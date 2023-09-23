#!/usr/bin/env python3

""" Manages communication to and from the science STM32 MCU.
The science codebase deals with reading and parsing NMEA-like messages
from the STM32 chip over UART to complete tasks for almost
every mission. These tasks include operating the science box and getting
relevant data during the Science task, as well as controlling the arm laser during the
Equipment Servicing task. It also transmits diagnostic data on temperature
and current on the 3.3V, 5V, and 12V lines for the PDB.
"""
import threading
from typing import Any, Callable, Dict, List

import rospy
import serial
import time as t

from mrover.msg import Diagnostic, HeaterData, ScienceTemperature, Spectral
from std_msgs.msg import Bool

from mrover.srv import (
    ChangeHeaterState,
    ChangeHeaterStateRequest,
    ChangeHeaterStateResponse,
    ChangeServoAngle,
    ChangeServoAngleRequest,
    ChangeServoAngleResponse,
    EnableDevice,
    EnableDeviceRequest,
    EnableDeviceResponse,
)

from std_srvs.srv import (
    SetBool,
    SetBoolRequest,
    SetBoolResponse,
)


class ScienceBridge:
    """Manages all the information and functions used for dealing with the
    bridge between the science board and the Jetson.
    One ScienceBridge will be made in main.
    :param _active_publisher: A publisher that publishes whether the
        science MCU is active or not (based on whether any messages were heard)
    :param _allowed_mosfet_names: A list of allowed mosfet names for the "enable
        mosfet" device service
    :param _mcu_active_timeout_s: This represents the maximum amount of time
        in seconds without receiving a UART message until the MCU is called
        not active.
    :param _mosfet_number_by_device_name: A dictionary that maps each actual
        device to a MOSFET device number.
    :param _num_diag_current: The number of diagnostic current sensors.
    :param _num_diag_thermistors: The number of diagnostic thermistors.
    :param _num_science_thermistors: The number of thermistors in the ISH box.
    :param _num_spectral: The number of spectral sensors in the ISH box.
    :param _handler_function_by_tag: A dictionay that maps each NMEA tag for a
        UART message to its corresponding callback function that returns a ROS
        struct with the packaged data.
    :param _ros_publisher_by_tag: A dictionary that maps each NMEA tag for a
        UART message to its corresponding ROS Publisher object.
    :param _sleep_amt_s: A float representing the sleep duration used for when the
        sleep function is called, in seconds.
    :param _time_since_last_received_msg: The time since last received a
        UART message.
    :param _uart_transmit_msg_len: An integer representing the maximum length
        for a transmitted UART message.
    :param _uart_lock: A lock used to prevent clashing over the UART transmit
        line.
    :param _last_mcu_active: The last note for whether the mcu was active or not.
    """

    NUM_SPECTRAL_CHANNELS = 6

    _active_publisher: rospy.Publisher
    _allowed_mosfet_names: List[str]
    _mcu_active_timeout_s: int
    _mosfet_number_by_device_name: Dict[str, int]
    _num_diag_current: int
    _num_diag_thermistors: int
    _num_science_thermistors: int
    _num_spectral: int
    _handler_function_by_tag: Dict[str, Callable[[str], Any]]
    _ros_publisher_by_tag: Dict[str, rospy.Publisher]
    _sleep_amt_s: float
    _time_since_last_received_msg: float
    _uart_transmit_msg_len: int
    _uart_lock: threading.Lock
    ser: serial.Serial
    _last_mcu_active: Bool

    def __init__(self) -> None:
        self._active_publisher = rospy.Publisher("science_mcu_active", Bool, queue_size=1)

        self._mcu_active_timeout_s = rospy.get_param("science/info/mcu_active_timeout_s")
        self._mosfet_number_by_device_name = rospy.get_param("science/device_mosfet_numbers")
        self._allowed_mosfet_names = ["arm_laser", "uv_led_carousel", "uv_led_end_effector", "white_led", "raman_laser"]

        self._num_diag_current = rospy.get_param("/science/info/num_diag_current")
        self._num_diag_thermistors = rospy.get_param("/science/info/num_diag_thermistors")
        self._num_science_thermistors = rospy.get_param("/science/info/num_science_thermistors")
        self._num_spectral = rospy.get_param("/science/info/num_spectral")

        self._handler_function_by_tag = {
            "AUTO_SHUTOFF": self._heater_auto_shutoff_handler,
            "DIAG": self._diagnostic_handler,
            "HEATER_DATA": self._heater_state_handler,
            "SCIENCE_TEMP": self._science_thermistor_handler,
            "SPECTRAL": self._spectral_handler,
        }
        self._ros_publisher_by_tag = {
            "AUTO_SHUTOFF": rospy.Publisher("science/heater_auto_shutoff_state_data", Bool, queue_size=1),
            "DIAG": rospy.Publisher("diagnostic_data", Diagnostic, queue_size=1),
            "HEATER_DATA": rospy.Publisher("science/heater_state_data", HeaterData, queue_size=1),
            "SCIENCE_TEMP": rospy.Publisher("science/temperatures", ScienceTemperature, queue_size=1),
            "SPECTRAL": rospy.Publisher("science/spectral", Spectral, queue_size=1),
        }

        self._sleep_amt_s = rospy.get_param("science/info/sleep")

        self._time_since_last_received_msg = t.time()

        self._uart_transmit_msg_len = rospy.get_param("science/info/uart_transmit_msg_len")
        self._uart_lock = threading.Lock()

        # Opens a serial connection to the STM32 chip
        self.ser = serial.Serial(
            port=rospy.get_param("science/serial/port"),
            baudrate=rospy.get_param("science/serial/baud_rate"),
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=rospy.get_param("science/serial/timeout"),
        )
        self._last_mcu_active = True

    def __del__(self) -> None:
        """
        Close serial connection to STM32 chip.
        """
        self.ser.close()

    def feed_uart_watchdog(self, event=None) -> bool:
        """Sends a message to the UART lines to feed the watchdog"""
        msg = "$WATCHDOG"
        success = self._send_msg(msg)
        return success

    def publish_mcu_active(self, event=None) -> bool:
        """Publish whether the mcu is active or not"""
        mcu_active = bool(t.time() - self._time_since_last_received_msg < self._mcu_active_timeout_s)
        self._last_mcu_active = mcu_active
        ros_msg = Bool(mcu_active)
        self._active_publisher.publish(ros_msg)
        return True

    def handle_enable_mosfet_device(self, req: EnableDeviceRequest) -> EnableDeviceResponse:
        """Process a request to change the state of a MOSFET device by issuing
        the command to the STM32 chip via UART.
        :param req: A boolean that is the requested arm laser state.
        :returns: A boolean that is the success of sent UART transaction.
        """

        if req.name in self._allowed_mosfet_names:
            success = self._send_mosfet_msg(req.name, req.enable)
            return EnableDeviceResponse(success)

        return EnableDeviceResponse(False)

    def handle_change_heater_auto_shutoff_state(self, req: SetBoolRequest) -> SetBoolResponse:
        """Process a request to change the auto shut off state of the
        carousel heaters by issuing the command to the STM32 chip via UART.
        :param req: A SetBoolRequest object that has
            a boolean that is the requested auto shut off state of the
            carousel heaters.
        :returns: A boolean that is the success of sent UART transaction.
        """
        success = self._heater_auto_shutoff_transmit(req.data)
        return SetBoolResponse(success=success, message="")

    def handle_change_heater_state(self, req: ChangeHeaterStateRequest) -> ChangeHeaterStateResponse:
        """Process a request to change the carousel heater state by issuing
        the command to the STM32 chip via UART.
        :param req: A ChangeHeaterStateRequest object that has the following:
            - an int that is the heater device that should be changed.
            - a boolean that is the requested heater state of that device.
        :returns: A boolean that is the success of sent UART transaction.
        """
        success = self._heater_transmit(req.device, req.enable)
        return ChangeHeaterStateResponse(success)

    def handle_change_servo_angle(self, req: ChangeServoAngleRequest) -> ChangeServoAngleResponse:
        """Process a request to change the angles of three carousel servos by
        issuing the command to the STM32 chip via UART.
        :param req: A ChangeServoAngleRequest object that has an int and
            a float representing the id and the angle respectively.
        :returns: A boolean that is the success of sent UART transaction.
        """
        if not 0 <= req.id < 3:
            # 0 <= req.id < 3 must be true since only 0, 1, and 2 are accepted.
            rospy.logerr(f"Site {req.id} for changing servo angle is invalid.")
            return ChangeServoAngleResponse(False)

        success = self._servo_transmit(req.id, req.angle)
        return ChangeServoAngleResponse(success)

    def _heater_auto_shutoff_transmit(self, enable: bool) -> bool:
        """Send a UART message to the STM32 chip commanding the auto shut off
        state of the carousel heaters.
            - Format: <"$AUTO_SHUTOFF,<VAL>">
        :param enable: A boolean that is the auto shut off state of the
            carousel heaters.
        :returns: A boolean that is the success of the transaction.
        """
        tx_msg = f"$AUTO_SHUTOFF,{int(enable)}"
        success = self._send_msg(tx_msg)
        return success

    def _heater_transmit(self, device: int, enable: bool) -> bool:
        """Send a UART message to the STM32 chip commanding the state of a
        particular heater device.
            - Format: <"$HEATER_CMD,<DEVICE>,<ENABLE>">
        :param device: An int that is the heater device that should be changed.
        :param enable: A boolean that is the requested heater state of that
            device.
        :returns: A boolean that is the success of the transaction.
        """
        tx_msg = f"$HEATER_CMD,{device},{int(enable)}"
        success = self._send_msg(tx_msg)
        return success

    def _servo_transmit(self, servo_id: int, angle: float) -> bool:
        """Send a UART message to the STM32 chip commanding the angles of the
        carousel servos.
        :param servo_id: A servo id. Should be 0, 1, or 2.
        :param angle: Three floats in an array that represent the angles of
            the servos. Note that this angle is what the servo interprets as
            the angle, and it may differ from servo to servo. Also note that
            the range of angles may vary (the safe range is between about 0
            and 150).
        :returns: A boolean that is the success of the transaction.
        """
        tx_msg = f"$SERVO,{servo_id},{angle}"
        success = self._send_msg(tx_msg)
        return success

    def _send_mosfet_msg(self, device_name: str, enable: bool) -> bool:
        """Send a MOSFET message on the UART transmit line to the STM32 chip.
        :param device_name: A string that is the device that wants to be
            changed.
        :param enable: A boolean that is the requested device state.
        :returns: A boolean that is the success of sent UART transaction.
        """
        device_num = self._mosfet_number_by_device_name[device_name]
        tx_msg = f"$MOSFET,{device_num},{int(enable)}"

        success = self._send_msg(tx_msg)
        return success

    def _send_msg(self, tx_msg: str) -> bool:
        """Send the message on the UART transmit line to the STM32 chip.
        :param tx_msg: The raw string that is to be sent to the UART transmit
            line.
        :returns: A boolean that is the success of sent UART transaction.
        """
        initial_len = len(tx_msg)

        if initial_len > self._uart_transmit_msg_len:
            rospy.logerr("UART message longer than expected.")
            return False

        # Add padding to the UART message until it is the length expected by the STM32 chip.
        tx_msg = tx_msg.ljust(self._uart_transmit_msg_len, ",")

        try:
            # Reset connection and send message.
            with self._uart_lock:
                self.ser.write(bytes(tx_msg, encoding="utf-8"))

        except serial.SerialException as exc:
            rospy.logerr(f"Error in _send_msg: {exc}")
            return False

        return True

    def receive(self) -> None:
        """Read in a message from the UART RX line and process it. If there
        is data, then the data will be published to the corresponding ROS
        topic. Depending on the NMEA tag of the message, the proper function
        will be called to process the message.
        """
        rx_msg = self._read_msg()
        if rx_msg == "":
            rospy.sleep(self._sleep_amt_s)
            return

        arr = rx_msg.split(",")

        tag = arr[0][3:]
        if not (tag in self._handler_function_by_tag):
            rospy.sleep(self._sleep_amt_s)
            return

        self._time_since_last_received_msg = t.time()

        try:
            self._handler_function_by_tag[tag](rx_msg)
        except ValueError as e:
            rospy.logerr(f"Received invalid message. Exception: {e}")

    def _read_msg(self) -> str:
        """Read a message on the UART receive line from the STM32 chip.
        :returns: A string that is the received message. Note that this may be
            an empty string if there are no messages.
        """
        try:
            with self._uart_lock:
                msg = self.ser.readline()

        except serial.SerialException as exc:
            rospy.logerr(f"Errored in _read_msg: {exc}")
            return ""

        return str(msg)

    def _diagnostic_handler(self, rx_msg: str) -> None:
        """Process a UART message that contains data of the temperature
        and current data of the 3.3V, 5V, and 12V lines. Then publish to the
        associated ROS topic.
        :param rx_msg: A string that was received from UART that contains data
            of the temperature and current data of the 3.3V, 5V, and 12V lines.
            - Format: $DIAG,<TEMP_0>,<TEMP_1>,<TEMP_2>,<CURR_0>,<CURR_1>,<CURR_2>
        """
        temperature_values = []
        current_values = []

        arr = rx_msg.split(",")

        if len(arr) < 1 + self._num_diag_thermistors + self._num_diag_current:
            rospy.logerr(f"Only {len(arr)} parameters in diagnostic handler")
            return

        for i in range(self._num_diag_thermistors):
            temperature_values.append(float(arr[i + 1]))
        for i in range(self._num_diag_current):
            current_values.append(float(arr[self._num_diag_thermistors + i + 1]))

        ros_msg = Diagnostic(temperatures=temperature_values, currents=current_values)
        self._ros_publisher_by_tag[arr[0][3:]].publish(ros_msg)

    def _heater_auto_shutoff_handler(self, rx_msg: str) -> None:
        """Process a UART message that contains data of the auto shut off
        state of the carousel heaters. Then publish to the associated ROS topic.
        :param rx_msg: A string that was received from UART that contains data
            of the carousel heater auto shut off state.
            - Format: <"$AUTO_SHUTOFF,enable">
        """
        arr = rx_msg.split(",")

        if len(arr) < 2:
            rospy.logerr(f"Only {len(arr)} parameters in auto shutoff handler")
            return
        ros_msg = Bool(bool(int(arr[1])))
        self._ros_publisher_by_tag[arr[0][3:]].publish(ros_msg)

    def _heater_state_handler(self, rx_msg: str) -> None:
        """Process a UART message that contains data of the state of a
        carousel heaters. Then publish to the associated ROS topic.
        :param rx_msg: A string that was received from UART that contains data
            of the carousel heater auto shut off state.
            - Format: <"$HEATER_DATA,<STATE_0>,<STATE_1>,<STATE_2>">
        """
        arr = rx_msg.split(",")

        if len(arr) < 4:
            rospy.logerr(f"Only {len(arr)} parameters in heater state handler")
            return

        heater_state = [bool(int(arr[1])), bool(int(arr[2])), bool(int(arr[3]))]
        ros_msg = HeaterData(state=heater_state)
        self._ros_publisher_by_tag[arr[0][3:]].publish(ros_msg)

    def _science_thermistor_handler(self, rx_msg: str) -> None:
        """Process a UART message that contains data of the
        thermistors onboard the carousel in Celsius. Then
        publish to the associated ROS topic.
        :param rx_msg: A string that was received from UART that contains data
            of the three carousel temperature sensors.
            - Format: $SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>,<EXTRA_PADDING>`
        """
        arr = rx_msg.split(",")

        if len(arr) < self._num_science_thermistors + 1:
            rospy.logerr(f"Only {len(arr)} parameters in science thermistor handler")
            return

        temperature_values = ScienceTemperature()

        for i in range(self._num_science_thermistors):
            temperature_values.temperatures[i] = float(arr[i + 1])

        self._ros_publisher_by_tag[arr[0][3:]].publish(temperature_values)

    def _spectral_handler(self, rx_msg: str) -> None:
        """Process a UART message that contains data of the carousel
        spectral sensor and publishes it to the corresponding topic. There are
        6 channels per sensor. Then publish to the associated ROS topic.
        :param rx_msg: A string that was received from UART that contains data
            of the three carousel spectral sensors.
            - Format: <"SPECTRAL,ch_0,ch_1,....ch_5">
        :returns: A Spectral struct that has a string has six floats that is
        the data of the carousel spectral sensor.
        """
        arr = rx_msg.split(",")

        if len(arr) < 1 + (self.NUM_SPECTRAL_CHANNELS * self._num_spectral):
            rospy.logerr(f"Only {len(arr)} parameters in science thermistor handler")
            return

        spectral_data = Spectral()
        for i in range(self.NUM_SPECTRAL_CHANNELS * self._num_spectral):
            spectral_data.data[i] = int(arr[i + 1])
        self._ros_publisher_by_tag[arr[0][3:]].publish(spectral_data)


def main():
    rospy.init_node("science")
    bridge = ScienceBridge()
    rospy.Service("enable_mosfet_device", EnableDevice, bridge.handle_enable_mosfet_device)
    rospy.Service("change_heater_state", ChangeHeaterState, bridge.handle_change_heater_state)
    rospy.Service("change_heater_auto_shutoff_state", SetBool, bridge.handle_change_heater_auto_shutoff_state)
    rospy.Service("change_servo_angle", ChangeServoAngle, bridge.handle_change_servo_angle)
    rospy.Timer(rospy.Duration(400.0 / 1000.0), bridge.feed_uart_watchdog)
    rospy.Timer(rospy.Duration(1.0), bridge.publish_mcu_active)

    while not rospy.is_shutdown():
        # receive() sleeps when no message is received.
        bridge.receive()


if __name__ == "__main__":
    main()
