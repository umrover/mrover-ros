#!/usr/bin/env python3

""" Manages communication to and from the science board.
The science_board codebase deals with reading and parsing NMEA like messages
from the STM32 chip on the science PCB over UART to complete tasks for almost
every mission. These tasks include operating the science box and getting
relevant data during the Science task, controlling the arm laser during the
Equipment Servicing task, and controlling the LED array used during the
Autonomous Traversal task.
"""
import threading
from typing import Any, Callable, Dict, List

# TODO: Look at Science/PDB/Fuse MCU Documentation, add handler for each topic, check yaml for number range for Publisher range

import numpy as np
import rospy
import serial
from mrover.msg import Current, Enable, Heater, Spectral, Triad, Temperature, ScienceTemperature, DiagTemperature
from mrover.srv import (
    ChangeAutonLEDState,
    ChangeAutonLEDStateRequest,
    ChangeAutonLEDStateResponse,
    ChangeDeviceState,
    ChangeDeviceStateRequest,
    ChangeDeviceStateResponse,
    ChangeHeaterState,
    ChangeHeaterStateRequest,
    ChangeHeaterStateResponse,
    ChangeServoAngles,
    ChangeServoAnglesRequest,
    ChangeServoAnglesResponse,
)


class ScienceBridge:
    """Manages all the information and functions used for dealing with the
    bridge between the science board and the Jetson.
    One ScienceBridge will be made in main.
    :param _id_by_color: A dictionary that maps the possible colors to an
        integer for the UART message.
    :param _handler_function_by_tag: A dictionay that maps each NMEA tag for a
        UART message to its corresponding callback function that returns a ROS
        struct with the packaged data.
    :param _mosfet_number_by_device_name: A dictionary that maps each actual
        device to a MOSFET device number.
    :param _ros_publisher_by_tag: A dictionary that maps each NMEA tag for a
        UART message to its corresponding ROS Publisher object.
    :param _sleep: A float representing the sleep duration used for when the
        sleep function is called.
    :param _uart_transmit_msg_len: An integer representing the maximum length
        for a transmitted UART message.
    :param _uart_lock: A lock used to prevent clashing over the UART transmit
        line.
    """

    _fuse_pdb_current_pub: rospy.Publisher
    _fuse_pdb_thermistor_pub: rospy.Publisher
    _id_by_color: Dict[str, int]
    _handler_function_by_tag: Dict[str, Callable[[str], Any]]
    _heater_auto_shut_off_pub: rospy.Publisher
    _heater_state_pub: rospy.Publisher
    _mosfet_number_by_device_name: Dict[str, int]
    _sleep: float
    _spectral_pub: rospy.Publisher
    _science_thermistor_pub: rospy.Publisher
    _triad_pub: rospy.Publisher
    _uart_transmit_msg_len: int
    _uart_lock: threading.Lock

    def __init__(self) -> None:
        self._fuse_pdb_thermistor_pub = rospy.Publisher(
            "diagnostic_data/temperatures", Temperature, queue_size=1)
        self._fuse_pdb_current_pub = rospy.Publisher(
            "diagnostic_data/current", Current, queue_size=1)
        self._id_by_color = rospy.get_param("/science_board/color_ids")
        self._mosfet_number_by_device_name = rospy.get_param(
            "science_board/device_mosfet_numbers")
        self._num_diag_current = rospy.get_param(
            "/science_board/info/diag_current")
        self._num_diag_thermistors = rospy.get_param(
            "/science_board/info/diag_thermistors")
        self._num_science_thermistors = rospy.get_param(
            "/science_board/info/science_thermistors")
        self._num_spectral = rospy.get_param(
            "/science_board/info/spectral")
        self._handler_function_by_tag = {
            "AUTOSHUTOFF": self._heater_auto_shut_off_handler,
            "DIAG_CURRENT": self._fuse_pdb_current_handler,
            "DIAG_TEMP": self._fuse_pdb_thermistor_handler,
            "HEATER": self._heater_state_handler,
            "SPECTRAL": self._spectral_handler,
            "SCIENCE_TEMP": self._science_thermistor_handler,
            "TRIAD": self._triad_handler,
        }
        self._heater_auto_shut_off_pub = rospy.Publisher(
            "science/heater_auto_shut_off_state_data", Enable, queue_size=1)
        self._heater_state_pub = rospy.Publisher(
            "science/heater_state_data", Heater, queue_size=1)
        self._science_thermistor_pub = rospy.Publisher(
            "science_data/temperatures", Temperature, queue_size=1)
        self._sleep = rospy.get_param("/science_board/info/sleep")
        self._spectral_pub = rospy.Publisher(
            "science_data/spectral", Spectral, queue_size=1)

        self._triad_pub = rospy.Publisher(
            "science/spectral_triad", Triad, queue_size=1)
        self._uart_transmit_msg_len = rospy.get_param(
            "/science_board/info/uart_transmit_msg_len")
        self._uart_lock = threading.Lock()

    def __enter__(self) -> None:
        """
        Opens a serial connection to the STM32 chip
        """

        self.ser = serial.Serial(
            port=rospy.get_param("/science_board/serial/port"),
            baudrate=rospy.get_param("/science_board/serial/baud_rate"),
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=rospy.get_param("/science_board/serial/timeout"),
        )

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        """
        Closes serial connection to STM32 chip
        """
        self.ser.close()

    def handle_change_arm_laser_state(self, req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
        """Processes a request to change the laser state of the arm by issuing
        the command to the STM32 chip via UART. Returns the success of the
        transaction.
        :param req: A boolean that is the requested arm laser state.
        :returns: A boolean that is the success of sent UART transaction.
        """
        success = self._send_mosfet_msg("arm_laser", req.enable)
        return ChangeDeviceStateResponse(success)

    def handle_change_auton_led_state(self, req: ChangeAutonLEDStateRequest) -> ChangeAutonLEDStateResponse:
        """Processes a request to change the auton LED array state by issuing
        the command to the STM32 chip via UART. Returns the success of the
        transaction.
        :param req: A string that is the color of the requested state of the
            auton LED array. Note that green actually means blinking green.
        :returns: A boolean that is the success of sent UART transaction.
        """
        success = self._auton_led_transmit(req.color.lower())
        return ChangeAutonLEDStateResponse(success)

    def handle_change_heater_auto_shut_off_state(self, req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
        """Processes a request to change the auto shut off state of the
        carousel heaters by issuing the command to the STM32 chip via UART.
        Returns the success of the transaction.
        :param req: A boolean that is the requested auto shut off state of the
            carousel heaters.
        :returns: A boolean that is the success of sent UART transaction.
        """
        success = self._heater_auto_shut_off_transmit(req.enable)
        return ChangeDeviceStateResponse(success)

    def handle_change_heater_state(self, req: ChangeHeaterStateRequest) -> ChangeHeaterStateResponse:
        """Processes a request to change the carousel heater state by issuing
        the command to the STM32 chip via UART. Returns the success of the
        transaction.
        :param req: A ChangeHeaterStateRequest object that has the following:
            - an int that is the heater device that should be changed.
            - a boolean that is the requested heater state of that device.
        :returns: A boolean that is the success of sent UART transaction.
        """
        success = self._heater_transmit(req.device, req.color)
        return ChangeHeaterStateResponse(success)

    def handle_change_servo_angles(self, req: ChangeServoAnglesRequest) -> ChangeServoAnglesResponse:
        """Processes a request to change the angles of three carousel servos by
        issuing the command to the STM32 chip via UART. Returns the success of
        transaction.
        :param req: A ChangeServoAnglesRequest object that has an int and
            a float representing the id and the angle respectively.
        :returns: A boolean that is the success of sent UART transaction.
        """
        success = self._servo_transmit(req.id, req.angle)
        return ChangeServoAnglesResponse(success)

    def handle_change_uv_led_carousel_state(self, req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
        """Processes a request to change the carousel UV LED by
        issuing the command to the STM32 chip via UART. Returns the success of
        transaction.
        :param req: A boolean that is the requested UV LED state.
        :returns: A boolean that is the success of sent UART transaction.
        """
        success = self._send_mosfet_msg("uv_led_carousel", req.enable)
        return ChangeDeviceStateResponse(success)

    def handle_change_uv_led_end_effector_state(self, req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
        """Processes a request to change the UV LED on the end effector by
        issuing the command to the STM32 chip via UART. Returns the success of
        transaction.
        :param req: A boolean that is the requested UV LED state.
        :returns: A boolean that is the success of sent UART transaction.
        """
        success = self._send_mosfet_msg("uv_led_end_effector", req.enable)
        return ChangeDeviceStateResponse(success)

    def handle_change_white_led_state(self, req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
        """Processes a request to change the carousel white LED by
        issuing the command to the STM32 chip via UART. Returns the success of
        transaction.
        :param: req: A boolean that is the requested white LED state.
        :returns: A boolean that is the success of sent UART transaction.
        """
        success = self._send_mosfet_msg("white_led", req.enable)
        return ChangeDeviceStateResponse(success)

    def receive(self) -> None:
        """Reads in a message from the UART RX line and processes it. If there
        is data, then the data will be published to the corresponding ROS
        topic.
        Depending on the NMEA tag of the message, the proper function
        will be called to process the message.
        """
        tx_msg = self._read_msg()
        arr = tx_msg.split(",")

        # error if tag doesn't start with '$' or is empty
        if len(tx_msg) == 0 or arr[0][0] != '$':
            rospy.loginfo(f"Error decoding message stream: {tx_msg}")
            rospy.sleep(self._sleep)

        tag = arr[0][1:]
        if not self._handler_function_by_tag.get_key(tag):
            rospy.loginfo(f"Error decoding message stream: {tx_msg}")
            rospy.sleep(self._sleep)

        rospy.loginfo(tx_msg)
        self._handler_function_by_tag(tag)

    def _add_padding(self, tx_msg: str) -> str:
        """Adds padding to a UART messages until it is a certain length.
        This certain length is determined by the input in the
        config/science_board.yaml file. This is so that the STM32 chip can
        expect to only receive messages of this particular length.
        :param tx_msg: The raw string that is to be sent without padding.
        :returns: The filled string that is to be sent with padding and is of
            certain length.
        """

        length = len(tx_msg)
        assert length <= self._uart_transmit_msg_len, "tx_msg should not be greater than self._uart_transmit_msg_len"
        list_msg = ["f{tx_msg}"]
        missing_characters = self._uart_transmit_msg_len - length
        list_dummy = [","] * missing_characters
        list_total = list_msg + list_dummy
        new_msg = "".join(list_total)
        return new_msg

    def _auton_led_transmit(self, color: str) -> bool:
        """Sends a UART message to the STM32 commanding the auton LED array
        state.
        :param color: A string that is the color of the requested state of
            the auton LED array. Note that green actually means blinking green.
            The string should be lower case.
        :returns: A boolean that is the success of the transaction. Note that
            this could be because an invalid color was sent.
        """
        assert color.islower(), "color should be lower case"
        if color not in self._id_by_color.keys():
            return False
        requested_state = self._id_by_color[color]
        msg = f"$LED,{requested_state}"
        success = self._send_msg(msg)
        return success

    def _format_mosfet_msg(self, device: int, enable: bool) -> str:
        """Creates a message that can be sent over UART to command a MOSFET
        device.
        :param device: An int that is the MOSFET device that can be changed.
        :param enable: A boolean that is the requested MOSFET device state.
            True means that the MOSFET device will connect to ground and
            activate the device.
        :returns: The raw string that has the device and enable information.
            Note that this is not yet ready to be transmitted to the STM32 chip
            over UART since it is not of the proper length yet.
        """
        tx_msg = f"$MOSFET,{device},{int(enable)}"
        return tx_msg

    def _heater_auto_shut_off_handler(self, tx_msg: str) -> None:
        """Processes a UART message that contains data of the auto shut off
        state of the carousel heaters and packages it into a ROS struct.
        :param tx_msg: A string that was received from UART that contains data
            of the carousel heater auto shut off state.
            - Format: <"$AUTOSHUTOFF,device,enable">
        :returns: An Enable struct that has a boolean that is the requested
            auto shut off state of the carousel heaters.
        """
        arr = tx_msg.split(",")
        ros_msg = Enable(enable=bool(int(arr[1])))
        self._heater_auto_shut_off_pub.publish(ros_msg)

    def _heater_auto_shut_off_transmit(self, enable: bool) -> bool:
        """Sends a UART message to the STM32 chip commanding the auto shut off
        state of the carousel heaters.
        :param enable: A boolean that is the auto shut off state of the
            carousel heaters.
        :returns: A boolean that is the success of the transaction.
        """
        tx_msg = f"$AUTOSHUTOFF,{enable}"
        success = self._send_msg(tx_msg)
        return success

    def _heater_state_handler(self, tx_msg: str) -> None:
        """Processes a UART message that contains data of the state of a
        carousel heater and packages it into a ROS struct.
        :param tx_msg: A string that was received from UART that contains data
            of the carousel heater auto shut off state.
            - Format: <"$HEATER,device,enable">
        :returns: An Heater struct that has the following:
            - an int that is the heater device that was changed.
            - a boolean that is the heater state of that device.
        """
        arr = tx_msg.split(",")
        ros_msg = Heater(device=int(arr[1]), enable=bool(int(arr[2])))
        self._heater_state_pub.publish(ros_msg)

    def _heater_transmit(self, device: int, enable: bool) -> bool:
        """Sends a UART message to the STM32 chip commanding the state of a
        particular heater device.
        :param device: An int that is the heater device that should be changed.
        :param enable: A boolean that is the requested heater state of that
            device.
        :returns: A boolean that is the success of the transaction.
        """
        heater_device_string = f"heater_{device}"
        translated_device = self._mosfet_number_by_device_name[heater_device_string]
        tx_msg = self._format_mosfet_msg(translated_device, enable)
        success = self._send_msg(tx_msg)
        return success

    def _read_msg(self) -> str:
        """Reads a message on the UART receive line from the STM32 chip.
        :returns: A string that is the received message. Note that this may be
            an empty string if there are no messages.
        """
        try:
            self._uart_lock.acquire()
            msg = self.ser.readline()
            self._uart_lock.release()
        except serial.SerialException:
            if self._uart_lock.locked():
                self._uart_lock.release()
            rospy.logerror("Errored in _read_msg")
        return str(msg)

    def _send_mosfet_msg(self, device_name: str, enable: bool) -> bool:
        """Sends a MOSFET message on the UART transmit line to the STM32 chip.
        :param device_name: A string that is the device that wants to be
            changed.
        :param enable: A boolean that is the requested device state.
        :returns: A boolean that is the success of sent UART transaction.
        """
        translated_device = self._mosfet_number_by_device_name[device_name]
        tx_msg = self._format_mosfet_msg(translated_device, enable)
        success = self._send_msg(tx_msg)
        return success

    def _send_msg(self, tx_msg: str) -> bool:
        """Sends the message on the UART transmit line to the STM32 chip.
        :param tx_msg: The raw string that is to be sent to the UART transmit
            line.
        :returns: A boolean that is the success of sent UART transaction.
        """
        try:
            tx_msg = self._add_padding(tx_msg)
            if len(tx_msg) > self._uart_transmit_msg_len:
                tx_msg = tx_msg[: self._uart_transmit_msg_len]
            rospy.loginfo(tx_msg)
            self._uart_lock.acquire()
            self.ser.close()
            self.ser.open()
            self.ser.write(bytes(tx_msg, encoding="utf-8"))
            self._uart_lock.release()
            return True
        except serial.SerialException as exc:
            if self._uart_lock.locked():
                self._uart_lock.release()
            rospy.logerror(f"Error in _send_msg: {exc}")
        return False

    def _servo_transmit(self, id: int, angle: float) -> bool:
        """Sends a UART message to the STM32 chip commanding the angles of the
        three carousel servos.
        :param angles: Three floats in an array that represent the angles of
            the servos. Note that this angle is what the servo interprets as
            the angle, and it may differ from servo to servo. Also note that
            the range of angles may vary (the safe range is between about 0
            and 150).
        :returns: A boolean that is the success of the transaction.
        """
        tx_msg = f"$SERVO,{id},{angle}"
        success = self._send_msg(tx_msg)
        return success

    def _spectral_handler(self, tx_msg: str) -> None:
        """Processes a UART message that contains data of the three carousel
        spectral sensors and packages it into a ROS struct.
        There are 3 spectral sensors, each having 6 channels. We read a
        uint16_t from each channel. We know which sensor is which because we
        know what the site is. The Jetson reads byte by byte, so the program
        combines every two bytes of information into a uint16_t.
        :param tx_msg: A string that was received from UART that contains data
            of the three carousel spectral sensors.
            - Format: <"SPECTRAL,site,ch_0,ch_1,....ch_5">
        :returns: A Spectral struct that has a string that is the site of the
            spectral and six floats that is the data of the a carousel
            spectral sensor.
        """
        arr = tx_msg.split(",")
        spectral_data = []
        for i in range(self._num_spectral):
            spectral_data.append(Spectral(int(arr[i + 1])))
        self._spectral_pub.publish(spectral_data)

    def _science_thermistor_handler(self, tx_msg: str) -> None:
        """TODO: explain function 
        :param tx_msg: A string that was received from UART that contains data
            of the three carousel spectral sensors.
            - Format: $SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>
        :returns: A Thermistor struct that has an int thermistor ID and a float
            that is the temperature of the carousel thermistor in Celsius.
        """
        arr = tx_msg.split(",")
        temperature_values = []

        for i in range(self._num_science_thermistors):
            temperature_values.append(Temperature(float(arr[i + 1])))

        ros_msg = ScienceTemperature(temperature_values)
        self._science_thermistor_pub.publish(ros_msg)

    def _fuse_pdb_thermistor_handler(self, tx_msg: str) -> None:
        """TODO: explain function 
            - Format: $DIAG_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>
        :returns: A Thermistor struct that has an int thermistor ID and a float
            that is the temperature of the carousel thermistor in Celsius.
        """
        arr = tx_msg.split(",")
        temperature_values = []

        for i in range(self._num_diag_thermistors):
            temperature_values.append(Temperature(float(arr[i + 1])))

        ros_msg = DiagTemperature(temperature_values)
        self._fuse_pdb_thermistor_pub.publish(ros_msg)

    def _fuse_pdb_current_handler(self, tx_msg: str) -> None:
        """ TODO: explain function 
        :param tx_msg: A string that was received from UART that contains data
            of the three carousel spectral sensors.
            - Format: $DIAG_CURRENT,<CURR_0>
        :returns: A Current struct that has the integer sensor ID and a float
            for the current in amps.
        """
        arr = tx_msg.split(",")
        current_values = []
        for i in range(self._num_diag_current):
            current_values.append(float(arr[i + 1]))
        ros_msg = Current(current_values)
        self._fuse_pdb_current_pub.publish(ros_msg)

    def _triad_handler(self, tx_msg: str) -> None:
        """Processes a UART message that contains data of the triad sensor on
        the end effector and packages it into a ROS struct.
        There are triad sensor has 18 channels. We read a uint16_t from each
        channel. The Jetson reads byte by byte, so the program combines every
        two bytes of information into a uint16_t.
        :param tx_msg: A string that was received from UART that contains data
            of the triad sensor.
            - Format: <"TRIAD,ch_0_0,ch_0_1,....ch_2_5">
        :returns: A Triad struct that has 18 floats that is the data of the
            triad sensor.
        """
        tx_msg.split(",")
        arr = [s.strip().strip("\x00") for s in tx_msg.split(",")]
        ros_msg = Triad()
        count = 1
        for index in range(len(ros_msg.data)):
            if not count >= len(arr):
                ros_msg.data[index] = 0xFFFF & (
                    (np.uint8(arr[count + 1]) << 8) | np.uint8(arr[count]))
            else:
                ros_msg.data[index] = 0
            count += 2
        self._triad_pub.publish(ros_msg)


def main():
    rospy.init_node("science_board")
    with ScienceBridge() as bridge:
        rospy.Service("change_arm_laser_state", ChangeDeviceState,
                      bridge.handle_change_arm_laser_state)
        rospy.Service("change_auton_led_state", ChangeAutonLEDState,
                      bridge.handle_change_auton_led_state)
        rospy.Service(
            "change_heater_auto_shut_off_state", ChangeDeviceState, bridge.handle_change_heater_auto_shut_off_state
        )
        rospy.Service("change_heater_state", ChangeHeaterState,
                      bridge.handle_change_heater_state)
        rospy.Service("change_servo_angles", ChangeServoAngles,
                      bridge.handle_change_servo_angles)
        rospy.Service("change_uv_led_carousel_state", ChangeDeviceState,
                      bridge.handle_change_uv_led_carousel_state)
        rospy.Service(
            "change_uv_led_end_effector_state", ChangeDeviceState, bridge.handle_change_uv_led_end_effector_state
        )
        rospy.Service("change_white_led_state", ChangeDeviceState,
                      bridge.handle_change_white_led_state)
        while not rospy.is_shutdown():
            bridge.receive()


if __name__ == "__main__":
    main()
