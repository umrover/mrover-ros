'''
Writes, reads and parses NMEA like messages from the onboard
science nucleo to operate the science boxes and get relevant data
'''
import numpy as np
import rospy
from mrover import Enable, Heater, Servo, Spectral, Thermistor
from sciencecomms import send_msg, read_msg


class ScienceBridge():
    """ScienceBridge class"""
    def __init__(self):
        self.nmea_handle_mapper = {
            "AUTOSHUTOFF": self.heater_auto_shut_off_handler,
            "HEATER": self.heater_state_handler,
            "SPECTRAL": self.spectral_handler,
            "THERMISTOR": self.thermistor_handler,
            "TRIAD": self.triad_handler
        }
        self.nmea_publisher_mapper = {
            "AUTOSHUTOFF": rospy.Publisher('heater_auto_shut_off_data', Enable, queue_size=1),
            "HEATER": rospy.Publisher('heater_state_data', Heater, queue_size=1),
            "SPECTRAL": rospy.Publisher('spectral_data', Spectral, queue_size=1),
            "THERMISTOR": rospy.Publisher('thermistor_data', Thermistor, queue_size=1),
            "TRIAD": rospy.Publisher('spectral_triad_data', Spectral, queue_size=1)
        }
        self.nmea_message_mapper = {
            "AUTOSHUTOFF": Enable(),
            "HEATER": Heater(),
            "SPECTRAL": Spectral(),
            "THERMISTOR": Thermistor(),
            "TRIAD": Spectral()
        }
        self.sleep = .01
        # Mapping of onboard devices to mosfet devices
        self.mosfet_dev_map = {
            "arm_laser": 1,
            "uv_bulb": 1,
            "uv_led": 4,
            "white_led": 5
        }
        self.heater_map = [7, 8, 9]

    def __enter__(self) -> None:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        pass

    def format_mosfet_msg(self, device: int, enable: bool) -> str:
        """Formats a mosfet message"""
        tx_msg = "$MOSFET,{dev},{en},"
        return tx_msg.format(dev=device, en=enable)

    def heater_auto_shut_off_transmit(self, ros_msg: Enable) -> None:
        """Transmits a heater auto shut off command over uart"""
        tx_msg = "$AUTOSHUTOFF,{enable}"
        tx_msg = tx_msg.format(enable=int(ros_msg.auto_shut_off_enabled))
        send_msg(tx_msg)

    def heater_auto_shut_off_handler(self, tx_msg: str, ros_msg: Enable) -> None:
        """Handles a received heater auto shut off message"""
        # tx_msg format: <"$AUTOSHUTOFF,device,enabled">
        arr = tx_msg.split(",")
        enabled = bool(int(arr[1]))
        ros_msg.enable = enabled

    def heater_state_handler(self, tx_msg: str, ros_msg: Heater) -> None:
        """Handles a received heater state message"""
        # tx_msg format: <"$HEATER,device,enabled">
        arr = tx_msg.split(",")
        ros_msg.device = int(arr[1])
        ros_msg.enable = bool(int(arr[2]))

    def heater_transmit(self, ros_msg: Heater) -> None:
        """Transmits a heater state command message"""
        translated_device = self.heater_map[ros_msg.device]
        tx_msg = self.format_mosfet_msg(translated_device, int(ros_msg.enable))
        send_msg(tx_msg)

    def mosfet_transmit(self, ros_msg: Enable, device_name: str) -> None:
        """Transmits a mosfet device state command message"""
        translated_device = self.mosfet_dev_map[device_name]
        tx_msg = self.format_mosfet_msg(translated_device, int(ros_msg.enable))
        send_msg(tx_msg)

    def servo_transmit(self, ros_msg: Servo) -> None:
        """Transmits a servo angle command message"""
        tx_msg = "$SERVO,{angle_0},{angle_1},{angle_2}"
        tx_msg = tx_msg.format(angle_0=ros_msg.angle_0, angle_1=ros_msg.angle_1,
                                   angle_2=ros_msg.angle_2)
        send_msg(tx_msg)

    def spectral_handler(self, msg: str, ros_msg: Spectral) -> None:
        """Handles a received spectral data message"""
        msg.split(',')
        arr = [s.strip().strip('\x00') for s in msg.split(',')]
        ros_msg_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                             "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                             "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]
        # There are 3 spectral sensors, each having 6 channels.
        # We read a uint16_t from each channel.
        # The jetson reads byte by byte, so the program combines every two byte of information
        # into a uint16_t.
        count = 1
        for var in ros_msg_variables:
            if not count >= len(arr):
                setattr(ros_msg, var, 0xFFFF & ((np.uint8(arr[count]) << 8) |
                        (np.uint8(arr[count + 1]))))
            else:
                setattr(ros_msg, var, 0)
            count += 2

    def thermistor_handler(self, tx_msg: str, ros_msg: Thermistor) -> None:
        """Handles a receieved thermistor data message"""
        arr = tx_msg.split(",")
        ros_msg.temp_0 = float(arr[1])
        ros_msg.temp_1 = float(arr[2])
        ros_msg.temp_2 = float(arr[3])

    def triad_handler(self, msg: str, ros_msg: Spectral) -> None:
        """Handles a received triad data message"""
        msg.split(',')
        arr = [s.strip().strip('\x00') for s in msg.split(',')]
        ros_msg_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                             "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                             "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]

        # There are 18 channels.
        # We read a uint16_t from each channel.
        # The jetson reads byte by byte, so the program combines every two byte of information
        # into a uint16_t.
        count = 1
        for var in ros_msg_variables:
            if not count >= len(arr):
                setattr(ros_msg, var, 0xFFFF & ((np.uint8(arr[count + 1]) << 8) |
                        (np.uint8(arr[count]))))
            else:
                setattr(ros_msg, var, 0)
            count += 2

    def receive(self) -> None:
        """Infinite loop that reads in messages from the UART RX line and processes them"""
        while True:
            tx_msg = read_msg()
            match_found = False
            for tag, handler_func in self.nmea_handle_mapper.items():
                if tag in tx_msg:
                    print(tx_msg)
                    match_found = True
                    handler_func(tx_msg, self.nmea_message_mapper[tag])
                    self.nmea_publisher_mapper[tag].publish(self.nmea_message_mapper[tag])
                    break
            if not match_found:
                if not tx_msg:
                    print(f'Error decoding message stream: {tx_msg}')
            rospy.sleep(self.sleep)


def main():
    """Main function"""
    rospy.init_node("science")

    with ScienceBridge() as bridge:
        rospy.Subscriber("arm_laser_cmd", Enable, bridge.mosfet_transmit, "arm_laser")
        rospy.Subscriber("heater_cmd", Heater, bridge.heater_transmit)
        rospy.Subscriber("heater_auto_shut_off_cmd", Enable, bridge.heater_auto_shut_off_transmit)
        rospy.Subscriber("servo_cmd", Servo, bridge.servo_transmit)
        rospy.Subscriber("uv_bulb_cmd", Enable, bridge.mosfet_transmit, "uv_bulb")
        rospy.Subscriber("uv_led_cmd", Enable, bridge.mosfet_transmit, "uv_led")
        rospy.Subscriber("white_led_cmd", Enable, bridge.mosfet_transmit, "white_led")

        while not rospy.is_shutdown():
            bridge.receive()


if __name__ == "__main__":
    main()
