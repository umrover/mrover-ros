'''
Writes, reads and parses NMEA like messages from the onboard
science nucleo to operate the science boxes and get relevant data
'''
import serial
# import Adafruit_BBIO.UART as UART  # for beaglebone use only
import numpy as np
from enum import Enum
import rospy
from mrover import AutonLED, Enable, Heater, Servo, Spectral, Thermistor


class LED_state(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2
    NONE = 3


# Mapping of Heater devices to actual mosfet devices
class Heater_devices(Enum):
    HEATER_0 = 7
    HEATER_1 = 8
    HEATER_2 = 9


Heater_Map = [Heater_devices.HEATER_0.value, Heater_devices.HEATER_1.value, Heater_devices.HEATER_2.value]

UART_TRANSMIT_uart_msg_LEN = 30


class ScienceBridge():
    def __init__(self):
        # UART.setup("UART4")  # Specific to beaglebone
        # maps NMEA uart_msgs to their handler
        self.NMEA_HANDLE_MAPPER = {
            "SPECTRAL": self.spectral_handler,
            "THERMISTOR": self.thermistor_handler,
            "TRIAD": self.triad_handler,
            "TXT": self.txt_handler,
            "HEATER": self.heater_state_handler,
            "AUTOSHUTOFF": self.heater_shut_off_handler
        }

        self.spectral_publisher = rospy.Publisher('spectral_data', Spectral, queue_size=1)
        self.thermistor_publisher = rospy.Publisher('thermistor_data', Thermistor, queue_size=1)
        self.triad_publisher = rospy.Publisher('spectral_triad_data', Spectral, queue_size=1)
        self.heater_publisher = rospy.Publisher('heater_state_data', Heater, queue_size=1)
        self.heater_auto_shut_off_publisher = rospy.Publisher('heater_auto_shut_off_data', Enable, queue_size=1)
        

        self.max_error_count = 20
        self.sleep = .01
        self.previous_auton_uart_msg = "Default"

        self.led_map = {
            "Red": LED_state.RED,
            "Blue": LED_state.BLUE,
            "Green": LED_state.GREEN
        }

        self.Mosfet_dev_map={
            "ArmLaser" : 1
            "RamanLaser" : 10
            "UVBulb" : 6
            "UVLED" : 4
            "WhiteLED" : 5
        }

    def __enter__(self):
        '''
        Opens a serial connection to the nucleo
        '''
        self.ser = serial.Serial(
            port='/dev/ttyTHS0',
            baudrate=38400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        '''
        Closes serial connection to nucleo
        '''
        self.ser.close()
    
    def add_padding(self, uart_msg):
        while(len(uart_msg) < UART_TRANSMIT_uart_msg_LEN):
            uart_msg += ","
        return uart_msg

    def auton_led(self, ros_msg):
        try:
            requested_state = self.led_map[ros_msg.color]
            print("Received new auton led request: Turning " + str(ros_msg.color))
        except KeyError:
            requested_state = LED_state.NONE
            print("Received invalid/Off auton led request: Turning OFF all colors")

        led_message = "$LED,{led_color}".format(led_color=requested_state.value)
        self.uart_send(led_message)

    def format_mosfet_message(self, device, enable):
        mosfet_message = "$MOSFET,{dev},{en},"
        return mosfet_message.format(dev=device, en=enable)

    def heater_auto_shut_off_transmit(self, ros_msg):
        # Send the nucleo a message telling if auto shut off for heaters is off or on
        message = "$AUTOSHUTOFF,{enable}"
        message = message.format(enable=int(ros_msg.auto_shut_off_enabled))
        self.uart_send(message)

    def heater_shut_off_handler(self, uart_msg, ros_msg):
        # uart_msg format: <"$AUTOSHUTOFF,device,enabled">
        try:
            arr = uart_msg.split(",")
            enabled = bool(int(arr[1]))
            ros_msg.enable = enabled
        except Exception as e:
            print(e)

    def heater_state_handler(self, uart_msg, ros_msg):
        # uart_msg format: <"$HEATER,device,enabled">
        try:
            arr = uart_msg.split(",")
            # Send back the heater and the state
            ros_msg.device = int(arr[1])
            ros_msg.enable = bool(int(arr[2]))
        except Exception as e:
            print(e)

    def heater_transmit(self, ros_msg):
        # Upon Receiving a heater on/off command, send a command for the appropriate mosfet
        print("Heater cmd callback")
        translated_device = Heater_Map[ros_msg.device]
        message = self.format_mosfet_message(translated_device, int(ros_msg.enable))
        self.uart_send(message)

    def mosfet_transmit(self, ros_msg, device_name):
        print(device_name + "callback")
        translated_device = self.Mosfet_dev_map[device_name]
        message = self.format_mosfet_message(translated_device, int(ros_msg.enable))
        self.uart_send(message)
        pass

    def servo_transmit(self, ros_msg):
        # get cmd lcm and send to nucleo
        print("Received Servo Cmd")
        # parse data into expected format
        message = "$SERVO,{angle_0},{angle_1},{angle_2}"
        message = message.format(angle_0=ros_msg.angle_0, angle_1=ros_msg.angle_1, angle_2=ros_msg.angle_2)
        self.uart_send(message)

    def spectral_handler(self, m, ros_msg):
        # uart_msg format: <"$SPECTRAL,d0_1,d0_2, .... , d2_6">
        try:
            m.split(',')
            arr = [s.strip().strip('\x00') for s in m.split(',')]
            ros_msg_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                                "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                                "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]

            # There are 3 spectral sensors, each having 6 channels.
            # We read a uint16_t from each channel.
            # The jetson reads byte by byte, so the program combines every two byte of information
            # into a uint16_t.
            count = 1
            for var in ros_msg_variables:
                if (not (count >= len(arr))):
                    setattr(ros_msg, var, 0xFFFF & ((np.uint8(arr[count]) << 8) | (np.uint8(arr[count + 1]))))
                else:
                    setattr(ros_msg, var, 0)
                count += 2
        except Exception as e:
            print("spectral exception:", e)
            pass

        # parse the spectral UART uart_msg
        # add in relevant error handling
        # set the ros_msg variables

    def thermistor_handler(self, uart_msg, ros_msg):
        # uart_msg format: <"$THERMISTOR,temp_0,temp_1,temp_2">
        try:
            arr = uart_msg.split(",")
            ros_msg.temp_0 = float(arr[1])
            ros_msg.temp_1 = float(arr[2])
            ros_msg.temp_2 = float(arr[3])
        except Exception as e:
            print("thermistor exception:", e)
            pass
        # parse the thermistor UART uart_msg
        # adding relevant error handling
        # set the ros_msg variables

    def triad_handler(self, m, ros_msg):
        # uart_msg format: <"$TRIAD,d0_1,d0_2, .... , d2_6">
        try:

            m.split(',')
            arr = [s.strip().strip('\x00') for s in m.split(',')]
            ros_msg_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                                "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                                "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]

            # There are 18 channels.
            # We read a uint16_t from each channel.
            # The jetson reads byte by byte, so the program combines every two byte of information
            # into a uint16_t.
            count = 1
            for var in ros_msg_variables:
                if (not (count >= len(arr))):
                    pass
                    setattr(ros_msg, var, 0xFFFF & ((np.uint8(arr[count+1]) << 8) | (np.uint8(arr[count]))))
                else:
                    setattr(ros_msg, var, 0)
                count += 2
        except Exception as e:
            print("triad exception:", e)
            pass

    def txt_handler(self, uart_msg, ros_msg):
        '''
        Prints info messages revieved from nucleo to screen
        '''
        print(uart_msg)

    def uart_send(self, message):
        message = self.add_padding(message)
        print(message)
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))

    async def receive(self, lcm):
        spectral = Spectral()
        thermistor = Thermistor()
        triad = Spectral()
        heater = Heater()
        heater_auto_shut_off = Enable()

        # Mark TXT as always seen because they are not necessary
        seen_tags = {tag: False if not tag == 'TXT' else True
                     for tag in self.NMEA_HANDLE_MAPPER.keys()}
        while True:
            # Wait for all tags to be seen
            while (not all(seen_tags.values())):  # TODO -
                try:
                    error_counter = 0  # TODO - DELETE
                    tx = self.ser.readline()
                    uart_msg = str(tx)
                except Exception as e:
                    print("Errored")
                    if error_counter < self.max_error_count:
                        error_counter += 1
                        print(e)
                        await asyncio.sleep(self.sleep)
                        continue
                    else:
                        raise e
                match_found = False
                for tag, func in self.NMEA_HANDLE_MAPPER.items():
                    if tag in uart_msg:  # TODO - why do we have tag in uart_msg, func is not even used
                        print(uart_msg)
                        match_found = True
                        try:
                            if (tag == "AUTOSHUTOFF"):
                                self.heater_shut_off_handler(uart_msg, heater_auto_shut_off)
                                heater_auto_shut_off_publisher.publish(heater_auto_shut_off)
                            
                            elif (tag == "HEATER"):
                                self.heater_state_handler(uart_msg, heater)
                                heater_publisher.publish(heater)

                            elif (tag == "SPECTRAL"):
                                self.spectral_handler(tx.decode(), spectral)  # TODO - why is this tx.decode()
                                spectral_publisher.publish(spectral)

                            elif (tag == "THERMISTOR"):
                                self.thermistor_handler(uart_msg, thermistor)
                                thermistor_publisher.publish(thermistor)
                                
                            elif (tag == "TRIAD"):
                                self.triad_handler(uart_msg, triad)
                                triad_publisher.publish(triad)
                            
                            seen_tags[tag] = True  # TODO - move to top so not hidden, or just don't use.
                        except Exception as e:
                            print(e)
                        break
                if not match_found:
                    if not uart_msg:
                        print('Error decoding message stream: {}'.format(uart_msg))
                await asyncio.sleep(self.sleep)
            seen_tags = {tag: False if not tag == 'TXT' else True
                         for tag in self.NMEA_HANDLE_MAPPER.keys()}
            await asyncio.sleep(self.sleep)


def main():

    rospy.init_node("science")

    with ScienceBridge() as bridge:
        rospy.Subscriber("auton_led", AutonLED, bridge.auton_led_transmit)
        rospy.Subscriber("arm_laser_cmd", Enable, bridge.mosfet_transmit, "ArmLaser")
        rospy.Subscriber("heater_cmd", Heater, bridge.heater_transmit)
        rospy.Subscriber("heater_auto_shut_off_cmd", Enable, bridge.heater_auto_shut_off_transmit)
        rospy.Subscriber("raman_laser_cmd", Enable, bridge.mosfet_transmit, "RamanLaser")
        rospy.Subscriber("servo_cmd", Servo, bridge.servo_transmit)
        rospy.Subscriber("uv_bulb_cmd", Enable, bridge.mosfet_transmit, "UVBulb")
        rospy.Subscriber("uv_led_cmd", Enable, bridge.mosfet_transmit, "UVLED")
        rospy.Subscriber("white_led_cmd", Enable, bridge.mosfet_transmit, "WhiteLED")
    
    rospy.spin()

if __name__ == "__main__":
    main()