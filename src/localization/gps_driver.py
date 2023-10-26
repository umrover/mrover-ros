'''
Reads and parses NMEA messages from the onboard GPS to provide
location data to the rover over LCM (/gps). Subscribes to
/rtcm and passes RTCM messages to the onboard gps to
acquire an RTK fix.
'''
import serial
import asyncio
import rospy
import threading
import numpy as np
from os import getenv
from rover_msgs import GPS, RTCM


class GPS_Driver():

    def __init__(self, port, baudrate):
        rospy.init_node('gps_driver')
        self.port = rospy.get_param("rover_gps_driver/port")
        self.baud = rospy.get_param("rover_gps_driver/baud")
        self.base_station_sub = rospy.Subscriber('/rtcm', RTCM, process_rtcm)
        self.gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=1)
        self.lock = threading.Lock()

    def connect(self):
        #open connection to rover GPS
        self.ser = serial.Serial(self.port, self.baudrate)
        return self

    def exit(self):
        #close connection
        self.ser.close()

    def process_rtcm(self, data):
        self.lock.acquire()
        rtcm_data = RTCM.decode(data)
        self.ser.write(rtcm_data)
        self.lock.release()

    def parse_rover_gps_data(self, rover_gps_data):
        pass

    def gps_data_thread(self):
        while not rospy.is_shutdown():
            self.lock.acquire()
            rover_gps_data = self.ser.readline()
            parsed_gps_data = self.parse_rover_gps_data(rover_gps_data)
            self.gps_pub.publish(parsed_gps_data)
            self.lock.release()


def main():
    #change values
    rtk_manager = GPS_Driver("port", baudrate=1500)
    rtk_manager.connect()
    rover_gps_thread = threading.Thread(rtk_manager.gps_data_thread)
    rover_gps_thread.start()


if __name__ == "__main__":
    main()