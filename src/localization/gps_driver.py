#!/usr/bin/env python3
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
# from rover_msgs import GPS, RTCM
from pyubx2 import UBXReader, UBX_PROTOCOL, RTCM3_PROTOCOL, protocol
from rtcm_msgs.msg import Message


class GPS_Driver():

    def __init__(self, port, baudrate):
        rospy.init_node('gps_driver')
        self.port = rospy.get_param("rover_gps_driver/port")
        self.baud = rospy.get_param("rover_gps_driver/baud")
        self.base_station_sub = rospy.Subscriber('/rtcm', Message, self.process_rtcm)
        # self.gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=1)
        self.lock = threading.Lock()


    def connect(self):
        #open connection to rover GPS
        self.ser = serial.Serial(self.port, self.baud)
        self.reader = UBXReader(self.ser, protfilter=(UBX_PROTOCOL | RTCM3_PROTOCOL))

        return self

    def exit(self) -> None:
        #close connection
        self.ser.close()

    def process_rtcm(self, data) -> None:
        print("processing RTCM")
        with self.lock:
            # rtcm_data = RTCM.decode(data)
            self.ser.write(data.message)

    def parse_rover_gps_data(self, rover_gps_data) -> None:
        msg = rover_gps_data
        # skip if message could not be parsed
        if not msg:
            return
        
        try:
            print(msg.identity)
            rospy.loginfo(vars(rover_gps_data))

        except:
            pass


        print("parsing GPS")
                
        if rover_gps_data.identity == "RXM-RTCM":
            print("RXM")
            # rospy.loginfo(vars(rover_gps_data))
        
        if rover_gps_data.identity == "NAV-PVT":
            print("PVT")
            rospy.loginfo(vars(rover_gps_data))

        if rover_gps_data.identity == "NAV-STATUS":
            print("NAV STATUS")



    def gps_data_thread(self) -> None:
        #TODO: add more message checks if needed
        while not rospy.is_shutdown():
            with self.lock:
                if self.ser.in_waiting:
                    raw, rover_gps_data = self.reader.read()
                    parsed_gps_data = self.parse_rover_gps_data(rover_gps_data)


def main():
    rospy.logerr("rover gps driver")
    #change values
    rtk_manager = GPS_Driver("port", baudrate=38400)
    rtk_manager.connect()
    #rover_gps_thread = threading.Thread(rtk_manager.gps_data_thread)
    #rover_gps_thread.start()
    rtk_manager.gps_data_thread()
    rtk_manager.exit()


if __name__ == "__main__":
    main()