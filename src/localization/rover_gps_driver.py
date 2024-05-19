#!/usr/bin/env python3

"""
Reads and parses NMEA messages from the onboard GPS to provide
location data to the rover over LCM (/gps). Subscribes to
/rtcm and passes RTCM messages to the onboard gps to
acquire an RTK fix.
"""
from __future__ import annotations

import threading

import serial
from pyubx2 import UBXReader, UBX_PROTOCOL, RTCM3_PROTOCOL

import rospy
from mrover.msg import RTKStatus
from rtcm_msgs.msg import Message
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header


class GpsDriver:
    port: str
    baud: int
    base_station_sub: rospy.Subscriber
    gps_pub: rospy.Publisher
    rtk_fix_pub: rospy.Publisher
    lock: threading.Lock
    valid_offset: bool
    time_offset: float
    ser: serial.Serial
    reader: UBXReader

    def __init__(self) -> None:
        rospy.init_node("gps_driver")
        self.port = rospy.get_param("rover_gps_driver/port")
        self.baud = rospy.get_param("rover_gps_driver/baud")
        self.base_station_sub = rospy.Subscriber("/rtcm", Message, self.process_rtcm)
        self.gps_pub = rospy.Publisher("gps/fix", NavSatFix, queue_size=1)
        self.rtk_fix_pub = rospy.Publisher("rtk_fix_status", RTKStatus, queue_size=1)

        self.lock = threading.Lock()
        self.valid_offset = False
        self.time_offset = -1

    def connect(self) -> GpsDriver:
        self.ser = serial.Serial(self.port, self.baud)
        self.reader = UBXReader(self.ser, protfilter=(UBX_PROTOCOL | RTCM3_PROTOCOL))
        return self

    def exit(self) -> None:
        self.ser.close()

    # rospy subscriber automatically runs this callback in separate thread
    def process_rtcm(self, data: Message) -> None:
        with self.lock:
            self.ser.write(data.message)

    def parse_ubx_message(self, msg) -> None:
        if not msg:
            return

        match msg.identity:
            case "RXM-RTCM":
                match msg.msgUsed:
                    case 0:
                        rospy.logwarn("RTCM usage unknown")
                    case 1:
                        rospy.logwarn("RTCM message not used")
                    case 2:
                        rospy.logdebug("RTCM message successfully used by receiver")

            case "NAV-PVT":
                if msg.lat == 0 or msg.lon == 0:
                    rospy.logwarn_throttle(1, "Zero satellite fix. Are we inside?")
                    return
                # TODO(quintin): Use the time from the GPS message
                # time = datetime.datetime(year=msg.year, month=msg.month, day=msg.day, hour=msg.hour, second=msg.second)
                # time = rospy.Time(secs=time.timestamp() + (msg.nano / 1e6))
                # if not self.valid_offset:
                #     self.time_offset = rospy.Time.now() - time
                #     self.valid_offset = True
                #
                # time = time + self.time_offset
                self.gps_pub.publish(
                    NavSatFix(
                        header=Header(stamp=rospy.Time.now(), frame_id="base_link"),
                        latitude=msg.lat,
                        longitude=msg.lon,
                        altitude=msg.hMSL,
                    )
                )
                self.rtk_fix_pub.publish(RTKStatus(msg.carrSoln))

                if msg.difSoln == 1:
                    rospy.logdebug_throttle(3, "Differential correction applied")
                if msg.carrSoln == 0:
                    rospy.logwarn_throttle(3, "No RTK")
                elif msg.carrSoln == 1:
                    rospy.logdebug_throttle(3, "Floating RTK Fix")
                elif msg.carrSoln == 2:
                    rospy.logdebug_throttle(3, "RTK FIX")

            case "NAV-STATUS":
                pass

    def gps_data_thread(self) -> None:
        while not rospy.is_shutdown():
            with self.lock:
                if self.ser.in_waiting:
                    raw, msg = self.reader.read()
                    self.parse_ubx_message(msg)


def main():
    driver = GpsDriver()
    driver.connect()
    driver.gps_data_thread()
    driver.exit()


if __name__ == "__main__":
    main()
