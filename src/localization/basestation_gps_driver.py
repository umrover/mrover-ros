#!/usr/bin/env python3
from pyubx2 import UBXReader, UBX_PROTOCOL, RTCM3_PROTOCOL, protocol
from serial import Serial

import rospy
from rtcm_msgs.msg import Message


def main() -> None:
    rtcm_pub = rospy.Publisher("rtcm", Message, queue_size=1)
    rospy.init_node("basestation_driver")
    port = rospy.get_param("basestation_gps_driver/port")
    baud = rospy.get_param("basestation_gps_driver/baud")
    svin_started = False
    svin_complete = False

    # connect to GPS over serial, only read UBX and RTCM messages
    with Serial(port, baud, timeout=1) as ser:
        reader = UBXReader(ser, protfilter=(UBX_PROTOCOL | RTCM3_PROTOCOL))
        while not rospy.is_shutdown():
            if ser.in_waiting:
                (raw_msg, msg) = reader.read()

                # skip if message could not be parsed
                if not msg:
                    continue

                # publish RTCM messages
                if protocol(raw_msg) == RTCM3_PROTOCOL:
                    rtcm_pub.publish(Message(message=raw_msg))

                # print survey-in status
                elif msg.identity == "NAV-SVIN":
                    if not svin_started and msg.active:
                        svin_started = True
                        rospy.loginfo("basestation survey-in started")
                    if not svin_complete and msg.valid:
                        svin_complete = True
                        rospy.loginfo(f"basestation survey-in complete, accuracy = {msg.meanAcc}")
                    if svin_started and not svin_complete:
                        rospy.loginfo(f"current accuracy: {msg.meanAcc}")

                # fix quality information
                elif msg.identity == "NAV-PVT":
                    rospy.loginfo(f"{'valid' if msg.gnssFixOk else 'invalid'} fix, {msg.numSV} satellites used")


if __name__ == "__main__":
    main()
