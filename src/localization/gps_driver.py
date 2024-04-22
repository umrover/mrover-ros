#!/usr/bin/env python3
"""
Reads and parses NMEA messages from the onboard GPS to provide
location data to the rover over LCM (/gps). Subscribes to
/rtcm and passes RTCM messages to the onboard gps to
acquire an RTK fix.
"""
import serial
import rospy
import threading
import rospy
from pyubx2 import UBXReader, UBX_PROTOCOL, RTCM3_PROTOCOL, POLL_LAYER_FLASH, UBXMessage
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from rtcm_msgs.msg import Message
from mrover.msg import rtkStatus
import datetime
import pyudev


class GPS_Driver:
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

    id_gps_done: threading.Event
    ID_GPS_CONFIG_KEY: str
    ID_GPS_BAUD: int
    ID_GPS_TIMEOUT: float
    ID_GPS_POLL_LAYER: int


    def identify_gps(self, stream: serial.Serial, lock: threading.Lock, reader: UBXReader):
        while self.id_gps_done.is_set() == False:
            with lock:
                if (stream.in_waiting):
                    raw, msg = reader.read()
                    if (msg.identity == "CFG-VALGET"):

                        if (msg.CFG_MSGOUT_UBX_RXM_RLM_UART2 == 1):
                            rospy.set_param("left_gps_driver/port", stream.port)
                            rospy.loginfo("GPS set as LEFT GPS")
                        else:
                            rospy.set_param("right_gps_driver/port", stream.port)
                            rospy.loginfo("GPS set as RIGHT GPS")

                        self.id_gps_done.set()

    def id_gps(self):
        context = pyudev.Context()

         # TODO: find enumeration parameters for gps units         
        for device in context.list_devices(sys_name="/dev/gps_*"):

            # create stream and reader
            port = device.device_path
            id_gps_stream = serial.Serial(port, self.ID_GPS_BAUD)
            id_gps_reader = UBXReader(id_gps_stream, protfilter=(UBX_PROTOCOL | RTCM3_PROTOCOL))
            
            # start reading thread to parse response
            id_gps_lock = threading.Lock()
            id_gps_thread = threading.Thread(self.identify_gps, args=(id_gps_stream, id_gps_lock, id_gps_reader), daemon=True)
            id_gps_thread.start()

            # send poll request
            success = False

            while self.id_gps_done.is_set() == False:
                position = 0
                poll_layer = self.ID_GPS_POLL_LAYER
                keys = [self.ID_GPS_CONFIG_KEY]
                msg = UBXMessage.config_poll(poll_layer, position, keys)
                id_gps_lock.acquire()
                id_gps_stream.write(msg.serialize())
                id_gps_lock.release()
                success = self.id_gps_done.wait(5)

            if success == False:
                rospy.logerr("Failed to ID GPS: " + str(port))

            # join reading thread to main thread and reset event
            id_gps_thread.join()
            self.id_gps_done.clear()

    def __init__(self):
        rospy.init_node("gps_driver")
        
        self.id_gps_done = threading.Event()
        self.ID_GPS_CONFIG_KEY = "CFG_MSGOUT_UBX_RXM_RLM_UART2"
        self.ID_GPS_POLL_LAYER = POLL_LAYER_FLASH
        self.ID_GPS_BAUD = 38400
        self.ID_GPS_TIMEOUT = 0.1

        self.id_gps()

        self.port = rospy.get_param("port")
        self.baud = rospy.get_param("baud")
        self.base_station_sub = rospy.Subscriber("/rtcm", Message, self.process_rtcm)
        self.gps_pub = rospy.Publisher("fix", NavSatFix, queue_size=1)
        self.rtk_fix_pub = rospy.Publisher("rtk_fix_status", rtkStatus, queue_size=1)

        self.lock = threading.Lock()
        self.valid_offset = False
        self.time_offset = -1

    def connect(self):
        # open connection to rover GPS
        self.ser = serial.Serial(self.port, self.baud)
        self.reader = UBXReader(self.ser, protfilter=(UBX_PROTOCOL | RTCM3_PROTOCOL))

        return self

    def exit(self) -> None:
        # close connection
        self.ser.close()

    # rospy subscriber automatically runs this callback in separate thread
    def process_rtcm(self, data) -> None:
        print("processing RTCM")
        with self.lock:
            self.ser.write(data.message)

    def parse_ubx_message(self, msg) -> None:
        # skip if message could not be parsed
        if not msg:
            return

        if msg.identity == "RXM-RTCM":
            rospy.loginfo("RXM")
            msg_used = msg.msgUsed

            if msg_used == 0:
                rospy.logwarn("RTCM Usage unknown\n")
            elif msg_used == 1:
                rospy.logwarn("RTCM message not used\n")
            elif msg_used == 2:
                rospy.loginfo("RTCM message successfully used by receiver\n")
            # rospy.loginfo(vars(rover_gps_data))

        elif msg.identity == "NAV-PVT":
            rospy.loginfo("PVT")
            parsed_latitude = msg.lat
            parsed_longitude = msg.lon
            parsed_altitude = msg.hMSL
            time = datetime.datetime(year=msg.year, month=msg.month, day=msg.day, hour=msg.hour, second=msg.second)
            time = rospy.Time(secs=time.timestamp() + (msg.nano / 1e6))
            if not self.valid_offset:
                self.time_offset = rospy.Time.now() - time
                self.valid_offset = True

            time = time + self.time_offset
            # rospy.loginfo_throttle(3, f"{time} {rospy.Time.now()} {time-rospy.Time.now()} {self.time_offset}")
            message_header = Header(stamp=time, frame_id="base_link")

            self.gps_pub.publish(
                NavSatFix(
                    header=message_header,
                    latitude=parsed_latitude,
                    longitude=parsed_longitude,
                    altitude=parsed_altitude,
                )
            )
            self.rtk_fix_pub.publish(rtkStatus(msg.carrSoln))

            if msg.difSoln == 1:
                rospy.loginfo_throttle(3, "Differential correction applied")
            if msg.carrSoln == 0:
                rospy.logwarn_throttle(3, "No RTK")
            elif msg.carrSoln == 1:
                rospy.loginfo_throttle(3, "Floating RTK Fix")
            elif msg.carrSoln == 2:
                rospy.loginfo_throttle(3, "RTK FIX")

        elif msg.identity == "NAV-STATUS":
            pass

    def gps_data_thread(self) -> None:
        while not rospy.is_shutdown():
            with self.lock:
                if self.ser.in_waiting:
                    raw, msg = self.reader.read()
                    self.parse_ubx_message(msg)


def main():
    rtk_manager = GPS_Driver()
    rtk_manager.connect()
    rtk_manager.gps_data_thread()
    rtk_manager.exit()


if __name__ == "__main__":
    main()
