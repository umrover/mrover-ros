import json
import math
from channels.generic.websocket import JsonWebsocketConsumer

import rospy
import tf2_ros
import threading
from util.SE3 import SE3
from tf.transformations import euler_from_quaternion

from mrover.msg import PDB, ControllerState, CalibrationStatus
from mrover.srv import EnableDevice
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class GUIConsumer(JsonWebsocketConsumer):

    def connect(self):
        self.accept()
        self.pdb_sub = rospy.Subscriber('/pdb_data', PDB, self.pdb_callback)
        self.arm_moteus_sub = rospy.Subscriber('/arm_controller_data', ControllerState, self.arm_controller_callback)
        self.gps_fix = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_fix_callback)
        self.imu_calibration = rospy.Subscriber('/imu/calibration', CalibrationStatus, self.imu_calibration_callback)
        # rospy.wait_for_service("enable_limit_switches")
        self.limit_switch_service = rospy.ServiceProxy("enable_limit_switches", EnableDevice)

        self.tf_listener = threading.Thread(target=self.flight_attitude_listener)
        self.tf_listener.join()


    def disconnect(self, close_code):
        self.pdb_sub.unregister()

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """
        message = json.loads(text_data)
        if message["type"] == "enable_decive_srv":
            self.enable_device_callback(message)
        elif message["type"] == "disable_auton_led":
            self.disable_auton_led(message)


    def pdb_callback(self, msg):
        self.send(text_data=json.dumps({
            'type': 'pdb',
            'temperatures': msg.temperatures,
            'currents': msg.currents
        }))

    def arm_controller_callback(self, msg): 
        self.send(text_data=json.dumps({
            'type': 'arm_controller',
            'name': msg.name,
            'state': msg.state,
            'error': msg.error
        }))

    def enable_device_callback(self, msg):
        try:
            result = self.limit_switch_service(name=msg["name"], enable=msg["enable"])
            self.send(text_data=json.dumps({
                'type': 'enable_device_srv',
                'result': result.success
            }))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def disable_auton_led(self, msg):
        led_pub = rospy.Publisher("/auton_led_cmd", String, queue_size=100)
        message = String()
        message.data = "off"
        led_pub.publish(message)
    
    def gps_fix_callback(self, msg):
        self.send(text_data=json.dumps({
            'type': 'nav_sat_fix',
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }))

    def imu_calibration_callback(self, msg) -> None:
        self.send(text_data=json.dumps({
            'type': 'calibration_status',
            'system_calibration': msg.system_calibration,
            'gyroscope_calibration': msg.gyroscope_calibration,
            'accelerometer_calibration': msg.accelerometer_calibration,
            'magnetometer_calibration': msg.magnetometer_calibration
        }))

    def flight_attitude_listener(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # threshold that must be exceeded to send JSON message
        threshold = 0.0001
        map_to_baselink = SE3()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                tf_msg = SE3.from_tf_tree(tf_buffer, "map", "base_link")

                if tf_msg.is_approx(map_to_baselink, threshold):
                    rate.sleep()
                    continue
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                rate.sleep()
                continue

            map_to_baselink = tf_msg
            rotation = map_to_baselink.rotation
            euler = euler_from_quaternion(rotation.quaternion)
            pitch = euler[0] * 180 / math.pi
            roll = euler[1] * 180 / math.pi

            self.send(text_data=json.dumps({
                'type': 'flight_attitude',
                'pitch': pitch,
                'roll': roll
            }))

            rate.sleep()