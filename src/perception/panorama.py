#!/usr/bin/env python3

import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from mrover.msg import MotorsStatus, Position
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from util.SE3 import SE3


import rospy
import sys
import actionlib

import cv2 as cv
import time

from mrover.msg import CapturePanoramaAction, CapturePanoramaActionFeedback, CapturePanoramaActionResult, CapturePanoramaGoal
from sensor_msgs.point_cloud2 import PointCloud2
from sensor_msgs import point_cloud2

class Panorama:

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, CapturePanoramaAction, execute_cb=self.capture_panorama, auto_start=False)
        self.tf_buffer = None
        self.listener = None
        self.img_list = []
        self.current_img = None
        self.current_pc = None
        self.arr_pc = None
        # TODO: Why no auto start?
        self._as.start()

    def rotate_pc(self, trans_mat : np.ndarray, pc : np.ndarray):
        # rotate the provided point cloud's x, y points by the se3_pose
        
        # remove NaNs and infs from pc
        pc = pc[np.isfinite(pc).all(axis=1)]

        # represent pc in homogenous coordinates
        points = np.hstack((pc[:,0:3], np.ones((pc.shape[0],1))))
        rotated_points = np.matmul(trans_mat, points.T).T
        pc[:,0:3] = np.delete(rotated_points, 3, 1)
        return pc

    def pc_callback(self, msg: PointCloud2):
        self.current_pc = msg

        # extract xyzrgb fields
        # TODO: dtype hard-coded to float32
        self.arr_pc = np.frombuffer(bytearray(msg.data), dtype=np.float32).reshape(msg.height * msg.width, int(msg.point_step / 4))[0::10,:]

    def image_callback(self, msg: Image):
        self.current_img = cv2.cvtColor(np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4), cv2.COLOR_RGBA2RGB)

    def capture_panorama(self, goal: CapturePanoramaGoal):
        # self.position_subscriber = rospy.Subscriber("/mast_status", MotorsStatus, self.position_callback, callback_args = Position, queue_size=1)
        self.pc_subscriber = rospy.Subscriber("/camera/left/points", PointCloud2, self.pc_callback, queue_size=1)
        self.img_subscriber = rospy.Subscriber("/camera/left/image", Image, self.image_callback, queue_size=1)
        self.mast_pose = rospy.Publisher("/mast_gimbal_position_cmd", Position, queue_size=1)
        self.pc_publisher = rospy.Publisher("/stitched_pc", PointCloud2)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        time.sleep(1)
        zed_in_base = SE3.transform_matrix(SE3.from_tf_time(self.tf_buffer, "odom", "zed_base_link")[0])
        # TODO: Don't hardcode or parametrize this?
        angle_inc = 0.2 # in radians
        current_angle = 0.0
        stitched_pc = np.empty((0,8), dtype=np.float32)
        
        while (current_angle < goal.angle):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted and stopped by operator' % self._action_name)
                self._as.set_preempted()
                break
            if self.current_img is None:
                continue
            try:
                zed_in_base_current = SE3.transform_matrix(SE3.from_tf_time(self.tf_buffer, "odom", "zed_base_link")[0])
                tf_diff = np.linalg.inv(zed_in_base) @ zed_in_base_current
                rotated_pc = self.rotate_pc(tf_diff, self.arr_pc)

                # pc_msg = PointCloud2()
                # pc_msg.width = rotated_pc.shape[0]
                # flat_pc = rotated_pc.flatten()
                # header = Header()
                # header.frame_id = 'base_link'
                # pc_msg.header = header
                # pc_msg.fields = self.current_pc.fields
                # pc_msg.is_bigendian = self.current_pc.is_bigendian
                # pc_msg.data = flat_pc.tobytes()
                # pc_msg.height = 1
                # pc_msg.point_step = int(len(pc_msg.data) / pc_msg.width)
                # pc_msg.is_dense = self.current_pc.is_dense
                # self.pc_publisher.publish(pc_msg)
            except:
                continue
            time.sleep(0.5)

            self.img_list.append(np.copy(self.current_img))
            stitched_pc = np.vstack((stitched_pc, rotated_pc))

            current_angle += angle_inc
            self.mast_pose.publish(Position(["mast_gimbal_z"], [current_angle]))
            # self._as.publish_feedback(CapturePanoramaActionFeedback(current_angle / goal.angle))

        rospy.loginfo("Creating Panorama using %s images...", str(len(self.img_list)))
        stitcher = cv.Stitcher.create()
        # status, pano = stitcher.stitch(self.img_list)
        # desktop_path = "/home/alison/catkin_ws/src/mrover/data/pano.png"
        # cv2.imwrite(filename=desktop_path, pano)

        # construct pc from stitched
        pc_msg = PointCloud2()
        pc_msg.width = stitched_pc.shape[0]
        stitched_pc = stitched_pc.flatten()
        header = Header()
        header.frame_id = 'base_link'
        pc_msg.header = header
        pc_msg.fields = self.current_pc.fields
        pc_msg.is_bigendian = self.current_pc.is_bigendian
        pc_msg.data = stitched_pc.tobytes()
        pc_msg.height = 1
        pc_msg.point_step = int(len(pc_msg.data) / pc_msg.width)
        pc_msg.is_dense = self.current_pc.is_dense
        while not rospy.is_shutdown():
            self.pc_publisher.publish(pc_msg)
            time.sleep(0.5)

        # return CapturePanoramaGoal(...)
            
        # def received_point_cloud(point: PointCloud2):
        #     # Extract RGB field
        #     pc = pointcloud2_to_array(point)
        #     pc = split_rgb_field(pc)
        #     shape = pc.shape + (3,)
        #     rgb = np.zeros(shape)
        #     rgb[..., 0] = pc["r"]
        #     rgb[..., 1] = pc["g"]
        #     rgb[..., 2] = pc["b"]
        # def received_motor_position(status: MotorsStatus):
        #     nonlocal latestPosition
        #     # rospy.loginfo("Received motor position")
        #     mast_throttle.publish(Position(["mast_gimbal_z"], [targetPosition]))
        #     latestPosition = status.joint_states.position
        #     # rospy.loginfo(status.joint_states.position)


        # Wait until mast_status starts publishing

def main() -> int:
    rospy.init_node(name="panorama")
    pano = Panorama(rospy.get_name())
    # panorama_service = rospy.Service("capture_panorama", CapturePanoramaAction, pano.capture_panorama)
    rospy.spin()
    # return 0


if __name__ == "__main__":
    sys.exit(main())
