#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from mrover.msg import Position
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from util.SE3 import SE3


import rospy
import sys
import actionlib

import cv2
import time

from mrover.msg import CapturePanoramaAction, CapturePanoramaFeedback, CapturePanoramaResult, CapturePanoramaGoal
from sensor_msgs.point_cloud2 import PointCloud2

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
        # get every tenth point to make the pc sparser
        # TODO: dtype hard-coded to float32
        self.arr_pc = np.frombuffer(bytearray(msg.data), dtype=np.float32).reshape(msg.height * msg.width, int(msg.point_step / 4))[0::10,:]

    def image_callback(self, msg: Image):
        self.current_img = cv2.cvtColor(np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4), cv2.COLOR_RGBA2RGB)

    def capture_panorama(self, goal: CapturePanoramaGoal):
        # self.position_subscriber = rospy.Subscriber("/mast_status", MotorsStatus, self.position_callback, callback_args = Position, queue_size=1)
        self.pc_subscriber = rospy.Subscriber("/mast_camera/left/points", PointCloud2, self.pc_callback, queue_size=1)
        self.img_subscriber = rospy.Subscriber("/mast_camera/left/image", Image, self.image_callback, queue_size=1)
        self.mast_pose = rospy.Publisher("/mast_gimbal_position_cmd", Position, queue_size=1)
        self.pc_publisher = rospy.Publisher("/stitched_pc", PointCloud2)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        time.sleep(1)

        try:
            for i in range(3):
                zed_in_base = SE3.transform_matrix(SE3.from_tf_time(self.tf_buffer, "odom", "zed_base_link")[0])
                break
        except:
            rospy.loginfo("Failed to get transform from map to zed_base_link")
        # TODO: Don't hardcode or parametrize this?
        angle_inc = 0.2 # in radians
        current_angle = 0.0
        stitched_pc = np.empty((0,8), dtype=np.float32)
        
        while (current_angle < goal.angle):

            # allow gimbal to come to rest
            time.sleep(0.5)
            
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted and stopped by operator' % self._action_name)
                self._as.set_preempted()
                break
            if self.current_img is None:
                pass
            try:
                zed_in_base_current = SE3.transform_matrix(SE3.from_tf_time(self.tf_buffer, "odom", "zed_base_link")[0])
                tf_diff = np.linalg.inv(zed_in_base) @ zed_in_base_current
                rotated_pc = self.rotate_pc(tf_diff, self.arr_pc)
                stitched_pc = np.vstack((stitched_pc, rotated_pc))
            except:
                pass

            self.img_list.append(np.copy(self.current_img))
            rospy.loginfo("rotating mast...")
            current_angle += angle_inc
            self.mast_pose.publish(Position(["mast_gimbal_z"], [current_angle]))
            self._as.publish_feedback(CapturePanoramaFeedback(current_angle / goal.angle))

        rospy.loginfo("Creating Panorama using %s images...", str(len(self.img_list)))
        stitcher = cv2.Stitcher.create()

        # TODO Handle exceptions in stitching and write to relative path
        status, pano = stitcher.stitch(self.img_list)

        # construct image msg
        try:
            header = Header()
            img_msg = Image()
            img_msg.header = header
            img_msg.height = pano.shape[0]
            img_msg.width = pano.shape[1]
            img_msg.encoding = "rgb8"
            img_msg.data = pano.tobytes()
            img_msg.step = len(pano[0]) * 3
        except:
            rospy.loginfo("Failed to create image message")
            self._as.set_aborted()
            return
        
        self._as.set_succeeded(CapturePanoramaResult(panorama=img_msg)) # TODO: replace with temp_img with stitched img

        # construct pc from stitched
        try:
            pc_msg = PointCloud2()
            pc_msg.width = stitched_pc.shape[0]
            stitched_pc = stitched_pc.flatten()
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
        except:
            # If image succeeds but pc fails, should we set action as succeeded?
            rospy.loginfo("Failed to create point cloud message")
            return

def main() -> int:
    rospy.init_node(name="panorama")
    pano = Panorama(rospy.get_name())
    rospy.spin()

if __name__ == "__main__":
    sys.exit(main())
