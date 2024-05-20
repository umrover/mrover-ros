#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from mrover.msg import Throttle
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from tf.transformations import euler_from_quaternion
from util.SE3 import SE3, SO3


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
        self._as = actionlib.SimpleActionServer(
            self._action_name, CapturePanoramaAction, execute_cb=self.capture_panorama, auto_start=False
        )
        self.tf_buffer = None
        self.listener = None
        self.img_list = []
        self.current_img = None
        self.current_pc = None
        self.arr_pc = None
        # TODO: Why no auto start?
        self._as.start()

    def rotate_pc(self, trans_mat: np.ndarray, pc: np.ndarray):
        # rotate the provided point cloud's x, y points by the se3_pose

        # remove NaNs and infs from pc
        pc = pc[np.isfinite(pc).all(axis=1)]

        # represent pc in homogenous coordinates
        points = np.hstack((pc[:, 0:3], np.ones((pc.shape[0], 1))))
        rotated_points = np.matmul(trans_mat, points.T).T
        pc[:, 0:3] = np.delete(rotated_points, 3, 1)
        return pc

    def pc_callback(self, msg: PointCloud2):
        self.current_pc = msg

        # extract xyzrgb fields
        # get every tenth point to make the pc sparser
        # TODO: dtype hard-coded to float32
        self.arr_pc = np.frombuffer(bytearray(msg.data), dtype=np.float32).reshape(
            msg.height * msg.width, int(msg.point_step / 4)
        )[0::10, :]

    def image_callback(self, msg: Image):
        self.current_img = cv2.cvtColor(
            np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4), cv2.COLOR_RGBA2RGB
        )

    def capture_panorama(self, goal: CapturePanoramaGoal):
        # self.position_subscriber = rospy.Subscriber("/mast_status", MotorsStatus, self.position_callback, callback_args = Position, queue_size=1)
        self.pc_subscriber = rospy.Subscriber("/camera/left/points", PointCloud2, self.pc_callback, queue_size=1)
        self.img_subscriber = rospy.Subscriber("/camera/left/image", Image, self.image_callback, queue_size=1)
        self.mast_throttle = rospy.Publisher("/mast_gimbal_throttle_cmd", Throttle, queue_size=1)
        self.pc_publisher = rospy.Publisher("/stitched_pc", PointCloud2)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # wait for tf buffer
        time.sleep(1)

        try:
            for i in range(3):
                zed_in_base = SE3.transform_matrix(
                    SE3.from_tf_tree_with_time(self.tf_buffer, "odom", "zed_base_link")[0]
                )
                break
        except:
            rospy.loginfo("Failed to get transform from map to zed_base_link")
            return 1
        # TODO: Don't hardcode or parametrize this?
        current_angle = 0
        stitched_pc = np.empty((0, 8), dtype=np.float32)

        while current_angle < goal.angle:
            # current_pos
            # calculate angle
            rospy.loginfo("rotating mast...")
            time_start = rospy.Time.now()
            while rospy.Time.now() - time_start < rospy.Duration(1.5):
                self.mast_throttle.publish(Throttle(["mast_gimbal_z"], [1.0]))
                rospy.Rate(10).sleep()

            self.mast_throttle.publish(Throttle(["mast_gimbal_z"], [0.0]))

            try:
                zed_in_base_current = SE3.transform_matrix(
                    SE3.from_tf_tree_with_time(self.tf_buffer, "odom", "zed_base_link")[0]
                )
                tf_diff = np.linalg.inv(zed_in_base) @ zed_in_base_current

                if self._as.is_preempt_requested():
                    rospy.loginfo("%s: Preempted and stopped by operator" % self._action_name)
                    self._as.set_preempted()
                    break
                if self.current_img is not None:
                    self.img_list.append(np.copy(self.current_img))
                if self.current_pc is not None:
                    rotated_pc = self.rotate_pc(tf_diff, self.arr_pc)
                    stitched_pc = np.vstack((stitched_pc, rotated_pc))

                current_angle = euler_from_quaternion(SO3.from_matrix(rotation_matrix=tf_diff[:3, :3]).quaternion)[2]
                rospy.loginfo(f"current angle {current_angle}")
                self._as.publish_feedback(CapturePanoramaFeedback(current_angle / goal.angle))

            except tf2_ros.TransformException as e:
                rospy.logwarn_throttle(f"Transform exception: {e}")

        self.mast_throttle.publish(Throttle(["mast_gimbal_z"], [0.0]))

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

        self._as.set_succeeded(CapturePanoramaResult(panorama=img_msg))  # TODO: replace with temp_img with stitched img

        # construct pc from stitched
        try:
            pc_msg = PointCloud2()
            pc_msg.width = stitched_pc.shape[0]
            stitched_pc = stitched_pc.flatten()
            header.frame_id = "base_link"
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
