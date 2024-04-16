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

type_mappings = [
    (PointField.INT8, np.dtype("int8")),
    (PointField.UINT8, np.dtype("uint8")),
    (PointField.INT16, np.dtype("int16")),
    (PointField.UINT16, np.dtype("uint16")),
    (PointField.INT32, np.dtype("int32")),
    (PointField.UINT32, np.dtype("uint32")),
    (PointField.FLOAT32, np.dtype("float32")),
    (PointField.FLOAT64, np.dtype("float64")),
]
pftype_to_nptype = dict(type_mappings)
# sizes (in bytes) of PointField types
pftype_sizes = {
    PointField.INT8: 1,
    PointField.UINT8: 1,
    PointField.INT16: 2,
    PointField.UINT16: 2,
    PointField.INT32: 4,
    PointField.UINT32: 4,
    PointField.FLOAT32: 4,
    PointField.FLOAT64: 8,
}

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

    def fields_to_dtype(fields, point_step):
        """Convert a list of PointFields to a numpy record datatype."""
        offset = 0
        np_dtype_list = []
        for f in fields:
            while offset < f.offset:
                # might be extra padding between fields
                np_dtype_list.append(("%s%d" % ("__", offset), np.uint8))
                offset += 1

            dtype = pftype_to_nptype[f.datatype]
            if f.count != 1:
                dtype = np.dtype((dtype, f.count))

            np_dtype_list.append((f.name, dtype))
            offset += pftype_sizes[f.datatype] * f.count

        # might be extra padding between points
        while offset < point_step:
            np_dtype_list.append(("%s%d" % ("__", offset), np.uint8))
            offset += 1

        return np_dtype_list


    def pointcloud2_to_array(cloud_msg, squeeze=True):
        """Converts a rospy PointCloud2 message to a numpy recordarray

        Reshapes the returned array to have shape (height, width), even if the height is 1.

        The reason for using np.frombuffer rather than struct.unpack is speed... especially
        for large point clouds, this will be <much> faster.
        """
        # construct a numpy record type equivalent to the point type of this cloud
        dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

        # parse the cloud into an array
        cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

        # remove the dummy fields that were added
        cloud_arr = cloud_arr[[fname for fname, _type in dtype_list if not (fname[: len("__")] == "__")]]

        if squeeze and cloud_msg.height == 1:
            return np.reshape(cloud_arr, (cloud_msg.width,))
        else:
            return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))


    def split_rgb_field(cloud_arr):
        """Takes an array with a named 'rgb' float32 field, and returns an array in which
        this has been split into 3 uint 8 fields: 'r', 'g', and 'b'.

        (pcl stores rgb in packed 32 bit floats)
        """
        rgb_arr = cloud_arr["rgb"].copy()
        rgb_arr.dtype = np.uint32
        r = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
        g = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
        b = np.asarray(rgb_arr & 255, dtype=np.uint8)

        # create a new array, without rgb, but with r, g, and b fields
        new_dtype = []
        for field_name in cloud_arr.dtype.names:
            field_type, field_offset = cloud_arr.dtype.fields[field_name]
            if not field_name == "rgb":
                new_dtype.append((field_name, field_type))
        new_dtype.append(("r", np.uint8))
        new_dtype.append(("g", np.uint8))
        new_dtype.append(("b", np.uint8))
        new_cloud_arr = np.zeros(cloud_arr.shape, new_dtype)

        # fill in the new array
        for field_name in new_cloud_arr.dtype.names:
            if field_name == "r":
                new_cloud_arr[field_name] = r
            elif field_name == "g":
                new_cloud_arr[field_name] = g
            elif field_name == "b":
                new_cloud_arr[field_name] = b
            else:
                new_cloud_arr[field_name] = cloud_arr[field_name]
        return new_cloud_arr

    def rotate_pc(self, trans_mat : np.ndarray, pc : np.ndarray):
        # rotate the provided point cloud's x, y points by the se3_pose
        points: np.ndarray[np.Any, np.dtype[np.floating[np._64Bit]]] = np.hstack((pc[:,0:3], np.ones((pc.shape[0],1))))
        rotated_points = np.matmul(trans_mat, points.T).T
        pc[:,0:3] = np.delete(rotated_points, 3, 1)
        return pc[~np.isnan(points).any(axis = 1)]

    def pc_callback(self, msg: PointCloud2):
        self.current_pc = msg

        # extract xyzrgb fields
        # TODO: dtype hard-coded to float32
        self.arr_pc = np.frombuffer(bytearray(msg.data), dtype=np.float32).reshape(msg.height * msg.width, int(msg.point_step / 4))[0::20,:]

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
                tf_diff = zed_in_base_current.T * zed_in_base
                rotated_pc = self.rotate_pc(tf_diff, self.arr_pc)
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
