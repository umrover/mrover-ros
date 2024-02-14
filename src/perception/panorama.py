#!/usr/bin/env python3
import os
import statistics
import struct

import cv2
import numpy as np
import collections
from sensor_msgs.msg import PointCloud2, PointField
from mrover.msg import MotorsStatus, Throttle, Position

import rospy
import sys

import cv2 as cv
from cv_bridge import CvBridge

from mrover.srv import CapturePanorama, CapturePanoramaRequest, CapturePanoramaResponse
from sensor_msgs.point_cloud2 import PointCloud2
from sensor_msgs import point_cloud2

type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}


def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % ('__', offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % ('__', offset), np.uint8))
        offset += 1

    return np_dtype_list


def pointcloud2_to_array(cloud_msg, squeeze=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray

    Reshapes the returned array to have shape (height, width), even if the height is 1.

    The reason for using np.frombuffer rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len('__')] == '__')]]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))


def split_rgb_field(cloud_arr):
    '''Takes an array with a named 'rgb' float32 field, and returns an array in which
    this has been split into 3 uint 8 fields: 'r', 'g', and 'b'.

    (pcl stores rgb in packed 32 bit floats)
    '''
    rgb_arr = cloud_arr['rgb'].copy()
    rgb_arr.dtype = np.uint32
    r = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
    g = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
    b = np.asarray(rgb_arr & 255, dtype=np.uint8)

    # create a new array, without rgb, but with r, g, and b fields
    new_dtype = []
    for field_name in cloud_arr.dtype.names:
        field_type, field_offset = cloud_arr.dtype.fields[field_name]
        if not field_name == 'rgb':
            new_dtype.append((field_name, field_type))
    new_dtype.append(('r', np.uint8))
    new_dtype.append(('g', np.uint8))
    new_dtype.append(('b', np.uint8))
    new_cloud_arr = np.zeros(cloud_arr.shape, new_dtype)

    # fill in the new array
    for field_name in new_cloud_arr.dtype.names:
        if field_name == 'r':
            new_cloud_arr[field_name] = r
        elif field_name == 'g':
            new_cloud_arr[field_name] = g
        elif field_name == 'b':
            new_cloud_arr[field_name] = b
        else:
            new_cloud_arr[field_name] = cloud_arr[field_name]
    return new_cloud_arr


def capture_panorama(request: CapturePanoramaRequest) -> CapturePanoramaResponse:
    image_list = []

    def received_point_cloud(point: PointCloud2):
        rospy.loginfo("Received point cloud data")
        # Extract RGB field
        pc = pointcloud2_to_array(point)
        pc = split_rgb_field(pc)
        shape = pc.shape + (3,)
        rgb = np.zeros(shape)
        rgb[..., 0] = pc['r']
        rgb[..., 1] = pc['g']
        rgb[..., 2] = pc['b']
        image_list.append(cv2.cvtColor((rgb * 255).astype(np.uint8), cv2.COLOR_BGR2RGB))
        rospy.loginfo("Converted and Saved Image")
        rospy.loginfo("Total Images Saved" + str(len(image_list)))

    def received_motor_position(status: MotorsStatus):
        rospy.loginfo("Received motor position")
        mast_throttle.publish(Position(["mast_joint"], [2]))
        rospy.loginfo(status.joint_states.position)

    position_subscriber = rospy.Subscriber("/mast_status", MotorsStatus, received_motor_position, queue_size=1)
    image_subscriber = rospy.Subscriber("/camera/left/points", PointCloud2, received_point_cloud, queue_size=1)
    mast_throttle = rospy.Publisher("/mast_position_cmd", Position, queue_size=1)
    # TODO, make motors spin around
    if (len(image_list) < 2):
        rospy.sleep(1)

    rospy.loginfo("Creating Panorama..." + str(len(image_list)))
    stitcher = cv2.Stitcher.create()
    (code, outPic) = stitcher.stitch(image_list)
    rospy.loginfo("Panorama Created" + str(len(image_list)))
    desktop_path = '/Users/ryankersten/Desktop/test.png'

    cv2.imwrite(desktop_path, image_list[0])

    return CapturePanoramaResponse(...)


def main() -> int:
    rospy.init_node("panorama")
    panorama_service = rospy.Service(
        "capture_panorama",
        CapturePanorama,
        capture_panorama
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
