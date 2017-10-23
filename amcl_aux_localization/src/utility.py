#!/usr/bin/env python
"""
Utility functions for manipulating poses in ROS.
Used mostly for extending coordinate transformations beyond the scope of transformations.py.
Written by Alex Zhu (alexzhu(at)seas.upenn.edu)
"""
# ROS imports
import roslib
import rospy
import numpy as np
import math

import roslib
from std_msgs.msg import (
    Header,
)
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Pose,
    Quaternion,
    PoseWithCovariance,
)
from tf.transformations import *

def get_t_R(pose):
    """
    Returns the translation vector (4x1) and rotation matrix (4x4) from a pose message
    """
    t = np.transpose(np.matrix([pose.position.x,pose.position.y,pose.position.z,1.0]))
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R_full = quaternion_matrix(quat)
    R = R_full
    return t,R

def get_angle_tag_2_bot_from_R_tag2cam(_R_tag_2_cam, is_camera_top_and_tag_top):
    # Modify the following lines if the tag is pasted on the ceil.
    # Return (-pi, pi]
    if is_camera_top_and_tag_top:
        # Case 1: The tags are pasted on the ceil
        # _R_tag1_2_cam = self._R_tag_2_tag1.dot(_R_tag_2_cam)
        _angle_tag_2_bot = np.arctan2(_R_tag_2_cam[1,0], _R_tag_2_cam[0,0])
    else:
        # Case 2: The tags are pasted on the wall
        _angle_tag_2_bot = -np.arctan2(-_R_tag_2_cam[2,0], np.sqrt(_R_tag_2_cam[2,0]**2 + _R_tag_2_cam[2,2]**2))

    # Prevent the sigularity
    if math.isnan(_angle_tag_2_bot):
        return None
    else:
        return _angle_tag_2_bot

def get_angle_tag_2_bot_from_R_tag2bot(_R_tag_2_cam, is_camera_top_and_tag_top):
    # Modify the following lines if the tag is pasted on the ceil.
    # Return (-pi, pi]
    if is_camera_top_and_tag_top:
        # Case 1: The tags are pasted on the ceil
        # _R_tag1_2_cam = self._R_tag_2_tag1.dot(_R_tag_2_cam)
        _angle_tag_2_bot = np.arctan2(_R_tag_2_cam[1,1], _R_tag_2_cam[0,1])
    else:
        # Case 2: The tags are pasted on the wall
        _angle_tag_2_bot = -np.arctan2(_R_tag_2_cam[1,2], _R_tag_2_cam[0,2])

    # Prevent the sigularity
    if math.isnan(_angle_tag_2_bot):
        return None
    else:
        return _angle_tag_2_bot

def make_pose_stamped_msg(t,R):
    """
    Returns a pose stamped message from a translation vector and rotation matrix (4x4) for publishing.
    NOTE: Does not set the target frame.
    """
    pose_stamped_msg = PoseStamped()
    #
    pose_stamped_msg.header = Header()
    pose_stamped_msg.header.stamp = rospy.Time.now()
    #
    pose_msg = Pose()
    pose_msg.position.x = t[0]
    pose_msg.position.y = t[1]
    pose_msg.position.z = t[2]
    #
    quat = quaternion_from_matrix(R)
    pose_msg.orientation.x = quat[0]
    pose_msg.orientation.y = quat[1]
    pose_msg.orientation.z = quat[2]
    pose_msg.orientation.w = quat[3]
    #
    pose_stamped_msg.pose = pose_msg

    return pose_stamped_msg

def make_pose_covariance_stamped_msg(t,R,Cov):
    """
    Returns a pose stamped message from a translation vector and rotation matrix (4x4) for publishing.
    NOTE: Does not set the target frame.
    """
    pose_cov_stamped_msg = PoseWithCovarianceStamped()
    #
    pose_cov_stamped_msg.header = Header()
    pose_cov_stamped_msg.header.stamp = rospy.Time.now()
    #
    pose_msg = Pose()
    pose_msg.position.x = t[0]
    pose_msg.position.y = t[1]
    pose_msg.position.z = t[2]
    #
    quat = quaternion_from_matrix(R)
    pose_msg.orientation.x = quat[0]
    pose_msg.orientation.y = quat[1]
    pose_msg.orientation.z = quat[2]
    pose_msg.orientation.w = quat[3]
    #
    pose_cov_stamped_msg.pose.pose = pose_msg
    pose_cov_stamped_msg.pose.covariance = Cov

    return pose_cov_stamped_msg

def make_pose_covariance_stamped_msg_quat(t, quat, Cov):
    """
    Returns a pose stamped message from a translation vector and rotation matrix (4x4) for publishing.
    NOTE: Does not set the target frame.
    """
    pose_cov_stamped_msg = PoseWithCovarianceStamped()
    #
    pose_cov_stamped_msg.header = Header()
    pose_cov_stamped_msg.header.stamp = rospy.Time.now()
    #
    pose_msg = Pose()
    pose_msg.position.x = t[0]
    pose_msg.position.y = t[1]
    pose_msg.position.z = t[2]
    #
    # quat = quaternion_from_matrix(R)
    # print "quat", quat
    pose_msg.orientation.x = quat[0]
    pose_msg.orientation.y = quat[1]
    pose_msg.orientation.z = quat[2]
    pose_msg.orientation.w = quat[3]
    #
    pose_cov_stamped_msg.pose.pose = pose_msg
    pose_cov_stamped_msg.pose.covariance = Cov

    return pose_cov_stamped_msg

def make_pose_covariance_stamped_msg_quat_timeStampIn(t, quat, Cov, timeStamp_in):
    """
    Returns a pose stamped message from a translation vector and rotation matrix (4x4) for publishing.
    NOTE: Does not set the target frame.
    """
    pose_cov_stamped_msg = PoseWithCovarianceStamped()
    #
    pose_cov_stamped_msg.header = Header()
    pose_cov_stamped_msg.header.stamp = timeStamp_in
    #
    pose_msg = Pose()
    pose_msg.position.x = t[0]
    pose_msg.position.y = t[1]
    pose_msg.position.z = t[2]
    #
    # quat = quaternion_from_matrix(R)
    # print "quat", quat
    pose_msg.orientation.x = quat[0]
    pose_msg.orientation.y = quat[1]
    pose_msg.orientation.z = quat[2]
    pose_msg.orientation.w = quat[3]
    #
    pose_cov_stamped_msg.pose.pose = pose_msg
    pose_cov_stamped_msg.pose.covariance = Cov

    return pose_cov_stamped_msg
