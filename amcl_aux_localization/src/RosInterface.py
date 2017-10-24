#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import Queue

# ROS imports
import roslib
import rospy

from std_msgs.msg import (
    Header,
)

from apriltags_ros.msg import (
    AprilTagDetectionArray,
    AprilTagDetection,
)


from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    Pose,
    Quaternion,
    PoseWithCovariance,
)

# import cv2
# import yaml
import numpy as np

# import sys

import tf
from tf.transformations import *

# Extra utility functions
from utility import *

class ROSInterface(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, camera_frame_id, base_frame_id, odom_frame_id, map_frame_id, ns_tag_detector):
        """
        Initialize the class
        """
        # Queue for tag detections
        self.tag_Queue = Queue.Queue() # Unlimited length

        # Is the camera installed on top or at front-side?
        self.camera_top_and_tag_top = True

        # tf
        self.tf_listener = tf.TransformListener()
        # tf frames
        self.camera_frame = "/%s" % camera_frame_id # /usb_cam
        self.base_frame = "/%s" % base_frame_id # "/base_footprint"
        self.odom_frame = "/%s" % odom_frame_id # "/odom"
        self.map_frame = "/%s" % map_frame_id # "/map"
        # Name space
        self.ns_tag_detector = "/%s" % ns_tag_detector #"/proc_image"

        # Get the camera's pose relative to base_frame
        trans_cam_at_bot = (0.0, 0.0, 0.0)
        is_tf_bot_2_cam_available = False
        while (not is_tf_bot_2_cam_available) and (not rospy.is_shutdown() ):
            # Try tf
            try:
                # Transform from /base_footprint to /usb_cam
                # We got the translation _t_cam_at_bot
                # and the rotation _R_cam_2_bot
                (trans_cam_at_bot, quaternion) = self.tf_listener.lookupTransform(self.base_frame, self.camera_frame, rospy.Time(0))
                #

                R_full_cam_2_bot = quaternion_matrix(quaternion)
                # Leave the loop
                # print ("tf from %s to %s got" % self.base_frame, self.camera_frame)
                print "Got the tf"
                is_tf_bot_2_cam_available = True
            except: # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # Sleep, then keep on trying
                print "tf exception..."
                rospy.sleep(0.5)

        #
        _t_cam_at_bot = np.transpose( np.array([trans_cam_at_bot]) )
        self._t_cam_at_bot = np.concatenate((_t_cam_at_bot,np.array([[1]])), axis=0)
        self._R_cam_2_bot = R_full_cam_2_bot
        #
        print "_t_cam_at_bot =", self._t_cam_at_bot
        print "_R_cam_2_bot =", self._R_cam_2_bot

        """
        if self.camera_top_and_tag_top:
            # Camera installed on the top
            self._R_cam_2_bot = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
        else:
            # Camera installed at fron-side
            self._R_cam_2_bot = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
        """

        # ROS publishers and subscribers
        rospy.Subscriber("%s/tag_detections" % self.ns_tag_detector, AprilTagDetectionArray,self._tag_pose_callback)
        # amcl
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_CB)
        self._pub_init_amcl = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        # Information form amcl
        self._amcl_poseStamp = None # Header()
        self._amcl_pose = None # Pose()
        self._amcl_cov = None # np.eye(6)

        # The transformation matrix for covariance matrix 6-DoF->3-DoF (x, y, theta)
        self._T_subState = np.zeros((3,6))
        self._T_subState[0,0] = 1.0
        self._T_subState[1,1] = 1.0
        self._T_subState[2,5] = 1.0




    def _tag_pose_callback(self, posearray):
        """
        Callback function for AprilTag measurements
        """
        num_detections = len(posearray.detections)
        if (num_detections == 0):
            return
        # Else, not empty --> put into queue
        self.tag_Queue.put(posearray)
        # self.tag_Queue.put_nowait(posearray)


    def get_measurements_tf(self):
        """
        Returns information about the last tag seen if any. Returns a list of Python list
        in the format of (x,y,theta,id). Returns None if no new tag is seen.
        """
        if self.tag_Queue.empty():
            return None
        # print "The size of tag_Queue:", self.tag_Queue.qsize()
        # Else, get one element from queue
        posearray = self.tag_Queue.get()
        num_detections = len(posearray.detections)

        # Note: all tags should be represented in robot's coordinate
        tag_list = list()
        for kk in range(num_detections): # range(self.num_detections):
            #
            _marker_num = posearray.detections[kk].id
            tagFramName = "/tag_%d" % _marker_num
            try:
                # From /usb_cam to a tag
                (trans, quaternion) = self.tf_listener.lookupTransform( self.camera_frame, tagFramName, rospy.Time(0))
                """
                now = rospy.Time.now()
                self.tf_listener.waitForTransform(self.camera_frame,tagFramName, now, rospy.Duration(0.5))
                (trans, quaternion) = self.tf_listener.lookupTransform(self.camera_frame,tagFramName, now)
                """
                # print "trans =", trans
                _t_tag_at_cam = np.transpose(np.matrix([trans[0], trans[1], trans[2],1.0]))
                # print "_t_tag_at_cam =", _t_tag_at_cam
                _R_tag_2_cam = quaternion_matrix(quaternion)
                #
            except: # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "tf exception while getting measurements"
                # continue
                print "Use pose from message"
                # Use the old data which is parsed at callback
                # _t_tag_at_cam is in camera fram
                (_t_tag_at_cam, _R_tag_2_cam) = get_t_R(posearray.detections[kk].pose.pose)
                #
            #-----------------------------#
            _angle_tag_at_bot = get_angle_tag_2_bot_from_R_tag2cam(_R_tag_2_cam, self.camera_top_and_tag_top)
            if _angle_tag_at_bot is None:
                continue

            _t_tag_at_bot = np.dot(self._R_cam_2_bot, _t_tag_at_cam) + self._t_cam_at_bot
            #-----------------------------#
            # A list of tag information
            # Each item: [x, y, theta, id]
            tag_list.append([_t_tag_at_bot[0,0], _t_tag_at_bot[1,0], _angle_tag_at_bot, _marker_num])
        #
        return tag_list

    def get_measurements_tf_directMethod(self):
        """
        Returns information about the last tag seen if any. Returns a list of Python list
        in the format of (x,y,theta,id). Returns None if no new tag is seen.
        """
        if self.tag_Queue.empty():
            return None
        # print "The size of tag_Queue:", self.tag_Queue.qsize()
        # Else, get one element from queue
        posearray = self.tag_Queue.get()
        num_detections = len(posearray.detections)

        # Note: all tags should be represented in robot's coordinate
        tag_list = list()
        for kk in range(num_detections): # range(self.num_detections):
            #
            _marker_num = posearray.detections[kk].id
            tagFramName = "/tag_%d" % _marker_num
            try:
                # From /base_footprint to a tag
                # (trans, quaternion) = self.tf_listener.lookupTransform( self.base_frame, tagFramName, rospy.Time(0))
                """
                now = rospy.Time.now()
                self.tf_listener.waitForTransform(self.base_frame, tagFramName, now, rospy.Duration(0.5))
                (trans, quaternion) = self.tf_listener.lookupTransform(self.base_frame,tagFramName, now)
                """
                # Intergrating the odom:
                # The transformation method which will compensate the delay
                # Since the tags are not moving, we can first project the tags to the /odom at the time they were discoverd,
                # and then calculate the tranformation from /basefootprint to /odom at now
                now = rospy.Time.now()
                time_tag_was_found = self.tf_listener.getLatestCommonTime(self.camera_frame, tagFramName) # Note that its from /usb_cam to /tag_xx
                self.tf_listener.waitForTransformFull(self.base_frame, now,
                                                        tagFramName, time_tag_was_found,
                                                        self.odom_frame, rospy.Duration(0.5))
                (trans, quaternion) = self.tf_listener.lookupTransformFull(self.base_frame, now,
                                                                            tagFramName, time_tag_was_found,
                                                                            self.odom_frame)
                # print "--Successfully got the tf of the tag!!--"
                # print "trans =", trans
                _t_tag_at_bot = np.transpose(np.matrix([trans[0], trans[1], trans[2],1.0]))
                # print "_t_tag_at_bot =", _t_tag_at_bot
                _R_tag_2_bot = quaternion_matrix(quaternion)
                _angle_tag_at_bot = get_angle_tag_2_bot_from_R_tag2bot(_R_tag_2_bot, self.camera_top_and_tag_top)
                if _angle_tag_at_bot is None:
                    continue
            except: # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "tf exception while getting measurements"
                # continue
                print "Use pose from message"
                # Use the old data which is parsed at callback
                # _t_tag_at_cam is in camera fram
                (_t_tag_at_cam, _R_tag_2_cam) = get_t_R(posearray.detections[kk].pose.pose)
                #
                _t_tag_at_bot = np.dot(self._R_cam_2_bot, _t_tag_at_cam) + self._t_cam_at_bot
                #
                _angle_tag_at_bot = get_angle_tag_2_bot_from_R_tag2cam(_R_tag_2_cam, self.camera_top_and_tag_top)
                if _angle_tag_at_bot is None:
                    continue
            #-----------------------------#
            # A list of tag information
            # Each item: [x, y, theta, id]
            tag_list.append([_t_tag_at_bot[0,0], _t_tag_at_bot[1,0], _angle_tag_at_bot, _marker_num])
        #
        return tag_list

    def amcl_pose_CB(self, PoseWithCovarianceStamped):
        self._amcl_poseStamp = PoseWithCovarianceStamped.header
        self._amcl_pose = PoseWithCovarianceStamped.pose.pose
        self._amcl_cov = np.array(PoseWithCovarianceStamped.pose.covariance).reshape(6,6)

    def get_amcl_pose(self):
        if self._amcl_poseStamp is None:
            return None
        else:
            # [x, y, theta].'
            pose_2D = np.zeros((3,1))
            pose_2D[0,0] = self._amcl_pose.position.x
            pose_2D[1,0] = self._amcl_pose.position.y
            #
            pose = self._amcl_pose
            quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            # roll = euler[0]
            # pitch = euler[1]
            yaw = euler[2]
            pose_2D[2,0] = yaw
            # Reduced-order covariance matrix
            cov_2D = np.dot(np.dot(self._T_subState, self._amcl_cov), np.transpose(self._T_subState) )
            #
            return (pose_2D, cov_2D, self._amcl_poseStamp.stamp)

    def get_amcl_pose_tf(self):
        # Get pose_2D from tf to reduce the effect of delay
        if self._amcl_poseStamp is None:
            return None
        else:
            # [x, y, theta].'
            pose_2D = np.zeros((3,1))
            #
            # Try tf
            try:
                # From /map to /base_footprint
                # For outputing the stamp
                stamp_amclPose = self.tf_listener.getLatestCommonTime(self.map_frame, self.base_frame)
                # stamp_amclPose = rospy.Time.now()
                # print "~~~ Delay of the amcl_pose: ", (rospy.Time.now() - stamp_amclPose).to_sec(), "sec."
                #
                (trans, quaternion) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
                #
                """
                now = rospy.Time.now()
                self.tf_listener.waitForTransform(self.map_frame,self.base_frame, now, rospy.Duration(1.0))
                (trans, quaternion) = self.tf_listener.lookupTransform(self.map_frame,self.base_frame, now)
                """
                pose_2D[0,0] = trans[0] # self._amcl_pose.position.x
                pose_2D[1,0] = trans[1] # self._amcl_pose.position.y
                # pose = self._amcl_pose
                # quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
            except: # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "tf exception while getting robot pose"
                # Use the topic /amcl_pose instead
                pose_2D[0,0] = self._amcl_pose.position.x
                pose_2D[1,0] = self._amcl_pose.position.y
                #
                pose = self._amcl_pose
                quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                #
                stamp_amclPose = self._amcl_poseStamp.stamp
            #
            # roll = euler[0]
            # pitch = euler[1]
            yaw = euler[2]
            pose_2D[2,0] = yaw
            # Reduced-order covariance matrix
            cov_2D = np.dot(np.dot(self._T_subState, self._amcl_cov), np.transpose(self._T_subState) )
            #
        return (pose_2D, cov_2D, stamp_amclPose)

    def set_amcl_pose(self, pose_2D, cov_2D):
        #
        t = [pose_2D[0,0], pose_2D[1,0], 0.0]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, pose_2D[2,0]) # raw, pitch, yaw
        #
        Cov_np = np.dot( np.dot( np.transpose(self._T_subState), cov_2D), self._T_subState )
        Cov = Cov_np.reshape(1,36).tolist()[0] # Convert to a python list (1-D)
        #
        pose_cov_stamped_msg = make_pose_covariance_stamped_msg_quat(t, quaternion, Cov)
        # Publish
        self._pub_init_amcl.publish(pose_cov_stamped_msg)

    def set_amcl_pose_timeStampIn(self, pose_2D, cov_2D, timeStamp_in):
        #
        t = [pose_2D[0,0], pose_2D[1,0], 0.0]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, pose_2D[2,0]) # raw, pitch, yaw
        #
        Cov_np = np.dot( np.dot( np.transpose(self._T_subState), cov_2D), self._T_subState )
        Cov = Cov_np.reshape(1,36).tolist()[0] # Convert to a python list (1-D)
        #
        pose_cov_stamped_msg = make_pose_covariance_stamped_msg_quat_timeStampIn(t, quaternion, Cov, timeStamp_in)
        # Publish
        self._pub_init_amcl.publish(pose_cov_stamped_msg)
