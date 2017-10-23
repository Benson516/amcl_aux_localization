#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy

import yaml
import numpy as np

import sys

from RosInterface import ROSInterface

# User files, uncomment as completed
# from math import atan2
# Extra utility functions
# from utility import *
#
from KalmanFilter import KalmanFilter



class AmclAuxLocalization(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, camera_frame_id, base_frame_id, odom_frame_id, map_frame_id, ns_tag_detector, tag_pose_id, pos_init):
        """
        Initialize the class
        """
        # Handles all the ROS related items
        self.ros_interface = ROSInterface(camera_frame_id, base_frame_id, odom_frame_id, map_frame_id, ns_tag_detector)
        self.time = rospy.get_time()
        # YOUR CODE AFTER THIS
        #-------------------------------------------#
        self.markers = tag_pose_id
        self.idx_target_marker = 0
        #-------------------------------------------#
        # Kalman filter
        self.kalman_filter = KalmanFilter(tag_pose_id)
        self.kalman_filter.mu_est = pos_init # 3*pos_init # For test


    def process_measurements(self):
        """
        YOUR CODE HERE
        This function is called at 10Hz
        """
        # tag_measurement = self.ros_interface.get_measurements()
        # tag_measurement = self.ros_interface.get_measurements_tf()
        tag_measurement = self.ros_interface.get_measurements_tf_directMethod()
        # amcl_pose = self.ros_interface.get_amcl_pose()

        """
        # Mark_for_running
        print '-----------------'
        print 'tag:'
        print tag_measurement
        """

        #----------------------------------------#
        # self.time = rospy.get_time()
        # print "self.time",self.time

        # Kalman filter
        # self.kalman_filter.step_filter(self.controlOut[0], (-1)*imu_meas, tag_measurement, rospy.get_time())
        self.kalman_filter.step_filter_by_amcl(self.ros_interface, tag_measurement, rospy.get_time())
        """
        print "After--"
        print "mu_est",self.kalman_filter.mu_est
        print "angle_est =", (self.kalman_filter.mu_est[2,0]*180.0/np.pi), "deg"
        """
        #
        return

def main(args):
    rospy.init_node('robot_control')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)

    # tf_frames
    camera_frame_id = np.array(params['camera_frame_id'])
    base_frame_id = np.array(params['base_frame_id'])
    odom_frame_id = np.array(params['odom_frame_id'])
    map_frame_id = np.array(params['map_frame_id'])
    #
    ns_tag_detector = np.array(params['ns_tag_detector'])
    # tags
    tag_pose_id = np.array(params['tag_pose_id'])
    # Initial pose
    pos_init = np.array(params['initial_pose'])
    #
    # _t_cam_at_bot = np.array(params['_t_cam_at_bot'])

    #
    # camera_frame_id = "usb_cam"

    # Intialize the AmclAuxLocalization object
    amcl_aux_localization = AmclAuxLocalization(camera_frame_id, base_frame_id, odom_frame_id, map_frame_id, ns_tag_detector, tag_pose_id, pos_init)

    # Call process_measurements at 10Hz
    r = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        amcl_aux_localization.process_measurements()
        r.sleep()
    # Done

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
