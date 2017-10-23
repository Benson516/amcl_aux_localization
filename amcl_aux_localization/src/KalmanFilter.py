#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
#import pylab
import time
import math
from math import cos
from math import sin
from math import atan2

from RosInterface import ROSInterface

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input:
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = None # Used to keep track of time between measurements

        # YOUR CODE HERE
        self.n = 3 # Number of states
        self.p = 2 # Number of inputs
        self.q = 3 # Number of outputs
        n = self.n
        p = self.p
        q = self.q

        # Covariance for motion
        self.Q_t = np.eye(p)

        # Covariance for measurements
        # self.R_t = np.eye(3)*(10.0**(-3))
        self.R_t = np.eye(q)
        self.R_t[0,0] = 0.01**(2) # 0.02**(2)
        self.R_t[1,1] = 0.01**(2) # 0.02**(2)
        self.R_t[2,2] = (1.0*np.pi/180.0)**(2) # (0.8*np.pi/180.0)**(2)
        # The ratio for the additional covariance increased with the square of distance
        self.distortionRate = 30.0 # 1.0

        # The threshold of each covariance
        self.cov_threshold = [0.5**2, 0.5**2, (10.0*np.pi/180.0)**2] # 2.5 cm, 2.5 cm, 0.5 deg
        # self.cov_threshold = [0.2**2, 0.2**2, (2.0*np.pi/180.0)**2] # 2.5 cm, 2.5 cm, 0.5 deg

        # Creat a diagonal matrix with diagonal elements self.cov_threshold
        self.cov_th = np.diag( np.array(self.cov_threshold) )
        self.cov_correctionRate = 0.2

        """
        # Extra variance for preventing over convergence
        self.R_extra = np.eye(q)
        self.R_extra[0,0] = 0.005**2
        self.R_extra[1,1] = 0.005**2
        self.R_extra[2,2] = (0.1*np.pi/180.0)**(2)
        #
        self.cov_increse_ratio = 1.005
        """

        #
        self.dt = 0; # second
        self.mu_est = np.zeros((n,1)) # Estimation of the mean of the states, x = [x,y,theta].'
        self.Sigma_est = float(10**5)*np.eye(n) # Estimation of covariance matrix
        # self.u = np.zeros((p,1)) # u = [v,w].'
        self.Kt = np.zeros((n,q)) # Kalman gain
        #
        # Input matrix
        self.input_matrix = np.zeros((n,p)) # B_u
        theta = 0.0
        self.input_matrix[0,0] = cos(theta)
        self.input_matrix[1,0] = sin(theta)
        self.input_matrix[2,1] = 1.0
        #
        self.dfx_dx = np.eye(n)
        self.dfx_dn = np.zeros((n,p))
        # dh(x)/dx
        self.dhx_dx = np.eye(n)

        # The zero vector in homography
        self.homography_zeroVec = np.array([[0.],[0.],[1.]])
        # Pre-calculate the information from known map-features
        # print type(self.markers)
        # print self.markers.shape
        num_markers = self.markers.shape[0]
        # print num_markers
        self.dict_markers = dict()
        self.H_marker_2_world = dict()
        for i in range(num_markers):
            marker = self.markers[i,:]
            id_marker = int(marker[3])
            self.dict_markers[id_marker] = marker[0:3]
            # self.H_marker_2_world.append(self.Homography((marker[0],marker[1],marker[2])))
            self.H_marker_2_world[id_marker] = self.Homography( (marker[0],marker[1],marker[2]) )

    def Homography(self, state):
        """
        Calculate the Homography matrix given the state = (x,y,theta)
        Inputs:
        state - (x,y,theta)
        Outputs:
        H - Homography matrix, numpy array
        """
        H = np.array([[cos(state[2]), -sin(state[2]), state[0]],[sin(state[2]), cos(state[2]), state[1]],[0., 0., 1.]])
        return H

    def sys_dynamics(self, x, dt, v, omega):
        """
        Calculate the f(x), where x(t+1) = f( x(t) )
        Inputs:
        v, omega - the inear-speed and rotational-speed command

        Calculate: four elements
        fx - f( x(t) )
        dfx_dx - df(x)/dx
        dfx_dn - df(x)/dn
        dfx_dx - df(x)/dx

        Outputs:
        fx
        """

        theta = self.mu_est[2,0]
        #
        c_theta = cos(theta)
        s_theta = sin(theta)

        # f( x(t) )
        self.input_matrix[0,0] = c_theta
        self.input_matrix[1,0] = s_theta
        dx = self.input_matrix.dot(np.array([[v],[omega]]))
        fx = x + dt*dx
        # df(x)/dx
        self.dfx_dx[0,2] = -dt*v*s_theta
        self.dfx_dx[1,2] = dt*v*c_theta
        # df(x)/dn
        self.dfx_dn[0,0] = dt*c_theta
        self.dfx_dn[1,0] = dt*s_theta
        self.dfx_dn[2,1] = dt
        # dh(x)/dx
        # self.dhx_dx = np.eye(n)

        return fx


    def prediction(self, v, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the predction of the state
        predicted_covariance - a 3 by 3 numpy array of the predction of the
            covariance
        """
        # YOUR CODE HERE
        omega = imu_meas[3]
        # theta = self.mu_est[2,0]

        """
        #
        self.input_matrix[0,0] = cos(theta)
        self.input_matrix[1,0] = sin(theta)

        # Predict the mean
        dx = self.input_matrix.dot(np.array([[v],[omega]]))
        self.mu_est = self.mu_est + self.dt*dx
        """

        # Predict the mean
        self.mu_est = self.sys_dynamics(self.mu_est, self.dt, v, omega)
        pi_2 = 2.0*np.pi
        if self.mu_est[2,0] > np.pi:
            self.mu_est[2,0] -= pi_2
        elif self.mu_est[2,0] < -np.pi:
            self.mu_est[2,0] += pi_2

        # Predict the covariance matrix
        # self.Q_t[0,0] = 100.0 + 100*abs(v)
        self.Sigma_est = self.dfx_dx.dot(self.Sigma_est).dot(self.dfx_dx.transpose())
        self.Sigma_est += self.dfx_dn.dot(self.Q_t).dot(self.dfx_dn.transpose())

        # print "dx",dx
        # print "mu_est",self.mu_est
        # print "Sigma_est",self.Sigma_est

        return (self.mu_est,self.Sigma_est)

    def update(self,z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        # YOUR CODE HERE
        # for every features, do the update
        num_markers = len(z_t)
        # print "num_markers =",num_markers
        if num_markers == 0:
            return (self.mu_est, self.Sigma_est)

        # print type(z_t[0]) # list

        # 2pi
        pi_2 = 2.0*np.pi

        # Calculate the pose-measurement by each apriltag
        x_meas_apriltag_list = list()
        cov_sum = np.zeros((self.q, self.q)) # For calculating the covariance of the averaged distribution
        for tag_pose in z_t:
            id_marker = int(tag_pose[3])
            H_robot_2_marker = np.linalg.inv(self.Homography((tag_pose[0],tag_pose[1],tag_pose[2])))
            """
            H_robot_2_world = self.H_marker_2_world[id_marker].dot(H_robot_2_marker)
            x_meas = H_robot_2_world[0,2]
            y_meas = H_robot_2_world[1,2]
            theta_meas = atan2(H_robot_2_world[1,0],H_robot_2_world[0,0])
            """
            try:
                x_y_1_meas = self.H_marker_2_world[id_marker].dot( H_robot_2_marker.dot(self.homography_zeroVec) )
                # x_meas = x_y_1_meas[0,0]
                # y_meas = x_y_1_meas[1,0]
                theta_meas = self.dict_markers[id_marker][2] - tag_pose[2]
            except: # No such tag in the dictionaries
                print ("=== Unexpected tag found: #%d" % id_marker)
                #
                num_markers = num_markers - 1 # Don't count this marker
                #
                # print "=== New num_markers =", num_markers
                #
                if num_markers <= 0:
                    print "=== No any known tag, return..."
                    return (self.mu_est, self.Sigma_est) # No-tag, return
                else:
                    # print "=== Skip this tag."
                    continue # Keep the for-loop
            #
            # pi_2 = 2.0*np.pi
            if theta_meas > np.pi:
                theta_meas -= pi_2
            elif theta_meas <= -np.pi:
                theta_meas += pi_2
            #
            # print "x_meas =",x_meas, "y_meas =",y_meas, "theta_meas =",(theta_meas*180.0/np.pi), "deg"
            # x_meas_apriltag_list.append(np.array([[x_meas], [y_meas], [theta_meas]])) # 3 by 1
            x_y_1_meas[2,0] = theta_meas
            """
            # Mark_for_running
            print "Tag suggested robot-pose: ", x_y_1_meas
            print "theta_measured =", (theta_meas*180.0/np.pi), "deg"
            """
            x_meas_apriltag_list.append(x_y_1_meas) # 3 by 1
            # Calculating the (estimated) covariance of the tag
            #---------------------#
            # The square of the tag-distance
            taf_dist_2 = (tag_pose[0]**2 + tag_pose[1]**2)
            # The estimated covariance of this tag,
            # which is the function of square distance between tag and robot
            # cov_tag = self.R_t*(1.0 + taf_dist_2)
            #
            # Accumulating the covariance of each tag
            # cov_sum += cov_tag
            cov_sum += self.R_t*(1.0 + self.distortionRate*taf_dist_2)
            #---------------------#
        #
        # print (x_meas_apriltag_list[0]).shape
        # print "x_meas_apriltag_list[0]", x_meas_apriltag_list[0]

        #
        # output: num_markers, x_meas_apriltag_list, cov_sum
        #


        #
        # pi_2 = 2.0*np.pi
        if self.mu_est[2,0] > np.pi:
            self.mu_est[2,0] -= pi_2
        elif self.mu_est[2,0] <= -np.pi:
            self.mu_est[2,0] += pi_2
        #
        # Calculate the pose-error related to each measured-pose by apriltag
        error_sum = np.zeros((3,1))
        for i in range( len(x_meas_apriltag_list) ): # range(num_markers):
            """
            print type(x_meas_apriltag_list[i])
            print (x_meas_apriltag_list[i]).shape
            print type(self.mu_est)
            print (self.mu_est).shape
            print type(error_sum)
            print (error_sum).shape
            """
            delta_x = (x_meas_apriltag_list[i] - self.mu_est)
            # pi_2 = 2.0*np.pi
            if delta_x[2,0] > np.pi:
                delta_x[2,0] -= pi_2
            elif delta_x[2,0] <= -np.pi:
                delta_x[2,0] += pi_2
            #
            error_sum += delta_x
        #


        # Averaging all the estimated pose
        pose_error = error_sum*(1.0/num_markers) # The averaged pose-error
        # var_sensors = self.R_t*(1.0/num_markers) # The variance of the averaged pose that is suggested by sensors
        var_sensors = cov_sum*((1.0/num_markers)**2) # The variance of the averaged pose that is suggested by sensors
        # Correction of the angle
        while pose_error[2,0] > np.pi:
            pose_error[2,0] -= pi_2
        while pose_error[2,0] <= -np.pi:
            pose_error[2,0] += pi_2

        ##--------------------------------------##
        # Check if the covariance is too small
        is_converged = False
        for kk in range(self.n):
            if self.Sigma_est[kk,kk] <= self.cov_threshold[kk]:
                is_converged = True
        ##--------------------------------------##
        if is_converged:
            # Boost up the covariance
            # self.Sigma_est += self.R_extra
            #
            # A filter that pull the covariance toward the threshold
            self.Sigma_est += self.cov_correctionRate*(self.cov_th - self.Sigma_est)
            # Still update the covariance
            is_converged = False
        else:
            pass
        ##--------------------------------------##

        #-------------------------#
        # Update the Kalman gain
        temp = self.dhx_dx.dot(self.Sigma_est).dot(self.dhx_dx.transpose()) + var_sensors
        self.Kt = self.Sigma_est.dot(self.dhx_dx.transpose()).dot(np.linalg.inv(temp))
        # self.Kt = self.Sigma_est.dot(self.dhx_dx.transpose()).dot(np.linalg.pinv(temp)) # Use pseudo-inverse
        # self.Kt = self.Kt*(1.0/num_markers)

        # Update mean
        self.mu_est = self.mu_est + self.Kt.dot( pose_error )
        # pi_2 = 2.0*np.pi
        if self.mu_est[2,0] > np.pi:
            self.mu_est[2,0] -= pi_2
        elif self.mu_est[2,0] <= -np.pi:
            self.mu_est[2,0] += pi_2

        # print "Sigma_est",self.Sigma_est

        # Update the covariance matrix
        if is_converged:
            # No update for the covariance matrix
            pass
            # Add additional variance
            # self.Sigma_est += self.R_extra
            # self.Sigma_est = self.Sigma_est*self.cov_increse_ratio
        else:
            # Update the covariance matrix
            # temp_matrix = num_markers*(self.Kt.dot(self.dhx_dx.dot(self.Sigma_est)))
            # print "temp_matrix",temp_matrix
            # self.Sigma_est -= num_markers*(self.Kt.dot(self.dhx_dx.dot(self.Sigma_est)))
            self.Sigma_est -= self.Kt.dot(self.dhx_dx.dot(self.Sigma_est))
        #
        # print "Kt",self.Kt
        # print "mu_est",self.mu_est
        # print "Sigma_est",self.Sigma_est
        return (self.mu_est, self.Sigma_est)

    def step_filter(self, v, imu_meas, z_t, time_now):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """
        # To run the filter, we require at least that the imu_meas is available
        # imu_meas is an numpy array or None
        if imu_meas is None:
            return self.mu_est

        # YOUR CODE HERE
        # Calculate the time betweeen
        # time_now = imu_meas[4];
        if self.last_time is None:
            self.dt = 0.01
        else:
            self.dt = time_now - self.last_time

        self.last_time = time_now

        # Prediction
        self.prediction(v, imu_meas)

        # Update, when measurements are available
        if not z_t is None and not z_t == []:
            self.update(z_t)

        return self.mu_est

    # Special version using 'amcl' as updating process
    def step_filter_by_amcl(self, ros_interface, z_t, time_now):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        ros_interface - for interfacing the amcl
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """
        # To run the filter, we require at least that the imu_meas is available

        # YOUR CODE HERE
        # Calculate the time betweeen
        if self.last_time is None:
            self.dt = 0.01
        else:
            self.dt = time_now - self.last_time
        #
        self.last_time = time_now

        """
        # Prediction
        # amcl_pose = ros_interface.get_amcl_pose() # From /amcl_pose
        amcl_pose = ros_interface.get_amcl_pose_tf() # From tf and /amcl_pose
        #
        if not (amcl_pose is None):
            self.mu_est = amcl_pose[0]
            self.Sigma_est = amcl_pose[1]
            print "Before--"
            print "mu_est",self.mu_est
            print "angle_est =", (self.mu_est[2,0]*180.0/np.pi), "deg"
        else:
            # No prediction was done, return
            return self.mu_est
        """

        # Update, when measurements are available
        if not (z_t is None) and not (z_t == []):
            # Once we get the measurement, get the amcl_pose
            #--------------------------------#
            # Prediction
            # amcl_pose = ros_interface.get_amcl_pose() # From /amcl_pose
            amcl_pose = ros_interface.get_amcl_pose_tf() # From tf and /amcl_pose
            if not (amcl_pose is None):
                self.mu_est = amcl_pose[0]
                self.Sigma_est = amcl_pose[1]
                #
                stamp_amclPose = amcl_pose[2] # test
                #
                """
                # Mark_for_running
                print "Before--"
                print "mu_est",self.mu_est
                print "angle_est =", (self.mu_est[2,0]*180.0/np.pi), "deg"
                """
            else:
                # No prediction was done, return
                return self.mu_est
            #--------------------------------#
            # If we got the pose_2D, do update
            # Update
            self.update(z_t)
            # Fix the prediction of the amcl_pose
            ros_interface.set_amcl_pose(self.mu_est,self.Sigma_est)
            # ros_interface.set_amcl_pose_timeStampIn(self.mu_est,self.Sigma_est, stamp_amclPose) # test
            """
            # Mark_for_running
            print "After--"
            print "mu_est",self.mu_est
            print "angle_est =", (self.mu_est[2,0]*180.0/np.pi), "deg"
            """
        else:
            # No update was Done, no pose_2D was gotten from amcl
            pass

        return self.mu_est
