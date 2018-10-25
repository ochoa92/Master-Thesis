#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# =====================================================================================================
# Name: jaco2_control_robot.py
# Author: Hélio Ochoa
# Version: 0.0
# Description: Joint Space Torque Control With Task Space Posture Reference (real robot)
# =====================================================================================================

import roslib; roslib.load_manifest('thesis_demo')
import rospy
import sys
import thread
import math
import tf

import numpy as np
from numpy.linalg import inv

from sensor_msgs.msg import JointState
import kinova_msgs.msg
from kinova_msgs.srv import *

from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_kinematics import KDLKinematics



## use service to send robot to home position
def homeRobot(prefix):
    service_address = '/' + prefix + '_driver/in/home_arm'
    rospy.wait_for_service(service_address)
    try:
        home = rospy.ServiceProxy(service_address, HomeArm)
        home()
        print("\nKINOVA ARM HAS BEEN RETURNED HOME!")
        return None
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


## use service to set torque control parameters
def setTorqueControlParameters(prefix):
    service_address = '/' + prefix + '_driver/in/set_torque_control_parameters'
    rospy.wait_for_service(service_address)
    try:
        setTorqueParameters = rospy.ServiceProxy(service_address, SetTorqueControlParameters)
        setTorqueParameters()
    except rospy.ServiceException, e:
        print "\nService call failed: %s" % e
        return None


## use service to switch controllers
# @ prefix: j2n6s300
# @ flag: 1 - TORQUE CONTROL MODE
# @ flag: 0 - POSITION CONTROL MODE
def setTorqueControlMode(prefix, flag):
    service_address = '/' + prefix + '_driver/in/set_torque_control_mode'
    rospy.wait_for_service(service_address)
    try:
        switchTorquemode = rospy.ServiceProxy(service_address, SetTorqueControlMode)
        if flag == 1:
            switchTorquemode(1)
            print("\nTORQUE CONTROL MODE!")
        if flag == 0:
            switchTorquemode(0)
            print("\nPOSITION CONTROL MODE!")
    except rospy.ServiceException, e:
        print "\nService call failed: %s" % e
        return None



##
# returns robot URDF
# @param urdf URDF object of robot.
def get_urdf_model(Robot):
    def usage():
        print("Tests for kdl_parser:\n")
        print("kdl_parser <urdf file>")
        print("\tLoad the URDF from file.")
        print("kdl_parser")
        print("\tLoad the URDF from the parameter server.")
        sys.exit(1)

    if len(sys.argv) > 2:
        usage()
    if len(sys.argv) == 2 and (sys.argv[1] == "-h" or sys.argv[1] == "--help"):
        usage()
    if (len(sys.argv) == 1):
        robot = Robot.from_parameter_server()
    else:
        f = file(sys.argv[1], 'r')
        robot = Robot.from_xml_string(f.read())
        f.close()

    return robot


##
# Thread to interrupt the loop main
def input_thread(L):
    raw_input("\nPress <ENTER> to interrupt the main loop!\n")
    L.append(None)


class RobotControl(object):
    ##
    # Constructor
    # @param urdf URDF object of robot.
    # @param base_link Name of the root link of the kinematic chain. (robot.get_root())
    # @param end_link Name of the end link of the kinematic chain. ("j2n6s300_end_effector")
    # @param topic_name Name of topic to get the joint states of the robot. ("/j2n6s300/joint_states")
    #
    def __init__(self, urdf, base_link, end_link, topic_name):
        self.urdf = urdf
        self.base_link = base_link
        self.end_link = end_link
        self.topic_name = topic_name
        self.joint_names = None
        self.joint_positions = None
        self.joint_velocities = None
        self.joint_efforts = None
        self.js_idx = None  # Joint state index

        rospy.Subscriber(self.topic_name, JointState, self.get_joint_state)


    ##
    # returns a list of the link names in kinematic chain.
    def get_link_names(self, joints=False, fixed=True):
        return self.urdf.get_chain(self.base_link, self.end_link, joints, fixed)

    ##
    # @returns a List of the joint names in the kinematic chain.
    def get_joint_names(self, links=False, fixed=False):
        return self.urdf.get_chain(self.base_link, self.end_link, links=links, fixed=fixed)

    ##
    # Joint states listener callback
    def get_joint_state(self, msg):
        if self.js_idx is None:
            joint_names_list = self.get_joint_names()
            self.js_idx = [msg.name.index(joint_name) for joint_name in joint_names_list]

        self.joint_positions = np.matrix([msg.position[i] for i in self.js_idx])
        self.joint_velocities = np.matrix([msg.velocity[i] for i in self.js_idx])
        self.joint_efforts = np.matrix([msg.effort[i] for i in self.js_idx])

    ##
    # Returns the current joint positions (rad)
    def get_joint_positions(self):
        if self.joint_positions is None:
            rospy.logwarn("\nJoint positions haven't been filled yet.")
            return None
        else:
            return self.joint_positions


    ##
    # Returns the current joint velocities
    def get_joint_velocities(self):
        if self.joint_velocities is None:
            rospy.logwarn("\nJoint velocities haven't been filled yet.")
            return None
        return self.joint_velocities

    ##
    # Returns the current joint efforts
    def get_joint_efforts(self):
        if self.joint_efforts is None:
            rospy.logwarn("\nJoint efforts haven't been filled yet.")
            return None
        return self.joint_efforts

    ##
    # Returns robot kinematics
    def get_jaco_kinematics(self):
        kin = KDLKinematics(self.urdf, self.base_link, self.end_link)
        return kin

    ## TORQUE CONTROL
    # Publish the joint torques
    # @param joint Commands
    # @param prefix(j2n6s300)
    def publishTorqueCmd(self, jointCmds):
        # publish joint torque commands
        topic_name = '/j2n6s300_driver/in/joint_torque'
        pub = rospy.Publisher(topic_name, kinova_msgs.msg.JointTorque, queue_size=1)
        jointCmd = kinova_msgs.msg.JointTorque()
        jointCmd.joint1 = jointCmds[0]
        jointCmd.joint2 = jointCmds[1]
        jointCmd.joint3 = jointCmds[2]
        jointCmd.joint4 = jointCmds[3]
        jointCmd.joint5 = jointCmds[4]
        jointCmd.joint6 = jointCmds[5]

        pub.publish(jointCmd)

    ##
    # Returns the current effective end effector force.
    def end_effector_force(self, JT, eff):
        tau = eff
        if JT is None or tau is None:
            return None
        Fe = inv(JT) * tau
        return Fe

    ##
    # Returns the position desired for a trajectory line
    # @param line start point
    # @param line end point
    # @param step time(t) -> t:[0, 1]
    def trajectory_line(self, line_start_point, line_end_point, alpha):

        position_d = line_start_point * (1 - alpha) + line_end_point * alpha
        return position_d

    ##
    # Returns the position desired for a polynomial 3 trajectory
    # @param initial position (pos_i)
    # @param final position (pos_f)
    # @param initial time(ti), final time(tf) and the time to finish the trajectory
    def polynomial3(self, pos_i, pos_f, ti, tf, t):

        pos_d = pos_i + ((3 * (pos_f - pos_i) * (t - ti) ** 2) / (tf - ti) ** 2) - (
                    2 * (pos_f - pos_i) * (t - ti) ** 3) / (tf - ti) ** 3
        return pos_d


def main():
    rospy.init_node('jaco2_control')

    robot = get_urdf_model(Robot)
    base_link = robot.get_root()
    end_link = "j2n6s300_end_effector"
    topic_name = "/j2n6s300_driver/out/joint_state"
    arm = RobotControl(robot, base_link, end_link, topic_name)
    kin = arm.get_jaco_kinematics()
    rospy.sleep(2)

    prefix = 'j2n6s300'
    setTorqueControlParameters(prefix)  # use service to set torque control parameters
    setTorqueControlMode(prefix, flag=1)  # use service to switch to torque control

    # =================================================================================================
    # KST FILE
    # =================================================================================================
    path = '/home/ochoa/kst_thesis/jaco2_control_robot.txt'
    file = open(path, "w")
    line1 = [' t; ',
             'pXc; ', 'pXd; ', 'pYc; ', 'pYd; ', 'pZc; ', 'pZd; ',
             'Qxc; ', 'Qxd; ', 'Qyc; ', 'Qyd; ', 'Qzc; ', 'Qzd; ', 'Qwc; ', 'Qwd', '\n']
    line2 = [' s; ',
             'm; ', 'm; ', 'm; ', 'm; ', 'm; ', 'm; ',
             'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q', '\n']
    file.writelines(line1)
    file.writelines(line2)


    # =================================================================================================
    # Initial Conditions
    # =================================================================================================
    home = np.matrix([[4.8046852], [2.92482], [1.002], [4.2031852], [1.4458], [1.3233]])
    # desired position(Xd) and desired Rotation(Rd)
    Xd, Rd = kin.FK(home)

    Tbe = kin.forward(home, end_link, base_link)
    Qd = tf.transformations.quaternion_from_matrix(Tbe)  # desired Quaternion Qd = [qx qy qz qw]

    last_delta_velocity = np.matrix([[0], [0], [0], [0], [0], [0]])

    dt = 0.01  # sample time(ms)

    ## trajectory conditions
    # line
    t = 0
    pi = Xd
    Qi = Qd

    pf = pi + np.matrix([[0.1],
                         [-0.1],
                         [0.1]])
    Qz = tf.transformations.quaternion_about_axis(-np.pi/2, (0, 0, 1))
    Qf = tf.transformations.quaternion_multiply(Qi, Qz)

    # Ellipse:
    t1 = 0
    tf1 = 60
    a = 0.1
    b = 0.1
    T = 12
    wr = (2 * np.pi) / T
    p0 = np.matrix([[0],  # initial point of the ellipse
                    [0],
                    [0]])

    # =================================================================================================
    # main loop
    # =================================================================================================
    duration_sec = 100
    count = 0
    rate = rospy.Rate(100)  # 100Hz
    L = []
    thread.start_new_thread(input_thread, (L,))
    while count < 100 * duration_sec:
        count = count + 1

        qc = np.transpose(arm.get_joint_positions())  # Current joint positions (rad)
        dqc = np.transpose(arm.get_joint_velocities())  # Current joint velocities (rad/s)

        # current Position(pc) and current Rotation(Rc)
        Xc, Rc = kin.FK(qc)

        Tbe = kin.forward(qc, end_link, base_link)
        Qc = tf.transformations.quaternion_from_matrix(Tbe)  # current Quaternion Qc = [qx qy qz qw]

        M = kin.inertia(qc)  # joint space mass matrix at the end link

        J = kin.jacobian(qc)  # jacobian matrix at the end_link
        Jinv = inv(J)  # inverse jacobian

        # =============================================================================================
        # Trajectory
        # =============================================================================================
        # line
        if count > 1000:
            if t <= 1:
                Xd = arm.trajectory_line(pi, pf, t)
                # Qd = arm.trajectory_line(Qi, Qf, t)
                p0 = Xd
                t = t + 0.005

        # # ellipse
        # if count > 2000:
        #     if t1 <= tf1:
        #         Xd = p0 + np.matrix([[0],
        #                              [a * np.cos(wr * t1) - a],
        #                              [b * np.sin(wr * t1)]])
        #         t1 = t1 + 0.01

        # =============================================================================================
        # Controller gains
        # =============================================================================================
        K1 = np.matrix([[5, 0, 0, 0, 0, 0],     #7
                        [0, 5, 0, 0, 0, 0],
                        [0, 0, 5, 0, 0, 0],
                        [0, 0, 0, 3, 0, 0],    #7
                        [0, 0, 0, 0, 3, 0],
                        [0, 0, 0, 0, 0, 3]])

        Kp = np.matrix([[25, 0, 0, 0, 0, 0],    #50
                        [0, 25, 0, 0, 0, 0],
                        [0, 0, 25, 0, 0, 0],
                        [0, 0, 0, 5, 0, 0],   #50
                        [0, 0, 0, 0, 5, 0],
                        [0, 0, 0, 0, 0, 5]])

        Kd = np.matrix([[0, 0, 0, 0, 0, 0],  # 0.02
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0],   #0.02
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0]])


        # =============================================================================================
        # Joint Space Torque Control With Task Space Posture Reference
        # =============================================================================================
        delta_Xcd = Xd - Xc  # task space position error

        Qc_inv = tf.transformations.quaternion_inverse(Qc)
        Qcd = tf.transformations.quaternion_multiply(Qd, Qc_inv)
        if Qcd[3] < 0:
            ecd = np.matrix([[0],
                             [0],
                             [0]])  # task space orientation error
        else:
            ecd = np.matrix([[Qcd[0]],
                             [Qcd[1]],
                             [Qcd[2]]])  # task space orientation error

        Xe = np.concatenate((delta_Xcd, ecd), axis=0)

        dqd = Jinv * K1 * Xe  # desired joint space velocity

        delta_velocity = dqd - dqc  # velocity error

        delta_acceleration = (delta_velocity - last_delta_velocity) / dt
        last_delta_velocity = delta_velocity

        alpha = Kp * delta_velocity + Kd * delta_acceleration

        # Command Torques (tau)
        tau = (M * alpha)   # with gravity(g) matrix of DSP

        # Publish Torque Commands
        jointCmds = [tau.item(0), tau.item(1), tau.item(2), tau.item(3), tau.item(4), tau.item(5)]
        # arm.publishTorqueCmd(jointCmds)

        # =============================================================================================
        # SEND TO FILE
        # =============================================================================================
        TIME = count * dt
        # position
        pXc = Xc.item(0)
        pXd = Xd.item(0)
        pYc = Xc.item(1)
        pYd = Xd.item(1)
        pZc = Xc.item(2)
        pZd = Xd.item(2)

        # orientation
        Qxc = Qc[0]
        Qxd = Qd[0]
        Qyc = Qc[1]
        Qyd = Qd[1]
        Qzc = Qc[2]
        Qzd = Qd[2]
        Qwc = Qc[3]
        Qwd = Qd[3]

        lines = ' ' + str(TIME) + '; ' \
                + str(pXc) + '; ' + str(pXd) + '; ' + str(pYc) + '; ' + str(pYd) + '; ' + str(pZc) + '; ' + str(
            pZd) + '; ' \
                + str(Qxc) + '; ' + str(Qxd) + '; ' \
                + str(Qyc) + '; ' + str(Qyd) + '; ' \
                + str(Qzc) + '; ' + str(Qzd) + '; ' \
                + str(Qwc) + '; ' + str(Qwd)
        file.write(lines + '\n')

        rate.sleep()
        if L:
            break

    file.close()

    # =============================================================================================
    # PRINTS
    # =============================================================================================
    print '\n ------------------------------------------------------------------------------------------------------------------------------------'
    print '\nPosition Error:', delta_Xcd
    print '\nOrientation Error:', ecd
    print '\nJoint velocity Error:', delta_velocity
    print '\nJoint acceleration Error:', delta_acceleration
    print '\nTau:', jointCmds
    rospy.loginfo("I have published to the topic %d", count)
    print '\n ------------------------------------------------------------------------------------------------------------------------------------'


    setTorqueControlMode(prefix, flag=0)  # use service to switch to position control
    homeRobot(prefix)  # use service to send robot to home position


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
