#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# =====================================================================================================
# Name: surface_polishing_Q.py
# Author: Hélio Ochoa
# Version: 0.0
# Description: Surface polishing task for redundant robots with Quaternion orientation.
# =====================================================================================================

import roslib; roslib.load_manifest('thesis_demo')
import rospy
import sys
import thread
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
    # @param wrapped If False returns the raw encoded positions, if True returns
    #                the angles with the forearm and wrist roll in the range -pi to pi
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

        position_d = line_start_point*(1 - alpha) + line_end_point*alpha
        return position_d

    ##
    # Returns the position desired for a polynomial 3 trajectory
    # @param initial position (pos_i)
    # @param final position (pos_f)
    # @param initial time(ti), final time(tf) and the time to finish the trajectory
    def polynomial3(self, pos_i, pos_f, ti, tf, t):

        pos_d = pos_i + ((3*(pos_f-pos_i)*(t-ti)**2)/(tf-ti)**2) - (2*(pos_f-pos_i)*(t-ti)**3)/(tf-ti)**3
        return pos_d


def main():
    rospy.init_node('surface_polishing_Q')

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

    flag1 = 0
    flag2 = 0
    flag3 = 0

    rate = rospy.Rate(100)  # 100hz
    while not rospy.is_shutdown():

        exit_menu = str(raw_input("\nPress <Enter> to continue or letter <e> to exit: "))
        if exit_menu == 'e':
            print("\nExit...")
            break

        else:
            pi = 0
            Qi = 0
            pf = 0
            Qf = 0
            while not flag1:

                print('\n =========================================================')
                print('\n                   Surface Polishing                      ')
                print('\n =========================================================')
                print('\n')

                flag3 = 0
                flag2 = 0

                choice = int(raw_input('\nPlease choose the final point, after that select the number (1): '))
                if choice == 1:

                    qf = np.transpose(arm.get_joint_positions())  # Joint Angles(rad)
                    ## final position (pf) and finalRotation (Rf)
                    pf, Rf = kin.FK(qf)

                    Tbe = kin.forward(qf, end_link, base_link)
                    Qf = tf.transformations.quaternion_from_matrix(Tbe)  # final Quaternion Qf = [qx qy qz qw]

                    print '\nPf = ', pf
                    print '\nQf = ', Qf

                    print('\nYou have selected the final point!')

                    flag1 = 1
                else:
                    print('\nPlease select the number (1) to continue...')

            while not flag2:
                choice = int(raw_input('\nPlease choose the initial point, after that select the number (2): '))
                if choice == 2:

                    qi= np.transpose(arm.get_joint_positions())  # Joint Angles(rad)
                    ## initial position (pi) and initial Rotation (Ri)
                    pi, Ri = kin.FK(qi)

                    Tbe = kin.forward(qi, end_link, base_link)
                    Qi = tf.transformations.quaternion_from_matrix(Tbe)  # final Quaternion Qi = [qx qy qz qw]

                    print '\nPi = ', pi
                    print '\nQi = ', Qi

                    print('\nYou have selected the initial point!')

                    flag2 = 1
                else:
                    print('\nPlease select the number (2) to continue...')

            while not flag3:
                choice = int(raw_input('\nPlease select (3) to start the polishing task: '))
                if choice == 3:

                    # =================================================================================================
                    # KST FILE
                    # =================================================================================================
                    path = '/home/ochoa/kst_thesis/surface_polishing_Q.txt'
                    file = open(path, "w")
                    line1 = [' t; ',
                             'pXc; ', 'pXd; ', 'pYc; ', 'pYd; ', 'pZc; ', 'pZd;',
                             'Qxc; ', 'Qxd; ', 'Qyc; ', 'Qyd; ', 'Qzc; ', 'Qzd; ', 'Qwc; ', 'Qwd; ',
                             'Fx; ', 'Fy; ', 'Fz', '\n']

                    line2 = [' s; ',
                             'm; ', 'm; ', 'm; ', 'm; ', 'm; ', 'm; ',
                             'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ',
                             'N; ', 'N; ', 'N', '\n']

                    file.writelines(line1)
                    file.writelines(line2)

                    # =================================================================================================
                    # Initial Conditions
                    # =================================================================================================
                    # Position Desired(pos_d) and Rotation Desired(Rd)
                    pd = pi
                    Qd = Qi

                    # Velocity Desired (vd, wd)
                    vd = np.matrix([[0], [0], [0]])  # linear velocity desired
                    wd = np.matrix([[0], [0], [0]])  # angular velocity desired

                    ad = np.matrix([[0], [0], [0], [0], [0], [0]])  # desired acceleration

                    dt = 0.01  # integration interval
                    error_sum_p = 0  # position error sum

                    # low-pass filter
                    qc_filter_last = np.matrix([[0], [0], [0], [0], [0], [0]])
                    dqc_filter_last = np.matrix([[0], [0], [0], [0], [0], [0]])
                    effc_filter_last = np.matrix([[0], [0], [0], [0], [0], [0]])
                    alpha = 100  # cut-off frequency(Hz)
                    h = 0.01  # sample time(ms)

                    # =================================================================================================
                    # Initial Trajectory Conditions
                    # =================================================================================================
                    # line
                    t = 0
                    pos_i = pi + np.matrix([[0],[0],[-0.02]])
                    pos_f = pf + np.matrix([[0],[0],[-0.02]])

                    # ellipse
                    t1 = 0
                    tf1 = 30
                    a = 0.05
                    b = 0.05
                    T = 8   # Period(s)
                    wr = (2*np.pi)/T    # resonance frequency
                    pos_0 = np.matrix([[0],  # Centre of the ellipse
                                       [0],
                                       [0]])

                    vel_i = np.matrix([[0], [0], [0]])


                    # =================================================================================================
                    # main loop
                    # =================================================================================================
                    duration_sec = 1000
                    count = 0
                    rate = rospy.Rate(100)  # 100Hz
                    L = []
                    thread.start_new_thread(input_thread, (L,))
                    while count < 100 * duration_sec:
                        count = count + 1

                        qc = np.transpose(arm.get_joint_positions())     # Current joint positions (rad)
                        dqc = np.transpose(arm.get_joint_velocities())   # Current joint velocities (rad/s)
                        effc = np.transpose(arm.get_joint_efforts())     # Current joint efforts (N/m)

                        qc_filter = (alpha * h * qc + qc_filter_last) / (1 + alpha * h)
                        dqc_filter = (alpha * h * dqc + dqc_filter_last) / (1 + alpha * h)
                        effc_filter = (alpha * h * effc + effc_filter_last) / (1 + alpha * h)

                        # Current Position(pc) and current Rotation(Rc)
                        pc, Rc = kin.FK(qc_filter)

                        Tbe = kin.forward(qc_filter, end_link, base_link)
                        Qc = tf.transformations.quaternion_from_matrix(Tbe)  # current Quaternion Qc = [qx qy qz qw]

                        J = kin.jacobian(qc_filter) # jacobian matrix at the end_link
                        JT = np.transpose(J)         # transpose jacobian

                        vel_current = J * dqc_filter  # current end-effector velocity(v, w)
                        vc = vel_current[0:3]  # current linear velocity
                        wc = vel_current[3:6]  # current angular velocity

                        M = kin.inertia(qc_filter)  # joint space mass matrix at the end_link

                        LAMBDA = inv(J*inv(M)*JT)  # cartesian space mass matrix at the end_link

                        Fe = arm.end_effector_force(JT, effc_filter)  # end-effector force (Fe = [Fx, Fy, Fz, Tx, Ty, Tz])

                        # =============================================================================================
                        # Trajectory
                        # =============================================================================================
                        if count > 500:
                            if t <= 1:
                                pd = arm.trajectory_line(pos_i, pos_f, t)
                                # Qd = arm.trajectory_line(Qi, Qf, t)
                                pos_0 = pd
                                t = t + 0.003

                        if count > 1500:
                            # ellipse
                            vd = np.matrix([[0], [0], [0]])
                            ad = np.matrix([[0], [0], [0], [0], [0], [0]])
                            if t1 <= tf1:
                                pd = pos_0 + np.matrix([[a*np.cos(wr*t1) - a],
                                                           [b*np.sin(wr*t1)],
                                                           [0]])

                                vd = np.add(vel_i, np.matrix([[-wr*a*np.sin(wr*t1)],
                                                              [wr*b*np.cos(wr*t1)],
                                                              [0]]))

                                ad = np.matrix([[-(wr**2)*a*np.cos(wr*t1)],
                                                [-(wr**2)*b*np.sin(wr*t1)],
                                                [0],
                                                [0],
                                                [0],
                                                [0]])

                                t1 = t1 + 0.01



                        # =============================================================================================
                        # Mass-Spring-Damper gains
                        # =============================================================================================
                        # Mass(A)
                        Ainv = J*inv(M)*JT  # NO INERTIA SHAPING -> Fe terms are canceled (A = LAMBDA(q))

                        # Damper(D) position(p) and orientation(o) gains
                        Dp = np.matrix([[24, 0, 0],
                                        [0, 24, 0],
                                        [0, 0, 24]])

                        Do = np.matrix([[3, 0, 0],
                                        [0, 3, 0],
                                        [0, 0, 3]])

                        # Spring(K) position(p) and orientation(o) gains
                        Kp = np.matrix([[400, 0, 0],
                                        [0, 400, 0],
                                        [0, 0, 500]])

                        Ko = np.matrix([[13, 0, 0],
                                        [0, 13, 0],
                                        [0, 0, 13]])    #13

                        # Integrative(I) position(p) and orientation(o) gains
                        Ip = np.matrix([[30, 0, 0],
                                        [0, 30, 0],
                                        [0, 0, 0]])

                        # =============================================================================================
                        # Impedance Controller (Task Space)
                        # =============================================================================================
                        # position
                        delta_p = pd - pc  # position error
                        error_sum_p += delta_p * dt  # error sum
                        delta_v = vd - vc  # linear velocity error
                        w_pos = Kp*delta_p + Dp*(delta_v) + Ip*error_sum_p

                        # orientation
                        Qc_inv = tf.transformations.quaternion_inverse(Qc)
                        Qcd = tf.transformations.quaternion_multiply(Qd, Qc_inv)
                        if Qcd[3] < 0:
                            delta_o = np.matrix([[0],
                                                 [0],
                                                 [0]])
                        else:
                            delta_o = np.matrix([[Qcd[0]],
                                                 [Qcd[1]],
                                                 [Qcd[2]]])
                        delta_w = wd - wc  # angular velocity error
                        w_ori = Do*delta_w + Ko*delta_o

                        w = ad + Ainv*(np.concatenate((w_pos, w_ori), axis=0))  # auxiliary variable -> w = ac (current acceleration)
                        Fc = LAMBDA*w  # cartesian force

                        # Command Torques (tau)
                        tau = (JT * Fc)  # with gravity(g) matrix of DSP

                        # Publish Torque Commands
                        jointCmds = [tau.item(0), tau.item(1), tau.item(2), tau.item(3), tau.item(4), tau.item(5)]
                        arm.publishTorqueCmd(jointCmds)

                        qc_filter_last = qc_filter
                        dqc_filter_last = dqc_filter
                        effc_filter_last = effc_filter
                        # =============================================================================================
                        # SEND TO FILE
                        # =============================================================================================
                        TIME = count * dt
                        # position
                        # position
                        pXc = pc.item(0)
                        pXd = pd.item(0)
                        pYc = pc.item(1)
                        pYd = pd.item(1)
                        pZc = pc.item(2)
                        pZd = pd.item(2)

                        # orientation
                        Qxc = Qc[0]
                        Qxd = Qd[0]
                        Qyc = Qc[1]
                        Qyd = Qd[1]
                        Qzc = Qc[2]
                        Qzd = Qd[2]
                        Qwc = Qc[3]
                        Qwd = Qd[3]

                        # End-Effector force
                        Fx = Fe.item(0)
                        Fy = Fe.item(1)
                        Fz = Fe.item(2)


                        lines = ' ' + str(TIME) + '; ' \
                                + str(pXc) + '; ' + str(pXd) + '; ' + str(pYc) + '; ' + str(pYd) + '; ' + str(pZc) + '; ' + str(pZd) + '; ' \
                                + str(Qxc) + '; ' + str(Qxd) + '; ' \
                                + str(Qyc) + '; ' + str(Qyd) + '; ' \
                                + str(Qzc) + '; ' + str(Qzd) + '; ' \
                                + str(Qwc) + '; ' + str(Qwd) + '; ' \
                                + str(Fx) + '; ' + str(Fy) + '; ' + str(Fz)
                        file.write(lines + '\n')

                        rate.sleep()
                        if L:
                            break

                    file.close()

                    # =============================================================================================
                    # PRINTS
                    # =============================================================================================
                    print '\n ------------------------------------------------------------------------------------------------------------------------------------'
                    print '\nPosition Error:', delta_p
                    print '\nOrientation Error:', delta_o
                    print '\nTau:', jointCmds
                    rospy.loginfo("I will publish to the topic %d", count)
                    print '\n ------------------------------------------------------------------------------------------------------------------------------------'

                    flag3 = 1
                    flag1 = 0
                else:
                    print('\nPlease select the number (3) to start polishing...')

        rate.sleep()

    setTorqueControlMode(prefix, flag=0)  # use service to switch to position control
    # homeRobot(prefix)  # use service to send robot to home position


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass