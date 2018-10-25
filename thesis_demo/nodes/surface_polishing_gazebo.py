#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# =====================================================================================================
# Name: surface_polishing_gazebo.py
# Author: Hélio Ochoa
# Version: 1.0
# Description: Polishing task in Gazebo simulator.
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
from std_msgs.msg import Float64
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_kinematics import KDLKinematics


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

        # Get info from URDF model
        self.joint_types = []
        for jnt_name in self.get_joint_names():
            jnt = urdf.joint_map[jnt_name]
            self.joint_types.append(jnt.type)

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
    def get_joint_positions(self, wrapped=False):
        if self.joint_positions is None:
            rospy.logwarn("\nJoint positions haven't been filled yet.")
            return None
        if wrapped:
            return self.wrap_angles(self.joint_positions)
        else:
            return self.joint_positions

    ##
    # Returns joint angles for continuous joints to a range [0, 2*PI)
    # @param q List of joint positions.
    # @return np.array of wrapped joint positions.
    def wrap_angles(self, q):
        continuous = self.joint_types == 'continuous'
        return np.where(continuous, np.mod(q, 2 * np.pi), q)

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

    ## TRAJECTORY CONTROL FOR JOINTS
    # Publish the joint commands to move the robot joints
    # @param number of joints(nbJoints)
    def moveJoint(self, jointcmds, nbJoints):
        topic_name = '/j2n6s300/effort_joint_trajectory_controller/command'
        pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
        jointCmd = JointTrajectory()
        point = JointTrajectoryPoint()
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        point.time_from_start = rospy.Duration.from_sec(5.0)
        for i in range(0, nbJoints):
            jointCmd.joint_names.append('j2n6s300_joint_' + str(i + 1))
            point.positions.append(jointcmds[i])
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0)
        jointCmd.points.append(point)
        rate = rospy.Rate(100)
        count = 0
        while count < 500:
            pub.publish(jointCmd)
            count = count + 1
            rate.sleep()

    ## TRAJECTORY CONTROL FOR FINGERS
    # Publish the joint commands to move the fingers
    # @param number of fingers(nbFingers)
    def moveFingers(self, jointcmds, nbFingers):
        topic_name = '/j2n6s300/effort_finger_trajectory_controller/command'
        pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
        jointCmd = JointTrajectory()
        point = JointTrajectoryPoint()
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        point.time_from_start = rospy.Duration.from_sec(5.0)
        for i in range(0, nbFingers):
            jointCmd.joint_names.append('j2n6s300_joint_finger_' + str(i + 1))
            point.positions.append(jointcmds[i])
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0)
        jointCmd.points.append(point)
        rate = rospy.Rate(100)
        count = 0
        while count < 100:
            pub.publish(jointCmd)
            count = count + 1
            rate.sleep()

    ##
    #  ============================= Switch controllers  =============================
    def switchController(self, flag):
        rospy.wait_for_service('/j2n6s300/controller_manager/switch_controller')
        try:
            switch_controller = rospy.ServiceProxy('/j2n6s300/controller_manager/switch_controller', SwitchController)

            if flag == 1:
                ## Switch to torque controller
                switch_controller(['joint_1_torque_controller',
                                   'joint_2_torque_controller',
                                   'joint_3_torque_controller',
                                   'joint_4_torque_controller',
                                   'joint_5_torque_controller',
                                   'joint_6_torque_controller',
                                   'finger_1_position_controller',
                                   'finger_2_position_controller',
                                   'finger_3_position_controller'],
                                  ['effort_joint_trajectory_controller',
                                   'effort_finger_trajectory_controller'], 1)

                print("\nTORQUE CONTROLLER MODE!")

            if flag == 0:
                ## Switch to position controller
                switch_controller(['effort_joint_trajectory_controller',
                                   'effort_finger_trajectory_controller'],
                                  ['joint_1_torque_controller',
                                   'joint_2_torque_controller',
                                   'joint_3_torque_controller',
                                   'joint_4_torque_controller',
                                   'joint_5_torque_controller',
                                   'joint_6_torque_controller',
                                   'finger_1_position_controller',
                                   'finger_2_position_controller',
                                   'finger_3_position_controller'], 1)

                print("\nTRAJECTORY CONTROLLER MODE!")


        except rospy.ServiceException, e:
            print
            "Service call failed: %s" % e

    ## TORQUE CONTROL
    # Publish the joint torque
    # @param jointCmds - joint commands
    # @param fingerCmds - finger commands (open/.../close)
    def publishTorqueCmd(self, jointCmds, fingerCmds):

        # Define publishers for each joint controller.
        pub1 = rospy.Publisher('/j2n6s300/joint_1_torque_controller/command', Float64, queue_size=1)
        pub2 = rospy.Publisher('/j2n6s300/joint_2_torque_controller/command', Float64, queue_size=1)
        pub3 = rospy.Publisher('/j2n6s300/joint_3_torque_controller/command', Float64, queue_size=1)
        pub4 = rospy.Publisher('/j2n6s300/joint_4_torque_controller/command', Float64, queue_size=1)
        pub5 = rospy.Publisher('/j2n6s300/joint_5_torque_controller/command', Float64, queue_size=1)
        pub6 = rospy.Publisher('/j2n6s300/joint_6_torque_controller/command', Float64, queue_size=1)

        self.jointCmd = [Float64().data, Float64().data, Float64().data, Float64().data, Float64().data, Float64().data]
        self.jointCmd = jointCmds
        pub1.publish(self.jointCmd[0])
        pub2.publish(self.jointCmd[1])
        pub3.publish(self.jointCmd[2])
        pub4.publish(self.jointCmd[3])
        pub5.publish(self.jointCmd[4])
        pub6.publish(self.jointCmd[5])

        # Define publishers for each finger controller.
        pub7 = rospy.Publisher('/j2n6s300/finger_1_position_controller/command', Float64, queue_size=1)
        pub8 = rospy.Publisher('/j2n6s300/finger_2_position_controller/command', Float64, queue_size=1)
        pub9 = rospy.Publisher('/j2n6s300/finger_3_position_controller/command', Float64, queue_size=1)

        self.fingerCmd = [Float64().data, Float64().data, Float64().data]
        self.fingerCmd = fingerCmds
        pub7.publish(self.fingerCmd[0])
        pub8.publish(self.fingerCmd[1])
        pub9.publish(self.fingerCmd[2])

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
    rospy.init_node('Surface_Polishing')

    robot = get_urdf_model(Robot)
    base_link = robot.get_root()
    end_link = "j2n6s300_end_effector"
    topic_name = "/j2n6s300/joint_states"
    arm = RobotControl(robot, base_link, end_link, topic_name)
    kin = arm.get_jaco_kinematics()
    rospy.sleep(2)

    ## Switch to torque controller
    arm.switchController(flag=1)

    # =================================================================================================
    # KST FILE
    # =================================================================================================
    path = '/home/ochoa/kst_thesis/polishing_gazebo.txt'
    file = open(path, "w")
    line1 = [' t; ', 'pXc; ', 'pXd; ', 'pYc; ', 'pYd; ', 'pZc; ', 'pZd;',
             'QXc; ', 'QXd; ', 'QYc; ', 'QYd; ', 'QZc; ', 'QZd; ', 'QWc; ', 'QWd; '
             'Fx; ', 'Fy; ', 'Fz', '\n']
    line2 = [' s; ', 'm; ', 'm; ', 'm; ', 'm; ', 'm; ', 'm; ',
             'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ',
             'N; ', 'N; ', 'N', '\n']
    file.writelines(line1)
    file.writelines(line2)

    # =================================================================================================
    # Initial Conditions
    # =================================================================================================
    home = np.matrix([[4.8046852], [2.92482], [1.002], [4.2031852], [1.4458], [1.3233]])
    qd = home  # HOME(rad)

    # desired position(pd) and desired Rotation(Rd)
    pd, Rd = kin.FK(qd)

    Tbe = kin.forward(qd, end_link, base_link)
    Qd = tf.transformations.quaternion_from_matrix(Tbe)    # desired Quaternion Qd = [qx qy qz qw]

    # Velocity Desired (vd, wd)
    vd = np.matrix([[0], [0], [0]])  # linear velocity desired
    wd = np.matrix([[0], [0], [0]])  # angular velocity desired

    ad = np.matrix([[0], [0], [0], [0], [0], [0]])  # desired acceleration

    dt = 0.01  # sample time
    error_sum_p = 0  # position error sum

    ## trajectory conditions
    # step1
    t1 = 0
    ti1 = 0
    tf1 = 5
    pi1 = pd
    Qi1 = Qd
    pf1 = pi1 + np.matrix([[0.0], [-0.1], [-0.1]])
    Qz = tf.transformations.quaternion_about_axis(-np.pi/2, (0, 1, 0))
    Qf1 = tf.transformations.quaternion_multiply(Qi1, Qz)

    # step2
    t2 = 0
    pi2 = pf1
    pf2 = pi2 + np.matrix([[0.0], [0.0], [-0.37]])
    # pf2 = pi2 + np.matrix([[0.0], [0.0], [-0.2]])

    # step3
    t3 = 0
    tf3 = 30

    ## ramp:
    # a = 0.1
    # b = 0.1
    ## card_box & cinder_block_wide:
    a = 0.05
    b = 0.05

    T = 8
    wr = (2*np.pi)/T
    p0 = np.matrix([[0], [0], [0]])
    v0 = np.matrix([[0], [0], [0]])

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

        qc = np.transpose(arm.get_joint_positions(wrapped=False))  # Current joint positions (rad)
        dqc = np.transpose(arm.get_joint_velocities())  # Current joint velocities (rad/s)
        effc = np.transpose(arm.get_joint_efforts())  # current joint efforts (N/m)

        # current Position(pc) and current Rotation(Rc)
        pc, Rc = kin.FK(qc)

        Tbe = kin.forward(qc, end_link, base_link)
        Qc = tf.transformations.quaternion_from_matrix(Tbe)  # current Quaternion Qc = [qx qy qz qw]

        J = kin.jacobian(qc)  # jacobian matrix at the end_link
        JT = np.transpose(J)  # transpose jacobian

        vel_current = J * dqc # current end-effector velocity(v, w)
        vc = vel_current[0:3]  # current linear velocity
        wc = vel_current[3:6]  # current angular velocity

        g = np.transpose(np.matrix(kin.gravity(qc)))  # gravity matrix

        M = kin.inertia(qc)  # joint space mass matrix at the end link

        LAMBDA = inv(J * inv(M) * JT)  # cartesian space mass matrix at the end_link

        Fe = arm.end_effector_force(JT, effc)  # end-effector force (Fe = [Fx, Fy, Fz, Tx, Ty, Tz])

        # =============================================================================================
        # POLISHING TASK
        # =============================================================================================
        # step1
        if count > 1000:
            if t1 <= tf1:
                pd = arm.polynomial3(pi1, pf1, ti1, tf1, t1)
                Qd = arm.polynomial3(Qi1, Qf1, ti1, tf1, t1)
                t1 = t1 + 0.01

        # step2
        if count > 2000:
            if t2 <= 1:
                pd = arm.trajectory_line(pi2, pf2, t2)
                p0 = pd
                t2 = t2 + 0.002

        # step3
        if count > 3000:
            vd = np.matrix([[0], [0], [0]])
            ad = np.matrix([[0], [0], [0], [0], [0], [0]])
            if t3 <= tf3:
                pd = p0 + np.matrix([[a * np.cos(wr * t3) - a],
                                     [b * np.sin(wr * t3)],
                                     [0]])

                vd = v0 + np.matrix([[-wr * a * np.sin(wr * t3)],
                                     [wr * b * np.cos(wr * t3)],
                                     [0]])

                ad = np.matrix([[-(wr ** 2) * a * np.cos(wr * t3)],
                                [-(wr ** 2) * b * np.sin(wr * t3)],
                                [0],
                                [0],
                                [0],
                                [0]])

                t3 = t3 + 0.01


        # =============================================================================================
        # Spring-Damper gains
        # =============================================================================================
        # Mass(A)
        Ainv = J * inv(M) * JT  # NO INERTIA SHAPING -> Fe terms are canceled (A = LAMBDA(q))

        # Damper(D) position(p) and orientation(o) gains
        Dp = np.matrix([[35, 0, 0],
                        [0, 35, 0],
                        [0, 0, 35]])    #30

        Do = np.matrix([[2, 0, 0],
                        [0, 2, 0],
                        [0, 0, 2]])   #1.5

        # Spring(K) position(p) and orientation(o) gains
        Kp = np.matrix([[400, 0, 0],
                        [0, 400, 0],
                        [0, 0, 500]])   #200

        Ko = np.matrix([[10, 0, 0],
                        [0, 10, 0],
                        [0, 0, 10]])     #5

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
        w_pos = Kp * delta_p + Dp * (delta_v) + Ip * error_sum_p

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
        w_ori = Do * delta_w + Ko * delta_o

        w = ad + Ainv * (np.concatenate((w_pos, w_ori), axis=0) - Fe)  # auxiliary variable -> w = ac (current acceleration)
        Fc = LAMBDA * w  # cartesian force

        # Command Torques (tau)
        tau = (JT * Fc) + g + (JT * Fe)

        # Publish Torque Commands
        jointCmds = [tau.item(0), tau.item(1), tau.item(2), tau.item(3), tau.item(4), tau.item(5)]
        finger_position = [0, 0, 0]  # Gripper Open
        # finger_position = [1, 1, 1]   # Gripper Close
        arm.publishTorqueCmd(jointCmds, finger_position)


        # =============================================================================================
        # SEND TO FILE
        # =============================================================================================
        TIME = count * 0.01
        # position
        pXc = pc.item(0)
        pXd = pd.item(0)
        pYc = pc.item(1)
        pYd = pd.item(1)
        pZc = pc.item(2)
        pZd = pd.item(2)

        # orientation
        QXd = Qd[0]
        QXc = Qc[0]
        QYd = Qd[1]
        QYc = Qc[1]
        QZd = Qd[2]
        QZc = Qc[2]
        QWd = Qd[3]
        QWc = Qc[3]

        # Fx = 0
        # Fy = 0
        # Fz = 0
        # if count > 500:
        Fx = Fe.item(0)
        Fy = Fe.item(1)
        Fz = Fe.item(2)

        lines = ' ' + str(TIME) + '; ' \
                + str(pXc) + '; ' + str(pXd) + '; ' + str(pYc) + '; ' + str(pYd) + '; ' + str(pZc) + '; ' + str(pZd) + '; ' \
                + str(QXc) + '; ' + str(QXd) + '; ' \
                + str(QYc) + '; ' + str(QYd) + '; ' \
                + str(QZc) + '; ' + str(QZd) + '; ' \
                + str(QWc) + '; ' + str(QWd) + '; ' \
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

    ## Switch to position controller
    arm.switchController(flag=0)

    # home robot
    home = [4.8046852, 2.92482, 1.002, 4.2031852, 1.4458, 1.3233]
    nbJoints = 6
    nbfingers = 3
    arm.moveJoint(home, nbJoints)
    arm.moveFingers([0, 0, 0], nbfingers)
    print('\nRobot has returned to HOME...')


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
