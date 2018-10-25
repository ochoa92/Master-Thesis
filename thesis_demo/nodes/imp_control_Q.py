#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# =====================================================================================================
# Name: imp_control_Q.py
# Author: Hélio Ochoa
# Version: 0.0
# Description: Quaternion orientation control.
# =====================================================================================================

import roslib; roslib.load_manifest('thesis_demo')
import rospy
import sys
import thread
import math

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
            print "Service call failed: %s" % e

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

    ##
    # Returns the Skew Symmetric operator
    # @param vector
    def SkewSymmetric(self, vector):
        a = vector.item(0)
        b = vector.item(1)
        c = vector.item(2)

        S = np.matrix([[0, -c, b],
                       [c, 0, -a],
                       [-b, a, 0]])

        return S

    ##
    # Returns the quaternion Q = [n,e]
    # @param the rotation matrix R
    def quaternion_from_rotation(self, R):
        """
        R = [[R(1,1) R(1,2) R(1,3)]
             [R(2,1) R(2,2) R(2,3)]
             [R(3,1) R(3,2) R(3,3)]]
        """

        n = 0.5*np.sqrt(R.item(0) + R.item(4) + R.item(8) + 1)

        # e = [ex, ey, ez]
        e = 0.5*np.matrix([[np.sign(R.item(7) - R.item(5)) * np.sqrt(R.item(0) - R.item(4) - R.item(8) + 1)],
                           [np.sign(R.item(2) - R.item(6)) * np.sqrt(R.item(4) - R.item(8) - R.item(0) + 1)],
                           [np.sign(R.item(3) - R.item(1)) * np.sqrt(R.item(8) - R.item(0) - R.item(4) + 1)]])

        return n, e


    ##
    # Returns the rotation matrix R
    # @param the quaternion Q = [n,e]
    def rotation_from_quaternion(self, n, e):

        S = self.SkewSymmetric(e)   # skew-symmetric matrix operator
        I = np.identity(3)          # identity matrix (3x3)

        aux = (n**2) - np.transpose(e)*e
        R = aux.item(0)*I + 2*e*np.transpose(e) + 2*n*S

        return R

    ##
    # Returns the Quaternion normed
    # @param the quaternion Q = [n,e]
    def quaternionNorm(self, n, e):

        n_temp = n

        # e = [ex, ey, ez]
        ex_temp = e.item(0)
        ey_temp = e.item(1)
        ez_temp = e.item(2)

        Qnorm = math.sqrt(ex_temp*ex_temp + ey_temp*ey_temp + ez_temp*ez_temp + n_temp*n_temp)
        qx = ex_temp/Qnorm
        qy = ey_temp/Qnorm
        qz = ez_temp/Qnorm
        qw = n_temp/Qnorm

        Q_normed = [qx, qy, qz, qw]

        return Q_normed


    ##
    # Returns the euler angles XYZ
    #             Roll: rotation in Z
    #             Pitch: rotation in Y
    #             Yaw: rotation in X
    # @param the quaternion Q = [n,e]
    def quaternion2EulerXYZ(self, n, e):

        Q_normed = self.quaternionNorm(n, e)
        qx = Q_normed[0]
        qy = Q_normed[1]
        qz = Q_normed[2]
        qw = Q_normed[3]

        tx = math.atan2((2 * qw * qx - 2 * qy * qz), (qw * qw - qx * qx - qy * qy + qz * qz))
        ty = math.asin(2 * qw * qy + 2 * qx * qz)
        tz = math.atan2((2 * qw * qz - 2 * qx * qy), (qw * qw + qx * qx - qy * qy - qz * qz))
        EulerXYZ = [tx, ty, tz]

        return EulerXYZ


    ##
    # Returns the quaternion Q = [n,e]
    # @param the euler angles XYZ
    #            Roll: rotation in Z
    #            Pitch: rotation in Y
    #            Yaw: rotation in X
    def EulerXYZ2quaternion(self, EulerXYZ):

        tx, ty, tz = EulerXYZ[0:3]
        sx = math.sin(0.5 * tx)
        cx = math.cos(0.5 * tx)
        sy = math.sin(0.5 * ty)
        cy = math.cos(0.5 * ty)
        sz = math.sin(0.5 * tz)
        cz = math.cos(0.5 * tz)

        qx = sx * cy * cz + cx * sy * sz
        qy = -sx * cy * sz + cx * sy * cz
        qz = sx * sy * cz + cx * cy * sz
        qw = -sx * sy * sz + cx * cy * cz

        n = qw
        e = np.matrix([[qx],
                       [qy],
                       [qz]])

        return n, e


    ##
    # Returns the multiplication of two quaternions (Qa*Qb)
    def multiply_quaternions(self, na, ea, nb, eb):

        nab_temp = na*nb - np.transpose(ea)*eb
        nab = nab_temp.item(0)

        eab = na*eb + nb*ea + np.multiply(ea, eb)

        return nab, eab

    ##
    # Returns the quaternion Q = [n,e]
    # @param axis [x, y, z] and angle theta
    def quaternion_from_angle_axis(self, x, y, z, theta):

        """
        n >= 0 for theta: [-pi, pi]
        """
        n = np.cos(theta/2)

        e = np.matrix([[x*np.sin(theta/2)],
                       [y*np.sign(theta/2)],
                       [z*np.sin(theta/2)]])

        return n, e


def main():
    rospy.init_node('Impedance_Control')

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
    path = '/home/ochoa/kst_thesis/imp_control_Q.txt'
    file = open(path, "w")
    line1 = [' t; ', 'pXc; ', 'pXd; ', 'pYc; ', 'pYd; ', 'pZc; ', 'pZd;',
             'oXc; ', 'oXd; ', 'oYc; ', 'oYd; ', 'oZc; ', 'oZd', '\n']
    line2 = [' s; ', 'm; ', 'm; ', 'm; ', 'm; ', 'm; ', 'm; ',
             'rad; ', 'rad; ', 'rad; ', 'rad; ', 'rad; ', 'rad', '\n']
    file.writelines(line1)
    file.writelines(line2)



    # =================================================================================================
    # Initial Conditions
    # =================================================================================================
    home = np.matrix([[4.8046852], [2.92482], [1.002], [4.2031852], [1.4458], [1.3233]])
    q_desired = home  # HOME(rad)

    # desired position(pd) and desired Rotation(Rd)
    pd, Rd = kin.FK(q_desired)
    nd, ed = arm.quaternion_from_rotation(Rd)       # desired Quaternion Qd = [nd,ed]
    EulerXYZ_d = arm.quaternion2EulerXYZ(nd, ed)    # desired EulerXYZ angles

    # Velocity Desired (vd, wd)
    vd = np.matrix([[0], [0], [0]])    # linear velocity desired
    wd = np.matrix([[0], [0], [0]])    # angular velocity desired

    ad = np.matrix([[0], [0], [0], [0], [0], [0]])    # desired acceleration

    dt = 0.01  # integration interval
    error_sum_p = 0  # position error sum

    ## trajectory conditions
    t = 0

    # position
    pi = pd
    pf = pi + np.matrix([[0.0],
                         [-0.1],
                         [0.1]])

    # orientation
    ni = nd
    ei = ed
    n, e = arm.quaternion_from_angle_axis(0, 0, 1, -np.pi/6)
    nf, ef = arm.multiply_quaternions(ni, ei, n, e)


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

        q_current = np.transpose(arm.get_joint_positions(wrapped=False))  # Current joint positions (rad)
        dq_current = np.transpose(arm.get_joint_velocities())             # Current joint velocities (rad/s)
        eff_current = np.transpose(arm.get_joint_efforts())               # current joint efforts (N/m)

        # Current Position(pc) and current Rotation(Rc)
        pc, Rc = kin.FK(q_current)
        nc, ec = arm.quaternion_from_rotation(Rc)       # current Quaternion Qc = [nc,ec]
        EulerXYZ_c = arm.quaternion2EulerXYZ(nc, ec)    # current EulerXYZ angles

        J = kin.jacobian(q_current)  # jacobian matrix at the end_link
        JT = np.transpose(J)         # transpose jacobian

        vel_current = J*dq_current  # current end-effector velocity(v, w)
        vc = vel_current[0:3]  # current linear velocity
        wc = vel_current[3:6]  # current angular velocity

        g = np.transpose(np.matrix(kin.gravity(q_current)))  # gravity matrix

        M = kin.inertia(q_current)           # joint space mass matrix at the end link
        
        LAMBDA = inv(J * inv(M) * JT)   # cartesian space mass matrix at the end_link

        Fe = arm.end_effector_force(JT, eff_current)   # end-effector force (Fe = [Fx, Fy, Fz, Tx, Ty, Tz])

        F = np.matrix([[0], [0], [0], [0], [0], [0]])  # Simulated Force Interaction
        # Force Interaction F = [Fx, Fy, Fz, Tx, Tz, Ty]
        # if (count > 1000) and (count < 2000):
        #     F = np.matrix([[0], [0], [-10], [0], [0], [0]])

        # =============================================================================================
        # Trajectory
        # =============================================================================================
        if count > 1500:
            if t <= 1:

                pd = arm.trajectory_line(pi, pf, t)

                nd = arm.trajectory_line(ni, nf, t)
                ed = arm.trajectory_line(ei, ef, t)
                EulerXYZ_d = arm.quaternion2EulerXYZ(nd, ed)

                t = t + 0.001

        # =============================================================================================
        # Spring-Damper gains
        # =============================================================================================
        # Mass(A)
        Ainv = J * inv(M) * JT  # NO INERTIA SHAPING -> Fe terms are canceled (A = LAMBDA(q))

        # Damper(D) position(p) and orientation(o) gains
        Dp = np.matrix([[24, 0, 0],
                        [0, 24, 0],
                        [0, 0, 24]])

        Do = np.matrix([[0.05, 0, 0],
                        [0, 0.05, 0],
                        [0, 0, 0.05]])  # 0.05

        # Spring(K) position(p) and orientation(o) gains
        Kp = np.matrix([[100, 0, 0],
                        [0, 100, 0],
                        [0, 0, 100]])

        Ko = np.matrix([[4, 0, 0],
                        [0, 4, 0],
                        [0, 0, 4]])

        # Integrative(I) position(p) and orientation(o) gains
        Ip = np.matrix([[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]])

        # =============================================================================================
        # Impedance Controller (Task Space)
        # =============================================================================================
        # position
        delta_p = pd - pc           # position error
        error_sum_p += delta_p*dt   # error sum
        delta_v = vd - vc           # linear velocity error
        w_pos = Kp*delta_p + Dp*(delta_v) + Ip*error_sum_p

        # orientation
        delta_o = nc*ed - nd*ec - arm.SkewSymmetric(ed)*ec  # orientation error
        delta_w = wd - wc                                   # angular velocity error
        w_ori = Do*delta_w + Ko*delta_o

        w = ad + Ainv * (np.concatenate((w_pos, w_ori), axis=0) - Fe)   # auxiliary variable -> w = ac (current acceleration)
        Fc = LAMBDA*w   # cartesian force

        # Command Torques (tau)
        tau = (JT * Fc) + g + (JT * Fe) + (JT * F)

        # Publish Torque Commands
        jointCmds = [tau.item(0), tau.item(1), tau.item(2), tau.item(3), tau.item(4), tau.item(5)]
        finger_position = [0, 0, 0]  # Gripper Open
        # finger_position = [1, 1, 1]   # Gripper Close
        arm.publishTorqueCmd(jointCmds, finger_position)


        # =============================================================================================
        # SEND TO FILE
        # =============================================================================================
        TIME = count * 0.010
        # position
        pXc = pc.item(0)
        pXd = pd.item(0)
        pYc = pc.item(1)
        pYd = pd.item(1)
        pZc = pc.item(2)
        pZd = pd.item(2)

        # orientation
        oXd = EulerXYZ_d[0]
        oXc = EulerXYZ_c[0]
        oYd = EulerXYZ_d[1]
        oYc = EulerXYZ_c[1]
        oZd = EulerXYZ_d[2]
        oZc = EulerXYZ_c[2]

        lines = ' ' + str(TIME) + '; ' \
                + str(pXc) + '; ' + str(pXd) + '; ' + str(pYc) + '; ' + str(pYd) + '; ' + str(pZc) + '; ' + str(pZd) + '; ' \
                + str(oXc) + '; ' + str(oXd) + '; ' + str(oYc) + '; ' + str(oYd) + '; ' + str(oZc) + '; ' + str(oZd)
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
