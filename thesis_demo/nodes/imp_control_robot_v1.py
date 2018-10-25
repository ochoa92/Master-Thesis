#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# =====================================================================================================
# Name: imp_control_robot_v1.py
# Author: Hélio Ochoa
# Version: 1.0
# Description: Impedance control in the task space with force sensing.
# =====================================================================================================

import roslib; roslib.load_manifest('thesis_demo')
import rospy
import sys
import thread

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
        return np.where(continuous, np.mod(q, 2*np.pi), q)

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
    # Returns the angle and axis (theta, v)
    # @param rotation matrix R
    def angle_axis_from_rotation(self, R, threshold):
        """
        R = [[R(1,1) R(1,2) R(1,3)]
             [R(2,1) R(2,2) R(2,3)]
             [R(3,1) R(3,2) R(3,3)]]
        """
        theta = np.arccos(0.5 * (R.item(0) + R.item(4) + R.item(8) - 1))

        if (theta >= -threshold) and (theta <= threshold):
            v = np.matrix([[0],
                           [0],
                           [0]])
        elif (theta >= (np.pi - threshold)) and (theta <= (np.pi + threshold)):
            v = np.matrix([[0],
                           [0],
                           [0]])
        else:
            v1 = (R.item(7) - R.item(5))/(2 * np.sin(theta))
            v2 = (R.item(2) - R.item(6))/(2 * np.sin(theta))
            v3 = (R.item(3) - R.item(1))/(2 * np.sin(theta))
            v = np.matrix([[v1],
                           [v2],
                           [v3]])

        return theta, v

    ##
    # Returns a rotation matrix(R)
    # @param the angle and the axis (theta, v)
    def rotation_from_angle_axix(self, theta, v):
        """
        theta
        v = [vx, vy, vz]

        """
        vx = v.item(0)
        vy = v.item(1)
        vz = v.item(2)

        from numpy import cos as c
        from numpy import sin as s

        R = np.matrix([[(vx**2)*(1-c(theta))+c(theta), vx*vy*(1-c(theta))-vz*s(theta), vx*vz*(1-c(theta))+vy*s(theta)],
                       [vx*vy*(1-c(theta))+vz*s(theta), (vy**2)*(1-c(theta))+c(theta), vy*vz*(1-c(theta))-vx*s(theta)],
                       [vx*vz*(1-c(theta))-vy*s(theta), vy*vz*(1-c(theta))+vx*s(theta), (vz**2)*(1-c(theta))+c(theta)]])

        return R

    ##
    # Returns the rotation vector r = theta*v
    # @param R - the rotation matrix
    def R2r(self, R, threshold=0.05):
        """
        R = [[R(1,1) R(1,2) R(1,3)]
             [R(2,1) R(2,2) R(2,3)]
             [R(3,1) R(3,2) R(3,3)]]
        """
        theta = np.arccos(0.5 * (R.item(0) + R.item(4) + R.item(8) - 1))

        if (theta >= -threshold) and (theta <= threshold):
            v = np.matrix([[0],
                           [0],
                           [0]])
        elif (theta >= (np.pi - threshold)) and (theta <= (np.pi + threshold)):
            v = np.matrix([[0],
                           [0],
                           [0]])
        else:
            v1 = (R.item(7) - R.item(5))/(2 * np.sin(theta))
            v2 = (R.item(2) - R.item(6))/(2 * np.sin(theta))
            v3 = (R.item(3) - R.item(1))/(2 * np.sin(theta))
            v = np.matrix([[v1],
                           [v2],
                           [v3]])

        r = theta*v

        return r

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
    rospy.init_node('Impedance_Control')

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
    path = '/home/ochoa/kst_thesis/imp_control_robot_v1.txt'
    file = open(path, "w")
    line1 = [' t; ',
             'pXc; ', 'pXd; ', 'pYc; ', 'pYd; ', 'pZc; ', 'pZd;',
             'oXc; ', 'oXd; ', 'oYc; ', 'oYd; ', 'oZc; ', 'oZd; ',
             'vX; ', 'vXd; ', 'vY; ', 'vYd; ', 'vZ; ', 'vZd; ',
             'wX; ', 'wXd; ', 'wY; ', 'wYd; ', 'wZ; ', 'wZd; ',
             'Fx; ', 'Fy; ', 'Fz', '\n']
    line2 = [' s; ',
             'm; ', 'm; ', 'm; ', 'm; ', 'm; ', 'm; ',
             'rad; ', 'rad; ', 'rad; ', 'rad; ', 'rad; ', 'rad; ',
             'm/s; ', 'm/s; ', 'm/s; ', 'm/s; ', 'm/s; ', 'm/s; ',
             'rad/s; ', 'rad/s; ', 'rad/s; ', 'rad/s; ', 'rad/s; ', 'rad/s; ',
             'N; ', 'N; ', 'N', '\n']
    file.writelines(line1)
    file.writelines(line2)

    # =================================================================================================
    # Initial Conditions
    # =================================================================================================
    home = np.matrix([[4.8046852], [2.92482], [1.002], [4.2031852], [1.4458], [1.3233]])
    q_desired = home  # HOME(rad)
    # Position Desired(pos_d) and Rotation Desired(Rd)
    pos_d, Rd = kin.FK(q_desired)
    ori_d = arm.R2r(Rd, threshold=0.05)

    # Velocity Desired(velocity_d)
    velocity_d = np.matrix([[0], [0], [0], [0], [0], [0]])
    linearV_d = np.matrix([[velocity_d.item(0)], [velocity_d.item(1)], [velocity_d.item(2)]])  # desired linear velocity
    angularW_d = np.matrix([[velocity_d.item(3)], [velocity_d.item(4)], [velocity_d.item(5)]])  # desired angular velocity

    # Acceleration Desires(acceleration_d)
    acceleration_d = np.matrix([[0], [0], [0], [0], [0], [0]])

    dt = 0.01  # integration interval
    error_sum_p = 0  # position error sum

    # low-pass filter
    q_filtered = np.matrix([[0], [0], [0], [0], [0], [0]])
    dq_filtered = np.matrix([[0], [0], [0], [0], [0], [0]])
    eff_filtered = np.matrix([[0], [0], [0], [0], [0], [0]])
    alpha = 100    # cut-off frequency(Hz)
    h = 0.01    # sample time(ms)

    # =================================================================================================
    # Initial Trajectory Conditions
    # =================================================================================================
    # spline (polynomial3)
    t = 0
    ti = 0
    tf = 10
    pos_i = pos_d
    pos_f = pos_i + np.matrix([[0.1], [-0.1], [0.1]])

    # line
    t1 = 0
    Ri = Rd
    ori_i = ori_d
    Rz = arm.rotation_from_angle_axix(-np.pi/2, np.matrix([0, 0, 1]))
    Rf = Ri * Rz
    ori_f = arm.R2r(Rf, threshold=0.05)

    # ellipse:
    t2 = 0
    tf2 = 30
    a = 0.1
    b = 0.1
    T = 10
    wr = (2*np.pi)/T

    pos_0 = np.matrix([[0], [0], [0]])  # initial point of the ellipse
    vel_i = np.matrix([[0], [0], [0]])  # initial velocity of the ellipse



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

        q_current = np.transpose(arm.get_joint_positions(wrapped=False))   # Current joint positions (rad)
        dq_current = np.transpose(arm.get_joint_velocities())             # Current joint velocities (rad/s)
        eff_current = np.transpose(arm.get_joint_efforts())  # Current joint efforts (N/m)

        q_filtered = (alpha*h*q_current + q_filtered) / (1 + alpha*h)
        dq_filtered = (alpha*h*dq_current + dq_filtered) / (1 + alpha*h)
        eff_filtered = (alpha*h*eff_current + eff_filtered) / (1 + alpha * h)

        # Current Position(pos_c) and current Rotation(Rc)
        pos_c, Rc = kin.FK(q_filtered)
        ori_c = arm.R2r(Rc, threshold=0.05)

        J = kin.jacobian(q_filtered) # jacobian matrix at the end_link
        JT = np.transpose(J)        # transpose jacobian

        velocity_c = np.dot(J, dq_filtered)     # current end-effector velocity(v, w)
        linearV_c = np.matrix([[velocity_c.item(0)], [velocity_c.item(1)], [velocity_c.item(2)]])   # current linear velocity
        angularW_c = np.matrix([[velocity_c.item(3)], [velocity_c.item(4)], [velocity_c.item(5)]])  # current angular velocity

        # g = np.transpose(np.matrix(kin.gravity(q_current)))  # gravity matrix

        M = kin.inertia(q_filtered)  # joint space mass matrix at the end_link

        LAMBDA = inv(J*inv(M)*JT)  # cartesian space mass matrix at the end_link

        Fe = arm.end_effector_force(JT, eff_filtered)  # end-effector force (Fe = [Fx, Fy, Fz, Tx, Ty, Tz])

        # =============================================================================================
        # Trajectory
        # =============================================================================================
        if count > 1000:
            # ellipse
            if t <= tf:
                pos_d = arm.polynomial3(pos_i, pos_f, ti, tf, t)
                pos_0 = pos_d
                t = t + 0.01


            if t1 <= 1:
                Rd = arm.trajectory_line(Ri, Rf, t1)
                ori_d = arm.trajectory_line(ori_i, ori_f, t1)
                t1 = t1 + 0.005


        if count > 2500:
            # ellipse
            linearV_d = np.matrix([[0], [0], [0]])
            acceleration_d = np.matrix([[0], [0], [0], [0], [0], [0]])
            if t2 <= tf2:
                pos_d = pos_0 + np.matrix([[0],
                                           [a*np.cos(wr*t2) - a],
                                           [b*np.sin(wr*t2)]])

                linearV_d = np.add(vel_i, np.matrix([[0],
                                                     [-wr*a*np.sin(wr*t2)],
                                                     [wr*b*np.cos(wr*t2)]]))

                acceleration_d = np.matrix([[0],
                                            [-(wr**2)*a*np.cos(wr*t2)],
                                            [-(wr**2)*b*np.sin(wr*t2)],
                                            [0],
                                            [0],
                                            [0]])

                t2 = t2 + 0.01



        # =============================================================================================
        # Mass-Spring-Damper gains
        # =============================================================================================
        # Mass(A)
        Ainv = J*inv(M)*JT  # NO INERTIA SHAPING -> Fe terms are canceled (A = LAMBDA(q))

        # Damper(D) position(p) and orientation(o) gains
        Dp = np.matrix([[25, 0, 0],
                        [0, 25, 0],
                        [0, 0, 25]])  # 25

        Do = np.matrix([[0.5, 0, 0],
                        [0, 0.5, 0],
                        [0, 0, 0.5]])  # 0.1

        # Spring(K) position(p) and orientation(o) gains
        Kp = np.matrix([[150, 0, 0],
                        [0, 150, 0],
                        [0, 0, 150]])  # 150

        Ko = np.matrix([[5, 0, 0],
                        [0, 5, 0],
                        [0, 0, 5]])  # 10

        # Integrative(I) position(p) and orientation(o) gains
        Ip = np.matrix([[15, 0, 0],
                        [0, 15, 0],
                        [0, 0, 15]])  # 10

        # =============================================================================================
        # Impedance Controller (Task Space)
        # =============================================================================================
        # Position
        delta_pos = pos_d - pos_c   # position error
        error_sum_p += delta_pos * dt
        w_pos = Dp*(linearV_d - linearV_c) + Kp*delta_pos  + Ip*error_sum_p

        # Orientation
        cRcd = inv(Rc)*Rd
        Rcd = Rc*cRcd*inv(Rc)   # Rotation error
        # Rcd = Rd*np.transpose(Rc)   # Rotation error
        delta_ori = arm.R2r(Rcd, threshold=0.05)
        w_ori = Do*(angularW_d - angularW_c) + Ko*delta_ori

        w = acceleration_d + Ainv*(np.concatenate((w_pos, w_ori), axis=0))
        Fc = LAMBDA*w

        # Command Torques (tau)
        tau = JT*Fc # with gravity(g) matrix of DSP

        # Publish Torque Commands
        jointCmds = [tau.item(0), tau.item(1), tau.item(2), tau.item(3), tau.item(4), tau.item(5)]
        arm.publishTorqueCmd(jointCmds)


        # =============================================================================================
        # SEND TO FILE
        # =============================================================================================
        TIME = count * 0.010
        # position
        pXc = pos_c.item(0)
        pXd = pos_d.item(0)
        pYc = pos_c.item(1)
        pYd = pos_d.item(1)
        pZc = pos_c.item(2)
        pZd = pos_d.item(2)

        # orientation
        oXd = ori_d.item(0)
        oXc = ori_c.item(0)
        oYd = ori_d.item(1)
        oYc = ori_c.item(1)
        oZd = ori_d.item(2)
        oZc = ori_c.item(2)

        # Linear velocity
        vX = linearV_c.item(0)
        vXd = linearV_d.item(0)
        vY = linearV_c.item(1)
        vYd = linearV_d.item(1)
        vZ = linearV_c.item(2)
        vZd = linearV_d.item(2)

        # Angular velocity
        wX = angularW_c.item(0)
        wXd = angularW_d.item(0)
        wY = angularW_c.item(1)
        wYd = angularW_d.item(1)
        wZ = angularW_c.item(2)
        wZd = angularW_d.item(2)


        # End-Effector force
        Fx = Fe.item(0)
        Fy = Fe.item(1)
        Fz = Fe.item(2)


        lines = ' ' + str(TIME) + '; ' \
                + str(pXc) + '; ' + str(pXd) + '; ' + str(pYc) + '; ' + str(pYd) + '; ' + str(pZc) + '; ' + str(pZd) + '; ' \
                + str(oXc) + '; ' + str(oXd) + '; ' + str(oYc) + '; ' + str(oYd) + '; ' + str(oZc) + '; ' + str(oZd) + '; ' \
                + str(vX) + '; ' + str(vXd) + '; ' + str(vY) + '; ' + str(vYd) + '; ' + str(vZ) + '; ' + str(vZd) + '; ' \
                + str(wX) + '; ' + str(wXd) + '; ' + str(wY) + '; ' + str(wYd) + '; ' + str(wZ) + '; ' + str(wZd) + '; ' \
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
    print '\nPosition Error:', delta_pos
    print '\nOrientation Error:', delta_ori
    print '\nTau:', jointCmds
    rospy.loginfo("I will publish to the topic %d", count)
    print '\n ------------------------------------------------------------------------------------------------------------------------------------'


    setTorqueControlMode(prefix, flag=0)  # use service to switch to position control
    homeRobot(prefix)  # use service to send robot to home position


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
