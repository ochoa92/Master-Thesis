#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# =====================================================================================================
# Name: calibration_robot.py
# Author: Hélio Ochoa
# Version: 0.0
# Description: A .py file to calibrate the joint torques
# =====================================================================================================

import roslib; roslib.load_manifest('kinova_thesis')
import rospy
import sys
import actionlib

import kinova_msgs.msg
from kinova_msgs.srv import *


def joint_position_client(angle_set, prefix):
    action_address = '/' + prefix + '_driver/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()
    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]

    client.send_goal(goal)

    client.wait_for_result(rospy.Duration(100.0))

    # Prints out the result of executing the action
    return client.get_result()


def homeRobot(prefix):
	service_address = '/' + prefix + '_driver/in/home_arm'
	rospy.wait_for_service(service_address)
        try:
           home = rospy.ServiceProxy(service_address, HomeArm)
           home()
           return None
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e


def ZeroTorque(prefix):
	#move robot to candle like pose
	#result = joint_position_client([180]*6)

	print "torque before setting zero"
	topic_name = '/' + prefix + '_driver/out/joint_torques'
	sub_once = rospy.Subscriber(topic_name, kinova_msgs.msg.JointAngles, printTorqueVaules)
	rospy.wait_for_message(topic_name, kinova_msgs.msg.JointAngles, timeout=2)
	sub_once.unregister()

	#call zero torque
	service_address = '/' + prefix + '_driver/in/set_zero_torques'
	rospy.wait_for_service(service_address)
	try:
		zeroTorques = rospy.ServiceProxy(service_address, ZeroTorques)
		zeroTorques()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		return None

	rospy.sleep(0.5)
	print "torque after setting zero"
	sub_once = rospy.Subscriber(topic_name, kinova_msgs.msg.JointAngles, printTorqueVaules)
	rospy.wait_for_message(topic_name, kinova_msgs.msg.JointAngles, timeout=2)
	sub_once.unregister()


def printTorqueVaules(torques):
	print "Torque: {}, {}, {}, {}, {}, {}".format(torques.joint1,
                                                   torques.joint2,
                                                   torques.joint3,
                                                   torques.joint4,
                                                   torques.joint5,
                                                   torques.joint6)


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


def main():
    rospy.init_node('calibration_robot', anonymous=False)

    # angle_set = [180, 180, 180, 0, 0, 180]  # Reset Position
    angle_set = [180, 180, 180, 180, 180, 180]  # Reset Position
    prefix = 'j2n6s300'

    setTorqueControlMode(prefix, flag=0)  # use service to switch to position control

    joint_position_client(angle_set, prefix)
    ZeroTorque(prefix)
    rospy.sleep(3)

    print("\nHOMING THE ROBOT AFTER CALIBRATION!")
    homeRobot(prefix)
    rospy.sleep(3)

    setTorqueControlParameters(prefix)  # use service to set torque control parameters
    setTorqueControlMode(prefix, flag=1)  # use service to switch to torque control

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")