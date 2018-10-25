#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import argparse


def moveJoint (jointcmds,prefix,nbJoints):
    topic_name = '/' + prefix + '/effort_joint_trajectory_controller/command'
    pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
    jointCmd = JointTrajectory()
    point = JointTrajectoryPoint()
    jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    point.time_from_start = rospy.Duration.from_sec(5.0)
    for i in range(0, nbJoints):
        jointCmd.joint_names.append(prefix +'_joint_'+str(i+1))
        point.positions.append(jointcmds[i])
        point.velocities.append(0)
        point.accelerations.append(0)
        point.effort.append(0)
    jointCmd.points.append(point)
    rate = rospy.Rate(100)
    count = 0
    while (count < 500):
        pub.publish(jointCmd)
        count = count + 1
        rate.sleep()

def moveFingers (jointcmds,prefix,nbJoints):
    topic_name = '/' + prefix + '/effort_finger_trajectory_controller/command'
    pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
    jointCmd = JointTrajectory()
    point = JointTrajectoryPoint()
    jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    point.time_from_start = rospy.Duration.from_sec(5.0)
    for i in range(0, nbJoints):
        jointCmd.joint_names.append(prefix +'_joint_finger_'+str(i+1))
        point.positions.append(jointcmds[i])
        point.velocities.append(0)
        point.accelerations.append(0)
        point.effort.append(0)
    jointCmd.points.append(point)
    rate = rospy.Rate(100)
    count = 0
    while (count < 100):
        pub.publish(jointCmd)
        count = count + 1
        rate.sleep()


if __name__ == '__main__':
  try:
    rospy.init_node('command_robot_home_pose')
    prefix = 'j2n6s300'
    nbJoints = 6
    nbfingers = 3
    #allow gazebo to launch
    rospy.sleep(1)

    #home robot
    moveJoint ([4.8046852, 2.92482, 1.002, 4.2031852, 1.4458, 1.3233], prefix, nbJoints)
    moveFingers ([0,0,0],prefix,nbfingers)


  except rospy.ROSInterruptException:
    print "program interrupted before completion"
