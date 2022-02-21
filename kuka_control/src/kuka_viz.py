#!/usr/bin/env python

from math import sin, cos, pi
import rospy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
import std_msgs.msg
from tf2_ros import TransformBroadcaster, TransformStamped
from iiwa_msgs.msg import CartesianPose, JointPosition
import random
def callback(curr_joints_msg):
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
    joint_state.position = [curr_joints_msg.position.a1, curr_joints_msg.position.a2, curr_joints_msg.position.a3, curr_joints_msg.position.a4, \
    curr_joints_msg.position.a5, curr_joints_msg.position.a6, curr_joints_msg.position.a7]
    joint_pub.publish(joint_state)

def listener():
    rospy.init_node('kuka_joint_state_publisher')

    global joint_pub
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    rospy.Subscriber('iiwa/state/JointPosition', JointPosition, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

