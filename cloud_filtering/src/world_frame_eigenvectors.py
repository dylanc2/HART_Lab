#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from iiwa_msgs.msg import CartesianPose, JointPosition
from trac_ik_python.trac_ik import IK
from pykdl_utils.kdl_kinematics import KDLKinematics
import PyKDL
from urdf_parser_py.urdf import URDF
import tf
import message_filters
import numpy as np

from cloud_filtering.msg import PrincipalComponentStamped
from cloud_filtering.msg import Float64MultiArrayStamped
from std_msgs.msg import String


def callback(first_pc_msg, joints_msg):
	# print("pc: ")
	# print(first_pc_msg)
	# print("joints: ")
	# print(joints_msg)

	curr_joints = [joints_msg.position.a1, joints_msg.position.a2, joints_msg.position.a3, joints_msg.position.a4,
		joints_msg.position.a5, joints_msg.position.a6, joints_msg.position.a7]

	fk_pose = kdl_kin.forward(curr_joints) # Cartesian pose at tip of tactile device, 4x4 homogenous matrix

	pc = np.asarray(first_pc_msg.principal_component)
	pc_homogeneous = np.pad(pc, (0, 1), 'constant').reshape((4, 1))

	world_frame_pc = fk_pose * pc_homogeneous

	world_frame_pc_msg = PrincipalComponentStamped()
	world_frame_pc_msg.principal_component[0] = world_frame_pc[0]
	world_frame_pc_msg.principal_component[1] = world_frame_pc[1]
	world_frame_pc_msg.principal_component[2] = world_frame_pc[2]

	world_frame_pc_msg.eigenvalue = first_pc_msg.eigenvalue
	world_frame_pc_msg.header.stamp = rospy.get_rostime()

	world_frame_pc_pub.publish(world_frame_pc_msg)

def listener():
	print("Running")
	rospy.init_node('world_frame_eigenvectors', anonymous=True)

	# Set up IK and FK solver
	urdf_string = open("/home/isabella/dylan_ws/src/iiwa14.urdf", 'r').read()
	urdf_str = "/home/isabella/dylan_ws/src/iiwa14.urdf"
	rospy.set_param('robot_description', urdf_str)
	robot_urdf = URDF.from_xml_string(urdf_string)

	global kdl_kin
	kdl_kin = KDLKinematics(robot_urdf, "iiwa_link_0", "tool_link_ee")
	ik_solver = IK("iiwa_link_0",
	               "tool_link_ee",
	               urdf_string=urdf_string)

	lower_bound, upper_bound = ik_solver.get_joint_limits()
	# Set up publisher for eigenvectors in world frame
	global world_frame_pc_pub
	world_frame_pc_pub = rospy.Publisher('/world_frame_pc', PrincipalComponentStamped, queue_size = 1)

	first_pc_sub = message_filters.Subscriber("/first_pc", PrincipalComponentStamped)
	joints_sub = message_filters.Subscriber("/iiwa/state/JointPosition", JointPosition)
	ts = message_filters.ApproximateTimeSynchronizer([first_pc_sub, joints_sub], 10, 1)
  	ts.registerCallback(callback)

	rospy.spin()

if __name__ == '__main__':
	listener()
