#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from iiwa_msgs.msg import CartesianPose, JointPosition
from trac_ik_python.trac_ik import IK
from pykdl_utils.kdl_kinematics import KDLKinematics
import PyKDL
from urdf_parser_py.urdf import URDF
import tf
import numpy as np



if __name__ == '__main__':
	rospy.init_node('track_edge', anonymous=True)

	# Set up IK and FK solver
	urdf_string = open("/home/isabella/dylan_ws/src/iiwa14.urdf", 'r').read()
	urdf_str = "/home/isabella/dylan_ws/src/iiwa14.urdf"
	rospy.set_param('robot_description', urdf_str)
	robot_urdf = URDF.from_xml_string(urdf_string)
	kdl_kin = KDLKinematics(robot_urdf, "iiwa_link_0", "tool_link_ee")
	ik_solver = IK("iiwa_link_0",
	               "tool_link_ee",
	               urdf_string=urdf_string)

	lower_bound, upper_bound = ik_solver.get_joint_limits()

	# Set up publisher for updated joints
	new_joints_msg = JointPosition()
	joint_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)



	while not rospy.is_shutdown():
		curr_joints_msg = rospy.wait_for_message('/iiwa/state/JointPosition', JointPosition)
		curr_joints = [curr_joints_msg.position.a1, curr_joints_msg.position.a2, curr_joints_msg.position.a3, curr_joints_msg.position.a4,
			curr_joints_msg.position.a5, curr_joints_msg.position.a6, curr_joints_msg.position.a7]
		print(curr_joints)
		joints = ik_solver.get_ik(curr_joints,
                0.433, 0.057, 0.63,  # X, Y, Z
                -0.42, 0.89, -0.018, 0.166)  # QX, QY, QZ, QW
		print(joints)

		fk_pose = kdl_kin.forward(curr_joints) # Cartesian pose at tip of tactile device
		fk_pos = np.squeeze(np.asarray(fk_pose[:3, 3]))
		fk_quat = tf.transformations.quaternion_from_matrix(fk_pose)

		# Move to the next position
		new_pose = np.array([fk_pos[0], \
			fk_pos[1], \
			fk_pos[2] + 0.01, \
			fk_quat[0], \
			fk_quat[1], \
			fk_quat[2], \
			fk_quat[3]])

		new_joints = ik_solver.get_ik(curr_joints,
		                new_pose[0], new_pose[1], new_pose[2],  # X, Y, Z
		                new_pose[3], new_pose[4], 
		                new_pose[5], new_pose[6])  # QX, QY, QZ, QW

		joint_difference = [np.abs(new_joints[i] - curr_joints[i]) for i in range(len(joints))]
		print "Max joint change", max(joint_difference)
		keypress = raw_input("Press any key to continue, q to quit\n")
		if keypress is "q":
			break


		new_joints_msg.position.a1 = new_joints[0]
		new_joints_msg.position.a2 = new_joints[1]
		new_joints_msg.position.a3 = new_joints[2]
		new_joints_msg.position.a4 = new_joints[3]
		new_joints_msg.position.a5 = new_joints[4]
		new_joints_msg.position.a6 = new_joints[5]
		new_joints_msg.position.a7 = new_joints[6]

		r = rospy.Rate(5) # 10hz
		for i in range(1):
		   joint_pub.publish(new_joints_msg)
		   r.sleep()

