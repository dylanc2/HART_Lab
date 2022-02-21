#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from iiwa_msgs.msg import CartesianPose, JointPosition
from geometry_msgs.msg import Vector3Stamped
from trac_ik_python.trac_ik import IK
from pykdl_utils.kdl_kinematics import KDLKinematics
from sensor_msgs.msg import PointCloud2
import ros_numpy
import PyKDL
from urdf_parser_py.urdf import URDF
import tf
import numpy as np
import math

from cloud_filtering.msg import PrincipalComponentStamped

filtered_xyz_array = None

def distance_from_centroid_to_undeformed_cloud(centroid, xyz_cloud):
	centroid_np = np.array([centroid.vector.x, centroid.vector.y, centroid.vector.z])
	dists = np.linalg.norm(xyz_cloud - centroid_np, axis=1)
	return np.min(dists)

def save_point_cloud(pc):
	pass
	# Temporary, to just save point cloud
	'''
	filtered_msg = rospy.wait_for_message('filtered_membrane', PointCloud2)
	deformed_msg = rospy.wait_for_message('deformed_cloud', PointCloud2)
	filtered_xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(filtered_msg)
	deformed_xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(deformed_msg)
	np.save('filtered_cloud_plane.npy', filtered_xyz) # save
	np.save('deformed_cloud_plane.npy', deformed_xyz) # save
	# b = np.load('data.npy') # load
	quit()
	'''

if __name__ == '__main__':
	rospy.init_node('track_edge', anonymous=True)

	global filtered_xyz_array

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

	NUM_MSGS_WAIT = 1
	while not rospy.is_shutdown():
		print("Planning next step")

		for i in range(NUM_MSGS_WAIT):
			pc_msg = rospy.wait_for_message('/world_frame_first_pc', PrincipalComponentStamped)
			evals_msg = rospy.wait_for_message('pca_evals', Vector3Stamped)

		pc = np.array(pc_msg.principal_component)
		ev = pc_msg.eigenvalue

		contact = ev > 1e-5
		
		# If ev is absolutely zero, then save undeformed pointcloud
		if ev == 0.0 and np.any(filtered_xyz_array == None):
			print("Recording undeformed point cloud")
			filtered_membrane_msg = rospy.wait_for_message('/filtered_membrane', PointCloud2)
			filtered_xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(filtered_membrane_msg)


		for i in range(NUM_MSGS_WAIT):
			curr_joints_msg = rospy.wait_for_message('/iiwa/state/JointPosition', JointPosition)
		curr_joints = [curr_joints_msg.position.a1, curr_joints_msg.position.a2, curr_joints_msg.position.a3, curr_joints_msg.position.a4,
			curr_joints_msg.position.a5, curr_joints_msg.position.a6, curr_joints_msg.position.a7]
		fk_pose = kdl_kin.forward(curr_joints) # Cartesian pose at tip of tactile device

		fk_pos = np.squeeze(np.asarray(fk_pose[:3, 3]))
		fk_quat = tf.transformations.quaternion_from_matrix(fk_pose)

		print("in contact", contact, ev)
		if not contact:

			# Move arm down
			new_joints = ik_solver.get_ik(curr_joints,
		                fk_pos[0], fk_pos[1], fk_pos[2] - 0.01,  # X, Y, Z
		                fk_quat[0], fk_quat[1], 
		                fk_quat[2], fk_quat[3])  # QX, QY, QZ, QW

			new_joints_msg.position.a1 = new_joints[0]
			new_joints_msg.position.a2 = new_joints[1]
			new_joints_msg.position.a3 = new_joints[2]
			new_joints_msg.position.a4 = new_joints[3]
			new_joints_msg.position.a5 = new_joints[4]
			new_joints_msg.position.a6 = new_joints[5]
			new_joints_msg.position.a7 = new_joints[6]


		elif contact: 

			direction = 1
			if pc[0] > 0: # Only move in the negative x direction
				direction = -1 
			
			pc_scaled = [e * 0.01 * direction for e in pc]


			v = fk_pose * np.array([0, 0, 1, 0]).reshape(4, 1)
			v = np.asarray(v.reshape(1, 4))[0][:3] # Change v to non-homogeneous coordinates

			u = v - pc * np.dot(v, pc) / np.linalg.norm(pc)
			w = np.cross(v, u)
			theta = math.acos(np.dot(v, u) / (np.linalg.norm(v) * np.linalg.norm(u)))

			# Limit the amount of rotation each time
			theta_constant = 0.5
			r = tf.transformations.rotation_matrix(theta_constant *theta, w)

			new_transformation = r * fk_pose
			new_quat = tf.transformations.quaternion_from_matrix(new_transformation)

			# Get distance from centroid to undeformed cloud
			centroid_msg = rospy.wait_for_message('/centroid', Vector3Stamped)
			centroid_dist = distance_from_centroid_to_undeformed_cloud(centroid_msg, filtered_xyz_array)
			depth_error = centroid_dist - 0.01

			if centroid_msg.vector.x == 0 and centroid_msg.vector.y == 0 and centroid_msg.vector.z == 0:
				depth_error = 0
			print("Depth error", depth_error)
			print(centroid_dist)

			# To modulate amount of pressure
			u_norm = u / np.linalg.norm(u)
			u_norm_scaled = [e * -depth_error for e in u_norm]
			assert(not np.all(filtered_xyz_array == None))
			centroid_msg = rospy.wait_for_message('centroid', Vector3Stamped)



			# u_norm points INTO the surface along surface normal

			# The works
			new_pose = np.array([fk_pos[0] + pc_scaled[0] + u_norm_scaled[0], \
				fk_pos[1] + pc_scaled[1] + u_norm_scaled[1], \
				fk_pos[2] + pc_scaled[2] + u_norm_scaled[2], \
				new_quat[0], \
				new_quat[1], \
				new_quat[2], \
				new_quat[3]])

			# Only depth correction
			# new_pose = np.array([fk_pos[0] + u_norm_scaled[0], \
			# 	fk_pos[1] + u_norm_scaled[1], \
			# 	fk_pos[2] + u_norm_scaled[2], \
			# 	fk_quat[0], \
			# 	fk_quat[1], \
			# 	fk_quat[2], \
			# 	fk_quat[3]])


			# Move to the next position with rotation
			# new_pose = np.array([fk_pos[0] + pc_scaled[0], \
			# 	fk_pos[1] + pc_scaled[1], \
			# 	fk_pos[2] + pc_scaled[2], \
			# 	new_quat[0], \
			# 	new_quat[1], \
			# 	new_quat[2], \
			# 	new_quat[3]])

			# Just do rotations
			# new_pose = np.array([fk_pos[0], \
			# 	fk_pos[1], \
			# 	fk_pos[2], \
			# 	new_quat[0], \
			# 	new_quat[1], \
			# 	new_quat[2], \
			# 	new_quat[3]])

			# Don't move at all
			# new_pose = np.array([fk_pos[0], \
			# 	fk_pos[1], \
			# 	fk_pos[2], \
			# 	fk_quat[0], \
			# 	fk_quat[1], \
			# 	fk_quat[2], \
			# 	fk_quat[3]])


			new_joints = ik_solver.get_ik(curr_joints,
							new_pose[0], new_pose[1], new_pose[2],  # X, Y, Z
							new_pose[3], new_pose[4], 
							new_pose[5], new_pose[6])  # QX, QY, QZ, QW

			print(new_joints)
		
		


		joint_difference = [np.abs(new_joints[i] - curr_joints[i]) for i in range(len(curr_joints))]
		print "Max joint change", max(joint_difference)
		if max(joint_difference) > 1:
			print("Max joint change too large, quitting")
			break
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
		print("Publishing new joints")
		for i in range(10):
		   joint_pub.publish(new_joints_msg)
		   r.sleep()

