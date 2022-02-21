#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from iiwa_msgs.msg import CartesianPose, JointPosition
from trac_ik_python.trac_ik import IK
from pykdl_utils.kdl_kinematics import KDLKinematics
import PyKDL
from urdf_parser_py.urdf import URDF
import message_filters
from sensor_msgs.msg import PointCloud2 
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import PointCloud2, do_transform_cloud 
import numpy as np

import ros_numpy
from cloud_filtering.msg import PrincipalComponentStamped
from cloud_filtering.msg import Float64MultiArrayStamped
from std_msgs.msg import String

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.decomposition import PCA

from geometry_msgs.msg import Vector3

def callback(cloud_msg):

    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_msg)
    X, Y, Z = xyz_array.T
    pc_centroid = np.mean(xyz_array, axis=0)

    # Self-defined quantities
    plane_normal = np.array([1, 0, 0]) 
    threshold = 1e-3

    points_to_centroid = xyz_array - pc_centroid
    projection_dist = np.abs(np.dot(points_to_centroid, plane_normal))
    close_idx = np.where(projection_dist < threshold)

    # Run PCA on the points that are intersected by the plane (up to a threshold)
    close_coords = np.array((X[close_idx], Y[close_idx], Z[close_idx])).T
    pca = PCA(n_components=1)
    pca.fit(close_coords)
    direction_vector = pca.components_[0]

    # Publish direction vector
    vector_msg = Vector3()
    vector_msg.x = direction_vector[0]
    vector_msg.y = direction_vector[1]
    vector_msg.z = direction_vector[2]

    intersected_direction_vector_pub.publish( vector_msg)

    # Only move in the negative y direction
    direction = 1
    if direction_vector[1] > 0: 
        direction = -1 
    
    direction_vector = direction_vector * direction

    print("Direction vector: {}".format(direction_vector))

    # # Plot
    # # Define a line from the direction vector
    # close_centroid = np.mean(close_coords, axis=0)
    # euclidian_distance = np.linalg.norm(close_coords - close_centroid, axis=1)
    # extent = np.max(euclidian_distance)

    # line = np.vstack((close_centroid - direction_vector * extent,
    #                 close_centroid + direction_vector * extent))

    # plt.figure()
    # ax = plt.axes(projection='3d')
    # ax.scatter(X, Y, Z, alpha=0.1)
    # ax.scatter(X[close_idx], Y[close_idx], Z[close_idx], color='red')

    # ax.plot(line[:, 0], line[:, 1], line[:, 2], 'r')

    # ax.set_xlabel('x')
    # ax.set_ylabel('y')

    # # Set equal aspect ratios
    # max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0

    # mid_x = (X.max()+X.min()) * 0.5
    # mid_y = (Y.max()+Y.min()) * 0.5
    # mid_z = (Z.max()+Z.min()) * 0.5
    # ax.set_xlim(mid_x - max_range, mid_x + max_range)
    # ax.set_ylim(mid_y - max_range, mid_y + max_range)
    # ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # plt.show()

    # rospy.signal_shutdown("My own reason")

if __name__ == '__main__':
    world_frame_point_cloud_pub = rospy.Publisher('/world_frame_point_cloud', PointCloud2, queue_size = 1)
    rospy.init_node('plane_intersection_node', anonymous=True)

    global intersected_pc_pub
    intersected_pc_pub = rospy.Publisher('/intersected_pc', PointCloud2, queue_size = 1)

    global intersected_direction_vector_pub
    intersected_direction_vector_pub = rospy.Publisher('/intersected_direction_vector', Vector3, queue_size = 1)

    pc_msg = rospy.Subscriber("/deformed_cloud", PointCloud2, callback)
    rospy.spin()
