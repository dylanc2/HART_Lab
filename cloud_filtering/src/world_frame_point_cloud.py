#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Vector3Stamped, PointStamped
from iiwa_msgs.msg import CartesianPose, JointPosition
from trac_ik_python.trac_ik import IK
from pykdl_utils.kdl_kinematics import KDLKinematics
import PyKDL
from urdf_parser_py.urdf import URDF
import message_filters
from sensor_msgs.msg import PointCloud2
import tf
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import PointCloud2, do_transform_cloud 
import tf2_geometry_msgs

import numpy as np

import ros_numpy

from cloud_filtering.msg import PrincipalComponentStamped
from cloud_filtering.msg import Float64MultiArrayStamped
from std_msgs.msg import String

def do_transform_centroid(centroid_in, trans):
    centroid_in_point = PointStamped()
    centroid_in_point.point.x, centroid_in_point.point.y, centroid_in_point.point.z = centroid_in.vector.x, centroid_in.vector.y, centroid_in.vector.z
    centroid_out_point = tf2_geometry_msgs.do_transform_point(centroid_in_point, trans)
    centroid_out_point.header = centroid_in.header
    return centroid_out_point


def do_transform_principal_component(first_pc_in, trans):
    # Perform transformation of principal component in Vector3 form first
    first_pc_in_vec = Vector3Stamped()
    first_pc_in_vec.vector.x, first_pc_in_vec.vector.y, first_pc_in_vec.vector.z = first_pc_in.principal_component
    first_pc_out_vec = tf2_geometry_msgs.do_transform_vector3(first_pc_in_vec, trans)

    first_pc_out = PrincipalComponentStamped()
    first_pc_out.header = first_pc_in.header
    first_pc_out.principal_component = [first_pc_out_vec.vector.x, first_pc_out_vec.vector.y, first_pc_out_vec.vector.z]
    first_pc_out.eigenvalue = first_pc_in.eigenvalue
    return first_pc_out

def callback(cloud_in, first_pc_in, centroid_in):
    tf_listener.waitForTransform('base_link', 'royale_camera_optical_frame', rospy.Time(), rospy.Duration(5.0))
    trans_tuple = tf_listener.lookupTransform('base_link', 'royale_camera_optical_frame', rospy.Time(0))
    trans = TransformStamped()
    trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z = trans_tuple[0]
    trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w = trans_tuple[1]

    # Perform transformations
    cloud_out = do_transform_cloud(cloud_in, trans)
    first_pc_out = do_transform_principal_component(first_pc_in, trans)
    centroid_out_point = do_transform_centroid(centroid_in, trans)


    world_frame_point_cloud_pub.publish(cloud_in)
    world_frame_first_pc_pub.publish(first_pc_out)
    world_frame_centroid_pub.publish(centroid_out_point)

def listener():
    rospy.init_node('world_frame_point_cloud', anonymous=True)

    global world_frame_point_cloud_pub
    world_frame_point_cloud_pub = rospy.Publisher('/world_frame_point_cloud', PointCloud2, queue_size = 5)

    global world_frame_first_pc_pub 
    world_frame_first_pc_pub = rospy.Publisher('/world_frame_first_pc', PrincipalComponentStamped, queue_size=5)

    global world_frame_centroid_pub
    world_frame_centroid_pub = rospy.Publisher('/world_frame_centroid', PointStamped, queue_size=5)

    global tf_listener
    tf_listener = tf.TransformListener()


    # Subscribe to all the quantities that have to be transformed to the world frame
    deformed_membrane_sub = message_filters.Subscriber('/deformed_cloud', PointCloud2)
    first_pc_sub = message_filters.Subscriber("/first_pc", PrincipalComponentStamped)
    centroid_sub = message_filters.Subscriber("/centroid", Vector3Stamped)

    ts = message_filters.ApproximateTimeSynchronizer([deformed_membrane_sub, first_pc_sub, centroid_sub], 10, 0.3)
    ts.registerCallback(callback)

    rospy.spin()



if __name__ == '__main__':
    listener()
