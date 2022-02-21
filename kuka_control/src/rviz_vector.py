#!/usr/bin/env python

from math import sin, cos, pi
import rospy
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3Stamped, PointStamped, Point

import message_filters
from cloud_filtering.msg import PrincipalComponentStamped

from sensor_msgs.msg import JointState
import std_msgs.msg
from tf2_ros import TransformBroadcaster, TransformStamped
from iiwa_msgs.msg import CartesianPose, JointPosition
import random
from visualization_msgs.msg import Marker
def callback(directional_pc, centroid):
    marker = Marker()
    marker.type = marker.ARROW
    marker.header = centroid.header
    marker.header.frame_id = 'base_link'
    arrow_start = Point()
    arrow_start.x, arrow_start.y, arrow_start.z = centroid.point.x, centroid.point.y, centroid.point.z
    arrow_end = Point()
    marker.scale.x, marker.scale.y, marker.scale.z = 0.05, 0.05, 0.1
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.5, 1.0

    direction = directional_pc.principal_component
    arrow_end.x, arrow_end.y, arrow_end.z = centroid.point.x + direction[0], centroid.point.y + direction[1], \
        centroid.point.z + direction[2]

    marker.points = [arrow_start, arrow_end]
    marker_pub.publish(marker)
    print(directional_pc)



def listener():
    rospy.init_node('rviz_vector')

    global marker_pub
    marker_pub = rospy.Publisher('control_vector', Marker, queue_size=10)
    print("Initialized")
    world_frame_first_pc_sub = message_filters.Subscriber("/world_frame_first_pc", PrincipalComponentStamped)
    world_frame_centroid_sub = message_filters.Subscriber("/world_frame_centroid", PointStamped)
    ts = message_filters.ApproximateTimeSynchronizer([world_frame_first_pc_sub, world_frame_centroid_sub], 10, 1)
    ts.registerCallback(callback)

    rospy.spin()


if __name__ == '__main__':
    listener()

