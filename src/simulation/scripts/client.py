#!/usr/bin/env python

"""
ROS Bridge for carla

Using python2 for now (default setup for kinetic and lunar)
Project created by George Laurent

self-driving vehicle research platform
(c) Neil Nie, 2018. All Rights Reserved
Contact: contact@neilnie.com

# TODO: group tf in same message ?

"""

import rospy
import tf

from carla.client import make_carla_client
from carla.sensor import Transform as carla_Transform

from geometry_msgs.msg import TransformStamped, Transform, Pose
from visualization_msgs.msg import MarkerArray, Marker

from carla_ros_bridge import CarlaROSBridge
from carla_ros_bridge import CarlaROSBridgeWithBag

def carla_transform_to_ros_transform(carla_transform):
    transform_matrix = carla_transform.matrix

    x, y, z = tf.transformations.translation_from_matrix(transform_matrix)
    quat = tf.transformations.quaternion_from_matrix(transform_matrix)


    ros_transform = Transform()
    # remember that we go from left-handed system (unreal) to right-handed system (ros)
    ros_transform.translation.x = x
    ros_transform.translation.y = -y
    ros_transform.translation.z = z

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
    roll = -roll
    pitch = pitch
    yaw = -yaw

    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    ros_transform.rotation.x = quat[0]
    ros_transform.rotation.y = quat[1]
    ros_transform.rotation.z = quat[2]
    ros_transform.rotation.w = quat[3]

    return ros_transform


def carla_transform_to_ros_pose(carla_transform):
    transform_matrix = Transform(carla_transform).matrix

    x, y, z = tf.transformations.translation_from_matrix(transform_matrix)
    quat = tf.transformations.quaternion_from_matrix(transform_matrix)

    ros_transform = Transform()
    ros_transform.translation.x = x
    ros_transform.translation.y = y
    ros_transform.translation.z = z

    ros_transform.rotation.x = quat[0]
    ros_transform.rotation.y = quat[1]
    ros_transform.rotation.z = quat[2]
    ros_transform.rotation.w = quat[3]

    return ros_transform


def _ros_transform_to_pose(ros_transform):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = ros_transform.translation.x, \
                                                        ros_transform.translation.y, \
                                                        ros_transform.translation.z

    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = ros_transform.rotation.x, \
                                                                                     ros_transform.rotation.y, \
                                                                                     ros_transform.rotation.z,\
                                                                                     ros_transform.rotation.w
    return pose


def update_marker_pose(object, base_marker):
    ros_transform = carla_transform_to_ros_transform(carla_Transform(object.transform))
    base_marker.pose = _ros_transform_to_pose(ros_transform)

    base_marker.scale.x = object.box_extent.x * 2.0
    base_marker.scale.y = object.box_extent.y * 2.0
    base_marker.scale.z = object.box_extent.z * 2.0

    base_marker.type = Marker.CUBE


lookup_table_marker_id = {}  # <-- TODO: migrate this in a class
def get_vehicle_marker(object, header, agent_id=88, player=False):
    """
    :param pb2 object (vehicle, pedestrian or traffic light)
    :param base_marker: a marker to fill/update
    :return: a marker
    """
    marker = Marker(header=header)
    marker.color.a = 0.3
    if player:
        marker.color.g = 1
        marker.color.r = 0
        marker.color.b = 0
    else:
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0

    if agent_id not in lookup_table_marker_id:
        lookup_table_marker_id[agent_id] = len(lookup_table_marker_id)
    _id = lookup_table_marker_id[agent_id]

    marker.id = _id
    marker.text = "id = {}".format(_id)
    update_marker_pose(object, marker)

    if not player:  # related to bug request #322
        marker.scale.x = marker.scale.x / 100.0
        marker.scale.y = marker.scale.y / 100.0
        marker.scale.z = marker.scale.z / 100.0

    # the box pose seems to require a little bump to be well aligned with camera depth
    marker.pose.position.z += marker.scale.z / 2.0

    return marker


if __name__ == "__main__":

    rospy.init_node("carla_client", anonymous=True)

    params = rospy.get_param('carla')
    host = params['host']
    port = params['port']

    rospy.loginfo("Trying to connect to {host}:{port}".format(host=host, port=port))

    with make_carla_client(host, port) as client:
        rospy.loginfo("Connection is ok")

        bridge_cls = CarlaROSBridgeWithBag if rospy.get_param('enable_rosbag') else CarlaROSBridge
        with bridge_cls(client=client, params=params) as carla_ros_bridge:
            carla_ros_bridge.run()





