#!/usr/bin/env python

import roslib
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import lanelet2


stb = None
static_transform = None


def timer_callback(event):
    global stb, static_transform
    static_transform.header.stamp = rospy.Time.now()
    stb.sendTransform(static_transform)


if __name__ == '__main__':

    rospy.init_node('map_frame_to_utm_tf_publisher')

    lat_origin = None
    lon_origin = None
    map_frame_id = None
    actual_utm_with_no_offset_frame_id = None
    init = False

    for i in range(3000):
        lat_origin = float(rospy.get_param("/lanelet2_interface_ros/lat_origin"))
        lon_origin = float(rospy.get_param("/lanelet2_interface_ros/lon_origin"))

        map_frame_id = rospy.get_param("/lanelet2_interface_ros/map_frame_id")
        actual_utm_with_no_offset_frame_id = rospy.get_param("/lanelet2_interface_ros/actual_utm_with_no_offset_frame_id")
        if lat_origin is not None and lon_origin is not None \
            and map_frame_id is not None and actual_utm_with_no_offset_frame_id is not None:
            init = True
            break

        rospy.sleep(0.01)

    if not init:
        rospy.logerr("map_frame_to_utm_tf_publisher: Could not initialize")
        exit()

    origin_latlon = lanelet2.core.GPSPoint(lat_origin, lon_origin)
    projector = lanelet2.projection.UtmProjector(
        lanelet2.io.Origin(origin_latlon), False, False)
    origin_xy = projector.forward(origin_latlon)

    stb = tf2_ros.TransformBroadcaster()

    static_transform = geometry_msgs.msg.TransformStamped()
    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = actual_utm_with_no_offset_frame_id
    static_transform.child_frame_id = map_frame_id
    static_transform.transform.translation.x = origin_xy.x
    static_transform.transform.translation.y = origin_xy.y
    static_transform.transform.translation.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    static_transform.transform.rotation.x = q[0]
    static_transform.transform.rotation.y = q[1]
    static_transform.transform.rotation.z = q[2]
    static_transform.transform.rotation.w = q[3]

    rospy.Timer(rospy.Duration(1.), timer_callback)

    rospy.spin()
