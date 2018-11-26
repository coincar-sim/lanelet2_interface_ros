#!/usr/bin/env python

# ROS Dependencies
import roslib
import rospy
import tf
import tf2_ros
import geometry_msgs.msg

# Regular Python Dependencies
import lanelet2


stb = None
static_transform = None


# def get_zone(lon, lat):
#     if 56 <= lat < 64 and 3 <= lon < 12:
#         return 32
#     if 72 <= lat < 84 and 0 <= lon < 42:
#         if lon < 9:
#             return 31
#         elif lon < 21:
#             return 33
#         elif lon < 33:
#             return 35
#         return 37
#     return int((lon + 180) / 6) + 1
#
#
# def ll2xy(lat, lon):
#     zone_ = get_zone(lon, lat)
#     p = pyproj.Proj(proj='utm', zone=zone_, ellps='WGS84')
#     [x, y] = p(lon, lat, errcheck=True)
#     return [x, y]


def timer_callback(event):
    global stb, static_transform
    static_transform.header.stamp = rospy.Time.now()
    stb.sendTransform(static_transform)


if __name__ == '__main__':

    rospy.init_node('map_to_mapWithOffset_tf_publisher')

    lat_origin = None
    lon_origin = None
    map_frame_id = None
    map_with_offset_frame_id = None
    init = False

    for i in range(3000):
        lat_origin = float(rospy.get_param("/lanelet2_interface_ros/lat_origin"))
        lon_origin = float(rospy.get_param("/lanelet2_interface_ros/lon_origin"))

        map_frame_id = rospy.get_param("/lanelet2_interface_ros/map_frame_id")
        map_with_offset_frame_id = rospy.get_param("/lanelet2_interface_ros/map_with_offset_frame_id")
        if lat_origin is not None and lon_origin is not None \
            and map_frame_id is not None and map_with_offset_frame_id is not None:
            init = True
            break

    if not init:
        rospy.logerr("map_to_mapWithOffset_tf_publisher: Could not initialize")
        exit()


    projector = lanelet2.projection.UtmProjector(
        lanelet2.io.Origin(lanelet2.core.GPSPoint(lat_origin, lon_origin)), False, False)
    origin_x = 0.
    origin_y = 0.
    stb = tf2_ros.TransformBroadcaster()

    static_transform = geometry_msgs.msg.TransformStamped()
    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = map_frame_id
    static_transform.child_frame_id = map_with_offset_frame_id
    static_transform.transform.translation.x = origin_x
    static_transform.transform.translation.y = origin_y
    static_transform.transform.translation.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    static_transform.transform.rotation.x = q[0]
    static_transform.transform.rotation.y = q[1]
    static_transform.transform.rotation.z = q[2]
    static_transform.transform.rotation.w = q[3]

    rospy.Timer(rospy.Duration(1.), timer_callback)

    rospy.spin()
