#!/usr/bin/env python
#
# Copyright (c) 2018
# FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
# KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import roslib
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import lanelet2


stb = None
static_transform = None
lat_origin = None
lon_origin = None
map_frame_id = None
actual_utm_with_no_offset_frame_id = None


def timer_callback(event):
    global stb, static_transform
    static_transform.header.stamp = rospy.Time.now()
    stb.sendTransform(static_transform)


def wait_for_params_successful():
    global lat_origin, lon_origin, map_frame_id, actual_utm_with_no_offset_frame_id

    for i in range(3000):
        try:
            lat_origin = float(rospy.get_param("/lanelet2_interface_ros/lat_origin"))
            lon_origin = float(rospy.get_param("/lanelet2_interface_ros/lon_origin"))
            map_frame_id = rospy.get_param("/lanelet2_interface_ros/map_frame_id")
            actual_utm_with_no_offset_frame_id = rospy.get_param(
                "/lanelet2_interface_ros/actual_utm_with_no_offset_frame_id")
        except Exception:
            rospy.sleep(0.01)
            continue

        return True

    return False


if __name__ == '__main__':

    rospy.init_node('map_frame_to_utm_tf_publisher')

    if not wait_for_params_successful():
        rospy.logerr("map_frame_to_utm_tf_publisher: Could not initialize")
        exit()

    origin_latlon = lanelet2.core.GPSPoint(lat_origin, lon_origin)
    projector = lanelet2.projection.UtmProjector(
        lanelet2.io.Origin(origin_latlon), False, False)
    origin_xy = projector.forward(origin_latlon)

    stb = tf2_ros.TransformBroadcaster()

    static_transform = geometry_msgs.msg.TransformStamped()
    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = map_frame_id
    static_transform.child_frame_id = actual_utm_with_no_offset_frame_id
    static_transform.transform.translation.x = -origin_xy.x
    static_transform.transform.translation.y = -origin_xy.y
    static_transform.transform.translation.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    static_transform.transform.rotation.x = q[0]
    static_transform.transform.rotation.y = q[1]
    static_transform.transform.rotation.z = q[2]
    static_transform.transform.rotation.w = q[3]

    rospy.Timer(rospy.Duration(1.), timer_callback)

    rospy.spin()
