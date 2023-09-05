#!/usr/bin/env python

# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
# Authors: David Butterworth <dbworth@cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy
import random  # randint
import numpy as np
from typing import Union
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA

Q90X = Quaternion(x=0.70710678, y=0., z=0., w=0.70710678)
Q90Y = Quaternion(x=0., y=0.70710678, z=0., w=0.70710678)
Q90Z = Quaternion(x=0., y=0., z=0.70710678, w=0.70710678)


class RvizMarkers(Node):

    """
    A class for publishing markers in Rviz
    """

    def __init__(self, base_frame, marker_topic, wait_time=None):
        super().__init__('RvizMarkersNode_')  # + marker_topic)
        self.base_frame = base_frame
        self.marker_topic = marker_topic

        # Set the default Marker parameters
        self.setDefaultMarkerParams()

        # Create the Rviz Marker Publisher
        self.loadMarkerPublisher(wait_time)

    def setDefaultMarkerParams(self):
        """
        Set the default parameters for each type of Rviz Marker
        """

        self.marker_lifetime = Duration(seconds=0.0).to_msg()
        self.muted = False
        self.alpha = 1.0

        # Set default parameters for Cylinder Marker
        self.cylinder_marker = Marker()
        self.cylinder_marker.header.frame_id = self.base_frame
        self.cylinder_marker.ns = "Cylinder"  # unique ID
        self.cylinder_marker.action = Marker().ADD
        self.cylinder_marker.type = Marker().CYLINDER
        self.cylinder_marker.lifetime = self.marker_lifetime

        # Reset Marker
        self.reset_marker = Marker()
        self.reset_marker.header.frame_id = self.base_frame
        self.reset_marker.header.stamp = self.get_clock().now().to_msg()
        self.reset_marker.action = 3

        # Arrow Marker
        self.arrow_marker = Marker()
        self.arrow_marker.header.frame_id = self.base_frame
        self.arrow_marker.ns = "Arrow"  # unique ID
        self.arrow_marker.action = Marker().ADD
        self.arrow_marker.type = Marker().ARROW
        self.arrow_marker.lifetime = self.marker_lifetime

        # Rectangle Marker
        self.rectangle_marker = Marker()
        self.rectangle_marker.header.frame_id = self.base_frame
        self.rectangle_marker.ns = "Rectangle"  # unique ID
        self.rectangle_marker.action = Marker().ADD
        self.rectangle_marker.type = Marker().CUBE
        self.rectangle_marker.lifetime = self.marker_lifetime

        # Line Marker
        self.line_marker = Marker()
        self.line_marker.header.frame_id = self.base_frame
        self.line_marker.ns = "Line"  # unique ID
        self.line_marker.action = Marker().ADD
        self.line_marker.type = Marker().LINE_STRIP
        self.line_marker.lifetime = self.marker_lifetime

        # Path Marker (Line List)
        self.path_marker = Marker()
        self.path_marker.header.frame_id = self.base_frame
        self.path_marker.ns = "Path"  # unique ID
        self.path_marker.action = Marker().ADD
        self.path_marker.type = Marker().LINE_LIST
        self.path_marker.lifetime = self.marker_lifetime
        self.path_marker.pose.position.x = 0.0
        self.path_marker.pose.position.y = 0.0
        self.path_marker.pose.position.z = 0.0
        self.path_marker.pose.orientation.x = 0.0
        self.path_marker.pose.orientation.y = 0.0
        self.path_marker.pose.orientation.z = 0.0
        self.path_marker.pose.orientation.w = 1.0

        # Sphere Marker (A single sphere)
        # This renders a low-quality sphere
        self.sphere_marker = Marker()
        self.sphere_marker.header.frame_id = self.base_frame
        self.sphere_marker.ns = "Sphere"  # unique ID
        self.sphere_marker.type = Marker().SPHERE
        self.sphere_marker.action = Marker().ADD
        self.sphere_marker.lifetime = self.marker_lifetime
        self.sphere_marker.pose.position.x = 0.0
        self.sphere_marker.pose.position.y = 0.0
        self.sphere_marker.pose.position.z = 0.0
        self.sphere_marker.pose.orientation.x = 0.0
        self.sphere_marker.pose.orientation.y = 0.0
        self.sphere_marker.pose.orientation.z = 0.0
        self.sphere_marker.pose.orientation.w = 1.0

        # Sphere Marker #2 (A single sphere)
        # A Sphere List with one sphere, this renders a
        # higher-quality sphere than the method above
        self.sphere_marker2 = Marker()
        self.sphere_marker2.header.frame_id = self.base_frame
        self.sphere_marker2.ns = "Sphere"  # unique ID
        self.sphere_marker2.type = Marker().SPHERE_LIST
        self.sphere_marker2.action = Marker().ADD
        self.sphere_marker2.lifetime = self.marker_lifetime
        self.sphere_marker2.pose.position.x = 0.0
        self.sphere_marker2.pose.position.y = 0.0
        self.sphere_marker2.pose.position.z = 0.0
        self.sphere_marker2.pose.orientation.x = 0.0
        self.sphere_marker2.pose.orientation.y = 0.0
        self.sphere_marker2.pose.orientation.z = 0.0
        self.sphere_marker2.pose.orientation.w = 1.0
        point1 = Point()
        self.sphere_marker2.points.append(point1)
        self.sphere_marker2.colors.append(self.getColor('blue'))

        # Spheres List (Multiple spheres)
        self.spheres_marker = Marker()
        self.spheres_marker.header.frame_id = self.base_frame
        self.spheres_marker.ns = "Spheres"  # unique ID
        self.spheres_marker.type = Marker().SPHERE_LIST
        self.spheres_marker.action = Marker().ADD
        self.spheres_marker.lifetime = self.marker_lifetime
        self.spheres_marker.pose.position.x = 0.0
        self.spheres_marker.pose.position.y = 0.0
        self.spheres_marker.pose.position.z = 0.0
        self.spheres_marker.pose.orientation.x = 0.0
        self.spheres_marker.pose.orientation.y = 0.0
        self.spheres_marker.pose.orientation.z = 0.0
        self.spheres_marker.pose.orientation.w = 1.0

        # Cube Marker (Block or cuboid)
        self.cube_marker = Marker()
        self.cube_marker.header.frame_id = self.base_frame
        self.cube_marker.ns = "Block"  # unique ID
        self.cube_marker.action = Marker().ADD
        self.cube_marker.type = Marker().CUBE
        self.cube_marker.lifetime = self.marker_lifetime

        # Cubes List (Multiple cubes)
        self.cubes_marker = Marker()
        self.cubes_marker.header.frame_id = self.base_frame
        self.cubes_marker.ns = "Cubes"  # unique ID
        self.cubes_marker.type = Marker().CUBE_LIST
        self.cubes_marker.action = Marker().ADD
        self.cubes_marker.lifetime = self.marker_lifetime
        self.cubes_marker.pose.position.x = 0.0
        self.cubes_marker.pose.position.y = 0.0
        self.cubes_marker.pose.position.z = 0.0
        self.cubes_marker.pose.orientation.x = 0.0
        self.cubes_marker.pose.orientation.y = 0.0
        self.cubes_marker.pose.orientation.z = 0.0
        self.cubes_marker.pose.orientation.w = 1.0

        # Cylinder Marker
        self.cylinder_marker = Marker()
        self.cylinder_marker.header.frame_id = self.base_frame
        self.cylinder_marker.ns = "Cylinder"  # unique ID
        self.cylinder_marker.action = Marker().ADD
        self.cylinder_marker.type = Marker().CYLINDER
        self.cylinder_marker.lifetime = self.marker_lifetime

        # Mesh Marker
        self.mesh_marker = Marker()
        self.mesh_marker.header.frame_id = self.base_frame
        self.mesh_marker.ns = "Mesh"  # unique ID
        self.mesh_marker.action = Marker().ADD
        self.mesh_marker.type = Marker().MESH_RESOURCE
        self.mesh_marker.lifetime = self.marker_lifetime

        # Text Marker
        self.text_marker = Marker()
        self.text_marker.header.frame_id = self.base_frame
        self.text_marker.ns = "Text"  # unique ID
        self.text_marker.action = Marker().ADD
        self.text_marker.type = Marker().TEXT_VIEW_FACING
        self.text_marker.lifetime = self.marker_lifetime

    def loadMarkerPublisher(self, wait_time=None):
        """
        Initialize the ROS Publisher.

        If wait_time != None, wait for specified number of
        seconds for a subscriber to connect.
        """

        # Check if the ROS Publisher has already been created
        if hasattr(self, 'pub_rviz_marker'):
            return

        # Create the Rviz Marker Publisher

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
            # lifesspan= Duration
            # deadline= Duration
            # liveliness=LivelinessPolicy.
        )

        self.pub_rviz_marker = self.create_publisher(
            Marker, self.marker_topic, qos_profile)

        self.get_logger().debug("Publishing Rviz markers on topic '%s'" % self.marker_topic)

        # Block for specified number of seconds,
        # or until there is 1 subscriber
        if wait_time != None:
            self.waitForSubscriber(self.pub_rviz_marker, wait_time)

    def waitForSubscriber(self, publisher, wait_time=1.0):
        """
        Wait until there is 1 subscriber to a ROS Publisher,
        or until some number of seconds have elapsed.
        """

        start_time = self.get_clock().now().to_msg()
        max_time = start_time + Duration(seconds=wait_time).to_msg()

        num_existing_subscribers = publisher.get_num_connections()

        rate = self.create_rate(100)
        while (num_existing_subscribers == 0):
            # print 'Number of subscribers: ', num_existing_subscribers
            rate.sleep()

            if (self.get_clock().now().to_msg() > max_time):
                self.get_logger().error("No subscribers connected to the '%s' topic after %f seconds",
                                        self.marker_topic, wait_time)
                return False

            num_existing_subscribers = publisher.get_num_connections()

        return True

    def publishMarker(self, marker):
        """
        Publish a Marker Msg
        """

        if (self.muted == True):
            return True

        # Check ROS Publisher
        self.loadMarkerPublisher()

        self.pub_rviz_marker.publish(marker)

        return True

    def deleteAllMarkers(self):
        """
        Publish a Msg to delete all Markers
        """

        return self.publishMarker(self.reset_marker)

    def getColor(self, color):
        """
        Convert a color name or RGB value to a ROS ColorRGBA type

        @param color name (string) or RGB color value (tuple or list)

        @return color (ColorRGBA)
        """

        result = ColorRGBA()
        result.a = self.alpha

        if (isinstance(color, tuple) or isinstance(color, list)):
            if len(color) == 3:
                result.r = color[0]
                result.g = color[1]
                result.b = color[2]
            elif len(color) == 4:
                result.r = color[0]
                result.g = color[1]
                result.b = color[2]
                result.a = color[3]
            else:
                raise ValueError(
                    'color must have 3 or 4 float values in getColor()')
        elif (color == 'red'):
            result.r = 0.8
            result.g = 0.1
            result.b = 0.1
        elif (color == 'green'):
            result.r = 0.1
            result.g = 0.8
            result.b = 0.1
        elif (color == 'blue'):
            result.r = 0.1
            result.g = 0.1
            result.b = 0.8
        elif (color == 'grey') or (color == 'gray'):
            result.r = 0.9
            result.g = 0.9
            result.b = 0.9
        elif (color == 'white'):
            result.r = 1.0
            result.g = 1.0
            result.b = 1.0
        elif (color == 'orange'):
            result.r = 1.0
            result.g = 0.5
            result.b = 0.0
        elif (color == 'translucent_light'):
            result.r = 0.1
            result.g = 0.1
            result.b = 0.1
            result.a = 0.1
        elif (color == 'translucent'):
            result.r = 0.1
            result.g = 0.1
            result.b = 0.1
            result.a = 0.25
        elif (color == 'translucent_dark'):
            result.r = 0.1
            result.g = 0.1
            result.b = 0.1
            result.a = 0.5
        elif (color == 'black'):
            result.r = 0.0
            result.g = 0.0
            result.b = 0.0
        elif (color == 'yellow'):
            result.r = 1.0
            result.g = 1.0
            result.b = 0.0
        elif (color == 'brown'):
            result.r = 0.597
            result.g = 0.296
            result.b = 0.0
        elif (color == 'pink'):
            result.r = 1.0
            result.g = 0.4
            result.b = 1.0
        elif (color == 'lime_green'):
            result.r = 0.6
            result.g = 1.0
            result.b = 0.2
        elif (color == 'clear'):
            result.r = 1.0
            result.g = 1.0
            result.b = 1.0
            result.a = 0.0
        elif (color == 'purple'):
            result.r = 0.597
            result.g = 0.0
            result.b = 0.597
        elif (color == 'random'):
            # Get a random color that is not too light
            while True:
                result.r = random.random()  # random float from 0 to 1
                result.g = random.random()
                result.b = random.random()
                if ((result.r + result.g + result.b) > 1.5):  # 0=black, 3=white
                    break
        else:
            self.get_logger().error(
                "getColor() called with unknown color name '%s', defaulting to 'blue'", color)
            result.r = 0.1
            result.g = 0.1
            result.b = 0.8

        return result

    def getRandomColor(self):
        """
        Get a random color.

        @return color (ColorRGBA)
        """

        # Make a list of the color names to choose from
        all_colors = []
        all_colors.append('red')
        all_colors.append('green')
        all_colors.append('blue')
        all_colors.append('grey')
        all_colors.append('white')
        all_colors.append('orange')
        all_colors.append('yellow')
        all_colors.append('brown')
        all_colors.append('pink')
        all_colors.append('lime_green')
        all_colors.append('purple')

        # Chose a random color name
        rand_num = random.randint(0, len(all_colors) - 1)
        rand_color_name = all_colors[rand_num]

        return rand_color_name

    def publishSphere(self, pose, color, scale, lifetime=None):
        """
        Publish a sphere Marker. This renders 3D looking sphere.

        @param pose (numpy matrix, numpy ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (isinstance(pose, np.matrix) or isinstance(pose, np.ndarray)):
            sphere_pose = mat_to_pose(pose)
        elif isinstance(pose, Pose):
            sphere_pose = pose
        elif isinstance(pose, Point):
            pose_msg = Pose()
            pose_msg.position = pose
            sphere_pose = pose_msg
        else:
            self.get_logger().error(
                "Pose is unsupported type '%s' in publishSphere()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if isinstance(scale, Vector3):
            sphere_scale = scale
        elif isinstance(scale, float):
            sphere_scale = Vector3(x=scale, y=scale, z=scale)
        else:
            self.get_logger().error(
                "Scale is unsupported type '%s' in publishSphere()", type(scale).__name__)
            return False

        # Increment the ID number
        self.sphere_marker.id += 1

        # Get the default parameters
        sphere_marker = self.sphere_marker

        if lifetime == None:
            sphere_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            sphere_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        sphere_marker.header.stamp = self.get_clock().now().to_msg()

        # Set marker size
        sphere_marker.scale = sphere_scale

        # Set marker color
        sphere_marker.color = self.getColor(color)

        # Set the pose
        sphere_marker.pose = sphere_pose

        return self.publishMarker(sphere_marker)

    def publishSphere2(self, pose, color, scale, lifetime=None):
        """
        Publish a sphere Marker. This renders a smoother, flatter-looking sphere.

        @param pose (numpy matrix, numpy ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (isinstance(pose, np.matrix) or isinstance(pose, np.ndarray)):
            sphere_pose = mat_to_pose(pose)
        elif isinstance(pose, Pose):
            sphere_pose = pose
        elif isinstance(pose, Point):
            pose_msg = Pose()
            pose_msg.position = pose
            sphere_pose = pose_msg
        else:
            self.get_logger().error(
                "Pose is unsupported type '%s' in publishSphere()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if isinstance(scale, Vector3):
            sphere_scale = scale
        elif isinstance(scale, float):
            sphere_scale = Vector3(x=scale, y=scale, z=scale)
        else:
            self.get_logger().error(
                "Scale is unsupported type '%s' in publishSphere()", type(scale).__name__)
            return False

        # Increment the ID number
        self.sphere_marker.id += 1

        # Get the default parameters
        sphere_marker = self.sphere_marker2  # sphere_marker2 = SPHERE_LIST

        if lifetime == None:
            sphere_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            sphere_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        sphere_marker.header.stamp = self.get_clock().now().to_msg()

        # Set marker size
        sphere_marker.scale = sphere_scale

        # Set marker color
        sphere_marker.color = self.getColor(color)

        # Set the pose of one sphere in the list
        sphere_marker.points[0] = sphere_pose.position
        sphere_marker.colors[0] = self.getColor(color)

        return self.publishMarker(sphere_marker)

    def publishArrow(self, pose, color, scale, lifetime=None):
        """
        Publish an arrow Marker.

        @param pose (numpy matrix, numpy ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (isinstance(pose, np.matrix) or isinstance(pose, np.ndarray)):
            arrow_pose = mat_to_pose(pose)
        elif isinstance(pose, Pose):
            arrow_pose = pose
        else:
            self.get_logger().error(
                "Pose is unsupported type '%s' in publishArrow()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if isinstance(scale, Vector3):
            arrow_scale = scale
        elif isinstance(scale, float):
            arrow_scale = Vector3(x=scale, y=0.1*scale, z=0.1*scale)
        else:
            self.get_logger().error(
                "Scale is unsupported type '%s' in publishArrow()", type(scale).__name__)
            return False

        # Increment the ID number
        self.arrow_marker.id += 1

        # Get the default parameters
        arrow_marker = self.arrow_marker

        if lifetime == None:
            arrow_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            arrow_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        arrow_marker.header.stamp = self.get_clock().now().to_msg()

        # Set the pose
        arrow_marker.pose = arrow_pose

        # Set marker size
        arrow_marker.scale = arrow_scale

        # Set marker color
        arrow_marker.color = self.getColor(color)

        return self.publishMarker(arrow_marker)

    def publishCube(self, pose, color, scale, lifetime=None):
        """
        Publish a cube Marker.

        @param pose (numpy matrix, numpy ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (isinstance(pose, np.matrix) or isinstance(pose, np.ndarray)):
            cube_pose = mat_to_pose(pose)
        elif isinstance(pose, Pose):
            cube_pose = pose
        else:
            self.get_logger().error(
                "Pose is unsupported type '%s' in publishCube()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if isinstance(scale, Vector3):
            cube_scale = scale
        elif isinstance(scale, float):
            cube_scale = Vector3(x=scale, y=scale, z=scale)
        else:
            self.get_logger().error(
                "Scale is unsupported type '%s' in publishCube()", type(scale).__name__)
            return False

        # Increment the ID number
        self.cube_marker.id += 1

        # Get the default parameters
        cube_marker = self.cube_marker

        if lifetime == None:
            cube_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            cube_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        cube_marker.header.stamp = self.get_clock().now().to_msg()

        # Set the pose
        cube_marker.pose = cube_pose

        # Set marker size
        cube_marker.scale = cube_scale

        # Set marker color
        cube_marker.color = self.getColor(color)

        return self.publishMarker(cube_marker)

    def publishCubes(self, list_of_cubes: list, color, scale: Union[Vector3, float], lifetime=None):
        """
        Publish a list of cubes.

        @param list_of_cubes (list of numpy matrix, list of numpy ndarray, list of ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Check input
        # if isinstance(list_of_cubes, list) == False:
        #     self.get_logger().error("list_of_cubes is unsupported type '%s' in publishCubes()",
        #                             type(list_of_cubes).__name__)
        #     return False

        # Convert input scale to a ROS Vector3 Msg
        if isinstance(scale, Vector3):
            cubes_scale = scale
        elif isinstance(scale, float):
            cubes_scale = Vector3(x=scale, y=scale, z=scale)
        else:
            self.get_logger().error(
                "Scale is unsupported type '%s' in publishCubes()", type(scale).__name__)
            return False

        # Increment the ID number
        self.cubes_marker.id += 1

        # Get the default parameters
        cubes_marker = self.cubes_marker

        if lifetime == None:
            cubes_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            cubes_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        cubes_marker.header.stamp = self.get_clock().now().to_msg()

        # Set marker size
        cubes_marker.scale = cubes_scale

        # Set marker color
        cubes_marker.color = self.getColor(color)

        cubes_color = self.getColor(color)

        # Set the cubes positions and color
        cubes_marker.points[:] = []  # clear
        cubes_marker.colors[:] = []
        for i in range(0, len(list_of_cubes)):

            # Each cube position needs to be a ROS Point Msg
            if isinstance(list_of_cubes[i], Pose):
                cubes_marker.points.append(list_of_cubes[i].position)
                cubes_marker.colors.append(cubes_color)
            elif (isinstance(list_of_cubes[i], np.matrix) or isinstance(list_of_cubes[i], np.ndarray)):
                pose_i = mat_to_pose(list_of_cubes[i])
                cubes_marker.points.append(pose_i.position)
                cubes_marker.colors.append(cubes_color)
            elif isinstance(list_of_cubes[i], Point):
                cubes_marker.points.append(list_of_cubes[i])
                cubes_marker.colors.append(cubes_color)
            else:
                self.get_logger().error("list_of_cubes contains unsupported type '%s' in publishCubes()",
                                        type(list_of_cubes[i]).__name__)
                return False

        return self.publishMarker(cubes_marker)

    def publishBlock(self, pose, color, scale, lifetime=None):
        """
        Publish a cube Marker.

        @param pose (numpy matrix, numpy ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        return self.publishCube(pose, color, scale, lifetime=lifetime)

    def publishCylinder(self, pose, color, height, radius, lifetime=None):
        """
        Publish a cylinder Marker.

        @param pose (numpy matrix, numpy ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param height (float)
        @param radius (float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (isinstance(pose, np.matrix) or isinstance(pose, np.ndarray)):
            cylinder_pose = mat_to_pose(pose)
        elif isinstance(pose, Pose):
            cylinder_pose = pose
        else:
            self.get_logger().error(
                "Pose is unsupported type '%s' in publishCylinder()", type(pose).__name__)
            return False

        # Increment the ID number
        self.cylinder_marker.id += 1

        # Get the default parameters
        cylinder_marker = self.cylinder_marker

        if lifetime == None:
            cylinder_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            cylinder_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        cylinder_marker.header.stamp = self.get_clock().now().to_msg()

        # Set the pose
        cylinder_marker.pose = cylinder_pose

        # Set marker size
        cylinder_marker.scale.x = radius
        cylinder_marker.scale.y = radius
        cylinder_marker.scale.z = height

        # Set marker color
        cylinder_marker.color = self.getColor(color)

        return self.publishMarker(cylinder_marker)

    def publishAxis(self, pose: Union[np.matrix, np.array, Pose], length=0.05, radius=0.01, lifetime=None):
        """
        Publish an axis Marker.

        @param pose (numpy matrix, numpy ndarray, ROS Pose)
        @param length axis length (float)
        @param radius axis radius (float)
        @param lifetime (float, None = never expire)
        """

        # Convert input pose to a numpy matrix
        if (isinstance(pose, np.matrix) or isinstance(pose, np.ndarray)):
            center_pose = mat_to_pose(pose)
        elif isinstance(pose, Pose):
            center_pose = pose
        else:
            self.get_logger().error(
                "Pose is unsupported type '%s' in publishAxis()", type(pose).__name__)
            return False

        x_rot = point_rotation_by_quaternion(
            Point(x=length/2), center_pose.orientation)

        self.publishCylinder(Pose(position=add_two_points(x_rot, center_pose.position),
                                  orientation=quaternion_multiply(center_pose.orientation, Q90Y)), 'red', length, radius)

        y_rot = point_rotation_by_quaternion(
            Point(y=length/2), center_pose.orientation)

        self.publishCylinder(Pose(position=add_two_points(y_rot, center_pose.position),
                                  orientation=quaternion_multiply(center_pose.orientation, Q90X)), 'green', length, radius)

        z_rot = point_rotation_by_quaternion(
            Point(z=length/2), center_pose.orientation)

        self.publishCylinder(Pose(position=add_two_points(z_rot, center_pose.position),
                                  orientation=quaternion_multiply(center_pose.orientation, Q90Z)), 'blue', length, radius)

        return True

    def publishAxisArray(self, pose_array, length=0.05, radius=0.01, lifetime=None):
        for pose in pose_array:
            self.publishAxis(pose, length, radius, lifetime=lifetime)

    def publishMesh(self, pose, file_name, color, scale, lifetime=None):
        """
        Publish a mesh Marker. The mesh file can be a binary STL or collada DAE file.

        @param pose (numpy matrix, numpy ndarray, ROS Pose)
        @param file_name (string)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (isinstance(pose, np.matrix) or isinstance(pose, np.ndarray)):
            mesh_pose = mat_to_pose(pose)
        elif isinstance(pose, Pose):
            mesh_pose = pose
        else:
            self.get_logger().error(
                "Pose is unsupported type '%s' in publishMesh()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if isinstance(scale, Vector3):
            mesh_scale = scale
        elif isinstance(scale, float):
            mesh_scale = Vector3(x=scale, y=scale, z=scale)
        else:
            self.get_logger().error(
                "Scale is unsupported type '%s' in publishMesh()", type(scale).__name__)
            return False

        # Increment the ID number
        self.mesh_marker.id += 1

        # Get the default parameters
        mesh_marker = self.mesh_marker

        if lifetime == None:
            mesh_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            mesh_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        mesh_marker.header.stamp = self.get_clock().now().to_msg()

        # Set marker size
        mesh_marker.scale = mesh_scale

        # Set marker color
        if color == None:
            mesh_marker.color = ColorRGBA()  # no color
        else:
            mesh_marker.color = self.getColor(color)

        # Set the pose
        mesh_marker.pose = mesh_pose

        # Set the mesh
        mesh_marker.mesh_resource = file_name
        mesh_marker.mesh_use_embedded_materials = True

        return self.publishMarker(mesh_marker)

    def publishRectangle(self, point1, point2, color, lifetime=None):
        """
        Publish a rectangle Marker between two points. If the z-values are not the same then this will result in a cuboid.

        @param point1 (ROS Point)
        @param point2 (ROS Point)
        @param color name (string) or RGB color value (tuple or list)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input points to ROS Point Msgs
        if isinstance(point1, Point):
            rect_point1 = point1
        else:
            self.get_logger().error(
                "Point1 is unsupported type '%s' in publishRectangle()", type(point1).__name__)
            return False
        if isinstance(point2, Point):
            rect_point2 = point2
        else:
            self.get_logger().error(
                "Point2 is unsupported type '%s' in publishRectangle()", type(point2).__name__)
            return False

        # Increment the ID number
        self.rectangle_marker.id += 1

        # Get the default parameters
        rectangle_marker = self.rectangle_marker

        if lifetime == None:
            rectangle_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            rectangle_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        rectangle_marker.header.stamp = self.get_clock().now().to_msg()

        # Set marker color
        rectangle_marker.color = self.getColor(color)

        # Calculate the center pose
        rect_pose = Pose()
        rect_pose.position.x = (
            rect_point1.x - rect_point2.x) / 2.0 + rect_point2.x
        rect_pose.position.y = (
            rect_point1.y - rect_point2.y) / 2.0 + rect_point2.y
        rect_pose.position.z = (
            rect_point1.z - rect_point2.z) / 2.0 + rect_point2.z
        rectangle_marker.pose = rect_pose

        # Calculate scale
        rectangle_marker.scale.x = np.fabs(rect_point1.x - rect_point2.x)
        rectangle_marker.scale.y = np.fabs(rect_point1.y - rect_point2.y)
        rectangle_marker.scale.z = np.fabs(rect_point1.z - rect_point2.z)

        return self.publishMarker(rectangle_marker)

    def publishPlane(self, pose, depth, width, color, lifetime=None):
        """
        Publish a plane Marker.

        @param pose (numpy matrix, numpy ndarray, ROS Pose)
        @param depth (float)
        @param width (float)
        @param color name (string) or RGB color value (tuple or list)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (isinstance(pose, np.matrix) or isinstance(pose, np.ndarray)):
            rect_pose = mat_to_pose(pose)
        elif isinstance(pose, Pose):
            rect_pose = pose
        else:
            self.get_logger().error(
                "Pose is unsupported type '%s' in publishRectangle()", type(pose).__name__)
            return False

        # Increment the ID number
        self.rectangle_marker.id += 1

        # Get the default parameters
        rectangle_marker = self.rectangle_marker

        if lifetime == None:
            rectangle_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            rectangle_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        rectangle_marker.header.stamp = self.get_clock().now().to_msg()

        # Set marker color
        rectangle_marker.color = self.getColor(color)

        # Set the pose
        rectangle_marker.pose = rect_pose

        # Set the scale
        rectangle_marker.scale.x = depth
        rectangle_marker.scale.y = width
        rectangle_marker.scale.z = 0.0

        return self.publishMarker(rectangle_marker)

    def publishLine(self, point1, point2, color, width, lifetime=None):
        """
        Publish a line Marker between two points.

        @param point1 (ROS Point, ROS Pose, numpy matrix, numpy ndarray)
        @param point2 (ROS Point, ROS Pose, numpy matrix, numpy ndarray)
        @param color name (string) or RGB color value (tuple or list)
        @param width (float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input points to ROS Point Msgs
        if isinstance(point1, Point):
            line_point1 = point1
        elif isinstance(point1, Pose):
            position = point1.position
            line_point1 = Point(x=position.x, y=position.y, z=position.z)
        elif (isinstance(point1, np.matrix) or isinstance(point1, np.ndarray)):
            pose = mat_to_pose(point1)
            position = pose.position
            line_point1 = Point(x=position.x, y=position.y, z=position.z)
        else:
            self.get_logger().error(
                "Point1 is unsupported type '%s' in publishLine()", type(point1).__name__)
            return False

        if isinstance(point2, Point):
            line_point2 = point2
        elif isinstance(point2, Pose):
            position = point2.position
            line_point2 = Point(x=position.x, y=position.y, z=position.z)
        elif (isinstance(point2, np.matrix) or isinstance(point2, np.ndarray)):
            pose = mat_to_pose(point2)
            position = pose.position
            line_point2 = Point(x=position.x, y=position.y, z=position.z)
        else:
            self.get_logger().error(
                "Point2 is unsupported type '%s' in publishLine()", type(point2).__name__)
            return False

        # Increment the ID number
        self.line_marker.id += 1

        # Get the default parameters
        line_marker = self.line_marker

        if lifetime == None:
            line_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            line_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        line_marker.header.stamp = self.get_clock().now().to_msg()

        # Set marker color
        line_marker.color = self.getColor(color)

        # Set the start and end points
        line_marker.points[:] = []  # clear
        line_marker.points.append(line_point1)
        line_marker.points.append(line_point2)

        # Set the line width
        line_marker.scale.x = width

        return self.publishMarker(line_marker)

    def publishPath(self, path, color, width, lifetime=None):
        """
        Publish a path Marker using a set of waypoints.

        @param path (list of ROS Points)
        @param color name (string) or RGB color value (tuple or list)
        @param width (float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Check input
        if isinstance(path, list):
            path_path = path  # :-)
        else:
            self.get_logger().error(
                "Path is unsupported type '%s' in publishPath()", type(path).__name__)
            return False

        # Increment the ID number
        self.path_marker.id += 1

        # Get the default parameters
        path_marker = self.path_marker

        if lifetime == None:
            path_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            path_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        path_marker.header.stamp = self.get_clock().now().to_msg()

        # Set the path width
        path_marker.scale.x = width

        path_color = self.getColor(color)

        # Set the path points and color
        path_marker.points[:] = []  # clear
        path_marker.colors[:] = []
        for i in range(1, len(path)):

            # Each path waypoint needs to be a ROS Point Msg
            if isinstance(path[i], Point):
                # Start of segment is previous point
                path_marker.points.append(path[i-1])
                path_marker.colors.append(path_color)
                # End of segment is current point
                path_marker.points.append(path[i])
                path_marker.colors.append(path_color)
            elif isinstance(path[i], Pose):
                # Start of segment is previous point
                position = path[i-1].position
                point = Point(x=position.x, y=position.y, z=position.z)
                path_marker.points.append(point)
                path_marker.colors.append(path_color)
                # End of segment is current point
                position = path[i].position
                point = Point(x=position.x, y=position.y, z=position.z)
                path_marker.points.append(point)
                path_marker.colors.append(path_color)
            elif (isinstance(path[i], np.matrix) or isinstance(path[i], np.ndarray)):
                # Start of segment is previous point
                pose = mat_to_pose(path[i-1])
                position = pose.position
                point = Point(x=position.x, y=position.y, z=position.z)
                path_marker.points.append(point)
                path_marker.colors.append(path_color)
                # End of segment is current point
                pose = mat_to_pose(path[i])
                position = pose.position
                point = Point(x=position.x, y=position.y, z=position.z)
                path_marker.points.append(point)
                path_marker.colors.append(path_color)
            else:
                self.get_logger().error(
                    "path list contains unsupported type '%s' in publishPath()", type(path[i]).__name__)
                return False

        return self.publishMarker(path_marker)

    def publishPolygon(self, polygon, color, width, lifetime=None):
        """
        Publish a polygon Marker.

        @param polygon (ROS Polygon)
        @param color name (string) or RGB color value (tuple or list)
        @param width line width (float)
        @param lifetime (float, None = never expire)

        a path with the start and end points connected
        """

        if (self.muted == True):
            return True

        # Check input
        if isinstance(polygon, Polygon):
            polygon_msg = polygon
        else:
            self.get_logger().error(
                "Path is unsupported type '%s' in publishPolygon()", type(polygon).__name__)
            return False

        # Copy points from ROS Polygon Msg into a list
        polygon_path = []
        for i in range(0, len(polygon_msg.points)):
            x = polygon_msg.points[i].x
            y = polygon_msg.points[i].y
            z = polygon_msg.points[i].z
            polygon_path.append(Point(x=x, y=y, z=z))

        # Add the first point again
        x = polygon_msg.points[0].x
        y = polygon_msg.points[0].y
        z = polygon_msg.points[0].z
        polygon_path.append(Point(x=x, y=y, z=z))

        return self.publishPath(polygon_path, color, width, lifetime)

    def publishSpheres(self, list_of_spheres: list, color, scale, lifetime=None):
        """
        Publish a list of spheres. This renders smoother, flatter-looking spheres.

        @param list_of_spheres (list of numpy matrix, list of numpy ndarray, list of ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Check input
        # if isinstance(list_of_spheres, list:
        #     self.get_logger().error("list_of_spheres is unsupported type '%s' in publishSpheres()",
        #                             type(list_of_spheres).__name__)
        #     return False

        # Convert input scale to a ROS Vector3 Msg
        if isinstance(scale, Vector3):
            spheres_scale = scale
        elif isinstance(scale, float):
            spheres_scale = Vector3(x=scale, y=scale, z=scale)
        else:
            self.get_logger().error(
                "Scale is unsupported type '%s' in publishSpheres()", type(scale).__name__)
            return False

        # Increment the ID number
        self.spheres_marker.id += 1

        # Get the default parameters
        spheres_marker = self.spheres_marker

        if lifetime == None:
            spheres_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            spheres_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        spheres_marker.header.stamp = self.get_clock().now().to_msg()

        # Set marker size
        spheres_marker.scale = spheres_scale

        # Set marker color
        spheres_marker.color = self.getColor(color)

        spheres_color = self.getColor(color)
        # spheres_marker.color = spheres_color

        # Set the sphere positions and color
        spheres_marker.points[:] = []  # clear
        spheres_marker.colors[:] = []
        for i in range(0, len(list_of_spheres)):

            # Each sphere position needs to be a ROS Point Msg
            if isinstance(list_of_spheres[i], Pose):
                spheres_marker.points.append(list_of_spheres[i].position)
                spheres_marker.colors.append(spheres_color)
            elif (isinstance(list_of_spheres[i], np.matrix) or isinstance(list_of_spheres[i], np.ndarray)):
                pose_i = self.mat_to_pose(list_of_spheres[i])
                spheres_marker.points.append(pose_i.position)
                spheres_marker.colors.append(spheres_color)
            elif isinstance(list_of_spheres[i], Point):
                spheres_marker.points.append(list_of_spheres[i])
                spheres_marker.colors.append(spheres_color)
            else:
                self.get_logger().error("list_of_sphere contains unsupported type '%s' in publishSphere()",
                                        type(list_of_spheres[i]).__name__)
                return False

        return self.publishMarker(spheres_marker)

    def publishText(self, pose, text, color, scale, lifetime=None):
        """
        Publish a text Marker

        @param pose (numpy matrix, numpy ndarray, ROS Pose)
        @param text (string)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (isinstance(pose, np.matrix) or isinstance(pose, np.ndarray)):
            text_pose = mat_to_pose(pose)
        elif isinstance(pose, Pose):
            text_pose = pose
        else:
            self.get_logger().error(
                "Pose is unsupported type '%s' in publishText()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if isinstance(scale, Vector3):
            text_scale = scale
        elif isinstance(scale, float):
            text_scale = Vector3(x=scale, y=scale, z=scale)
        else:
            self.get_logger().error(
                "Scale is unsupported type '%s' in publishText()", type(scale).__name__)
            return False

        # Increment the ID number
        self.text_marker.id += 1

        # Get the default parameters
        text_marker = self.text_marker

        if lifetime == None:
            text_marker.lifetime = Duration(
                seconds=0.0).to_msg()  # 0 = Marker never expires
        else:
            text_marker.lifetime = Duration(
                seconds=lifetime).to_msg()  # in seconds

        # Set the timestamp
        text_marker.header.stamp = self.get_clock().now().to_msg()

        # Set the pose
        text_marker.pose = text_pose

        # Set marker size
        text_marker.scale = text_scale

        # Set marker color
        text_marker.color = self.getColor(color)

        text_marker.text = text

        return self.publishMarker(text_marker)


def pose_to_mat(pose: Pose):
    """
    Convert a ROS Pose msg to a 4x4 matrix.

    @param pose (ROS geometry_msgs.msg.Pose)

    @return mat 4x4 matrix (np.matrix)
    """

    mat = np.eye(4)
    mat[0:3, 0:3] = quaternion_to_rotation_matrix(pose.orientation)
    mat[0:3, 3] = np.asarray(
        [pose.position.x, pose.position.y, pose.position.z])

    return mat


def mat_to_pose(mat: Union[np.array, list[list]]):
    """
    Convert a homogeneous transformation matrix to a ROS Pose msg.

    @param mat 4x4 homogenous transform (np.matrix or np.ndarray)

    @return pose (ROS geometry_msgs.msg.Pose)
    """

    return Pose(position=Point(x=mat[0, 3], y=mat[1, 3], z=mat[2, 3]),
                orientation=roatoion_matrix_to_quaternion(mat))


def roatoion_matrix_to_quaternion(m):
    """
    Covert a rotation matrix into a quartenion.

    Input
    :param m: 3x3 rotation matrix

    Output
    :return: geometry_msgs/Quaternion

    credit: https://stackoverflow.com/questions/69482958/reimplement-eigen-rotation-matrix-conversion-to-quaternions-in-python
    """
    t = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if (t > 0):
        t = np.sqrt(t + 1)
        q[3] = 0.5 * t
        t = 0.5/t
        q[0] = (m[2, 1] - m[1, 2]) * t
        q[1] = (m[0, 2] - m[2, 0]) * t
        q[2] = (m[1, 0] - m[0, 1]) * t

    else:
        i = 0
        if (m[1, 1] > m[0, 0]):
            i = 1
        if (m[2, 2] > m[i, i]):
            i = 2
        j = (i+1) % 3
        k = (j+1) % 3

        t = np.sqrt(m[i, i] - m[j, j] - m[k, k] + 1)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (m[k, j] - m[j, k]) * t
        q[j] = (m[j, i] + m[i, j]) * t
        q[k] = (m[k, i] + m[i, k]) * t

    return Quaternion(x=q[0], y=[1], z=q[2], w=q[3])


def quaternion_to_rotation_matrix(q: Quaternion):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion.

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
            This rotation matrix converts a point in the local reference
            frame to a point in the global reference frame.

    """

    r00 = 2 * (q.w * q.w + q.x * q.x) - 1
    r01 = 2 * (q.x * q.y - q.w * q.z)
    r02 = 2 * (q.x * q.z + q.w * q.y)
    r10 = 2 * (q.x * q.y + q.w * q.z)
    r11 = 2 * (q.w * q.w + q.y * q.y) - 1
    r12 = 2 * (q.y * q.z - q.w * q.x)
    r20 = 2 * (q.x * q.z - q.w * q.y)
    r21 = 2 * (q.y * q.z + q.w * q.x)
    r22 = 2 * (q.w * q.w + q.z * q.z) - 1

    return np.array([[r00, r01, r02],
                     [r10, r11, r12],
                     [r20, r21, r22]])


def quaternion_multiply(q1: Quaternion, q0: Quaternion):
    return Quaternion(x=q1.x * q0.w + q1.y * q0.z - q1.z * q0.y + q1.w * q0.x,
                      y=-q1.x * q0.z + q1.y * q0.w + q1.z * q0.x + q1.w * q0.y,
                      z=q1.x * q0.y - q1.y * q0.x + q1.z * q0.w + q1.w * q0.z,
                      w=-q1.x * q0.x - q1.y * q0.y - q1.z * q0.z + q1.w * q0.w)


def point_rotation_by_quaternion(point: Point, q: Quaternion):
    '''
    q: qw, qx, qy, qz
    '''
    # r = [0]+point
    r = Quaternion(x=point.x, y=point.y, z=point.z, w=0.0)
    # q_conj = [q[0], -1*q[1], -1*q[2], -1*q[3]]
    q_re = quaternion_multiply(
        quaternion_multiply(q, r), quaternion_conjugate(q))
    return Point(x=q_re.x, y=q_re.y, z=q_re.z)


def add_two_points(p1: Point, p2: Point):
    return Point(x=p1.x+p2.x,
                 y=p1.y+p2.y,
                 z=p1.z+p2.z)


def quaternion_conjugate(q: Quaternion):
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)
