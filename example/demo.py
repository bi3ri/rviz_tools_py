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


"""
This is a demo of Rviz Tools for python which tests all of the
available functions by publishing lots of Markers in Rviz.
"""

# Python includes
import numpy as np
import threading

# ROS includes
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from rviz_tools_py.rviz_tools import RvizMarkers
from ament_index_python.packages import get_package_share_directory

# Initialize the ROS Node
rclpy.init()
node = rclpy.create_node('rviz_tools_py_demo')

# markers = rviz_tools_py.rviz_tools.RvizMarkers('/map', 'visualization_marker')
markers = RvizMarkers('/world', 'rviz_tools_py_demo')

thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()
rate = node.create_rate(1)

try:
    while rclpy.ok():

        # Axis:

        # Publish an axis using a numpy transform matrix
        mat = np.eye(4)
        axis_length = 0.4
        axis_radius = 0.05
        # pose, axis length, radius, lifetime
        markers.publishAxis(mat, axis_length, axis_radius)

        # Publish an axis using a ROS Pose Msg
        P = Pose(position=Point(x=2.0))
        axis_length = 0.4
        axis_radius = 0.05
        # pose, axis length, radius, lifetime
        markers.publishAxis(P, axis_length, axis_radius)

        # Line:

        # Publish a line between two ROS Point Msgs
        point1 = Point(x=-2.0, y=1.0)
        point2 = Point(x=2.0, y=1.0)
        width = 0.05
        # point1, point2, color, width, lifetime
        markers.publishLine(point1, point2, 'green', width, 5.0)

        # Publish a line between two ROS Poses
        P1 = Pose(position=Point(x=-2.0, y=1.1))
        P2 = Pose(position=Point(x=2.0, y=1.1))
        width = 0.02
        # point1, point2, color, width, lifetime
        markers.publishLine(P1, P2, 'red', width, 5.0)

        # Publish a line between two numpy transform matrices
        P1 = Point(x=-2.0, y=1.2)
        P2 = Point(x=2.0, y=1.2)

        width = 0.02
        # point1, point2, color, width, lifetime
        markers.publishLine(P1, P2, 'blue', width, 5.0)

        # Path:

        # Publish a path using a list of ROS Point Msgs
        path = []
        path.append(Point(x=0.0, y=-0.5, z=0.0))
        path.append(Point(x=1.0, y=-0.5, z=0.0))
        path.append(Point(x=1.5, y=-0.2, z=0.0))
        path.append(Point(x=2.0, y=-0.5, z=0.0))
        path.append(Point(x=2.5, y=-0.2, z=0.0))
        path.append(Point(x=3.0, y=-0.5, z=0.0))
        path.append(Point(x=4.0, y=-0.5, z=0.0))
        width = 0.02
        # path, color, width, lifetime
        markers.publishPath(path, 'orange', width, 5.0)

        # Plane / Rectangle:

        # Publish a rectangle between two points (thin, planar surface)
        # If the z-values are different, this will produce a cuboid
        point1 = Point(x=-1.0)
        point2 = Point(x=-2.0, y=-1.0)
        markers.publishRectangle(point1, point2, 'blue', 5.0)

        # Publish a rotated plane using a numpy transform matrix
        mat = np.eye(4)
        # rotation matrix 30deg y=axis
        R_y = [[0.9330127,  0.0669873,  0.3535534],
               [0.0669873,  0.9330127, -0.3535534],
               [-0.3535534,  0.3535534,  0.8660254]]

        T = np.asarray([-3, -1.5, 0])
        mat[0:3, 0:3] = R_y
        mat[0:3, 3] = T

        depth = 1.1
        width = 1.5
        # pose, depth, width, color, lifetime
        markers.publishPlane(mat, depth, width, 'purple', 5.0)

        # Publish a plane using a ROS Pose Msg
        P = Pose(position=Point(x=-3.0))
        depth = 1.3
        width = 1.3
        # pose, depth, width, color, lifetime
        markers.publishPlane(P, depth, width, 'brown', 5.0)

        # Polygon:

        # Publish a polygon using a ROS Polygon Msg
        polygon = Polygon()
        polygon.points.append(Point(x=0.0, y=-1.0))
        polygon.points.append(Point(x=0.0, y=-2.0))
        polygon.points.append(Point(x=-1.0, y=-2.0))
        polygon.points.append(Point(x=-1.0, y=-1.0))
        # path, color, width, lifetime
        markers.publishPolygon(polygon, 'red', 0.02, 5.0)

        # Text:

        # Publish some text using a ROS Pose Msg
        P = Pose(position=Point(x=3.0, y=-1.0))
        scale = Vector3(x=0.2, y=0.2, z=0.2)
        # pose, text, color, scale, lifetime
        markers.publishText(P, 'This is some text', 'white', scale, 5.0)

        # Arrow:

        # Publish an arrow using a numpy transform matrix
        mat = np.eye(4)
        mat[0:3, 3] = np.asarray([1, -2, 0])
        scale = Vector3(x=1.0, y=0.2, z=0.2)  # x=length, y=height, z=height
        # pose, color, scale, lifetime
        markers.publishArrow(mat, 'blue', scale, 5.0)

        # Publish an arrow using a ROS Pose Msg
        P = Pose(position=Point(x=1.0, y=-3.0))
        arrow_length = 2.0  # single value for length (height is relative)
        # pose, color, arrow_length, lifetime
        markers.publishArrow(P, 'pink', arrow_length, 5.0)

        # Cube / Cuboid:

        # Publish a cube using a numpy transform matrix
        mat = np.eye(4)
        mat[0:3, 3] = np.asarray([-3, 2.2, 0])
        cube_width = 0.5  # cube is 0.5x0.5x0.5
        # pose, color, cube_width, lifetime
        markers.publishCube(mat, 'green', cube_width, 5.0)

        # Publish a cube using a ROS Pose Msg
        P = Pose(position=Point(x=-2.0, y=2.2))
        cube_width = 0.6
        # pose, color, cube_width, lifetime
        markers.publishCube(P, 'blue', cube_width, 5.0)

        # Publish a cube using wrapper function publishBlock()
        P = Pose(position=Point(x=-1.0, y=2.2))
        cube_width = 0.7
        # pose, color, cube_width, lifetime
        markers.publishBlock(P, 'orange', cube_width, 5.0)

        # Publish a cuboid using a numpy transform matrix
        mat[0:3, 3] = np.asarray([0.6, 2.2, 0])
        scale = Vector3(x=1.5, y=0.2, z=0.2)
        # pose, color, scale, lifetime
        markers.publishCube(mat, 'yellow', scale, 5.0)

        # Publish a cuboid using a ROS Pose Msg
        P = Pose(position=Point(x=2.2, y=2.2))
        scale = Vector3(x=1.1, y=0.2, z=0.8)
        # pose, color, scale, lifetime
        markers.publishCube(P, 'brown', scale, 5.0)

        # List of cubes:

        # Publish a set of cubes using a list of ROS Point Msgs
        points = []
        z_height = 0.1
        points.append(Point(x=3.5+0*0.2, y=0.5, z=z_height))  # row 1
        points.append(Point(x=3.5+1*0.2, y=0.5, z=z_height))
        points.append(Point(x=3.5+2*0.2, y=0.5, z=z_height))
        points.append(Point(x=3.5+0*0.2, y=0.5+1*0.2, z=z_height))  # row 2
        points.append(Point(x=3.5+1*0.2, y=0.5+1*0.2, z=z_height))
        points.append(Point(x=3.5+2*0.2, y=0.5+1*0.2, z=z_height))
        points.append(Point(x=3.5+0*0.2, y=0.5+2*0.2, z=z_height))  # row 3
        points.append(Point(x=3.5+1*0.2, y=0.5+2*0.2, z=z_height))
        points.append(Point(x=3.5+2*0.2, y=0.5+2*0.2, z=z_height))
        points.append(Point(x=3.5+0*0.2, y=0.5+2*0.2,
                      z=z_height+0.2))  # 2nd layer
        diameter = 0.2-0.005
        # path, color, diameter, lifetime
        markers.publishCubes(points, 'red', diameter, 5.0)

        # Sphere:

        # Publish a sphere using a numpy transform matrix
        mat = np.eye(4)
        mat[0:3, 3] = np.asarray([-3, 3.2, 0.0])
        scale = Vector3(x=0.5, y=0.5, z=0.5)  # diameter
        color = [0.0, 1.0, 0.0]  # list of RGB values (green)
        # pose, color, scale, lifetime
        markers.publishSphere(mat, color, scale, 5.0)

        # Publish a sphere using a ROS Pose
        P = Pose(position=Point(x=-2.0, y=3.2))
        scale = Vector3(x=0.6, y=0.6, z=0.6)  # diameter
        color = (0.0, 0.0, 1.0)  # tuple of RGB values (blue)
        # pose, color, scale, lifetime
        markers.publishSphere(P, color, scale, 5.0)

        # Publish a sphere using a ROS Point
        point = Point(x=-1.0, y=3.2)
        scale = Vector3(x=0.7, y=0.7, z=0.7)  # diameter
        color = 'orange'
        # pose, color, scale, lifetime
        markers.publishSphere(point, color, scale, 5.0)

        # Publish a sphere by passing diameter as a float
        point = Point(y=3.2)
        diameter = 0.8
        # pose, color, diameter, lifetime
        markers.publishSphere(point, 'yellow', diameter, 5.0)

        # Publish a sphere with higher render quality (this is one sphere in a SPHERE_LIST)
        point = Point(x=1.0, y=3.2)
        diameter = 0.9
        # pose, color, scale, lifetime
        markers.publishSphere2(point, 'brown', diameter, 5.0)

        # List of spheres:

        # Publish a set of spheres using a list of ROS Point Msgs
        points = []
        points.append(Point(x=-3.0, y=4.0))
        points.append(Point(x=-2.0, y=4.0))
        points.append(Point(x=-1.0, y=4.0))
        points.append(Point(x=0.0, y=4.0))
        diameter = 0.3
        # path, color, diameter, lifetime
        markers.publishSpheres(points, 'white', diameter, 5.0)

        # Publish a set of spheres using a list of ROS Pose Msgs
        poses = []
        poses.append(Pose(position=Point(x=1.0, y=4.0)))
        poses.append(Pose(position=Point(x=2.0, y=4.0)))
        poses.append(Pose(position=Point(x=3.0, y=4.0)))
        scale = Vector3(x=0.5, y=0.5, z=0.5)  # diameter
        # path, color, scale, lifetime
        markers.publishSpheres(poses, 'blue', scale, 5.0)

        # Publish a set of spheres using a list of numpy transform matrices
        poses = []
        poses.append(Pose(position=Point(x=4.0, y=4.0)))
        poses.append(Pose(position=Point(x=5.0, y=4.0)))
        diameter = 0.6
        # path, color, scale, lifetime
        markers.publishSpheres(poses, 'green', diameter, 5.0)

        # Cylinder:

        # Publish a cylinder using a numpy transform matrix
        mat = np.eye(4)
        mat[0:3, 3] = np.asarray([-3, 5, 0])
        # pose, color, height, radius, lifetime
        markers.publishCylinder(mat, 'green', 1.0, 0.5, 5.0)

        # Publish a cylinder using a ROS Pose
        P = Pose(position=Point(x=-2.0, y=5.0))
        # pose, color, height, radius, lifetime
        markers.publishCylinder(P, 'blue', 1.0, 0.5, 5.0)

        # Publish a cylinder of a random color (method #1)
        P = Pose(position=Point(x=-1.0, y=5.0))
        # pose, color, height, radius, lifetime
        markers.publishCylinder(P, markers.getRandomColor(), 1.0, 0.5, 5.0)

        # Publish a cylinder of a random color (method #2)
        P = Pose(position=Point(y=5.0))
        # pose, color, height, radius, lifetime
        markers.publishCylinder(P, 'random', 1.0, 0.5, 5.0)

        # Model mesh:

        # Publish STL mesh of box, colored green
        mat = np.eye(4)
        mat[0:3, 3] = np.asarray([3, 1, 0])
        scale = Vector3(x=1.5, y=1.5, z=1.5)
        # mesh_file1 = "package://rviz_tools_py/meshes/box_mesh.stl"
        mesh_file1 = "$(find rviz_tools_py)/meshes/box_mesh.stl"
        # pose, mesh_file_name, color, mesh_scale, lifetime
        markers.publishMesh(Pose(position=Point(x=3.0, y=1.0, z=0.0)),
                            mesh_file1, 'lime_green', scale, 5.0)

        # Display STL mesh of bottle, re-scaled to smaller size
        P = Pose(position=Point(x=4.0, y=1.0))
        scale = Vector3(x=0.6, y=0.6, z=0.6)
        rviz_tools_py_pkg_path = get_package_share_directory('rviz_tools_py')

        mesh_file2 = rviz_tools_py_pkg_path + "/meshes/fuze_bottle_collision.stl"
        # pose, mesh_file_name, color, mesh_scale, lifetime
        markers.publishMesh(P, mesh_file2, 'blue', scale, 5.0)

        # Display collada model with original texture (no coloring)
        P = Pose(position=Point(x=5.0, y=1.0))
        mesh_file3 = "$(find rviz_tools_py)/meshes/fuze_bottle_visual.dae"
        mesh_scale = 4.0
        # pose, mesh_file_name, color, mesh_scale, lifetime
        markers.publishMesh(P, mesh_file3, None, mesh_scale, 5.0)

        rate.sleep()

except KeyboardInterrupt:
    pass
