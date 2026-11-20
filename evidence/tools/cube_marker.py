#!/usr/bin/env python3
"""Publish the target cube as an RViz marker in base_link (latched).

The marker is drawn purely from the cube's known position and TF. In the RViz
Camera display it is rendered from the camera's TF pose over the real image,
so it only lines up with the red cube if TF and the intrinsics are right.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker

rclpy.init()
node = Node("evidence_cube_marker")
pub = node.create_publisher(Marker, "/evidence/cube_marker",
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
m = Marker()
m.header.frame_id = "base_link"
m.ns, m.id, m.type, m.action = "cube", 0, Marker.CUBE, Marker.ADD
m.pose.position.x, m.pose.position.y, m.pose.position.z = 0.5, 0.0, 0.2
m.pose.orientation.w = 1.0
m.scale.x = m.scale.y = m.scale.z = 0.05
m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.2, 0.55
def tick():
    pub.publish(m)  # stamp 0 = use the latest TF; works with sim time
node.create_timer(0.5, tick)
rclpy.spin(node)
