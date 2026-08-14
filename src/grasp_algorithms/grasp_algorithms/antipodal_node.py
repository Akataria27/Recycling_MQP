#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from grasp_interfaces.msg import GraspCandidate
from geometry_msgs.msg import Pose
import numpy
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String
from std_msgs.msg import Float32

class AntipodalNode(Node):
    def __init__(self):
        super().__init__('antipodal_node')

        #Super Subscriptions
        self.sub = self.create_subscription(PointCloud2, '/object_cloud', self.cb, 1)
        self.subtojesus = self.create_subscription(String, '/command', self.command_cb, 1)
        self.anglesub = self.create_subscription(Float32, '/angle', self.angle_cb, 1)
        self.search = "wait"
        self.angle = None

        # My Publisher
        self.pub = self.create_publisher(GraspCandidate, '/grasp_candidates', 1)
        
        self.get_logger().info("Antipodal Node Started (Targeting Object TOP).")
    
    def command_cb(self, msg):
        self.search = msg.data

    def angle_cb(self, msg):
        self.angle = msg.data

    def cb(self, msg):
        if self.search != "start": 
            return

        gen = pc2.read_points(cloud=msg, field_names=("x", "y", "z"), skip_nans=True)

        raw_pts = numpy.array([[p[0], p[1], p[2]] for p in gen], dtype=numpy.float32)
        self.get_logger().info(f'NUMBER OF ROWS IN DATA: {len(raw_pts)}')

        # Camera Coordinates
        center_cam_x = ((raw_pts[:, 0]).max() - (raw_pts[:, 0]).min()) / 2  + (raw_pts[:, 0]).min()
        center_cam_y = ((raw_pts[:, 1]).max() - (raw_pts[:, 1]).min()) / 2 + (raw_pts[:, 1]).min()
        center_cam_z = raw_pts[:,2].min()

        # Coordinate Transformation
        y = (center_cam_x - .03) * -1.0 + 0.02
        x = -1*center_cam_y + .35
        top_z = .6 - center_cam_z + .265 +.015

        self.get_logger().info(f"\n Target Pose: \n x: {x} \n y: {y} \n top_z: {top_z}\n")

        pose = Pose()

        pose.position.x = x
        pose.position.y = y 
        pose.position.z = top_z
        pose.orientation.x = .926
        pose.orientation.y = -.378
        pose.orientation.z = -0.002
        pose.orientation.w = -0.001

        g = GraspCandidate()
        g.pose = pose
        g.angle = self.angle
        
        self.pub.publish(g)
        self.get_logger().info(f"Top-Down Grasp Published. Locking.")
        
        self.search = "wait"

def main(args=None):
    rclpy.init(args=args)
    node = AntipodalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()