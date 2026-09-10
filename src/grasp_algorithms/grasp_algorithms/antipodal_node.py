import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from grasp_interfaces.msg import GraspCandidate
from geometry_msgs.msg import Pose, Point, Quaternion
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String, Float32
import numpy

class AntipodalNode(Node):
    def __init__(self):
        super().__init__('antipodal_node')
        self.get_logger().info("Antipodal Node Started.")

        # Super Subscriptions
        self.sub = self.create_subscription(PointCloud2, '/object_cloud', self.callback, 1)
        self.subtojesus = self.create_subscription(String, '/command', self.command_cb, 1)
        self.anglesub = self.create_subscription(Float32, '/angle', self.angle_cb, 1)

        # Subscription Data
        self.search = "wait"
        self.angle = None

        # My Publisher
        self.pub = self.create_publisher(GraspCandidate, '/grasp_candidates', 1)
    
    def command_cb(self, msg):
        self.search = msg.data

    def angle_cb(self, msg):
        self.angle = msg.data

    # Main Callback
    def callback(self, msg):

        if self.search != "start": 
            return

        # 1. Read sensor_msgs_py.point_cloud2
        pointcloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw_pts = numpy.array([[p[0], p[1], p[2]] for p in pointcloud], dtype=numpy.float32)

        self.get_logger().fatal(f'NUMBER OF ROWS IN DATA: {len(raw_pts)}')

        # 2. Define Camera Coordinates
        center_cam_x = ((raw_pts[:, 0]).max() + (raw_pts[:, 0]).min()) / 2
        center_cam_y = ((raw_pts[:, 1]).max() + (raw_pts[:, 1]).min()) / 2
        center_cam_z = raw_pts[:,2].min()

        # 3. Transform Camera Coordinates to Base Coordinates
        BASE_LINK_TO_TABLE = 0.098
        CAMERA_T0_TABLE = 0.47
        X_TRANSLATION = .84 # used to be .35
        Y_TRANSLATION = .37
        Z_TRANSLATION = None
        x = (center_cam_x + X_TRANSLATION) + 0.015
        y = center_cam_y + Y_TRANSLATION
        top_z = .9 - center_cam_z + .265 + .015

        self.get_logger().info(f"\n Before Transformation: \n x: {center_cam_x} \n y: {center_cam_y} \n top_z: {center_cam_z}")
        self.get_logger().warn(f"\n After Transformation: \n x: {x} \n y: {y} \n top_z: {top_z}")

        # 4. Define Grasp Candidate
        g = GraspCandidate()
        g.pose = Pose(position=Point(x=x, y=y, z=top_z), 
                      orientation=Quaternion(x=0.926, y=-0.378, z=-0.002, w=-0.001))
        g.angle = self.angle
        g.type = 'bottle'
        
        self.pub.publish(g)
        self.get_logger().info(f"Grasp Candidate Is Published.")

        # 5. End State
        self.search = "wait"

def main(args=None):
    rclpy.init(args=args)
    node = AntipodalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()