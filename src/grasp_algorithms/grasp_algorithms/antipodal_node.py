import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from grasp_interfaces.msg import GraspCandidate
from geometry_msgs.msg import Pose, Point, Quaternion
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String, Float32, Int32
import numpy

class AntipodalNode(Node):
    def __init__(self):
        super().__init__('antipodal_node')
        self.get_logger().info("Antipodal Node Started.")

        # Super Subscriptions
        self.sub = self.create_subscription(PointCloud2, '/object_cloud', self.callback, 1)
        self.subtojesus = self.create_subscription(String, '/command', self.command_cb, 1)
        self.anglesub = self.create_subscription(Float32, '/angle', self.angle_cb, 1)
        self.objSub = self.create_subscription(Int32, '/type', self.handle_type, 1)

        # Subscription Data
        self.search = "wait"
        self.angle = None
        self.objType = None

        # My Publisher
        self.pub = self.create_publisher(GraspCandidate, '/grasp_candidates', 1)
    
    def command_cb(self, msg):
        self.search = msg.data

    def angle_cb(self, msg):
        self.angle = msg.data

    def handle_type(self, msg):
        self.objType = msg.data

    # Main Callback
    def callback(self, msg):

        # 1. Read sensor_msgs_py.point_cloud2
        pointcloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw_pts = numpy.array([[p[0], p[1], p[2]] for p in pointcloud], dtype=numpy.float32)
        if self.objType == 2:
            mask = (raw_pts[:, 2] <= raw_pts[:,2].min() + 0.01)
        else:
            mask = (raw_pts[:, 2] <= raw_pts[:,2].min() + 0.05)
        raw_pts = raw_pts[mask]

        # 2. Define Camera Coordinates
        center_cam_x = numpy.mean(raw_pts[:, 0])
        center_cam_y = numpy.mean(raw_pts[:, 1])
        center_cam_z = raw_pts[:,2].min()

        # 3. Transform Camera Coordinates to Base Coordinates
        BASE_LINK_TO_TABLE = 0.098
        CAMERA_T0_TABLE = 0.47
        X_TRANSLATION = .37
        Y_TRANSLATION = .515
        Z_TRANSLATION = 0.81
        x = (center_cam_x + X_TRANSLATION) + 0.05
        y = -center_cam_y +  Y_TRANSLATION - 0.01
        top_z = Z_TRANSLATION -center_cam_z + .28

        self.get_logger().info(f"\n Before Transformation: \n x: {center_cam_x} \n y: {center_cam_y} \n top_z: {center_cam_z}")
        self.get_logger().warn(f"\n After Transformation: \n x: {x} \n y: {y} \n top_z: {top_z}")

        # 4. Define Grasp Candidate
        g = GraspCandidate()
        g.type = self.objType
        g.pose = Pose(position=Point(x=x, y=y, z=top_z), 
                      orientation=Quaternion(x=0.926, y=-0.378, z=-0.002, w=-0.001))
        if g.type == 2:
            g.angle = 0.0
        else:
            g.angle = self.angle
        
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