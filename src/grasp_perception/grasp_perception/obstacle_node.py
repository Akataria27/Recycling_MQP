#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy
import open3d
import pyrealsense2
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32


class RealSenseObstacleNode(Node):
    def __init__(self):
        super().__init__('realsense_obstacle_node')

        # IMPORTANT: Use a dedicated optical frame for the cloud
        self.declare_parameter('frame_id', 'camera_frame') 

        # RealSense Camera Setup
        self.pipeline = pyrealsense2.pipeline()
        self.config = pyrealsense2.config()
        self.config.enable_device('337122071438') # to obtain serial number: open terminal > rs-enumerate-devices
        self.config.enable_stream(pyrealsense2.stream.depth, 640, 480, pyrealsense2.format.z16, 30)
        self.config.enable_stream(pyrealsense2.stream.color, 640, 480, pyrealsense2.format.rgb8, 30)
        self.r = True
        # Start Camera
        try:
            profile = self.pipeline.start(self.config)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.get_logger().info(f"RealSense Connected. Scale: {self.depth_scale}")
            depth_sensor.set_option(pyrealsense2.option.enable_auto_exposure, 0)
            depth_sensor.set_option(pyrealsense2.option.exposure, 5000.0)

            # Wait...
            for _ in range(100):
                self.pipeline.wait_for_frames()

        except Exception as e:
            self.get_logger().fatal(f"Camera Connection Failed: {e}")
            raise e

        # Processing Blocks
        self.align = pyrealsense2.align(pyrealsense2.stream.color)
        self.pc_gen = pyrealsense2.pointcloud() 

        # --- ROS2 ---
        # My Subscription
        self.subtojesus = self.create_subscription(Float32MultiArray, '/coord', self.command_cb, 1)

        # My Publisher
        self.pub = self.create_publisher(PointCloud2, '/object_cloud', 1)
        self.pub1 = self.create_publisher(Float32, '/angle', 1)

        # My Timer
        self.timer = self.create_timer(1.0/30.0, self.timer_cb)

        self.prev = numpy.zeros((1, 10))

        # Coordinates baby!
        self.x_obj = 100
        self.y_obj = 100


    def command_cb(self, msg):
        print(float(f"{msg.data[0]:.5f}"), float(f"{msg.data[1]:.5f}"))
        self.x_obj = float(f"{msg.data[0]:.5f}")
        self.y_obj = float(f"{msg.data[1]:.5f}")

    def timer_cb(self):
        # 1. Get Frames

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame: 
            return

        # 2. Generate Pyrealsense2 Point Cloud
        points_rs = self.pc_gen.calculate(depth_frame)
        
        # 3. Convert to NumPy Array
        vtx = numpy.asanyarray(points_rs.get_vertices())
        points_np = vtx.view(numpy.float32).reshape(-1, 3)

        # 4. Remove Table
        min_depth = .2
        max_depth = 0.44 # we think at 42 cm it cant see table, try 43.5, 44, etc
        mask = (points_np[:, 2] >= min_depth) & (points_np[:, 2] <= max_depth) # creates a list of booleans
        points_np = points_np[mask] # numpy uses boolean indexing

        # 5. Square XY
        search = 0.05
        if len(points_np) > 0:
            mask = (points_np[:, 0] >= self.x_obj-search) & (points_np[:, 0] <= self.x_obj + search) # must use & instead of and
            points_np = points_np[mask]
        if len(points_np) > 0:
            mask = (points_np[:, 1] >= self.y_obj-search) & (points_np[:, 1] <= self.y_obj + search)
            points_np = points_np[mask]

        # 6. Minimum Z + Offset
        offset = 0.02
        if len(points_np) > 0:
            min_z = points_np[:, 2].min()
            mask = (points_np[:, 2] >= min_z) & (points_np[:, 2] <= min_z + offset)
            points_np = points_np[mask]

        if len(points_np) < 200: 
            return  
    
        # 7. Create Point Cloud In Open3D
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(points_np)

        xy = numpy.asarray(points_np, dtype=float)[:, :2]
        center = xy.mean(axis=0)
        centered = xy - center
        cov = numpy.cov(centered, rowvar=False)
        evals, evecs = numpy.linalg.eigh(cov)
        minor = evecs[:, 0]
        yaw_grip = numpy.arctan2(minor[1], minor[0])
        self.pub1.publish(Float32(data=yaw_grip))

        # 8. Publish Open3D Point Cloud
        self.publish_cloud(pcd)

    def publish_cloud(self, pcd):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('frame_id').value 
        
        points = numpy.asarray(pcd.points)

        if points.shape[0] == 0:
            print('\n There are no points in the cloud!!!!!! \n')
            return

        # Create and Publish Open3D Point Cloud Data As sensor_msgs_py.point_cloud2
        msg = pc2.create_cloud_xyz32(header, points)
        self.pub.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node() # call parent Node class orignal destroy_node function

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()