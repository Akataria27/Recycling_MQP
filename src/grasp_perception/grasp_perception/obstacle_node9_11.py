import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import numpy
import open3d
import pyrealsense2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Float32, Float32MultiArray

MIN_DEPTH = .20
MAX_DEPTH = 0.80 # we think at .80 m it cant see table, try more values

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

        self.spatial = pyrealsense2.spatial_filter()
        self.temporal = pyrealsense2.temporal_filter()
        self.holefill = pyrealsense2.hole_filling_filter()

        # Start Camera
        try:
            profile = self.pipeline.start(self.config)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.get_logger().info(f"RealSense Connected. Scale: {self.depth_scale}")
            depth_sensor.set_option(pyrealsense2.option.enable_auto_exposure, 0)
            depth_sensor.set_option(pyrealsense2.option.exposure, 4000.0)

            # Wait...
            for _ in range(100):
                self.pipeline.wait_for_frames()

        except Exception as e:
            self.get_logger().fatal(f"Camera Connection Failed: {e}")
            raise e

        # Processing Blocks
        self.align = pyrealsense2.align(pyrealsense2.stream.color)
        self.pointcloud = pyrealsense2.pointcloud() 

        # --- ROS2 ---
        # My Subscription
        self.subtojesus = self.create_subscription(Float32MultiArray, '/coord', self.command_cb, 1)

        # My Publisher
        self.pub = self.create_publisher(PointCloud2, '/object_cloud', 1)
        self.pub1 = self.create_publisher(Float32, '/angle', 1)

        # My Timer
        self.timer = self.create_timer(1.0/30.0, self.timer_cb)

        # Coordinates
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
            self.get_logger().fatal('not good')
            return
        
        depth_frame = self.temporal.process(depth_frame)
        depth_frame = self.spatial.process(depth_frame)
        # depth_frame = self.holefill.process(depth_frame)

        # 2. Generate Pyrealsense2 Point Cloud
        points = self.pointcloud.calculate(depth_frame)

        if points.size() == 0:
            print('0')
            return
        
        # 3. Convert to NumPy Array
        vtx = numpy.asanyarray(points.get_vertices())
        points_np = vtx.view(numpy.float32).reshape(-1, 3)

        # 4. Z Clamp
        mask = (points_np[:, 2] >= MIN_DEPTH) & (points_np[:, 2] <= MAX_DEPTH)
        points_np = points_np[mask]

        # 4. X Clamp
        mask = (points_np[:, 0] >= -0.23) & (points_np[:, 0] <= 0.37)
        points_np = points_np[mask]

        if len(points_np) <= 1000: 
            return 
        
        pcd = open3d.t.geometry.PointCloud()
        pcd.point.positions = open3d.core.Tensor(points_np)

        # DENSITY SCAN
        # labels = pcd.cluster_dbscan(eps=0.008, min_points=105)
        # labels = labels.numpy()

        labels = pcd.cluster_dbscan(eps=0.009, min_points=105)
        labels = labels.numpy()

        # this selects by the biggest object (RAGGY SHAGGY SPECIAL)
        # should change to select closests to object coordinate
        if labels.max() >= 0:
            counts = numpy.bincount(labels[labels >= 0])
            biggest = numpy.argmax(counts)
            keep = numpy.where(labels == biggest)[0]
            pcd = pcd.select_by_index(keep)
        else:
            print('no clusters') #shouldnt happen bruh
            return
        
        # Principle Component Analysis (PCA)
        pcd_pts = pcd.point.positions.numpy()
        xy = numpy.asarray(pcd_pts, dtype=float)[:, :2]
        center = xy.mean(axis=0)
        centered = xy - center
        cov = numpy.cov(centered, rowvar=False)
        evals, evecs = numpy.linalg.eigh(cov)
        minor = evecs[:, 0]
        yaw_grip = numpy.arctan2(minor[1], minor[0])
        self.pub1.publish(Float32(data=yaw_grip))

        # 8. Ros2 Publish
        self.publish_cloud(pcd)

    def publish_cloud(self, pcd):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('frame_id').value

        points = pcd.point.positions.numpy()

        if points.shape[0] == 0:
            print('\n There are no points in the cloud!!!!!! \n')
            return

        # Create and Publish Open3D Point Cloud Data As sensor_msgs_py.point_cloud2
        cluster1 = pc2.create_cloud_xyz32(header, points)
        self.pub.publish(cluster1)

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