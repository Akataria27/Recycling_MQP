import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import numpy
import open3d
import pyrealsense2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Float32, Float32MultiArray
from sensor_msgs.msg import Image



class Camera(Node):
    def __init__(self):
        super().__init__('camera')

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
        self.depth_pub = self.create_publisher(Image, '/depth_frame',1)
        self.color_pub = self.create_publisher(Image, '/color_frame',1)

        self.timer = self.create_timer(1.0/30.0, self.timer_cb)

    def timer_cb(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        self.depth_pub