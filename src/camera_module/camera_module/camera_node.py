#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from camera_interfaces.srv import CameraSrv  # Custom service definition for capturing frames
import pyrealsense2 as rs
from cv_bridge import CvBridge
import numpy as np
import cv2

class RealSenseServiceNode(Node):
    """
    A ROS2 node that provides a service to capture RGB and Depth frames from an Intel RealSense camera.
    """
    def __init__(self):
        # 1. Initialize the Node with a name
        super().__init__('realsense_service_node')

        # 2. Initialize the camera
        self.pipeline = None
        if not self.initialize_camera():
            self.get_logger().fatal("Failed to initialize camera. Shutting down node.")
            raise RuntimeError("Camera initialization failed.")

        # Bridge to convert between OpenCV images and ROS Image messages
        self.bridge = CvBridge()

        # 3. Create the service
        self.service = self.create_service(CameraSrv, 'camera_service', self.capture_frames_callback)
        self.get_logger().info("RealSense service is ready to capture images.")

    def initialize_camera(self):
        """
        Initializes the RealSense camera pipeline.

        :return: True if the camera is initialized successfully, False otherwise.
        """
        try:
            # Create a RealSense pipeline and configure the streams
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)  # RGB stream
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream
            
            # Start the pipeline with the configured streams
            profile = self.pipeline.start(config)

            # Align depth frames to the color frames
            self.align = rs.align(rs.stream.color)
            
            # Retrieve the depth scale (used to convert depth values to meters)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            self.get_logger().info("Camera pipeline started successfully.")
            return True
        except Exception as e:
            # Log any errors and stop the pipeline if initialization fails
            self.get_logger().error(f"Error while initializing camera: {e}")
            if self.pipeline:
                self.pipeline.stop()
            return False

    def capture_frames_callback(self, request, response):
        """
        Callback function to handle the service request for capturing RGB and Depth frames.

        :param request: The service request object.
        :param response: The service response object.
        :return: The populated response object containing the captured frames and timestamp.
        """
        self.get_logger().info('Incoming request for RGB and Depth frames.')
        try:
            max_attempts = 10  # Maximum number of attempts to capture valid frames
            for attempt in range(max_attempts):
                # Wait for a new set of frames from the camera
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)  # Align depth to color frames
                
                # Retrieve the aligned depth and color frames
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if aligned_depth_frame and color_frame:
                    # Convert the depth frame to a NumPy array and scale it to meters
                    depth_image_np = np.asanyarray(aligned_depth_frame.get_data()) * self.depth_scale
                    # Convert the color frame to a NumPy array
                    color_image_np = np.asanyarray(color_frame.get_data())

                    # Convert NumPy arrays to ROS Image messages
                    response.rgb_image = self.bridge.cv2_to_imgmsg(color_image_np, encoding="rgb8")
                    response.depth_image = self.bridge.cv2_to_imgmsg(depth_image_np, encoding="32FC1")  # Use 32FC1 for depth data

                    # 4. Use the node's clock for timestamping
                    response.timestamp = self.get_clock().now().to_msg()
                    
                    self.get_logger().info(f"RGB and Depth pair collected at timestamp: {response.timestamp.sec}.{response.timestamp.nanosec}")
                    return response

                # Log a warning if valid frames are not received
                self.get_logger().warn(f"Attempt {attempt + 1}/{max_attempts}: Valid frames not received. Retrying...")

            # Log an error if valid frames are not captured after max_attempts
            self.get_logger().error(f"Failed to capture valid frames after {max_attempts} attempts.")
            # Return the response object, even if empty, as the service signature requires it
            return response

        except Exception as e:
            # Log any errors that occur during frame capture
            self.get_logger().error(f"Error while capturing frames: {e}")
            return response

    def on_shutdown(self):
        """
        Called upon node destruction to clean up resources.
        Stops the RealSense camera pipeline if it is running.
        """
        if self.pipeline:
            self.get_logger().info("Stopping camera pipeline.")
            self.pipeline.stop()

def main(args=None):
    """
    Main entry point for the ROS2 node.
    Initializes the node, starts the service, and handles shutdown gracefully.
    """
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    node = None
    try:
        # Create and spin the RealSenseServiceNode
        node = RealSenseServiceNode()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt) as e:
        # Handle exceptions during node execution
        if isinstance(e, RuntimeError):
            print(f"Node failed to start: {e}")  # Logger is not available if __init__ fails
        pass  # Handle shutdown gracefully
    finally:
        # Ensure the node is properly destroyed and ROS2 is shut down
        if node:
            node.on_shutdown()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()