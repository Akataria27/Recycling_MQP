#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

class RealSenseObstacleNode(Node):
    def __init__(self):
        super().__init__('realsense_obstacle_node')

        # --- Parameters ---
        self.declare_parameter('voxel_size', 0.005)     # 5mm (High detail)
        self.declare_parameter('z_min', 0.10)           # 10cm min dist
        self.declare_parameter('z_max', 1.0)            # 1.0m max dist (Limit this to just the table range)
        self.declare_parameter('ransac_thresh', 0.01)   # 1cm plane threshold
        self.declare_parameter('device_id', '')         # Empty = Auto-detect first camera
        
        # IMPORTANT: Use a dedicated optical frame for the cloud
        self.declare_parameter('frame_id', 'camera_frame') 

        # --- RealSense Setup ---
        self.pipeline = rs.pipeline()
        self.config = rs.config()
         
        device_id = self.get_parameter('device_id').value
        if device_id:
            self.config.enable_device(str(device_id))

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        # Start Camera
        try:
            profile = self.pipeline.start(self.config)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.get_logger().info(f"RealSense Connected. Scale: {self.depth_scale}")
        except Exception as e:
            self.get_logger().fatal(f"Camera Connection Failed: {e}")
            raise e

        # Processing Blocks
        self.align = rs.align(rs.stream.color)
        self.pc_gen = rs.pointcloud() 
        
        self.pub = self.create_publisher(PointCloud2, '/object_cloud', 1)
        self.timer = self.create_timer(1.0/30.0, self.timer_cb)

    def timer_cb(self):
        # 1. Get Frames
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame: return

        # 2. Generate Point Cloud
        points_rs = self.pc_gen.calculate(depth_frame)

        # 3. Convert to NumPy Array
        if points_rs.size() == 0: return

        vtx = np.asanyarray(points_rs.get_vertices())
        points_np = vtx.view(np.float32).reshape(-1, 3)

        # 4. Pre-Filter: Remove Zero/Infinity
        valid_mask = (points_np[:, 2] > 0) & (np.isfinite(points_np[:, 2]))
        points_np = points_np[valid_mask]
        if points_np.shape[0] < 100: return

        # 5. Open3D Processing
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)

        # A. Voxel Downsample
        voxel_size = self.get_parameter('voxel_size').value
        pcd = pcd.voxel_down_sample(voxel_size)

        # B. Passthrough Filter (Z-Range)
        pts = np.asarray(pcd.points)
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value

        mask = (pts[:, 2] > z_min) & (pts[:, 2] < z_max)
        pcd = pcd.select_by_index(np.where(mask)[0])
        if len(pcd.points) < 50: return

        # C. RANSAC Plane Removal (Remove Table)
        dist_thresh = self.get_parameter('ransac_thresh').value

        plane_model, inliers = pcd.segment_plane(distance_threshold=dist_thresh,
                                                 ransac_n=3,
                                                 num_iterations=1000)

        objects = pcd.select_by_index(inliers, invert=True)
        if len(objects.points) < 20: return

        # D. Statistical Outlier Removal (kill floating noise)
        objects, _ = objects.remove_statistical_outlier(nb_neighbors=20,
                                                        std_ratio=2.0)
        if len(objects.points) < 20: return

        # E. Keep only the largest cluster (the actual object)
        labels = np.array(objects.cluster_dbscan(eps=voxel_size * 4,
                                                  min_points=10))
        if labels.max() >= 0:
            counts = np.bincount(labels[labels >= 0])
            biggest = np.argmax(counts)
            keep = np.where(labels == biggest)[0]
            objects = objects.select_by_index(keep)

        # 6. Publish
        self.publish_cloud(objects)

    def publish_cloud(self, pcd):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        # Use the parameter frame_id
        header.frame_id = self.get_parameter('frame_id').value 
        
        points = np.asarray(pcd.points)

        if points.shape[0] == 0: return

        msg = pc2.create_cloud_xyz32(header, points)

        self.pub.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

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