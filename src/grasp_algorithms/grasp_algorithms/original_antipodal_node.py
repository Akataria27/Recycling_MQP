#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from grasp_interfaces.msg import GraspCandidate, GraspCandidateArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener

class AntipodalNode(Node):
    def __init__(self):
        super().__init__('antipodal_node')
        self.sub = self.create_subscription(PointCloud2, '/object_cloud', self.cb, 1)
        self.pub = self.create_publisher(GraspCandidateArray, '/grasp_candidates', 1)
        self.grasp_found = False 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- TUNABLE OFFSETS ---
        # 1. HEIGHT (Z): Distance from the wrist to the object Top.
        # Since we are now detecting the Top_Z, -0.15 (15cm) is usually a good hovering distance
        # for a Panda gripper (fingers are ~10cm long).
        self.z_offset = -0.15
        
        # 2. LEFT/RIGHT (Y): Shift Right/Left relative to gripper
        self.y_offset = 0.0
        
        # 3. FORWARD/BACK (X): Shift Forward/Back relative to gripper
        self.x_offset = 0

        self.get_logger().info("Antipodal Node Started (Targeting Object TOP).")

    def get_transform(self, target_frame, source_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time())
            return t
        except Exception as e:
            self.get_logger().warn(f"TF Lookup failed: {e}")
            return None

    def transform_points(self, points, trans):
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        t = np.array([tx, ty, tz])

        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w
        r = R.from_quat([qx, qy, qz, qw])
        rot_mat = r.as_matrix()

        return points @ rot_mat.T + t

    def check_reachability(self, x, y, z):
        if z < 0.02: 
            self.get_logger().error(f"REJECTED: Target Z ({z:.2f}) is too close to ground!")
            return False
        
        radius = np.sqrt(x**2 + y**2 + z**2)
        if radius > 0.80:
            self.get_logger().error(f"REJECTED: Too far ({radius:.2f}m)!")
            return False

        if radius < 0.20:
            self.get_logger().error(f"REJECTED: Too close to base!")
            return False
        return True

    def cb(self, msg):
        if self.grasp_found: return

        target_frame = "panda_link0"

        if not hasattr(self, "_printed_cloud_frame"):
            self.get_logger().info(
                f"PointCloud frame: {msg.header.frame_id}  ->  target frame: {target_frame}"
            )
            self._printed_cloud_frame = True

        trans = self.get_transform(target_frame, msg.header.frame_id)
        if trans is None: return

        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw_pts = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float32)
        if raw_pts.shape[0] < 50: return

        pts = self.transform_points(raw_pts, trans)
        
       #--- CHANGED: FIND TOP OF OBJECT ---
        #nstead of mean() for everything, we split it up.
        # center_x = np.median(pts[:, 0])# Keep X centered
        # center_y = np.median(pts[:, 1]) # Keep Y centered
        # top_z = pts[:, 2].max() + 0.1 # Find the HIGHEST point (The Top)

        center_x = (pts[:, 0]).mean() 
        center_y = (pts[:, 1]).mean() 
        top_z = pts[:, 2].max() + 0.1 # Find the HIGHEST point (The Top)

        center_x = 0.5 # Keep X centered
        center_y = 0 # Keep Y centered
        top_z = 0.29 # Find the HIGHEST point (The Top)

        # We create the "anchor" point at the very top center of the object
        anchor_point = np.array([center_x, center_y, top_z]) # -1 is to flip y axis direction

        # PCA for orientation (Standard)
        cov = np.cov((pts - pts.mean(axis=0)).T)
        eigvals, eigvecs = np.linalg.eig(cov)
        sort_indices = np.argsort(eigvals)
        
        # Strict Top-Down Orientation
        grasp_z = np.array([0.0, 0.0, -1.0])

        minor_axis_3d = eigvecs[:, sort_indices[0]] 
        grasp_y = np.array([minor_axis_3d[0], minor_axis_3d[1], 0.0])

        if np.linalg.norm(grasp_y) < 0.1:
            middle_axis_3d = eigvecs[:, sort_indices[1]]
            grasp_y = np.array([middle_axis_3d[0], middle_axis_3d[1], 0.0])

        grasp_y = grasp_y / np.linalg.norm(grasp_y)
        grasp_x = np.cross(grasp_y, grasp_z)
        grasp_x = grasp_x / np.linalg.norm(grasp_x)
        grasp_y = np.cross(grasp_z, grasp_x)

        rot_matrix = np.column_stack((grasp_x, grasp_y, grasp_z))
        r = R.from_matrix(rot_matrix)
        quat = r.as_quat()

        # Apply Offsets to the Anchor Point (The Top)
        offset_local = np.array([self.x_offset, self.y_offset, self.z_offset])
        offset_world = r.apply(offset_local)
        
        final_pos = anchor_point # + offset_world
        print(offset_world)
        self.get_logger().info(f"Top Z Found: {top_z:.3f}")
        self.get_logger().info(f"Target Pose: {final_pos}")

        if not self.check_reachability(final_pos[0], final_pos[1], final_pos[2]):
            return

        pose = PoseStamped()
        pose.header.frame_id = target_frame 
        pose.header.stamp = self.get_clock().now().to_msg() 
        
        pose.pose.position.x = float(final_pos[0])
        pose.pose.position.y = float(final_pos[1])
        pose.pose.position.z = float(final_pos[2])
        # pose.pose.orientation.x = quat[0]
        # pose.pose.orientation.y = quat[1]
        # pose.pose.orientation.z = quat[2]
        # pose.pose.orientation.w = quat[3]

        # final_pos[1] = final_pos[1] *-1
        # projected = pts @ grasp_y
        # width = float(projected.max() - projected.min())
        width = 0.04

        g = GraspCandidate()
        g.header = pose.header
        g.pose = pose.pose
        g.width = width
        g.score = 1.0
        
        arr = GraspCandidateArray()
        arr.candidates = [g]
        self.pub.publish(arr)
        
        self.get_logger().info(f"Top-Down Grasp Published. Locking.")
        self.grasp_found = True

def main(args=None):
    rclpy.init(args=args)
    node = AntipodalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()