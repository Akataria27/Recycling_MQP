#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from grasp_interfaces.msg import GraspCandidateArray
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs 
from moveit_msgs.action import MoveGroup
try:
    from franka_msgs.action import Grasp as FrankaGrasp
except ImportError:
    from control_msgs.action import GripperCommand as FrankaGrasp

import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class PandaGraspExecutor(Node):
    def __init__(self):
        super().__init__('panda_grasp_executor')
        self.sub = self.create_subscription(GraspCandidateArray, '/grasp_candidates', self.callback, 1)
        self.move_action_client = ActionClient(self, MoveGroup, 'move_action')
        self.gripper_client = ActionClient(self, FrankaGrasp, '/panda_gripper/grasp') 

        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        
        self.grasping_active = False 
        
        # --- CONFIGURATION ---
        # Must match the z_offset in your AntipodalNode!
        self.approach_distance = -0.15
        
        self.cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            GraspCandidateArray, '/grasp_candidates', self.callback, 1,
            callback_group=self.cb_group)
        self.move_action_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self.cb_group)
        self.gripper_client = ActionClient(
            self, FrankaGrasp, '/panda_gripper/grasp', callback_group=self.cb_group)

        self.get_logger().info("Grasp Executor Ready. Waiting for candidates...")
        

    def callback(self, msg: GraspCandidateArray):
        if self.grasping_active: return
        if len(msg.candidates) == 0: return

        self.grasping_active = True
        self.get_logger().info("Candidate received! Locking executor...")

        best = max(msg.candidates, key=lambda g: g.score)
        target_frame = 'panda_link0' 

        try:
            if not self.tfbuf.can_transform(target_frame, best.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().warn("Waiting for TF...")
                self.grasping_active = False 
                return

            p_stamped = PoseStamped()
            p_stamped.header = best.header
            p_stamped.pose = best.pose
            
            # This is the HOVER POSE (approx 15cm above object)
            hover_pose = self.tfbuf.transform(p_stamped, target_frame)
            
        except Exception as e:
            self.get_logger().error(f"TF Error: {e}")
            self.grasping_active = False
            return

        # --- 1. MOVE TO PRE-GRASP (HOVER) ---
        self.get_logger().info(f"1. Moving to Pre-Grasp (Z={hover_pose.pose.position.z:.3f})...")
        if not self.move_to_pose(hover_pose): 
            print("im stinky")
            self.grasping_active = False
            return

        
        drop = PoseStamped()
        drop.header.frame_id = 'panda_link0'
        drop.pose.position.x = hover_pose.pose.position.x 
        drop.pose.position.y = hover_pose.pose.position.y
        drop.pose.position.z = hover_pose.pose.position.z - 0.2 # this num changes how low the pick pose is
        drop.pose.orientation.x = .921
        drop.pose.orientation.y = -.390
        drop.pose.orientation.z = -0.001
        drop.pose.orientation.w = 0.002
        self.move_to_pose(drop)

 
        # --- 2. OPEN GRIPPER ---
        # Ensure gripper is open before descending
        # (Assuming you have a moveit/action for open, or just use grasp with large width)
        # self.trigger_gripper(width=0.08) 

        # --- 3. DESCEND TO GRASP ---
        self.get_logger().info("2. Descending to Grasp...")
        
        grasp_pose = PoseStamped()
        grasp_pose.header = hover_pose.header
        grasp_pose.pose = hover_pose.pose
        
        # CRITICAL FIX: Subtract the offset to go DOWN to the object
        # We go slightly lower (0.005) to ensure fingers surround the top surface
        grasp_pose.pose.position.z -= (self.approach_distance + 0.005)

  
        # --- 4. CLOSE GRIPPER ---
        self.get_logger().info("3. Closing Gripper...")
        self.trigger_gripper(0.04)
        # self.trigger_gripper(width=best.width - 0.005) # Close slightly tighter than object USES CAMERA
        

        # --- 5. LIFT UP ---
        # self.get_logger().info("4. Lifting...")
        # lift_pose = PoseStamped()
        # lift_pose.header = hover_pose.header
        # lift_pose.pose = hover_pose.pose

        lift = PoseStamped()
        lift.header.frame_id = 'panda_link0'
        lift.pose.position.x = hover_pose.pose.position.x
        lift.pose.position.y = hover_pose.pose.position.y
        lift.pose.position.z = hover_pose.pose.position.z + 0.05
        lift.pose.orientation.x = .921
        lift.pose.orientation.y = -.390
        lift.pose.orientation.z = -0.001
        lift.pose.orientation.w = 0.002

        self.move_to_pose(lift)

        self.get_logger().info("Grasp Complete. Unlocking in 5s...")

        self.trigger_gripper(0.12)


        #BACK TO READY POS
        ready_joints = {
        'panda_joint1': 0.0,
        'panda_joint2': -0.785398163,
        'panda_joint3': 0.0,
        'panda_joint4': -2.356194490,
        'panda_joint5': 0.0,
        'panda_joint6': 1.570796327,
        'panda_joint7': 0.785398163,
        }
       
        from moveit_msgs.msg import Constraints, JointConstraint
        c = Constraints()
        for name, pos in ready_joints.items():
            jc = JointConstraint(joint_name=name, position=pos,
                                tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            c.joint_constraints.append(jc)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "panda_arm"
        goal_msg.request.goal_constraints.append(c)

        goal_handle = self._wait_for(self.move_action_client.send_goal_async(goal_msg), timeout=10.0)
        self._wait_for(goal_handle.get_result_async(), timeout=30.0)
        self.get_logger().info("Ready Reached")
        
        self.grasping_active = False 
#_______________________________________________________________________________________________________________

    def _wait_for(self, future, timeout=None):
        # The MultiThreadedExecutor in main() services the callbacks on
        # another thread; we just block on the result here.
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            return None
        return future.result()


    def move_to_pose(self, pose_stamped):
        if not self.move_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("MoveIt action server not available!")
            return False
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "panda_arm" 
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        # Reduce velocity for the actual grasp approach for safety
        goal_msg.request.max_velocity_scaling_factor = 0.1 
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive

        c = Constraints()
        c.name = "target_pose"
        
        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = "panda_link8"
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
        pc.constraint_region.primitive_poses.append(pose_stamped.pose)
        pc.weight = 1.0
        
        oc = OrientationConstraint()
        oc.header = pose_stamped.header
        oc.link_name = "panda_link8"
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        goal_msg.request.goal_constraints.append(c)


        # DEBUG: print the actual pose command being sent to MoveIt
        self.get_logger().info("========== MOVEIT GOAL ==========")
        self.get_logger().info(f"Planning group: {goal_msg.request.group_name}")
        self.get_logger().info(f"Target frame: {pose_stamped.header.frame_id}")
        self.get_logger().info(f"Target link: panda_link8")
        self.get_logger().info(
            f"Position: x={pose_stamped.pose.position.x:.4f}, "
            f"y={pose_stamped.pose.position.y:.4f}, "
            f"z={pose_stamped.pose.position.z:.4f}"
        )
        self.get_logger().info(
            f"Orientation: x={pose_stamped.pose.orientation.x:.4f}, "
            f"y={pose_stamped.pose.orientation.y:.4f}, "
            f"z={pose_stamped.pose.orientation.z:.4f}, "
            f"w={pose_stamped.pose.orientation.w:.4f}"
        )

        send_future = self.move_action_client.send_goal_async(goal_msg)
        goal_handle = self._wait_for(send_future, timeout=10.0) #Boolean to confirm moveit handled goal succesfully
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveIt goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        final_res = self._wait_for(result_future, timeout=30.0)
        if final_res is None:
            self.get_logger().error("MoveIt result timed out")
            return False
        if final_res.result.error_code.val == 1:
            return True

        self.get_logger().error("MoveIt planning failed")
        return False

    def trigger_gripper(self, width, force=40.0):
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Gripper action not available")
            return

        goal = FrankaGrasp.Goal()
        goal.width = width
        goal.speed = 0.05
        goal.force = force
        
        goal.epsilon.inner = 0.05
        goal.epsilon.outer = 0.05
        
        send_future = self.gripper_client.send_goal_async(goal)
        goal_handle = self._wait_for(send_future, timeout=5.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Gripper goal rejected")
            return
        self._wait_for(goal_handle.get_result_async(), timeout=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = PandaGraspExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()