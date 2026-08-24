import rclpy
import pymoveit2
import panda_py
from rclpy.node import Node
from grasp_interfaces.msg import GraspCandidate
from std_msgs.msg import String
from franka_msgs.msg import FrankaState
from math import pi
from scipy.spatial.transform import Rotation

Z_offset = 0.225 # 0.25 is good

def final_angle(angle):
    if angle > pi/2:
        angle = angle - pi
    elif angle < -pi/2:
        angle = angle + pi
    return angle

class PandaGraspExecutor(Node):
    def __init__(self):
        super().__init__('panda_grasp_executor')
        self.collide = False
        
        # good group
        good = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # subscribe to my youtube channel
        self.sub = self.create_subscription(GraspCandidate, '/grasp_candidates', self.callback, 1)
        self.state_sub = self.create_subscription(FrankaState,'/franka_robot_state_broadcaster/robot_state', self.franka_state_callback, 1)

        # My Publishers
        self.send_state = self.create_publisher(String, '/state', 1)

        # PyMoveit2
        self.moveit2 = pymoveit2.MoveIt2(
            node = self,
            joint_names = ["panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"],
            base_link_name = "panda_link0",
            end_effector_name = "panda_link8",
            group_name = "panda_arm", 
            callback_group=good
        )

        # Declare State
        self.state = "not_runnin"

    # Reflex Behaviour (ASK ROS2 GUY)
    def franka_state_callback(self, msg):
        if msg.robot_mode == 4:
            self.get_logger().fatal('REFLEX')
            self.moveit2.cancel_execution()
            panda_py.Panda('172.16.0.2').recover()
            panda_py.Panda('172.16.0.2').move_to_start()
            panda_py.libfranka.Gripper('172.16.0.2').move(0.08, 0.1)

    # Main Callback
    def callback(self, msg):
        self.get_logger().info("Candidate received!")
        self.state = "runnin"
        self.send_state.publish(String(data=self.state))
        
        if self.state == 'runnin':
            grasp_candidate = msg

            # 1. HOVER
            self.get_logger().info(f"1. Moving to Pre-Grasp")

            # Multiply 2 Quaternions
            q_ref = Rotation.from_quat([grasp_candidate.pose.orientation.x, grasp_candidate.pose.orientation.y, grasp_candidate.pose.orientation.z, grasp_candidate.pose.orientation.w])
            q_future = Rotation.from_euler('z', final_angle(grasp_candidate.angle))
            q = q_ref * q_future
            q = q.as_quat()

            grasp_candidate.pose.orientation.x = q[0]
            grasp_candidate.pose.orientation.y = q[1]
            grasp_candidate.pose.orientation.z = q[2]
            grasp_candidate.pose.orientation.w = q[3]
            
            self.moveit2.move_to_pose(
                position=[grasp_candidate.pose.position.x, grasp_candidate.pose.position.y, grasp_candidate.pose.position.z],
                quat_xyzw=[grasp_candidate.pose.orientation.x, grasp_candidate.pose.orientation.y, grasp_candidate.pose.orientation.z, grasp_candidate.pose.orientation.w],
                frame_id="panda_link0",
                cartesian=True
            )
            self.moveit2.wait_until_executed()
       
            # 2. Drop
            self.get_logger().info("2. Descending to Grasp...")
            grasp_candidate.pose.position.z -= Z_offset
            self.moveit2.move_to_pose(
                position=[grasp_candidate.pose.position.x, grasp_candidate.pose.position.y, grasp_candidate.pose.position.z],
                quat_xyzw=[grasp_candidate.pose.orientation.x, grasp_candidate.pose.orientation.y, grasp_candidate.pose.orientation.z, grasp_candidate.pose.orientation.w],
                frame_id="panda_link0",
                cartesian=True
            )
            self.moveit2.wait_until_executed()

            # 3. Gripper Close
            self.get_logger().info("3. Closing Gripper...")
            panda_py.libfranka.Gripper('172.16.0.2').grasp(width=0.0, speed=0.1, force=100, epsilon_inner=0.1, epsilon_outer=1.0)

            # 4. Lift
            grasp_candidate.pose.position.z += Z_offset
            self.moveit2.move_to_pose(
                position=[grasp_candidate.pose.position.x, grasp_candidate.pose.position.y, grasp_candidate.pose.position.z],
                quat_xyzw=[grasp_candidate.pose.orientation.x, grasp_candidate.pose.orientation.y, grasp_candidate.pose.orientation.z, grasp_candidate.pose.orientation.w],
                frame_id="panda_link0",
                cartesian=True
            )
            self.moveit2.wait_until_executed()

            # 5. Move to Somewhere
            grasp_candidate.pose.position.x -= .1
            grasp_candidate.pose.position.y += .1
                    
            self.moveit2.move_to_pose(
                position=[grasp_candidate.pose.position.x, grasp_candidate.pose.position.y, grasp_candidate.pose.position.z],
                quat_xyzw=[grasp_candidate.pose.orientation.x, grasp_candidate.pose.orientation.y, grasp_candidate.pose.orientation.z, grasp_candidate.pose.orientation.w],
                frame_id="panda_link0",
                cartesian=True
            )
            self.moveit2.wait_until_executed()

            # 6. Gripper Open
            self.get_logger().info("Grasp Complete. Unlocking...")
            panda_py.libfranka.Gripper('172.16.0.2').move(0.08, 0.1)

            # 7. Return to Ready
            ready_joints = [
                0.0,          # panda_joint1
                -0.785398163, # panda_joint2
                0.0,          # panda_joint3
                -2.356194490, # panda_joint4
                0.0,          # panda_joint5
                1.570796327,  # panda_joint6
                0.785398163   # panda_joint7
            ]
            self.moveit2.move_to_configuration(
                joint_positions = ready_joints,
                joint_names = None,
                tolerance = 0.001,
                weight = 1.0
            )
            self.moveit2.wait_until_executed()

            # 8. End State
            self.state = 'not runnin'
            self.get_logger().info("Done")
            self.send_state.publish(String(data=self.state))

def main():
    rclpy.init()
    node = PandaGraspExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

