import rclpy
import pymoveit2
import panda_py
from rclpy.node import Node
from grasp_interfaces.msg import GraspCandidate
from std_msgs.msg import String
from franka_msgs.msg import FrankaState
from math import pi
from scipy.spatial.transform import Rotation
from franka_msgs.srv import ErrorRecovery
from time import sleep
from grasp_interfaces.msg import GraspCandidate, TrackedObject, TrackedObjectArray
from geometry_msgs.msg import Quaternion

Z_drop = 0.185 #0.185 is good
Z_lift = 0.25 #0.25

CONVEYOR_SPEED = 0.0785398163397

def final_angle(angle):
    if angle > pi/2:
        angle = angle - pi
    elif angle < -pi/2:
        angle = angle + pi
    return angle - pi/2

class PandaGraspExecutor(Node):
    def __init__(self):
        super().__init__('panda_grasp_executor')

        # Declare States
        self.collide = False
        self.state = "runnin"

        # subscribe to my youtube channel
        self.state_sub = self.create_subscription(FrankaState,'/franka_robot_state_broadcaster/robot_state', self.franka_state_callback, 1, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.sub2 = self.create_subscription(TrackedObject, '/franka/target', self.tracked_object_callback, 1)

        # Publishers
        self.send_state = self.create_publisher(String, '/state', 1)

        # Client
        self.recover = self.create_client(ErrorRecovery, '/error_recovery_service_server/error_recovery')

        # PyMoveit2
        self.moveit2 = pymoveit2.MoveIt2(
            node = self,
            joint_names = ["panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"],
            base_link_name = "panda_link0",
            end_effector_name = "panda_link8",
            group_name = "panda_arm", 
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )

        # David (don't delete)
        # this only affects move to config????
        # self.moveit2.max_velocity = 0.1
        # self.moveit2.max_acceleration = 0.1

        self.get_logger().info('Panda Grasp Executor was Initialized')

    def ready_state(self):
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

    # Reflex Behaviour
    def franka_state_callback(self, msg):
        if self.collide:
            if msg.robot_mode == 4:
                req = ErrorRecovery.Request()
                future = self.recover.call_async(req)
                # spin_until_future_complete breaks nodes 
                print(future.result()) # this will probably not print success because reentrant group
                if future:
                    self._logger.warn('Successfully Performed Recovery Service... Now Waiting for next Goal')
                return

        elif msg.robot_mode == 4: # if not self.collide and msg.robot_mode == 4:
            self.state = 'not runnin'
            self.send_state.publish(String(data=self.state))
            self.get_logger().fatal('Reflex Detected. Aborting Movement...')
            self.moveit2.cancel_execution() # COULD BE REDUNDANT, DO GOALS GET REMOVED WHEN REFLEX STATE?
            self.collide = True

    def franka_recover(self):
        if self.collide:
            self.get_logger().info("Returning to Ready State...")
            self.ready_state()
            self.moveit2.wait_until_executed()
            sleep(1)
            self.state = 'runnin'
            self.send_state.publish(String(data=self.state))
            self.collide = False
            self.get_logger().info("Successfully Returned to Ready State...")
            return

    def tracked_object_callback(self, msg):
        self.state = 'runnin'
        self.send_state.publish(String(data=self.state))
        print(msg)

        X_TRANSLATION = 0.37
        Y_TRANSLATION = 0.515
        Z_TRANSLATION = 0.81

        # The obstacle node stores RealSense (x, y, z) as position (z, y, x)
        cam_z = msg.position.z
        cam_y = msg.position.y
        cam_x = msg.position.x
        
        candidate = GraspCandidate()
        candidate.pose.position.x = (cam_x + X_TRANSLATION) + 0.05
        candidate.pose.position.y = -cam_y + Y_TRANSLATION - 0.01
        candidate.pose.position.z = Z_TRANSLATION - cam_z + 0.28
        candidate.pose.orientation = Quaternion(x=0.926, y=-0.378, z=-0.002, w=-0.001)
        candidate.angle = float(msg.angle)
        candidate.type = 2 # int(obj.obj_class) figure out how to find type nomenclature

        self.callback(candidate)
        self.state = 'not_runnin'
        self.send_state.publish(String(data=self.state))

    # Main Callback
    def callback(self, msg):
        self.get_logger().info("Candidate received!")
        self.send_state.publish(String(data=self.state))
        
        grasp_candidate = msg
            
        self.get_logger().fatal(f'angle: {grasp_candidate.angle} \n type: {grasp_candidate.type}')

        # 0. Multiply 2 Quaternions
        q_ref = Rotation.from_quat([grasp_candidate.pose.orientation.x, grasp_candidate.pose.orientation.y, grasp_candidate.pose.orientation.z, grasp_candidate.pose.orientation.w])
        q_future = Rotation.from_euler('z', final_angle(grasp_candidate.angle))
        q = q_ref * q_future
        q = q.as_quat()

        # 1. HOVER
        self.get_logger().info(f"1. Hover Above Object")

        grasp_candidate.pose.orientation.x = q[0]
        grasp_candidate.pose.orientation.y = q[1]
        grasp_candidate.pose.orientation.z = q[2]
        grasp_candidate.pose.orientation.w = q[3]

        grasp_candidate.pose.position.y -= 6 * CONVEYOR_SPEED
        self.moveit2.move_to_pose(
            position=[grasp_candidate.pose.position.x, grasp_candidate.pose.position.y, grasp_candidate.pose.position.z],
            quat_xyzw=[grasp_candidate.pose.orientation.x, grasp_candidate.pose.orientation.y, grasp_candidate.pose.orientation.z, grasp_candidate.pose.orientation.w],
            frame_id="panda_link0",
            cartesian=True
        )
        self.moveit2.wait_until_executed()

        # 1.5. Check for Collision (David patent)
        self.franka_recover()
    
        # 2. Drop
        self.get_logger().info("2. Descending to Grasp...")

        grasp_candidate.pose.position.z -= Z_drop
        self.moveit2.move_to_pose(
            position=[grasp_candidate.pose.position.x, grasp_candidate.pose.position.y, grasp_candidate.pose.position.z],
            quat_xyzw=[grasp_candidate.pose.orientation.x, grasp_candidate.pose.orientation.y, grasp_candidate.pose.orientation.z, grasp_candidate.pose.orientation.w],
            frame_id="panda_link0",
            cartesian=True
        )
        self.moveit2.wait_until_executed()

        # 2.5. Check for Collision (David patent)
        self.franka_recover()
        
        # 3. Gripper Close
        self.get_logger().info("3. Closing Gripper...")
        panda_py.libfranka.Gripper('172.16.0.2').grasp(width=0.0, speed=1.0, force=100, epsilon_inner=0.1, epsilon_outer=1.0)

        # 4. Lift
        grasp_candidate.pose.position.z += Z_lift 
        self.moveit2.move_to_pose(
            position=[grasp_candidate.pose.position.x, grasp_candidate.pose.position.y, grasp_candidate.pose.position.z],
            quat_xyzw=[grasp_candidate.pose.orientation.x, grasp_candidate.pose.orientation.y, grasp_candidate.pose.orientation.z, grasp_candidate.pose.orientation.w],
            frame_id="panda_link0",
            cartesian=True
        )
        self.moveit2.wait_until_executed()

        # 5. Move to Bin (Very Stupid Right Now)

        print(f'object type: {grasp_candidate.type}')

        TYPE_TARGETS = {
            2: (-0.26, -0.3), # Paper
            4: (-0.24, 0.35), # Plastic
            1: (-0.35, 0.38), # Metal
            3: (0.55, 0.080),
        }
        grasp_candidate.pose.position.x, grasp_candidate.pose.position.y = TYPE_TARGETS[grasp_candidate.type]
        self.moveit2.move_to_pose(
            position=[grasp_candidate.pose.position.x, grasp_candidate.pose.position.y, grasp_candidate.pose.position.z + 0.25],
            quat_xyzw=[grasp_candidate.pose.orientation.x, grasp_candidate.pose.orientation.y, grasp_candidate.pose.orientation.z, grasp_candidate.pose.orientation.w],
            frame_id="panda_link0",
            cartesian=False
        )
        self.moveit2.wait_until_executed()
        # -0.177, -0.735, 0.191 BIG TRASH BIN LOCATION xyz
        # let us add obstalces to planning

        # 6. Gripper Open
        self.get_logger().info("6. Unlocking Gripper...")
        panda_py.libfranka.Gripper('172.16.0.2').move(0.08, 0.1)

        # 7. Return to Ready
        self.ready_state()
        self.moveit2.wait_until_executed()

        # 8. End State
        self.get_logger().info("Grasp Completed")

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