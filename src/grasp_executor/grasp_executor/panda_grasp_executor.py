import rclpy
import pymoveit2
from pymoveit2.moveit2 import MoveIt2State
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

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
from geometry_msgs.msg import Quaternion, Pose
from rclpy.time import Time
import threading
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

Z_drop = 0.185 #0.185 is good
Z_lift = 0.25 #0.25

CONVEYOR_SPEED = 0.0785398163397
T_APPROACH = 2.8       # measured: "Approach took 2.80s"
MAX_TRACK_ERR = 0.10   # m, give up on the object if tracking falls this far behind

# Servo loop
DT = 0.01              # servo loop period (100 Hz)
KP = 3.0               # position correction gain
KI = 3.0               # integral gain: removes the steady lag behind the object
I_MAX = 0.10           # m/s, cap on the integral's contribution (anti-windup)
V_MAX = 0.15           # m/s, cap on the correction
SERVO_LEAD = 0.03      # s, compensates command latency
XY_TOL = 0.008         # m, alignment needed before/while descending
Z_TOL = 0.005          # m
DESCEND_SPEED = 0.08   # m/s
LIFT_SPEED = 0.10      # m/s
GRIPPER_IP = '172.16.0.2'
A_MAX = 0.5           # m/s^2, max change in commanded velocity (smooths every command)

# Bin (x, y) per class id from the perception node
TYPE_TARGETS = {
    # 0: (?, ?),       # Glass: planner sends glass to the arm, needs a bin
    1: (-0.35, 0.38),  # Metal
    2: (-0.26, -0.30), # Paper
    3: (0.55, 0.080),  # Cardboard
    4: (-0.24, 0.35),  # Plastic
}

def final_angle(angle):
    """Yaw to rotate the gripper by, relative to the hover orientation.

    A parallel gripper looks the same rotated 180 deg, so a and a +/- pi are the
    same grasp. Wrap to [-pi/2, pi/2) so the wrist never turns more than 90 deg.
    """
    a = angle - pi/2                      # same offset as before
    return (a + pi/2) % pi - pi/2         # shortest equivalent rotation

class PandaGraspExecutor(Node):
    def __init__(self):
        super().__init__('panda_grasp_executor')

        # Declare States
        self.collide = False
        self.state = "not runnin"

        # Subscriptions to target object
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

        self.phase = 'IDLE'
        self.target = None
        self.robot_mode = None
        self.z_offset = Z_drop
        self.stable = 0
        self.err_int = np.zeros(3)
        self.v_prev = np.zeros(3)
        self.t_prev = None
        self.grasp_done = False

        self.gripper_open()   # start with the gripper open
        self.grasp_ok = False
        self.grasp_t0 = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt Servo: talk to servo_node directly (no pymoveit2 helper)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.servo_start_cli = self.create_client(Trigger, '/servo_node/start_servo')
        self.recover_t0 = None

        # Servo's start service takes several seconds, so start it ONCE at startup and
        # leave it running. With no commands it just holds still and stops publishing
        # (incoming_command_timeout), so it doesn't interfere with planned moves.
        self.servo_ready = False
        self.servo_start_future = None
        self.create_timer(1.0, self._ensure_servo_started)

        self.create_timer(DT, self.servo_tick, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info('Panda Grasp Executor was Initialized')


    def transform_cam2franka(self, TrackedObject):
        X_TRANSLATION = 0.37
        Y_TRANSLATION = 0.515
        Z_TRANSLATION = 0.81

        cam_z = TrackedObject.position.z
        cam_y = TrackedObject.position.y
        cam_x = TrackedObject.position.x


        TrackedObject.position.x = (cam_x + X_TRANSLATION) + 0.05
        TrackedObject.position.y = -cam_y + Y_TRANSLATION - 0.21
        TrackedObject.position.z = Z_TRANSLATION - cam_z + 0.28

        TrackedObject.angle = float(TrackedObject.angle)
        return TrackedObject

    def tracked_object_callback(self, msg):
        # Only accept a new target when idle and not recovering
        if self.state == 'not runnin' and self.phase == 'IDLE' and not self.collide:
            if not self.servo_ready:
                self.get_logger().warn('Servo not started yet, ignoring target', throttle_duration_sec=2.0)
                return
            if msg.obj_class not in TYPE_TARGETS:
                self.get_logger().warn(f'No bin for class {msg.obj_class}, ignoring')
                return

            obj = self.transform_cam2franka(msg)
            MAX_BELT_TIME = 25
            Y_MIN = -0.2
            Y_MAX = 0.7
            t_seen = Time.from_msg(obj.header.stamp)
            belt_time = (self.get_clock().now() - t_seen).nanoseconds * 1e-9
            y_cur = obj.position.y - CONVEYOR_SPEED * belt_time
            y_arrive = y_cur - CONVEYOR_SPEED * T_APPROACH

            if obj.header.stamp.sec == 0 and obj.header.stamp.nanosec == 0:
                reason = 'unstamped'
            elif belt_time < 0.0:
                reason = f'stamp in the future ({belt_time:.3f}s)'
            elif belt_time > MAX_BELT_TIME:
                reason = f'too old ({belt_time:.1f}s)'
            elif not (Y_MIN <= y_arrive <= Y_MAX):
                reason = f'out of reach at arrival (y={y_arrive:.3f})'
            else:
                reason = None
            if reason:
                self.get_logger().info(reason)
                return

            q = (Rotation.from_quat([0.926, -0.378, -0.002, -0.001])* Rotation.from_euler('z', final_angle(obj.angle))).as_quat()

            # Hand the target to the servo loop
            self.target = {
                't0': t_seen,
                'x': obj.position.x,
                'y0': obj.position.y,
                'z_hover': obj.position.z,
                'z_grasp': obj.position.z - Z_drop,
                'quat': q.tolist(),
                'type': int(obj.obj_class),
                'track_id': obj.track_id,
            }
            self.get_logger().info(f'Accepted track {obj.track_id} (belt_time {belt_time:.2f}s)')
            self.phase = 'APPROACH'
            self.state = 'runnin'
            self.send_state.publish(String(data=self.state))

    # ---------------- Helpers ----------------
    def predict_y(self, lead=0.0):
        dt = (self.get_clock().now() - self.target['t0']).nanoseconds * 1e-9 + lead
        return self.target['y0'] - CONVEYOR_SPEED * dt

    def ee_position(self):
        try:
            tf = self.tf_buffer.lookup_transform('panda_link0', 'panda_link8', Time())
        except Exception:
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z])

    def planned_move(self, position, quat):
        """Blocking MoveIt move. Returns True only if it actually succeeded.

        False means a reflex (self.collide is True) OR planning/execution failed
        (e.g. 'execute_trajectory ... STATUS_ABORTED'). Callers check self.collide
        to tell the two apart.
        """
        self.get_logger().info(f'[DEBUG] planned_move -> {np.round(position, 3)}')
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat,
                                  frame_id='panda_link0', cartesian=False)
        self.get_logger().info('[DEBUG] planned_move: waiting for execution')
        result = self.moveit2.wait_until_executed()
        # Newer pymoveit2 returns True/False here; older versions return None.
        ok = (result is not False) and not self.collide
        self.get_logger().info(f'[DEBUG] planned_move: done (ok={ok}, result={result}, collide={self.collide})')
        return ok

    def gripper_open(self):
        """Open the gripper on a fresh connection (a long-lived one can drop: 'TCP send bytes: I/O error')."""
        for attempt in (1, 2):
            try:
                panda_py.libfranka.Gripper(GRIPPER_IP).move(0.08, 0.1)
                return True
            except Exception as e:
                self.get_logger().error(f'Gripper open failed (attempt {attempt}): {e}')
                sleep(0.2)
        return False

    def go_home_and_reset(self, open_gripper=True):
        """Go to ready, optionally open the gripper, and free the arm for the next target."""
        if open_gripper:
            self.gripper_open()
        self.ready_state()
        if self.moveit2.wait_until_executed() is False:
            self.get_logger().error('Return to ready FAILED. Check the arm before sending more targets.')
        self.target = None
        self.phase = 'IDLE'
        self.state = 'not runnin'
        self.send_state.publish(String(data=self.state))

    def send_twist(self, v):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'panda_link0'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(a) for a in v)
        self.twist_pub.publish(msg)

    def _ensure_servo_started(self):
        """Runs every 1 s until servo_node confirms it has started."""
        if self.servo_ready or self.servo_start_future is not None:
            return
        if not self.servo_start_cli.service_is_ready():
            self.get_logger().warn('Waiting for /servo_node/start_servo...', throttle_duration_sec=5.0)
            return
        self.get_logger().info('Starting MoveIt Servo (takes a few seconds)...')
        self.servo_start_future = self.servo_start_cli.call_async(Trigger.Request())
        self.servo_start_future.add_done_callback(self._on_servo_started)

    def _on_servo_started(self, future):
        res = future.result()
        if res is not None and res.success:
            self.servo_ready = True
            self.get_logger().info('MoveIt Servo started, ready for targets')
        else:
            self.get_logger().error(f'Servo start failed: {res}. Retrying...')
            self.servo_start_future = None   # timer will try again

    def stop_servo(self, ramp=True):
        """Bring the arm to rest and stop streaming. Servo stays started.

        ramp=True: slow down at A_MAX (smooth hand-off to the planner).
        ramp=False: send zero straight away (collisions).
        """
        if ramp:
            v = self.v_prev.copy()
            while np.linalg.norm(v) > 1e-3:
                v = v - np.clip(v, -A_MAX * DT, A_MAX * DT)
                self.send_twist(v)
                sleep(DT)
        self.send_twist((0.0, 0.0, 0.0))
        self.v_prev = np.zeros(3)
        sleep(0.5)   # let servo finish its halt messages and the arm settle before the planner takes over

    def abort_pick(self):
        """Stop servo, open gripper, go home, and free the arm for the next target."""
        self.stop_servo()
        self.go_home_and_reset(open_gripper=True)

    def _do_grasp(self):
        # Runs in its own thread so tracking continues while the fingers close.
        # Fresh gripper connection per call, like your original code.
        try:
            self.grasp_ok = bool(panda_py.libfranka.Gripper(GRIPPER_IP).grasp(
                width=0.0, speed=1.0, force=100, epsilon_inner=0.1, epsilon_outer=1.0))
        except Exception as e:
            self.get_logger().error(f'Gripper grasp failed: {e}')
            self.grasp_ok = False
        self.grasp_done = True   # always set, so the state machine never waits forever

    # ---------------- Servo loop ----------------
    def servo_tick(self):
        """Wrapper: an unexpected error stops this pick instead of killing the node."""
        try:
            self._servo_tick()
        except Exception as e:
            import traceback
            self.get_logger().error(f'Unexpected error in {self.phase}: {e}\n{traceback.format_exc()}')
            try:
                self.stop_servo(ramp=False)
            except Exception:
                pass
            self.get_logger().error('Pick abandoned. Arm left where it is; check it before the next target.')
            self.target = None
            self.phase = 'IDLE'
            self.state = 'not runnin'
            self.send_state.publish(String(data=self.state))

    def _servo_tick(self):
        if self.collide:
            self.handle_collision()
            return
        if self.phase == 'IDLE' or self.target is None:
            return
        tg = self.target
        
        # Planned approach to hover above where the object WILL be
        if self.phase == 'APPROACH':
            t_start = self.get_clock().now()
            y_goal = self.predict_y(lead=T_APPROACH)
            if not self.planned_move([tg['x'], y_goal, tg['z_hover']], tg['quat']):
                if self.collide:
                    return   # collision handler takes over next tick
                self.get_logger().error('Approach move failed, skipping this object')
                self.go_home_and_reset(open_gripper=False)
                return
            took = (self.get_clock().now() - t_start).nanoseconds * 1e-9
            self.get_logger().info(f'Approach took {took:.2f}s (T_APPROACH={T_APPROACH})')
            self.z_offset = Z_drop
            self.stable = 0
            self.err_int = np.zeros(3)
            self.v_prev = np.zeros(3)   # arm is at rest after the planned move
            self.t_prev = None
            self.get_logger().info('[DEBUG] Starting TRACK (servo already running)')
            self.phase = 'TRACK'
            return

        # Place: planned moves again
        if self.phase == 'PLACE':
            self.stop_servo()
            bx, by = TYPE_TARGETS[tg['type']]
            bin_pose = [bx, by, tg['z_grasp'] + Z_lift + 0.25]

            # Try the bin move twice (a fresh plan from the settled pose often works)
            placed = False
            for attempt in (1, 2):
                if self.planned_move(bin_pose, tg['quat']):
                    placed = True
                    break
                if self.collide:
                    return   # collision handler takes over next tick
                self.get_logger().warn(f'Move to bin failed (attempt {attempt})')
                sleep(0.5)

            if placed:
                self.gripper_open()
                self.get_logger().info(f'Track {tg["track_id"]} placed')
            else:
                self.get_logger().error('Could not reach the bin. Returning to ready; the object will be released there.')
            self.go_home_and_reset(open_gripper=not placed)
            return

        # Servo phases: TRACK / DESCEND / GRASP / LIFT
        p = self.ee_position()
        if p is None:
            self.get_logger().warn('[DEBUG] No TF panda_link0 -> panda_link8', throttle_duration_sec=1.0)
            return
        goal = np.array([tg['x'], self.predict_y(lead=SERVO_LEAD), tg['z_grasp'] + self.z_offset])
        err = goal - p

        # Safety: if we've fallen too far behind, abandon this object instead of chasing it
        if np.linalg.norm(err[:2]) > MAX_TRACK_ERR:
            self.get_logger().warn(f'[DEBUG] Tracking error {err.round(3)} too large, aborting track {tg["track_id"]}')
            self.abort_pick()
            return

        # Real time since the last tick (the timer isn't exactly 100 Hz in Python).
        # Using the real dt keeps the moving z goal and the z feedforward in step.
        now = self.get_clock().now()
        dt = DT if self.t_prev is None else (now - self.t_prev).nanoseconds * 1e-9
        dt = min(max(dt, 0.001), 0.05)
        self.t_prev = now

        err_xy = np.linalg.norm(err[:2])
        xy_ok = err_xy < XY_TOL

        # Vertical feedforward: the z goal moves at a known speed, so command that speed
        # directly instead of making the P term chase it. While descending, slow the
        # descent smoothly as alignment gets worse (full speed inside XY_TOL, zero at 2x)
        # instead of switching it on and off.
        vz_ff = 0.0
        if self.phase == 'DESCEND' and self.z_offset > 0.0:
            align = float(np.clip(2.0 - err_xy / XY_TOL, 0.0, 1.0))
            vz_ff = -DESCEND_SPEED * align
        elif self.phase == 'LIFT' and self.z_offset < Z_drop:
            vz_ff = LIFT_SPEED

        # PI control on top of the feedforward. Integral on x/y only: it removes the
        # lag behind the belt. z has its own feedforward, so no integral there.
        self.err_int[:2] = np.clip(self.err_int[:2] + err[:2] * dt, -I_MAX / KI, I_MAX / KI)
        correction = KP * err
        correction[:2] += KI * self.err_int[:2]
        v_target = np.array([0.0, -CONVEYOR_SPEED, vz_ff]) + np.clip(correction, -V_MAX, V_MAX)

        # Acceleration limit: never change the command by more than A_MAX per second.
        dv = np.clip(v_target - self.v_prev, -A_MAX * dt, A_MAX * dt)
        v = self.v_prev + dv
        self.v_prev = v

        self.send_twist(v)
        self.get_logger().info(
            f'[DEBUG] {self.phase} ee={p.round(3)} goal={goal.round(3)} err={err.round(3)} v={v.round(3)}',
            throttle_duration_sec=0.5)

        # Move the z goal at the same speed we just commanded as feedforward
        if self.phase == 'DESCEND':
            self.z_offset = max(0.0, self.z_offset + vz_ff * dt)
        elif self.phase == 'LIFT':
            self.z_offset = min(Z_drop, self.z_offset + vz_ff * dt)

        if self.phase == 'TRACK':
            self.stable = self.stable + 1 if xy_ok else 0
            if self.stable >= 20:
                self.phase = 'DESCEND'

        elif self.phase == 'DESCEND':
            if self.z_offset == 0.0 and abs(err[2]) < Z_TOL:
                self.grasp_done = False
                self.grasp_ok = False
                self.grasp_t0 = self.get_clock().now()
                threading.Thread(target=self._do_grasp, daemon=True).start()
                self.get_logger().info('Closing gripper (still tracking)')
                self.phase = 'GRASP'

        elif self.phase == 'GRASP':
            if self.grasp_done:
                self.get_logger().info(f'Gripper closed (grasp_ok={self.grasp_ok}), lifting')
                self.phase = 'LIFT'
            elif (self.get_clock().now() - self.grasp_t0).nanoseconds * 1e-9 > 3.0:
                self.get_logger().error('Gripper took over 3 s, aborting pick')
                self.abort_pick()

        elif self.phase == 'LIFT':
            if self.z_offset >= Z_drop:
                self.phase = 'PLACE'


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
        self.robot_mode = msg.robot_mode

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
            self.moveit2.cancel_execution()
            self.collide = True

    def handle_collision(self):
        # First tick after the reflex: stop servo once
        if self.phase != 'RECOVERING':
            self.get_logger().warn(f'Collision during {self.phase}, recovering')
            self.stop_servo(ramp=False)   # don't keep moving after a reflex
            self.phase = 'RECOVERING'
            self.recover_t0 = self.get_clock().now()

        # Wait (one check per tick, no blocking) for error recovery to clear the reflex
        if self.robot_mode == 4:
            if (self.get_clock().now() - self.recover_t0).nanoseconds * 1e-9 > 5.0:
                self.get_logger().error('Reflex not cleared after 5s', throttle_duration_sec=5.0)
            return

        self.franka_recover()          # goes to ready, clears self.collide
        self.target = None
        self.phase = 'IDLE'
        self.state = 'not runnin'
        self.send_state.publish(String(data=self.state))

    def franka_recover(self):
        if self.collide:
            self.get_logger().info("Returning to Ready State...")
            self.ready_state()
            self.moveit2.wait_until_executed()
            sleep(1)
            self.collide = False
            self.get_logger().info("Successfully Returned to Ready State...")
            return

def main():
    rclpy.init()
    node = PandaGraspExecutor()
    try:
        # Single-threaded on purpose: pymoveit2's blocking calls spin the node
        # themselves, and a second executor on the same node crashes rclpy.
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()