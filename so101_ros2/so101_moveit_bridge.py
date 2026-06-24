"""
so101_moveit_bridge.py  –  MoveIt2 ↔ SO101 hardware bridge

Replaces ros2_control entirely. Provides exactly what your moveit_controllers.yaml expects:

  Action servers (matching moveit_controllers.yaml):
    /arm_controller/follow_joint_trajectory      ← arm trajectory (FollowJointTrajectory)
    /arm_effort_controller/follow_joint_trajectory  ← same, effort mode alias
    /gripper_controller/gripper_cmd              ← gripper (GripperCommand)
    /gripper_effort_controller/follow_joint_trajectory ← gripper FJT alias

  Publisher:
    /joint_states   ← live arm positions (radians) at ~30 Hz

Joint name mapping  (MoveIt YAML → SO101 bus):
  Shoulder_Rotation → shoulder_pan
  Shoulder_Pitch    → shoulder_lift
  Elbow             → elbow_flex
  Wrist_Pitch       → wrist_flex
  Wrist_Roll        → wrist_roll
  Gripper           → gripper

Usage:
  ros2 launch so101_ros2 so101_moveit_bridge_launch.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint

from so101_ros2.lerobot.so101 import SO101


# ── Joint name translation ────────────────────────────────────────────────────
# Keys   = names MoveIt uses (from your moveit_controllers.yaml / URDF)
# Values = names the SO101 bus uses (from so101_inference_node.py JOINT_NAMES)

MOVEIT_TO_BUS: dict[str, str] = {
    "Shoulder_Rotation": "shoulder_pan",
    "Shoulder_Pitch":    "shoulder_lift",
    "Elbow":             "elbow_flex",
    "Wrist_Pitch":       "wrist_flex",
    "Wrist_Roll":        "wrist_roll",
    "Gripper":           "gripper",
}

BUS_TO_MOVEIT: dict[str, str] = {v: k for k, v in MOVEIT_TO_BUS.items()}

ARM_MOVEIT_JOINTS  = ["Shoulder_Rotation", "Shoulder_Pitch", "Elbow",
                       "Wrist_Pitch", "Wrist_Roll"]
GRIPPER_MOVEIT_JOINT = "Gripper"

# Gripper open/close limits in degrees (tune to your hardware)
GRIPPER_OPEN_DEG   =  0.0
GRIPPER_CLOSED_DEG = 50.0   # adjust to your actual gripper range


class SO101MoveItBridge(Node):
    """
    Bridges MoveIt2 planned trajectories → SO101 physical arm.

    No ros2_control / controller_manager needed.
    """

    def __init__(self):
        super().__init__('so101_moveit_bridge')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('port',         '/dev/ttyACM0')
        self.declare_parameter('robot_name',   'so101_follower')
        self.declare_parameter('recalibrate',  False)
        self.declare_parameter('publish_rate', 30.0)

        self.port        = self.get_parameter('port').value
        self.robot_name  = self.get_parameter('robot_name').value
        self.recalibrate = self.get_parameter('recalibrate').value
        pub_rate_hz      = self.get_parameter('publish_rate').value

        # ── Arm connection ────────────────────────────────────────────────────
        self.robot = self._connect_arm()

        # ── Callback group: lets action + timer run concurrently ──────────────
        self._cb = ReentrantCallbackGroup()

        # ── /joint_states publisher ───────────────────────────────────────────
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / pub_rate_hz, self._publish_joint_states,
                          callback_group=self._cb)

        # ── Action servers (match your moveit_controllers.yaml exactly) ───────

        # 1. Primary arm controller (default: true)
        self._arm_server = ActionServer(
            self, FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self._exec_arm_trajectory,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb,
        )

        # 2. Effort arm controller alias (default: false — MoveIt may still
        #    activate it as fallback; keep it functional)
        self._arm_effort_server = ActionServer(
            self, FollowJointTrajectory,
            'arm_effort_controller/follow_joint_trajectory',
            execute_callback=self._exec_arm_trajectory,   # same handler
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb,
        )

        # 3. Gripper controller – GripperCommand (default: true)
        self._gripper_server = ActionServer(
            self, GripperCommand,
            'gripper_controller/gripper_cmd',
            execute_callback=self._exec_gripper_command,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb,
        )

        # 4. Gripper effort controller – FollowJointTrajectory alias
        self._gripper_effort_server = ActionServer(
            self, FollowJointTrajectory,
            'gripper_effort_controller/follow_joint_trajectory',
            execute_callback=self._exec_gripper_fj_trajectory,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb,
        )

        self._cancel_flags: dict[str, bool] = {}

        self.get_logger().info(
            f"SO101MoveItBridge ready | port={self.port} | arm={self.robot_name}"
        )
        self.get_logger().info(
            "Action servers:\n"
            "  /arm_controller/follow_joint_trajectory\n"
            "  /arm_effort_controller/follow_joint_trajectory\n"
            "  /gripper_controller/gripper_cmd\n"
            "  /gripper_effort_controller/follow_joint_trajectory"
        )

    # ── Arm connection ─────────────────────────────────────────────────────────

    def _connect_arm(self) -> SO101:
        robot = SO101(port=self.port, name=self.robot_name,
                      recalibrate=self.recalibrate)
        try:
            self.get_logger().info("Connecting to SO101 arm …")
            robot.connect()
            self.get_logger().info("SO101 arm connected.")
            return robot
        except Exception as e:
            self.get_logger().fatal(f"Could not connect to arm: {e}")
            raise

    # ── /joint_states ──────────────────────────────────────────────────────────

    def _publish_joint_states(self):
        if self.robot is None:
            return
        try:
            # get_device_state() returns {bus_name: degrees}
            state = self.robot.get_device_state()

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()

            # Publish using MoveIt joint names so the planning scene matches URDF
            msg.name     = []
            msg.position = []
            for bus_name, deg in state.items():
                moveit_name = BUS_TO_MOVEIT.get(bus_name, bus_name)
                msg.name.append(moveit_name)
                msg.position.append(math.radians(deg))

            self.state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"joint_states read failed: {e}")

    # ── Generic action callbacks ───────────────────────────────────────────────

    def _accept_goal(self, _):
        return GoalResponse.ACCEPT

    def _accept_cancel(self, goal_handle):
        gid = str(goal_handle.goal_id.uuid)
        self._cancel_flags[gid] = True
        return CancelResponse.ACCEPT

    def _is_cancelled(self, goal_handle) -> bool:
        gid = str(goal_handle.goal_id.uuid)
        return self._cancel_flags.get(gid, False)

    def _clear_cancel(self, goal_handle):
        gid = str(goal_handle.goal_id.uuid)
        self._cancel_flags.pop(gid, None)

    # ── Arm FollowJointTrajectory executor ────────────────────────────────────

    def _exec_arm_trajectory(self, goal_handle: ServerGoalHandle):
        """Execute a FollowJointTrajectory goal for the arm joints."""
        self._clear_cancel(goal_handle)
        traj      = goal_handle.request.trajectory
        jnt_names = traj.joint_names          # MoveIt names (Shoulder_Rotation …)
        points    = traj.points

        if not points:
            self.get_logger().warn("Received empty arm trajectory – aborting.")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        n_pts = len(points)
        total = _duration_sec(points[-1])
        self.get_logger().info(
            f"Arm trajectory: {n_pts} waypoints over {total:.2f} s"
        )

        feedback = FollowJointTrajectory.Feedback()
        start_wall = time.monotonic()

        for i, point in enumerate(points):
            if self._is_cancelled(goal_handle) or not goal_handle.is_active:
                self.get_logger().info("Arm trajectory cancelled.")
                goal_handle.canceled()
                self._clear_cancel(goal_handle)
                return FollowJointTrajectory.Result()

            # Sleep to waypoint time
            target_t = _duration_sec(point)
            elapsed  = time.monotonic() - start_wall
            wait     = target_t - elapsed
            if wait > 0:
                time.sleep(wait)

            # Translate MoveIt joint names → bus names, radians → degrees
            goal_deg: dict[str, float] = {}
            for moveit_name, pos_rad in zip(jnt_names, point.positions):
                bus_name = MOVEIT_TO_BUS.get(moveit_name, moveit_name)
                goal_deg[bus_name] = math.degrees(pos_rad)

            try:
                self.robot._bus.sync_write("Goal_Position", goal_deg)
            except Exception as e:
                self.get_logger().error(f"sync_write failed at waypoint {i}: {e}")
                goal_handle.abort()
                self._clear_cancel(goal_handle)
                return FollowJointTrajectory.Result()

            # Publish feedback (best-effort)
            try:
                state = self.robot.get_device_state()
                feedback.joint_names = [
                    BUS_TO_MOVEIT.get(n, n) for n in state.keys()
                ]
                feedback.actual.positions = [
                    math.radians(d) for d in state.values()
                ]
            except Exception:
                pass
            goal_handle.publish_feedback(feedback)

        self.get_logger().info("Arm trajectory complete.")
        goal_handle.succeed()
        self._clear_cancel(goal_handle)
        return FollowJointTrajectory.Result()

    # ── Gripper GripperCommand executor ───────────────────────────────────────

    def _exec_gripper_command(self, goal_handle: ServerGoalHandle):
        """
        Execute a GripperCommand goal.

        MoveIt sends:
          goal.command.position   (float, metres or radians – treat as normalised 0–1
                                   or map directly; tune GRIPPER_OPEN/CLOSED_DEG above)
          goal.command.max_effort (float, ignored for position control)
        """
        self._clear_cancel(goal_handle)
        cmd_pos = goal_handle.request.command.position   # radians from MoveIt

        # Map MoveIt gripper position (radians) → hardware degrees
        # Adjust this formula to match your URDF gripper joint limits
        gripper_deg = math.degrees(cmd_pos)
        gripper_deg = max(GRIPPER_OPEN_DEG,
                          min(GRIPPER_CLOSED_DEG, gripper_deg))

        self.get_logger().info(
            f"Gripper command: {cmd_pos:.4f} rad → {gripper_deg:.1f} deg"
        )

        try:
            self.robot._bus.sync_write(
                "Goal_Position",
                {"gripper": gripper_deg}
            )
        except Exception as e:
            self.get_logger().error(f"Gripper sync_write failed: {e}")
            goal_handle.abort()
            self._clear_cancel(goal_handle)
            return GripperCommand.Result()

        # Brief settle time then read back
        time.sleep(0.3)

        result = GripperCommand.Result()
        try:
            state = self.robot.get_device_state()
            actual_deg = state.get("gripper", gripper_deg)
            result.position      = math.radians(actual_deg)
            result.reached_goal  = abs(actual_deg - gripper_deg) < 2.0  # 2° tolerance
            result.stalled       = False
            result.effort        = 0.0
        except Exception:
            result.position     = math.radians(gripper_deg)
            result.reached_goal = True

        goal_handle.succeed()
        self._clear_cancel(goal_handle)
        return result

    # ── Gripper FollowJointTrajectory alias ───────────────────────────────────

    def _exec_gripper_fj_trajectory(self, goal_handle: ServerGoalHandle):
        """
        Handle FollowJointTrajectory for gripper_effort_controller.
        Extracts the Gripper joint and executes it.
        """
        self._clear_cancel(goal_handle)
        traj      = goal_handle.request.trajectory
        jnt_names = traj.joint_names
        points    = traj.points

        if not points or GRIPPER_MOVEIT_JOINT not in jnt_names:
            self.get_logger().warn("Gripper FJT: no Gripper joint found – aborting.")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        gripper_idx = jnt_names.index(GRIPPER_MOVEIT_JOINT)
        start_wall  = time.monotonic()

        for i, point in enumerate(points):
            if self._is_cancelled(goal_handle) or not goal_handle.is_active:
                goal_handle.canceled()
                self._clear_cancel(goal_handle)
                return FollowJointTrajectory.Result()

            target_t = _duration_sec(point)
            wait     = target_t - (time.monotonic() - start_wall)
            if wait > 0:
                time.sleep(wait)

            gripper_deg = math.degrees(point.positions[gripper_idx])
            gripper_deg = max(GRIPPER_OPEN_DEG, min(GRIPPER_CLOSED_DEG, gripper_deg))

            try:
                self.robot._bus.sync_write(
                    "Goal_Position", {"gripper": gripper_deg}
                )
            except Exception as e:
                self.get_logger().error(f"Gripper FJT write failed at pt {i}: {e}")
                goal_handle.abort()
                self._clear_cancel(goal_handle)
                return FollowJointTrajectory.Result()

        goal_handle.succeed()
        self._clear_cancel(goal_handle)
        return FollowJointTrajectory.Result()

    # ── Cleanup ────────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.robot is not None and self.robot.is_connected:
            self.get_logger().info("Disconnecting SO101 arm …")
            self.robot.disconnect()
        super().destroy_node()


# ── Helpers ────────────────────────────────────────────────────────────────────

def _duration_sec(point: JointTrajectoryPoint) -> float:
    """Convert time_from_start (ROS Duration) to float seconds."""
    return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SO101MoveItBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()