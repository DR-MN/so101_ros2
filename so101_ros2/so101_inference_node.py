

#without the converstions ,straght values pass
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from so101_ros2.lerobot.so101 import SO101

class SO101InferenceNode(Node):
    """
    Single-arm node for inference.
    - Publishes  : /joint_states   (sensor_msgs/JointState)
    - Subscribes : /joint_commands (trajectory_msgs/JointTrajectory)
    No unit conversion — raw values passed through in both directions.
    """

    JOINT_NAMES = [
        "shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper",
    ]

    def __init__(self):
        super().__init__('so101_inference_node')

        # Parameters
        self.declare_parameter('robot_name', 'so101_follower')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('recalibrate', False)
        self.declare_parameter('publish_rate', 30.0)

        self.robot_name  = self.get_parameter('robot_name').value
        self.port        = self.get_parameter('port').value
        self.recalibrate = self.get_parameter('recalibrate').value
        pub_rate_hz      = self.get_parameter('publish_rate').value

        # Publisher: current joint states
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscriber: commanded joint positions from the policy (JointTrajectory)
        self.cmd_sub = self.create_subscription(
            JointTrajectory, '/joint_commands', self._command_callback, 10)

        # Timer: periodically read and publish arm state
        self.timer = self.create_timer(1.0 / pub_rate_hz, self._publish_joint_states)

        # Connect arm
        self.robot = self._init_arm()

        self.get_logger().info(
            f"SO101InferenceNode ready | port={self.port} | "
            f"name={self.robot_name} | rate={pub_rate_hz} Hz"
        )

    def _init_arm(self):
        robot = SO101(port=self.port, name=self.robot_name, recalibrate=self.recalibrate)
        try:
            self.get_logger().info("Connecting to SO101 arm...")
            robot.connect()
            self.get_logger().info("SO101 arm connected.")
            return robot
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            rclpy.shutdown()
            return None

    def _publish_joint_states(self):
        """Read arm → publish /joint_states (raw values, no conversion)."""
        if self.robot is None:
            return
        try:
            state: dict = self.robot.get_device_state()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name     = list(state.keys())
            msg.position = list(state.values())  # raw values as-is
            self.state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Could not read joint states: {e}")

    def _command_callback(self, msg: JointTrajectory):
        """Receive /joint_commands → drive arm (raw values, no conversion)."""
        if self.robot is None:
            self.get_logger().warn("Arm not initialised – ignoring command.")
            return

        if not msg.points:
            self.get_logger().warn("JointTrajectory has no points, ignoring.")
            return

        point = msg.points[0]

        # Pass values as-is, keep only known joints
        goal = {
            name: pos
            for name, pos in zip(msg.joint_names, point.positions)
            if name in self.JOINT_NAMES
        }

        if not goal:
            self.get_logger().warn("Command message had no valid joint names.")
            return

        try:
            self.robot._bus.sync_write("Goal_Position", goal)
            self.get_logger().debug(f"Command sent: {goal}")
        except Exception as e:
            self.get_logger().error(f"Error writing command to arm: {e}")

    def destroy_node(self):
        if self.robot is not None and self.robot.is_connected:
            self.get_logger().info("Disconnecting SO101 arm...")
            self.robot.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SO101InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




#publish raw values, joint state as joint state msg, joint command as trajectory msg.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from so101_ros2.lerobot.so101 import SO101


class SO101InferenceNode(Node):
    """
    Single-arm node for inference.
    - Publishes  : /joint_states   (sensor_msgs/JointState)
    - Subscribes : /joint_commands (sensor_msgs/JointState)

    No unit conversion — raw values passed through in both directions.
    """

    JOINT_NAMES = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]

    def __init__(self):
        super().__init__('so101_inference_node')

        # Parameters
        self.declare_parameter('robot_name', 'so101_follower')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('recalibrate', False)
        self.declare_parameter('publish_rate', 30.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.port = self.get_parameter('port').value
        self.recalibrate = self.get_parameter('recalibrate').value
        pub_rate_hz = self.get_parameter('publish_rate').value

        # Publisher: current joint states
        self.state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Subscriber: commanded joint positions
        self.cmd_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self._command_callback,
            10
        )

        # Timer: periodically read and publish arm state
        self.timer = self.create_timer(
            1.0 / pub_rate_hz,
            self._publish_joint_states
        )

        # Connect arm
        self.robot = self._init_arm()

        self.get_logger().info(
            f"SO101InferenceNode ready | port={self.port} | "
            f"name={self.robot_name} | rate={pub_rate_hz} Hz"
        )

    def _init_arm(self):
        robot = SO101(
            port=self.port,
            name=self.robot_name,
            recalibrate=self.recalibrate
        )

        try:
            self.get_logger().info("Connecting to SO101 arm...")
            robot.connect()
            self.get_logger().info("SO101 arm connected.")
            return robot

        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            rclpy.shutdown()
            return None

    def _publish_joint_states(self):
        """Read arm → publish /joint_states (raw values)."""
        if self.robot is None:
            return

        try:
            state: dict = self.robot.get_device_state()

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(state.keys())
            msg.position = list(state.values())

            self.state_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(
                f"Could not read joint states: {e}"
            )

    def _command_callback(self, msg: JointState):
        """Receive JointState → drive arm (raw values)."""
        if self.robot is None:
            self.get_logger().warn(
                "Arm not initialised – ignoring command."
            )
            return

        goal = {
            name: pos
            for name, pos in zip(msg.name, msg.position)
            if name in self.JOINT_NAMES
        }

        if not goal:
            self.get_logger().warn(
                "Command message had no valid joint names."
            )
            return

        try:
            self.robot._bus.sync_write(
                "Goal_Position",
                goal
            )
            self.get_logger().debug(
                f"Command sent: {goal}"
            )

        except Exception as e:
            self.get_logger().error(
                f"Error writing command to arm: {e}"
            )

    def destroy_node(self):
        if self.robot is not None and self.robot.is_connected:
            self.get_logger().info(
                "Disconnecting SO101 arm..."
            )
            self.robot.disconnect()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SO101InferenceNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()