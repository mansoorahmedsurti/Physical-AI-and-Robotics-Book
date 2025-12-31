# ROS 2 Node for Motion Planning

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planning_node')
        # Subscribe to commands (e.g., from voice recognition or task planner)
        self.command_subscriber = self.create_subscription(
            String, 'voice_commands', self.command_callback, 10)
        # Publish motion commands for motor control
        self.motion_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Motion Planning Node Initialized')

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')
        motion_cmd = self.generate_motion_command(command)
        if motion_cmd:
            self.publish_motion_command(motion_cmd)

    def generate_motion_command(self, command):
        # Placeholder for motion planning logic
        # This would translate high-level commands into specific joint/velocities
        twist_msg = Twist()
        if "forward" in command.lower():
            twist_msg.linear.x = 0.5 # Move forward at 0.5 m/s
        elif "backward" in command.lower():
            twist_msg.linear.x = -0.5 # Move backward at 0.5 m/s
        elif "left" in command.lower():
            twist_msg.angular.z = 0.5 # Turn left at 0.5 rad/s
        elif "right" in command.lower():
            twist_msg.angular.z = -0.5 # Turn right at 0.5 rad/s
        elif "stop" in command.lower():
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            self.get_logger().warn(f'Unknown command: "{command}" - No motion generated.')
            return None
        return twist_msg

    def publish_motion_command(self, motion_cmd):
        self.motion_publisher.publish(motion_cmd)
        self.get_logger().info('Published motion command')

def main(args=None):
    rclpy.init(args=args)
    motion_planning_node = MotionPlanningNode()
    rclpy.spin(motion_planning_node)
    motion_planning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
