import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SquareMovement(Node):

  def __init__(self):
    super().__init__('square_movement')
    self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
    self.send_commands()

  def send_commands(self):
    # Define movement commands (linear_x, angular_z, duration)
    commands = [
      (0.4, 0.0, 2.0),
      (0.0, 0.4, 1.57),
      (0.4, 0.0, 2.0),
      (0.0, 0.4, 1.57),
      (0.4, 0.0, 2.0),
      (0.0, 0.4, 1.57),
      (0.4, 0.0, 2.0),
    ]

    for cmd in commands:
      twist_msg = Twist()
      twist_msg.linear.x = cmd[0]
      twist_msg.angular.z = cmd[1]
      self.publisher_.publish(twist_msg)
      rclpy.time.sleep(rclpy.duration.Duration(seconds=cmd[2]))  # Wait between commands

def main():
  rclpy.init()
  node = SquareMovement()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()