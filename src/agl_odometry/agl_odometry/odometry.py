import rclpy
from rclpy.node import Node
import serial
from time import sleep
from geometry_msgs.msg import Twist
from agl_interfaces.msg import TwoAngularSpeeds
from agl_interfaces.msg import OdometryMsg
import math

class OdomPublisherSubscriber(Node):

  def __init__(self):
      super().__init__('odom_TwoAngularSpeeds_subscriber')

      self.declare_parameters(
      namespace='',
      parameters=[
          ('subs_topic', '/TwoAngularSpeeds'),
          ('pub_topic', '/OdometryMsg'),
          ('radius', 5.0),
          ('wheel_separation', 32.0)
      ]
      )

      self.subs_topic = self.get_param_str('subs_topic')
      self.pub_topic = self.get_param_str('pub_topic')
      self.radius = self.get_param_float('radius')
      self.wheel_separation = self.get_param_float('wheel_separation')

      self.deltaTimes = 1.0
      self.OdometryMsg = OdometryMsg()
      self.OdometryMsg.orientation = 0.0
      self.OdometryMsg.x_position = 0.0
      self.OdometryMsg.y_position = 0.0

      self.publisher = self.create_publisher(TwoAngularSpeeds, self.pub_topic, 10)
      self.subscription = self.create_subscription(TwoAngularSpeeds,self.subs_topic,self.listener_callback,10)
      self.subscription 

  def get_param_float(self, name):
    try:
      return float(self.get_parameter(name).get_parameter_value().double_value)
    except:
      pass
  def get_param_str(self, name):
    try:
      return self.get_parameter(name).get_parameter_value().string_value
    except:
      pass

  def listener_callback(self, msg):
    # This function will recieve feedback data from arduino in TwoAngularSpeeds formats.
      
      self.currentTime = time.time()

      self.deltaTimes = self.currentTime - self.lastTime

      self.linear_vel = (self.radius/2) * (msg.right_wheel_angular_speed + msg.left_wheel_angular_speed)
      self.angular_vel = (self.radius/self.wheel_separation) * (msg.right_wheel_angular_speed - msg.left_wheel_angular_speed)
      # A self.angular_vel deberíamos restarle la velocidad angular de la rueda loca?

      # Haremos un acumulativo con suma para sumarle o restarle a las posiciones o orientación y ver un cambio respecto a datos anteriores.
      OdometryMsg.orientation = OdometryMsg.orientation + (self.angular_vel * self.deltaTimes)
      OdometryMsg.x_position = OdometryMsg.x_position + (self.linear_vel * math.cos(self.orientation) * self.deltaTimes) 
      OdometryMsg.y_position = OdometryMsg.y_position + (self.linear_vel * math.sin(self.orientation) * self.deltaTimes) 

      self.publisher.publish(OdometryMsg)
      self.get_logger().info('Publishing: "%s"' % OdometryMsg)

      self.lastTime = time.time()

def main(args=None):
    rclpy.init(args=args)
    odom_TwoAngularSpeeds_subscriber = OdomPublisherSubscriber()
    rclpy.spin(odom_TwoAngularSpeeds_subscriber)
    odom_TwoAngularSpeeds_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()