import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from agl_interfaces.msg import TwoAngularSpeeds
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header

debug = False
no_EKF = True

class OdomPublisherSubscriber(Node):

  def __init__(self):
      super().__init__('odom_TwoAngularSpeeds_subscriber')

      self.declare_parameters(
      namespace='',
      parameters=[
          ('subs_topic', '/TwoAngularSpeeds'),
          ('pub_topic', '/Odometry/odom'),
          ('radius', 0.05),
          ('wheel_separation', 0.32)
      ]
      )

      self.subs_topic = self.get_param_str('subs_topic')
      self.pub_topic = self.get_param_str('pub_topic')
      self.radius = self.get_param_float('radius')
      self.wheel_separation = self.get_param_float('wheel_separation')

      self.first_time_in_subs_callback = True
      self.orientation = 0.0
      self.x_position = 0.0
      self.y_position = 0.0
      self.tf_broadcaster = TransformBroadcaster(self)

      self.publisher = self.create_publisher(Odometry, self.pub_topic, 10)
      self.subscription = self.create_subscription(TwoAngularSpeeds,self.subs_topic,self.listener_callback,10)
      self.subscription 

      self.x_coord_factor = 1.211

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

    if self.first_time_in_subs_callback:
       deltaTimes = 1.0
       self.first_time_in_subs_callback = False
    else:
      deltaTimes = self.currentTime - self.lastTime

    self.linear_vel = (self.radius/2) * (msg.right_wheel_angular_speed + msg.left_wheel_angular_speed)
    self.angular_vel = (self.radius/self.wheel_separation) * (msg.right_wheel_angular_speed - msg.left_wheel_angular_speed)
      
    # We'll acumulate in positions variables the increment or decrement of robot's x,y coordinates. 
    self.orientation =  (self.orientation + (self.angular_vel * deltaTimes))
    self.x_position = self.x_position + (self.linear_vel * math.cos(self.orientation) * deltaTimes) * self.x_coord_factor
    self.y_position = self.y_position + (self.linear_vel * math.sin(self.orientation) * deltaTimes)

    if self.orientation > 6.28:
      self.orientation -= 6.28
    elif self.orientation < -6.28:
      self.orientation += 6.28


    self.OdometryMsg = Odometry()

    self.OdometryMsg.header.stamp = rclpy.clock.Clock().now().to_msg()
    self.OdometryMsg.header.frame_id = "odom"
    self.OdometryMsg.child_frame_id = "base_link"

    self.OdometryMsg.pose.pose.position.x = self.x_position
    self.OdometryMsg.pose.pose.position.y = self.y_position
    self.OdometryMsg.pose.pose.position.z = 0.0

    self.OdometryMsg.twist.twist.linear.x = self.linear_vel
    self.OdometryMsg.twist.twist.linear.y = 0.0
    self.OdometryMsg.twist.twist.angular.z = self.angular_vel

    self.odom_quat = Quaternion()
    self.odom_quat.x = 0.0
    self.odom_quat.y = 0.0
    self.odom_quat.z = math.sin(self.orientation/2)
    self.odom_quat.w = math.cos(self.orientation/2)

    self.OdometryMsg.pose.pose.orientation = self.odom_quat

    #Publish transform if testing only with wheel odometry, no EKF no IMU

    if no_EKF:
  
      self.OdometryMsgTf = TransformStamped()

      self.OdometryMsgTf.header = Header()
      self.OdometryMsgTf.header.stamp = rclpy.clock.Clock().now().to_msg()
      self.OdometryMsgTf.header.frame_id = "odom"
      self.OdometryMsgTf.child_frame_id = "base_link"

      self.OdometryMsgTf.transform.translation.x = self.x_position;
      self.OdometryMsgTf.transform.translation.y = self.y_position;
      self.OdometryMsgTf.transform.translation.z = 0.0;
      self.OdometryMsgTf.transform.rotation = self.odom_quat;
    
      self.tf_broadcaster.sendTransform(self.OdometryMsgTf)

    self.publisher.publish(self.OdometryMsg)
    
    if debug:
      print(f"Linear velocity recieved: {self.linear_vel}") 
      print(f"Angular velocity recieved: {self.angular_vel}")
      self.get_logger().info('Publishing: "%s"' % self.OdometryMsg)

    self.lastTime = time.time()

def main(args=None):
    rclpy.init(args=args)
    odom_TwoAngularSpeeds_subscriber = OdomPublisherSubscriber()
    rclpy.spin(odom_TwoAngularSpeeds_subscriber)
    odom_TwoAngularSpeeds_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()