import rclpy
from rclpy.node import Node
import serial
from time import sleep
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):

  def __init__(self):
      super().__init__('cmd_vel_publisher')

      self.declare_parameters(
        namespace='',
        parameters=[
          ('device', '/dev/ttyACM0'), #device we are trasmitting to & recieving messages from
            ('topic', 'cmd_vel'),
        ]
      )

      self.device_name = self.get_param_str('device')
      self.topic = self.get_param_str('topic')
      self.ser = serial.Serial(self.device_name,
                            115200, #Note: Baud Rate must be the same in the arduino program, otherwise signal is not recieved!
                            timeout=0.1)
      self.ser.reset_input_buffer()

      self.publisher_leftWheel = self.create_publisher(Twist, self.topic, 10)
      timer_period = 0.5  # In seconds
      self.timer_leftWheel = self.create_timer(timer_period, self.timer_callback)

      self.publisher_rightWheel = self.create_publisher(Twist, self.topic, 10)
      self.timerRightWheel = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    num = 0.0
    strValue = 0
    num = 6.2
    strValue = (int)(num * 100)
    
    # Print angular speed for arduino via serial.
    self.write(str(strValue))

    # Read Wr, Vr, Wl, Vl, in that order, via serial from arduino.
    value = self.recv()

    print(f"RECEIVED DATA: {value}") 
    decoded = value.decode("ascii")
    cmd_vel_array = decoded.split()
    
    msgRightWheel = Twist()
    msgRightWheel.angular.z = float(cmd_vel_array[0])/100.0   # Arduino's speeds are in 100 order, divide by 100 to get real speeds.
    msgRightWheel.linear.x = float(cmd_vel_array[1])/100.0
    self.publisher_rightWheel.publish(msgRightWheel)
    self.get_logger().info('Publicando: "%s"' % msgRightWheel)

    msgLeftWheel = Twist()
    msgLeftWheel.angular.z = float(cmd_vel_array[2])/100.0
    msgLeftWheel.linear.x = float(cmd_vel_array[3])/100.0 
    self.publisher_leftWheel.publish(msgLeftWheel)
    self.get_logger().info('Publicando: "%s"' % msgLeftWheel)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    rclpy.spin(cmd_vel_publisher)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
