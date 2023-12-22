import rclpy
from rclpy.node import Node
import serial
from time import sleep
from geometry_msgs.msg import Twist
from agl_interfaces.msg import TwoAngularSpeeds

class CmdVelPublisherSubscriber(Node):

  def __init__(self):
      super().__init__('cmd_vel_publisher_subscriber')

      self.declare_parameters(
      namespace='',
      parameters=[
        ('device', '/dev/ttyACM0'), #device we are trasmitting to & recieving messages from
          ('topic', 'TwoAngularSpeeds'),
          ('radius', 5.0),
          ('wheel_separation', 32.0)
      ]
      )

      self.device_name = self.get_param_str('device')
      self.topic = self.get_param_str('topic')
      self.radius = self.get_param_float('radius')
      self.wheel_separation = self.get_param_float('wheel_separation')
      self.ser = serial.Serial(self.device_name,
                            115200, #Note: Baud Rate must be the same in the arduino program, otherwise signal is not recieved!
                            timeout=0.1)
      
      self.ser.reset_input_buffer()

      self.timer_period = 0.4  # In seconds

      self.publisher = self.create_publisher(TwoAngularSpeeds, self.topic, 10)
      self.timer = self.create_timer(self.timer_period, self.timer_callback)

      self.subscriber = self.create_subscription(TwoAngularSpeeds, self.topic, self.listener_callback,10)
      self.subscriber # See if necessary

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
        # self.get_logger().info('He escuchado: "%s"' % msg)
        
        self.rightLinearSpeed = msg.right_wheel_angular_speed * (self.radius)
        self.leftLinearSpeed = msg.left_wheel_angular_speed * (self.radius)

        self.rightWheelAngularSpeed = (self.rightLinearSpeed + msg.right_wheel_angular_speed * self.wheel_separation / 2) / self.radius
        self.leftWheelAngularSpeed = (self.leftLinearSpeed - msg.left_wheel_angular_speed * self.wheel_separation / 2) / self.radius

        self.valueToSendRightWheel = (int)(self.rightWheelAngularSpeed * 100)
        self.valueToSendLeftWheel = (int)(self.leftWheelAngularSpeed * 100)

        self.write(str(self.valueToSendRightWheel) + "\n")
        self.write(str(self.valueToSendLeftWheel )+ "\n")

        # self.valueToSendRightWheel = (int)(msg.angular.z * 100)
        # self.rightWheelAngularSpeed = (msg.linear.x + msg.angular.z * self.wheel_separation / 2) / self.radius;
        # self.leftWheelAngularSpeed = (msg.linear.x - msg.angular.z * self.wheel_separation / 2) / self.radius;
        # self.write(str(self.rightWheelAngularSpeed) + "\n")
        # self.write(str(self.leftWheelAngularSpeed )+ "\n")

        # self.valueToSendRightWheel = 500
        # self.write(str(self.valueToSendRightWheel) + "\n")
        # self.write(str(self.valueToSendRightWheel) + "\n")

  def timer_callback(self):
    # Read Wr, Wl, in that order, via serial from arduino.
    value = self.recv()

    print(f"RECEIVED DATA: {value}") 
    
    try:
      decoded = value.decode("ascii")
      cmd_vel_array = decoded.split()
      msgAngularSpeedsOfWheels = TwoAngularSpeeds()
      # Accessing arduino's cmd_vel_array based on its length allow us to gather the last two values from the serial buffer (Could be reading slower than we write in buffer).
      msgAngularSpeedsOfWheels.right_wheel_angular_speed = float(cmd_vel_array[len(cmd_vel_array) - 2])/100.0   # Arduino's speeds are in 100 order, divide by 100 to get real speeds.
      msgLeftWheel.left_wheel_angular_speed = float(cmd_vel_array[len(cmd_vel_array) - 1])/100.0
      self.publisher.publish(msgAngularSpeedsOfWheels)
      self.get_logger().info('Publishing: "%s"' % msgAngularSpeedsOfWheels)
    except:
      self.get_logger().warn('Fallo buffer')

  def recv(self):
    dataBytesRead = self.ser.inWaiting()
    data = self.ser.read(dataBytesRead)   # For reading floating values.
    return data

  def write(self, x):
    print(f"Valor que estamos enviando a nuestro arduino: {x}")
    self.ser.write(bytes(x, 'utf-8'))

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher_subscriber = CmdVelPublisherSubscriber()
    rclpy.spin(cmd_vel_publisher_subscriber)
    cmd_vel_publisher_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# El publisher tendrá que leer el puerto serie constantemente para asegurarse de que haya llegado información.