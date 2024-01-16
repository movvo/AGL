import rclpy
from rclpy.node import Node
import serial
from time import sleep
from geometry_msgs.msg import Twist
from agl_interfaces.msg import TwoAngularSpeeds

class ArduinoRosSerialServer(Node):

  def __init__(self):
      super().__init__('arduino_ros_serial_server')

      self.declare_parameters(
        namespace='',
        parameters=[
          ('device', '/dev/ttyACM0'), #device we are trasmitting to & recieving messages from
          ('topic', 'TwoAngularSpeeds'),
          ('subs_topic', '/cmd_vel'),
          ('radius', 0.05),
          ('wheel_separation', 0.32)
        ]
      )

      self.device_name = self.get_param_str('device')
      self.topic = self.get_param_str('topic')
      self.subs_topic = self.get_param_str('subs_topic')
      self.radius = self.get_param_float('radius')
      self.wheel_separation = self.get_param_float('wheel_separation')
      self.ser = serial.Serial(self.device_name,
                            230400, #Note: Baud Rate must be the same in the arduino program, otherwise signal is not recieved!
                            timeout=0.1)
      
      self.timer_period = 0.25  # In seconds
      self.setVelZero = True

      # We should subscribe to cmd_vel and publish via custom message.
      self.subscription = self.create_subscription(Twist,self.subs_topic,self.listener_callback,10)
      self.subscription 

      self.publisher = self.create_publisher(TwoAngularSpeeds, self.topic, 10)
      self.timer = self.create_timer(self.timer_period, self.timer_callback)

      
  def get_param_float(self, name):
    return float(self.get_parameter(name).get_parameter_value().double_value)
    
  def get_param_str(self, name):
    return self.get_parameter(name).get_parameter_value().string_value

  def listener_callback(self, msg):
    # This function will recieve data from joystick so it's necessary to process cmd_vel information not from the custom message we just created.

    if msg.angular.z != 0 or msg.linear.x != 0:

      self.setVelZero = True

      self.rightWheelAngularSpeed = (msg.linear.x + msg.angular.z * self.wheel_separation / 2) / self.radius
      self.leftWheelAngularSpeed = (msg.linear.x - msg.angular.z * self.wheel_separation / 2) / self.radius

      if msg.angular.z == 0.0:
        # To transform only the linear velocity, we can use a simple proportional relationship.
        # Our joystick values range from [-0.2; 0] U [0; 0.2] for linear velocity.
        # We can consider only the positive values.
        # By using a proportional relationship for values between 0 and 2, we can determine the equivalent values in [0, 620].
        # There's no need to check for values greater than 420, as the Arduino part already takes care of these aspects.

        correctedVel = (self.rightWheelAngularSpeed * 620)/4
        valueToSendRightWheel = (int)(correctedVel)
        valueToSendLeftWheel = (int)(correctedVel)
      else:
        correctedVelRight = (self.rightWheelAngularSpeed * 620)/4.64
        correctedVelLeft = (self.leftWheelAngularSpeed * 620)/4.64
        valueToSendRightWheel = (int)(correctedVelRight)
        valueToSendLeftWheel = (int)(correctedVelLeft)

        self.ser.reset_output_buffer()
        self.write(str(valueToSendRightWheel) + "\n")
        self.write(str(valueToSendLeftWheel )+ "\n")

    else:

      if self.setVelZero:
        self.setVelZero = False
        valueToSendRightWheel = 0
        valueToSendLeftWheel = 0

        self.ser.reset_output_buffer()
        self.write(str(valueToSendRightWheel) + "\n")
        self.write(str(valueToSendLeftWheel )+ "\n")
    

    

  def timer_callback(self):
    # Once we've published the feedback from arduino we should listen to this information in the odom package. (Subscriber to TwoAngularSpeeds)

    # Read Wr, Wl, in that order, via serial from arduino.
    left_right_wheel_speed_arduino_feedback = self.recv()    
    if len(left_right_wheel_speed_arduino_feedback) > 0:
      msgAngularSpeedsOfWheels = TwoAngularSpeeds()
      # Accessing arduino's cmd_vel_array based on its length allow us to gather the last two values from the serial buffer (Could be reading slower than we write in buffer).
      msgAngularSpeedsOfWheels.right_wheel_angular_speed = float(left_right_wheel_speed_arduino_feedback[len(left_right_wheel_speed_arduino_feedback) - 2])/100.0   # Arduino's speeds are in 100 order, divide by 100 to get real speeds.
      msgAngularSpeedsOfWheels.left_wheel_angular_speed = float(left_right_wheel_speed_arduino_feedback[len(left_right_wheel_speed_arduino_feedback) - 1])/100.0
      self.publisher.publish(msgAngularSpeedsOfWheels)

  def recv(self):
    dataBytesRead = self.ser.inWaiting()
    data = self.ser.read(dataBytesRead)   # For reading floating values.

    try:
      decoded = data.decode("ascii")
      cmd_vel_array = decoded.split()
    except:
      self.get_logger().warn('Serial recieve buffer empty')

    return cmd_vel_array

  def write(self, x):
    self.ser.write(bytes(x, 'utf-8'))

def main(args=None):
    rclpy.init(args=args)
    arduino_ros_serial_server_node = ArduinoRosSerialServer()
    rclpy.spin(arduino_ros_serial_server_node)
    arduino_ros_serial_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# El publisher tendrá que leer el puerto serie constantemente para asegurarse de que haya llegado información.