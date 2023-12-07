# # Serial server node
# import rclpy
# from rclpy.node import Node
# import serial   #sudo apt install python3-serial
# from time import sleep

# # Handle Twist messages, linear and angular velocity
# from geometry_msgs.msg import Twist
# class SerialServer(Node):

#   def __init__(self):
#     super().__init__('serial_server')
#     #Default Value declarations of ros2 params:
#     self.declare_parameters(
#     namespace='',
#     parameters=[
#       ('device', '/dev/ttyACM0'), #device we are trasmitting to & recieving messages from
#         ('wheel_instructions_topic', 'wheel_instructions_topic'),
#     ]
#     )
#     self.device_name = self.get_param_str('device')
#     self.wheel_topic_name = self.get_param_str('wheel_instructions_topic')
#     self.ser = serial.Serial(self.device_name,
#                            115200, #Note: Baud Rate must be the same in the arduino program, otherwise signal is not recieved!
#                            timeout=0.1)
#     self.ser.reset_input_buffer()

#   def get_param_str(self, name):
#     try:
#       return self.get_parameter(name).get_parameter_value().string_value
#     except:
#       pass
    
#   def recv(self):
#     dataBytesRead = self.ser.inWaiting()
#     data = self.ser.read(dataBytesRead)   # For reading floating values.
#     return data

#   def write(self, x):
#     print(f"Valor que estamos enviando a nuestro arduino: {x}")
#     self.ser.write(bytes(x, 'utf-8'))
#     # dataBytesRead = self.ser.inWaiting()
#     # data = self.ser.read(dataBytesRead)   # For reading floating values.
#     # return data


  

# def main(args=None):
#     rclpy.init(args=args)
#     serial_server = SerialServer()
#     # serial_server.recieve_cmd()
#     sleep(3)
#     num = 0.0
#     strValue = 0
#     while True:
#       # num = input("Enter a number: ") # Taking input from user
#       num = 6.2
#       strValue = (int)(num * 100)
      
#       # Print angular speed for arduino via serial.
#       serial_server.write(str(strValue))

#       # Read Wr, Vr, Wl, Vl, in that order, via serial from arduino.
#       value = serial_server.recv()
#       print(f"RECEIVED DATA: {value}") # printing the value
#       decoded = value.decode("ascii")
#       cmd_vel_array = decoded.split()
#       for iterator in cmd_vel_array:      
#         try:
#           decodedFloatingNumber = float(iterator)/100.0
#           print("Decoded String:", decodedFloatingNumber)
#           print("\n")
#         except:
#           pass

#       if num >8:
#         num=0
      
#       sleep(0.150)

# if __name__ == '__main__':
#   main()

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

    sleep(3)
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
    
    ########### TODO: Differential velocities
    wheel_1_angular_velocity = (msgRightWheel->linear.x + msgRightWheel->angular.z * parameters_.wheel_separation.as_double() / 2) / parameters_.wheel_radius.as_double();
    wheel_2_angular_velocity = (msgLeftWheel->linear.x - msgLeftWheel->angular.z * parameters_.wheel_separation.as_double() / 2) / parameters_.wheel_radius.as_double();

    sleep(0.150)


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    rclpy.spin(cmd_vel_publisher)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
