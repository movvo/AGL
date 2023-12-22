import rclpy
from rclpy.node import Node
import serial
from time import sleep
from geometry_msgs.msg import Twist

radius = 5.0

class CmdVelPublisherSubscriber(Node):

  def __init__(self):
      super().__init__('cmd_vel_publisher_subscriber')

      self.declare_parameters(
      namespace='',
      parameters=[
        ('device', '/dev/ttyACM0'), #device we are trasmitting to & recieving messages from
          ('topic', 'cmd_vel'),
          ('radius', 5.0),
          ('wheel_separation', 32.0)
      ]
      )

      self.device_name = self.get_param_str('device')
      self.topic = self.get_param_str('topic')
      self.ser = serial.Serial(self.device_name,
                            115200, #Note: Baud Rate must be the same in the arduino program, otherwise signal is not recieved!
                            timeout=0.1)
      self.ser.reset_input_buffer()

      self.radius = self.get_param_float('radius')
      self.wheel_separation = self.get_param_float('wheel_separation')
      self.timer_period = 0.9  # In seconds
      # self.publisher_leftWheel = self.create_publisher(Twist, self.topic, 10)
      # self.timer_leftWheel = self.create_timer(self.timer_period, self.timer_callback)

      # Two publishers for odometry readings. Could also make a custom interface message.

      self.publisher_rightWheel = self.create_publisher(Twist, self.topic, 10)
      self.timerRightWheel = self.create_timer(self.timer_period, self.timer_callback)

      self.subscription = self.create_subscription(Twist,self.topic,self.listener_callback,10)
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
        # Tendríamos que recibir dos mensajes para poder enviar las dos velocidades de las ruedas. Ver como hacerlo, dos subscriptions? Analizar como llega la info de joystick.
        self.get_logger().info('He escuchado: "%s"' % msg)

        # self.valueToSendRightWheel = (int)(msg.angular.z * 100)
        # self.rightWheelAngularSpeed = (msg.linear.x + msg.angular.z * self.wheel_separation / 2) / self.radius;
        # self.leftWheelAngularSpeed = (msg.linear.x - msg.angular.z * self.wheel_separation / 2) / self.radius;
        # self.write(str(self.rightWheelAngularSpeed) + "\n")
        # self.write(str(self.leftWheelAngularSpeed )+ "\n")

        self.valueToSendRightWheel = 500
        self.write(str(self.valueToSendRightWheel) + "\n")
        self.write(str(self.valueToSendRightWheel) + "\n")

  def timer_callback(self):
    # Read Wr, Wl, in that order, via serial from arduino.
    value = self.recv()

    print(f"RECEIVED DATA: {value}") 
    
    try:
      decoded = value.decode("ascii")
      cmd_vel_array = decoded.split()
      msgRightWheel = Twist()
      # Accessing arduino's cmd_vel_array based on its length allow us to gather the last two values from the serial buffer (Could be reading slower than we write in buffer).
      msgRightWheel.angular.z = float(cmd_vel_array[len(cmd_vel_array) - 2])/100.0   # Arduino's speeds are in 100 order, divide by 100 to get real speeds.
      msgRightWheel.linear.x = msgRightWheel.angular.z * radius
      self.publisher_rightWheel.publish(msgRightWheel)
      self.get_logger().info('Publishing: "%s"' % msgRightWheel)

      msgLeftWheel = Twist()
      msgLeftWheel.angular.z = float(cmd_vel_array[len(cmd_vel_array) - 1])/100.0
      msgLeftWheel.linear.x = msgLeftWheel.angular.z * radius 
      self.publisher_leftWheel.publish(msgLeftWheel)
      self.get_logger().info('Publishing: "%s"' % msgLeftWheel)
    except:
      self.get_logger().warn('Fallo buffer')
    
    ########### TODO: Differential velocities
    # wheel_1_angular_velocity = (msgRightWheel->linear.x + msgRightWheel->angular.z * parameters_.wheel_separation.as_double() / 2) / parameters_.wheel_radius.as_double();
    # wheel_2_angular_velocity = (msgLeftWheel->linear.x - msgLeftWheel->angular.z * parameters_.wheel_separation.as_double() / 2) / parameters_.wheel_radius.as_double();

    # sleep(0.150)

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

# El subscriber tendrá un timer mediante el cual haremos la escritura de las velocidades a nuestro arduino.
# El publisher tendrá que leer el puerto serie constantemente para asegurarse de que haya llegado información.

# La manera en la que está implementado actualmente es cada vez que enviemos datos a arduino leeremos del puerto serie pero sería incorrecto cuando no enviásemos info desde ros.