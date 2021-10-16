/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Iñaki Lorente
   Contact: support.idi@ageve.net
*/

#include <array>

#include "agl_joystick/joystick.hpp"

using std::placeholders::_1;

/*****************************************************************************
                    Joystick: JOYSTICK NODE CONSTRUCTOR
*****************************************************************************/

//================================================
Joystick::Joystick() : Node("Joystick_node"),
                       StateMachine(std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {}))
//================================================
{
    // Definir los State Machine timers a la frequencia deseada
    // y vincularlos a sus callback
    Unconfigured_timer = this->create_wall_timer(500ms, std::bind(&Joystick::Unconfigured,this));
    StandBy_timer = this->create_wall_timer(500ms, std::bind(&Joystick::StandBy,this));
    Run_timer = this->create_wall_timer(500ms, std::bind(&Joystick::Run,this));
    Fault_timer = this->create_wall_timer(500ms, std::bind(&Joystick::Fault,this));
    ShutDown_timer = this->create_wall_timer(500ms, std::bind(&Joystick::ShutDown,this));
    
    // Cancelar todos los timers excepto el inicial
    StandBy_timer->cancel();
    Run_timer->cancel();
    Fault_timer->cancel();
    ShutDown_timer->cancel();

    // Declare parameters
    this->declare_parameter("autorun");
    this->declare_parameter("commands.axes_pad_left_right");
    this->declare_parameter("commands.axes_pad_up_down");
    this->declare_parameter("commands.axes_r2");
    this->declare_parameter("commands.axes_l2");
    this->declare_parameter("commands.axes_arrow_left_right");
    this->declare_parameter("commands.axes_arrow_up_down");
    this->declare_parameter("commands.button_a");
    this->declare_parameter("commands.button_b");
    this->declare_parameter("commands.button_x");
    this->declare_parameter("commands.button_y");
    this->declare_parameter("commands.button_l1");
    this->declare_parameter("commands.button_r1");
    this->declare_parameter("commands.button_select");
    this->declare_parameter("commands.button_start");
    this->declare_parameter("commands.button_pad_left");
    this->declare_parameter("commands.button_pad_right");
    this->declare_parameter("speed.linear_gain");
    this->declare_parameter("speed.angular_gain");
    this->declare_parameter("speed.velocity");

}

/*****************************************************************************
                    State Machine: FUNCIONES ESTADO NODO
*****************************************************************************/

//================================================
void Joystick::Unconfigured()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(UNCONFIGURED);

    // Introducir las acciones a realizar
    // y la logica de cambio de estado

    if (Initialize()) //Condicion para pasar al siguiente
    { 
        ToggleState(StandBy_timer);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"Fallo inicializacion");
        updateErrorCritical(CONNECTION_FAILED,CONNECTION_FAILED_DESC);
        ToggleState(Fault_timer);    
    }
    if (onFault){
        ToggleState(Fault_timer);
    }
}

//================================================
void Joystick::StandBy()
//================================================
{

    // Indicamos a la maquina de estados el estado actual
    updateState(STANDBY);

    RCLCPP_DEBUG(this->get_logger(),"On Stand By");

    if (parameters.autorun.as_bool()){
        this->set_parameter(rclcpp::Parameter("autorun",false));
        ToggleState(Run_timer);
    }
    if (onFault){
        ToggleState(Fault_timer);
    }
}

//================================================
void Joystick::Run()
//================================================
{

    // Indicamos a la maquina de estados el estado actual
    updateState(RUN);
    RCLCPP_DEBUG(this->get_logger(),"On Run");

    // SRV onStop() --> Activation_srv con False
    if (onFault)
    {
        ToggleState(Fault_timer);
    }
}

//================================================
void Joystick::Fault()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(FAULT);
    RCLCPP_DEBUG(this->get_logger(),"On Fault");
    // Introducir las acciones a realizar
    // y la logica de cambio de estado
    if (critical & CONNECTION_FAILED)
    {
        if(Initialize()){
            updateSolvedCritical(CONNECTION_FAILED,CONNECTION_FAILED_DESC);
            ToggleState(StandBy_timer);
        }
    }

}

//================================================
void Joystick::ShutDown()
//================================================
{
    // Indicamos a la maquina de estados el estado actual
    updateState(SHUTDOWN);

    // Introducir las acciones a realizar
    // y la logica de cambio de estado
    rclcpp::shutdown();
}


/*****************************************************************************
                FUNCIONES PROPIAS FUNCIONAMIENTO JOYSTICK
*****************************************************************************/

//================================================
bool Joystick::Initialize()
//================================================
{
  RCLCPP_INFO(this->get_logger(), "Checking parameters...");
  
  try
  {
    this->parameters.autorun = this->get_parameter("autorun"); 
    this->parameters.axes_pad_left_right = this->get_parameter("commands.axes_pad_left_right"); 
    this->parameters.axes_pad_up_down = this->get_parameter("commands.axes_pad_up_down"); 
    this->parameters.axes_r2 = this->get_parameter("commands.axes_r2"); 
    this->parameters.axes_l2 = this->get_parameter("commands.axes_l2"); 
    this->parameters.axes_arrow_left_right = this->get_parameter("commands.axes_arrow_left_right"); 
    this->parameters.axes_arrow_up_down = this->get_parameter("commands.axes_arrow_up_down"); 
    this->parameters.button_a = this->get_parameter("commands.button_a"); 
    this->parameters.button_b = this->get_parameter("commands.button_b"); 
    this->parameters.button_x = this->get_parameter("commands.button_x"); 
    this->parameters.button_y = this->get_parameter("commands.button_y"); 
    this->parameters.button_l1 = this->get_parameter("commands.button_l1"); 
    this->parameters.button_r1 = this->get_parameter("commands.button_r1"); 
    this->parameters.button_select = this->get_parameter("commands.button_select");
    this->parameters.button_start = this->get_parameter("commands.button_start"); 
    this->parameters.button_pad_left = this->get_parameter("commands.button_pad_left"); 
    this->parameters.button_pad_right = this->get_parameter("commands.button_pad_right"); 
    this->parameters.linear_gain = this->get_parameter("speed.linear_gain"); 
    this->parameters.angular_gain = this->get_parameter("speed.angular_gain"); 
    this->parameters.velocity = this->get_parameter("speed.velocity"); 
  }
  catch (std::runtime_error &exc)
  {
      RCLCPP_INFO(this->get_logger(), exc.what());
      return false;
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            qos,
            std::bind(&Joystick::joy_callback, this, std::placeholders::_1));

  return true;       
}

//================================================
void Joystick::joy_callback(const sensor_msgs::msg::Joy::SharedPtr  msg)
//================================================
{
  if (state == RUN)
    {
      geometry_msgs::msg::Twist twist;

      // Increment velocity
      if (msg->buttons[parameters.button_y.as_int()])
      {
        velocity += 1.0;
      }
      // Reset velocity
      if (msg->buttons[parameters.button_a.as_int()])
      {
        velocity = parameters.velocity.as_double();
      }

      // ====================
      // Movement buttons
      // ====================
      if (static_cast<bool>(msg->buttons[parameters.button_select.as_int()])){
          if (joy_analog){
              joy_analog = false;
          }else{
              joy_analog = true;
          }
      }
      if (joy_analog){
        // JOYSTICK ANALOG 
        // ---------------
        if (msg->axes[parameters.axes_pad_up_down.as_int()]){
            twist.linear.x = parameters.linear_gain.as_double()*velocity*msg->axes[parameters.axes_pad_up_down.as_int()];
        }
        if (msg->axes[parameters.axes_pad_left_right.as_int()]){
            twist.angular.z = parameters.angular_gain.as_double()*velocity*msg->axes[parameters.axes_pad_left_right.as_int()];
        }
      }else{
        // JOYSTICK DIGITAL 
        // ---------------
        if (msg->axes[parameters.axes_arrow_up_down.as_int()]){
            twist.linear.x = parameters.linear_gain.as_double()*velocity*msg->axes[parameters.axes_arrow_up_down.as_int()];
        }
        if (msg->axes[parameters.axes_arrow_left_right.as_int()]){
            twist.angular.z = parameters.angular_gain.as_double()*velocity*msg->axes[parameters.axes_arrow_left_right.as_int()];
        }
      }
      
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;

      cmd_vel_pub_->publish(twist);
    }
}

/*****************************************************************************
** Main
*****************************************************************************/

//================================================
int main(int argc, char *argv[])
//================================================
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joystick>());
  rclcpp::shutdown();

  return 0;
}