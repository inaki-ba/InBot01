#include "MyBotTeleOp.h"

MyBotTeleOp::TeleopTurtle():
     linear_(1),
     angular_(2)
{
   
     nh_.param("axis_linear", linear_, linear_);
     nh_.param("axis_angular", angular_, angular_);
     nh_.param("scale_angular", a_scale_, a_scale_);
     nh_.param("scale_linear", l_scale_, l_scale_);
   
   
     vel_pub_ = nh_.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
   
   
     joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
   
}
   
void MyBotTeleOp::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
     turtlesim::Velocity vel;
     vel.angular = a_scale_*joy->axes[angular_];
     vel.linear = l_scale_*joy->axes[linear_];
     vel_pub_.publish(vel);
}
