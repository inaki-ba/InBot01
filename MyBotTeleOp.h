#include <turtle_actionlib/Velocity.h>
#include <sensor_msgs/Joy.h>

class MyBotTeleOp 
{
public:
	MyBotTeleOp();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr & joy);

	ros::NodeHandle nh_;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
	ros:Subscriber joy_sub_;
};


