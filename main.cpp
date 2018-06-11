/*
 * rosserial Planar Odometry Example
 */

#include <string.h>
#include <ros.h>
#include <ros/time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "mbed.h"

#include "pid.h"
#include "Sensing.h"
#include "Motor.h"
#include "MyBotTeleOp.h"

#define MOTORS_INA 1


//Serial pc(USBTX, USBRX);

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom[] = "/odom";
char s_imu[] = "/imu";


int main( void ) {

    //pc.baud(57600);
    nh.initNode();

    broadcaster.init(nh);

    MyBotTeleOp my_bot_teleop;
    MyImu imu(nh);

    nav_msgs::Odometry odom_msg;    
    ros::Publisher t_pub("odom", &odom_msg); 

    float value1, value2;
    char buffer1[32], buffer2[32];

    wait(1.5);
 
 #ifdef MOTORS_INA       
      pBridge = new InBridge(D2, A4, D5, D4, A0, A1);
      InMotor pMotorFL = new InMotor(pBridge, MOT_FL);
      InMotor pMotorFR = new InMotor(pBridge, MOT_FR);
      InMotor pMotorRL = new InMotor(pBridge, MOT_RL);
      InMotor pMotorRR = new InMotor(pBridge, MOT_RR);

#endif
    while (1) {

        ros::Time current_time = nh.now();

        // drive in a circle
        double dx = 0.2;
        double dtheta = 0.18;
        x += cos(theta)*dx*0.1;
        y += sin(theta)*dx*0.1;
        theta += dtheta*0.1;
        if (theta > 3.14)
            theta=-3.14;

        // tf odom->base_link
        t.header.frame_id = odom;
        t.child_frame_id = base_link;

        t.transform.translation.x = x;
        t.transform.translation.y = y;

        t.transform.rotation = tf::createQuaternionFromYaw(theta);
        t.header.stamp = nh.now();
        
        odom_msg.header.stamp = current_time;

        broadcaster.sendTransform(t);

        imu.Publish_env();
        imu.Publish_mag();
        imu.Publish_imu();

        nh.spinOnce();
        
        t.header.frame_id = odom;
        t.child_frame_id = base_link;
        
        t.transform.rotation = tf::createQuaternionFromYaw(theta);

#ifdef MOTORS_INA
          if (gStep > 0) {            
            /* Set speed of motor 0 to 5% */
            pMotorFL->runForward(50);            
        }
         if (gStep > 1) {            
            /* Set speed of motor 1 to 10 % */
            pMotorFR->runForward(50);             
        }
         if (gStep > 2) {            
            /* Set speed of motor 2 to 15 % */
            pMotorRL->runForward(150);             
        }
         if (gStep > 3)  {
            /* Set speed of motor 3 to 20 % */
            pMotorRR->runForward(150);
        }
 
        if (gStep > 0) {
            wait_ms(1000);
        
            pMotorFL->stop();   
            pMotorFR->stop();     
            pMotorRL->stop();     
            pMotorRR->stop();   
            
            wait_ms(1000);
        }
        flPulses  = pMotorFL->getQei(TRUE);
        frPulses  = pMotorFR->getQei(TRUE);
        rlPulses  = pMotorRL->getQei(TRUE);
        rrPulses  = pMotorRR->getQei(TRUE);
#endif
    }
}
