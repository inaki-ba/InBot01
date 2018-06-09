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


/* Functions -----------------------------------------------------------------*/
 
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  error number of the error
  * @retval None
  */
void my_error_handler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;
  
  /* Enter your own code here */
}
 
/**
* @brief  This function is the User handler for the flag interrupt
* @param  None
* @retval None
* @note   If needed, implement it, and then attach and enable it:
*           + motor->attach_flag_interrupt(my_flag_irq_handler);
*/
void my_flag_irq_handler(void)
{
  /* Code to be customised */
  /************************/
  /* Get the state of bridge A */
  uint16_t bridgeState  = motor->get_bridge_status(0);
  
  if (bridgeState == 0) {
    if ((motor->get_device_state(0) != INACTIVE)||
        (motor->get_device_state(1) != INACTIVE)) {
      /* Bridge A was disabling due to overcurrent or over temperature */
      /* When at least on of its  motor was running */
        my_error_handler(0XBAD0);
    }
  }
  
  /* Get the state of bridge B */
  bridgeState  = motor->get_bridge_status(1);
  
  if (bridgeState == 0)  {
    if ((motor->get_device_state(2) != INACTIVE)||
        (motor->get_device_state(3) != INACTIVE)) {
      /* Bridge A was disabling due to overcurrent or over temperature */
      /* When at least on of its  motor was running */
        my_error_handler(0XBAD1);
    }
  }  
}
 
 
/* Private functions ---------------------------------------------------------*/
 
/**
  * @brief  Button Irq
  * @param  None
  * @retval None
  */
 
void my_button_pressed(void)
{
    my_button_irq.disable_irq();
    gStep++;
    if (gStep > MAX_MOTOR) {
        gStep = 0;
    }
    wait_ms(200);
    my_button_irq.enable_irq();
}

/* Helper function for printing floats & doubles */
static char *print_double(char* str, double v, int decimalDigits=2)
{
  int i = 1;
  int intPart, fractPart;
  int len;
  char *ptr;

  /* prepare decimal digits multiplicator */
  for (;decimalDigits!=0; i*=10, decimalDigits--);

  /* calculate integer & fractinal parts */
  intPart = (int)v;
  fractPart = (int)((v-(double)(int)v)*i);

  /* fill in integer part */
  sprintf(str, "%i.", intPart);

  /* prepare fill in of fractional part */
  len = strlen(str);
  ptr = &str[len];

  /* fill in leading fractional zeros */
  for (i/=10;i>1; i/=10, ptr++) {
    if (fractPart >= i) {
      break;
    }
    *ptr = '0';
  }

  /* fill in (rest of) fractional part */
  sprintf(ptr, "%i", fractPart);

  return str;
}

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
            
    //pc.printf("\r\n--- Starting new run ---\r\n");

    hum_temp->read_id(&id);
    //pc.printf("HTS221  humidity & temperature    = 0x%X\r\n", id);
    press_temp->read_id(&id);
    //pc.printf("LPS22HB  pressure & temperature   = 0x%X\r\n", id);
    magnetometer->read_id(&id);
    //pc.printf("LSM303AGR magnetometer            = 0x%X\r\n", id);
    accelerometer->read_id(&id);
    //pc.printf("LSM303AGR accelerometer           = 0x%X\r\n", id);
    acc_gyro->read_id(&id);
    //pc.printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);

      
    wait(1.5);
 
 #ifdef MOTORS_INA       

      MyMotor *motor = new MyMotor(D2, A4, D5, D4, A0, A1);

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
            motor->set_speed(2,50);
            /* start motor 0 */
            motor->run(2, BDCMotor::FWD);
        }
 
        if (gStep > 1) {            
            /* Set speed of motor 1 to 10 % */
            motor->set_speed(3,100);
            /* start motor 1 */
            motor->run(3, BDCMotor::FWD);
        }
 
        if (gStep > 2) {            
            /* Set speed of motor 2 to 15 % */
            motor->set_speed(2,150);
            /* start motor 2 */
            motor->run(2, BDCMotor::FWD);
        }
 
        if (gStep > 3)  {
            /* Set speed of motor 3 to 20 % */
            motor->set_speed(3,200);
            /* start motor 3 */
            motor->run(3, BDCMotor::FWD);      
        }
 
        if (gStep > 0) {
            wait_ms(1000);
        
            motor->hard_hiz(2);   
            motor->hard_hiz(3);   
            motor->hard_hiz(2);   
            motor->hard_hiz(3);
            
            wait_ms(1000);
        }

        //leftPulses  = leftQei.getPulses();
        //rightPulses = rightQei.getPulses();
#endif
    }
}
