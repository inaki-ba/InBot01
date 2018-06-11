#include <Motor.h>

InBridge::InBridge( uint8_t ENA, uint8_t ENB, uint8_t INA1, 
	uint8_t INA2, uint8_t INB1, uint8_t INB2)
{
     /*----- Initialization. ----- */
 

      /* Initializing Motor Control Component. */
      
      //motor_ = new L6206(D2, A4, D5, D4, A0, A1);

 
     /* When init method is called with NULL pointer, the L6206 parameters are set   */
     /* with the predefined values from file l6206_target_config.h, otherwise the    */
     /* parameters are set using the init structure values.          */
     if (init(&init) != COMPONENT_OK) {
         exit(EXIT_FAILURE);
     }
 
      /* Attach the function my_flag_irq_handler (defined below) to the flag interrupt */
      attach_flag_interrupt(my_flag_irq_handler);
 
      /* Attach the function my_error_handler (defined below) to the error Handler*/
      attach_error_handler(my_error_handler);
 
      /* Printing to the console. */
      //printf("Motor Control Application Example for 4 Motors\r\n\n");
 
      /* Select the configuration with no bridge paralleling, two unidirectionnal motors on bridge A 
         and two unidirectionnal motors on bridge B */
      set_dual_full_bridge_config(PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B);
 
      /* Set PWM Frequency of bridge A inputs to 1000 Hz */ 
      set_bridge_input_pwm_freq(0,1000);
  
      /* Set PWM Frequency of bridge B inputs to 2000 Hz */ 
      set_bridge_input_pwm_freq(1,2000);
  
      // Attach my_button_pressed function to Irq
      my_button_irq.fall(&my_button_pressed);

      leftPulses_  = 0;    //How far the left wheel has travelled.
      rightPulses_ = 0;    //How far the right wheel has travelled.
      distance_    = 6000; //Number of pulses to travel.
}

InMotor::InMotor( InBridge* bridgeHangle, int motorIndex )
{
	bridgeHandle_ = bridgeHangle;
	motorIndex_ = motorIndex;
	pQei_ = new QEI(D11, NC, NC, 624);  //chanA, chanB, index, ppr	
}

bool InMotor::runForward( int Speed ) { 
	bridgeHandle_->set_speed(motorIndex_, Speed ); 
	bridgeHandle_->run(motorIndex_, BDCMotor::FWD);
}

BOOL InMotor::runBackwards( int Speed ) { 
	bridgeHandle_->set_speed(motorIndex_, Speed ); 
	bridgeHandle_->run(motorIndex_, BDCMotor::BWD);
}