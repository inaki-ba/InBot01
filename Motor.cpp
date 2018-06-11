#include <Motor.h>


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
  uint16_t bridgeState  = pBridge->get_bridge_status(0);
  
  if (bridgeState == 0) {
    if ((pBridge->get_device_state(0) != INACTIVE)||
        (pBridge->get_device_state(1) != INACTIVE)) {
      /* Bridge A was disabling due to overcurrent or over temperature */
      /* When at least on of its  motor was running */
        my_error_handler(0XBAD0);
    }
  }
  
  /* Get the state of bridge B */
  bridgeState  = motor->get_bridge_status(1);
  
  if (bridgeState == 0)  {
    if ((pBridge->get_device_state(2) != INACTIVE)||
        (pBridge->get_device_state(3) != INACTIVE)) {
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


InBridge::InBridge( PinName ENA, PinName ENB, PinName INA1, 
	PinName INA2, PinName INB1, PinName INB2)
	: L6206( ENA, ENB, INA1, INA2, INB1, INB2 )
{
     /*----- Initialization. ----- */
 

      /* Initializing Motor Control Component. */
      
      //motor_ = new L6206(D2, A4, D5, D4, A0, A1);

 
     /* When init method is called with NULL pointer, the L6206 parameters are set   */
     /* with the predefined values from file l6206_target_config.h, otherwise the    */
     /* parameters are set using the init structure values.          */
     if (init(&init_str) != COMPONENT_OK) {
         exit(EXIT_FAILURE);
     }
 
      /* Attach the function my_flag_irq_handler (defined below) to the flag interrupt */
      attach_flag_interrupt(::my_flag_irq_handler);
 
      /* Attach the function my_error_handler (defined below) to the error Handler*/
      attach_error_handler(::my_error_handler);
 
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
      my_button_irq.fall(::my_button_pressed);
}

InMotor::InMotor( InBridge* bridgeHangle, int motorIndex )
{
	bridgeHandle_ = bridgeHangle;
	motorIndex_ = motorIndex;
	pQei_ = new QEI(D11, NC, NC, 624);  //chanA, chanB, index, ppr	

    pulses_  = 0;    //How far the left wheel has travelled.      
    distance_    = 6000; //Number of pulses to travel.
}

bool InMotor::runForward( int Speed ) { 
	bridgeHandle_->set_speed(motorIndex_, Speed ); 
	bridgeHandle_->run(motorIndex_, BDCMotor::FWD);

	return TRUE;
}

bool InMotor::runBackwards( int Speed ) { 
	bridgeHandle_->set_speed(motorIndex_, Speed ); 
	bridgeHandle_->run(motorIndex_, BDCMotor::BWD);

	return TRUE;
}