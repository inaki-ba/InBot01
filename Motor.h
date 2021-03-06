
/* PID HEREDA O ABSTRAE ESTA CLASE */

#ifndef MOTOR_H
#define MOTOR_H
#endif

#include "L6206.h"
#include "QEI.h"

/* Definitions ---------------------------------------------------------------*/
 
#define MAX_MOTOR (4)
 
 
/* Variables -----------------------------------------------------------------*/
 
static volatile uint16_t gLastError;
static volatile uint8_t gStep = 0;
 
 
/* Variables -----------------------------------------------------------------*/
 
/* Initialization parameters. */
L6206_init_t init_str =
{
    L6206_CONF_PARAM_PARALLE_BRIDGES,
    {L6206_CONF_PARAM_FREQ_PWM1A, L6206_CONF_PARAM_FREQ_PWM2A, L6206_CONF_PARAM_FREQ_PWM1B, L6206_CONF_PARAM_FREQ_PWM2B},
    {100,100,100,100},
    {FORWARD,FORWARD,BACKWARD,FORWARD},
    {INACTIVE,INACTIVE,INACTIVE,INACTIVE},
    {FALSE,FALSE}
};
 
/* Motor Control Component. */
L6206 *motor;
 
/* User button on Nucleo board */
InterruptIn my_button_irq(USER_BUTTON);

class InBridge : public L6206 
{
public:
	InBridge( PinName ENA, PinName ENB, PinName INA1, PinName INA2, PinName INB1, PinName INB2);
	void my_error_handler(uint16_t error);
	void my_flag_irq_handler(void);
	void my_button_pressed(void);
	const InBridge *GetBridgeHandle() const { return this; }
};

InBridge *pBridge;

class InMotor 
{
public:
	InMotor( InBridge *bridgeHangle, int motorIndex );
	bool runForward( int Speed );
	bool runBackwards( int Speed );
	bool stop( bool bshutBridge = TRUE);
	int getQEI() { return pQei_->getPulses(); }

private:
	InBridge *bridgeHandle_;
	int motorIndex_;
	QEI *pQei_;  //chanA, chanB, index, ppr	

	uint16_t pulses_;    //How far the left wheel has travelled.    
    uint16_t distance_; //Number of pulses to travel.
};
