
/* PID HEREDA O ABSTRAE ESTA CLASE */

#ifndef MOTOR_H
#define MOTOR_H
#endif

#include "L6206.h"

/* Definitions ---------------------------------------------------------------*/
 
#define MAX_MOTOR (4)
 
 
/* Variables -----------------------------------------------------------------*/
 
static volatile uint16_t gLastError;
static volatile uint8_t gStep = 0;
 
 
/* Variables -----------------------------------------------------------------*/
 
/* Initialization parameters. */
L6206_init_t init =
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

class MyMotor : public L6206
{
public:
	MyMotor( uint8_t ENA, uint8_t ENB, uint8_t INA1, uint8_t INA2, uint8_t INB1, uint8_t INB2);

private:
	L6206 *motor_;

	uint16_t leftPulses_;    //How far the left wheel has travelled.
    uint16_t rightPulses_;    //How far the right wheel has travelled.
    uint16_t distance_; //Number of pulses to travel.
};