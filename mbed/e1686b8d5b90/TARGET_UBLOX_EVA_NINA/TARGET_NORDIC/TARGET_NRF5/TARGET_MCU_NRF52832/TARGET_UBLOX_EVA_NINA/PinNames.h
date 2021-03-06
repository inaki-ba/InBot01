#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PIN_INPUT,
    PIN_OUTPUT
} PinDirection;

#define PORT_SHIFT 3

typedef enum {
    // nRF52 pin names
    p0  = 0,
    p1  = 1,
    p2  = 2,
    p3  = 3,
    p4  = 4,
    p5  = 5,
    p6  = 6,
    p7  = 7,
    p8  = 8,
    p9  = 9,
    p10 = 10,
    p11 = 11,
    p12 = 12,
    p13 = 13,
    p14 = 14,
    p15 = 15,
    p16 = 16,
    p17 = 17,
    p18 = 18,
    p19 = 19,
    p20 = 20,
    p21 = 21,
    p22 = 22,
    p23 = 23,
    p24 = 24,
    p25 = 25,
    p26 = 26,
    p27 = 27,
    p28 = 28,
    p29 = 29,
    p30 = 30,
    p31 = 31,
    NC = (int)0xFFFFFFFF, // Not connected

    //NINA-B1 module pin names
    NINA_B1_GPIO_1 = p8,
    NINA_B1_GPIO_2 = p11,
    NINA_B1_GPIO_3 = p12,
    NINA_B1_GPIO_4 = p13,
    NINA_B1_GPIO_5 = p14,
    NINA_B1_GPIO_7 = p16,
    NINA_B1_GPIO_8 = p18,

    NINA_B1_GPIO_16 = p28,
    NINA_B1_GPIO_17 = p29,
    NINA_B1_GPIO_18 = p30,

    NINA_B1_GPIO_20 = p31,
    NINA_B1_GPIO_21 = p7,
    NINA_B1_GPIO_22 = p6,
    NINA_B1_GPIO_23 = p5,
    NINA_B1_GPIO_24 = p2,
    NINA_B1_GPIO_25 = p3,
    NINA_B1_GPIO_27 = p4,
    NINA_B1_GPIO_28 = p9,
    NINA_B1_GPIO_29 = p10,
    
    // Board pins
    LED1 = NINA_B1_GPIO_7, // ORANGE
    LED2 = NC,
    LED3 = NC,
    LED4 = NC,
    
    // Nordic SDK pin names
    RX_PIN_NUMBER = p5,
    TX_PIN_NUMBER = p6,
    CTS_PIN_NUMBER = p7,
    RTS_PIN_NUMBER = p31,
    I2C_SDA0 = p2,
    I2C_SCL0 = p3,

    // mBed interface pins
    USBTX = TX_PIN_NUMBER,
    USBRX = RX_PIN_NUMBER
} PinName;

typedef enum {
    PullNone = 0,
    PullDown = 1,
    PullUp = 3,
    PullDefault = PullUp
} PinMode;

#ifdef __cplusplus
}

#endif
#endif
