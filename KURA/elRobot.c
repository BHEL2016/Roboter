//***********************************************************************************
// elRobot: state machine source file
// Version: 2014_02_05
// author:  W. Kuran
//***********************************************************************************

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include "elRobot.h"

#ifndef __OPTIMIZE__
# warning Compiler optimizations disabled!
# warning use: Project>BildOptions>Optimize generated code (for size) - Os
#endif

#define SPEEDY_VERSION

//*************************************************************************************************
//*** MOTOR ***
//*************************************************************************************************

#define MOT_L_F			5			// PF5
#define MOT_L_B			6			// PF6
#define MOT_R_F			7			// PF7
#define MOT_R_B			6			// PC6

#define MOTOR_R			6			// PB6
#define MOTOR_L			7			// PD7

#ifdef  SPEEDY_VERSION
#define MOT_FORWARD		1
#define MOT_BACKWARD	0
#else
#define MOT_FORWARD		0
#define MOT_BACKWARD	1
#endif

#define PWM_STOPP		0

//*************************************************************************************************
//LED
//*************************************************************************************************

// LED Pins
#define LED_RF			0			// PB0
#define LED_RB			1			// PB1
#define LED_LF			2			// PB2
#define LED_LB			3			// PB3
// Duo LED
#define LED_RED			2			// PD2
#define LED_GREEN		3			// PD3

//*************************************************************************************************
//IR TRANSMIT RECEIVE
//*************************************************************************************************

#define IR_TRANSMITTER	6			// PD6
#define IR_RECEIVER_R	1			// ADC1
#define IR_RECEIVER_L	0			// ADC0

//*************************************************************************************************
//WHEEL CONTROL
//*************************************************************************************************

#define WHEEL_RIGHT		4			// PD4
#define WHEEL_LEFT		7			// PC7

//*************************************************************************************************
//BEEPER
//*************************************************************************************************

#define BEEPER			7			// PB7
#define NONE			0
#define BEEP			8

//*************************************************************************************************
//TIMER 0
//*************************************************************************************************

#define TICK_ONE_SEC      62


//*************************************************************************************************
// internals:
void pwm_timer_init(void);
void pwm_timer_start(void);
void pwm_timer_stop(void);
void odometer_timer_init(void);
void odometer_left_start(void);
void odometer_right_start(void);
void odometer_left_stop(void);
void odometer_right_stop(void);
void led(uint8_t x);

// internal variables:
volatile uint16_t speedR[11], speedL[11];
volatile uint16_t tick;

//*************************************************************************************************
void set_fuses(void)
{
	MCUCR |= (1<<JTD);
	MCUCR |= (1<<JTD);
	CLKPR = 0b10000000;
	CLKPR = 1;
}

void init(void)
{
	DDRB |= (1 << BEEPER)  |
	        (1 << MOTOR_R) |
 			(1 << LED_LB)  |
			(1 << LED_LF)  |
			(1 << LED_RB)  |
			(1 << LED_RF);

	DDRC |= (1 << MOT_R_B);

	DDRF |= (1 << MOT_L_F) |
	        (1 << MOT_L_B) |
			(1 << MOT_R_F);

	DDRD |= (1 << MOTOR_L)   |
	        (1 << IR_TRANSMITTER) |
			(1 << LED_GREEN) |
			(1 << LED_RED);

	PORTB = 0;

	PORTF = 0;

	PORTD = 0;


	speedR[10] = speedR[0] = 250;
	speedR[ 9] = speedR[1] = 230;
	speedR[ 8] = speedR[2] = 210;
	speedR[ 7] = speedR[3] = 190;
	speedR[ 6] = speedR[4] = 170;
    speedR[ 5] = 0;


	speedL[10] = speedL[0] = 250;
	speedL[ 9] = speedL[1] = 230;
	speedL[ 8] = speedL[2] = 210;
	speedL[ 7] = speedL[3] = 190;
	speedL[ 6] = speedL[4] = 170;
    speedL[ 5] = 0;


	pwm_timer_init();

	odometer_timer_init();

//  Timer 0:
    TIMSK0 = (1 << TOIE0);
//	TCNT0 = 206;
	TCCR0B = (1 << CS00);



    ms = tick = 0;

	sei();
}

void ledGreenOn(void)
{
    PORTD |= (1 << LED_GREEN);
}

void ledGreenOff(void)
{
    PORTD &= ~(1 << LED_GREEN);
}

void ledRedOn(void)
{
    PORTD |= (1 << LED_RED);
}

void ledRedOff(void)
{
    PORTD &= ~(1 << LED_RED);
}

void led(uint8_t x)
{
    if ((x & 0x1) == 0x1) PORTB |= (1 << LED_LB); else PORTB &= ~(1 << LED_LB);
    if ((x & 0x2) == 0x2) PORTB |= (1 << LED_LF); else PORTB &= ~(1 << LED_LF);
    if ((x & 0x4) == 0x4) PORTB |= (1 << LED_RB); else PORTB &= ~(1 << LED_RB);
    if ((x & 0x8) == 0x8) PORTB |= (1 << LED_RF); else PORTB &= ~(1 << LED_RF);
}

void pwm_timer_init(void)
{
	// Timer 4:
	//MOTOR RIGHT:
	TCCR4A |= (1 << COM4B1) |
	          (1 << PWM4B);

	//MOTOR LEFT:
	TCCR4C |= (1 << COM4D1) |
	          (1 << PWM4D);
}

void odometer_timer_init(void)
{
	TCCR1A = 0;
	TCCR1B = (1 << ICNC1);					// Input Capture Mode aktiviert
	TCCR1C = 0;
	TCCR3A = 0;
	TCCR3B = (1 << ICNC3);					// Input Capture Mode aktiviert
	TCCR3C = 0;
}

void start(void)
{
    pwm_timer_start();
    odometer_left_start();
	odometer_right_start();
}

void pwm_timer_start(void)
{
	// Timer 4
	//MOTOR RIGHT
	TCCR4A |= (1 << COM4B1)
	        | (1 << PWM4B);
	//MOTOR LEFT
	TCCR4C |= (1 << COM4D1) |
	          (1 << PWM4D);

	TCCR4B = 0;
	TCCR4D = 0;
	TCCR4E = 0;

	TCNT4 = 0;
	DT4 = 0;								// Dead Time Generator off

	OCR4B = PWM_STOPP;						// PWM 0%
	OCR4D = PWM_STOPP;						// PWM 0%

	TCCR4B |= (1 << CS40);					// e.g. 64kHz

}

void pwm_timer_stop(void)
{
	TCCR4B = 0;
	TCCR4A = 0;
	TCCR4C = 0;
}

void odometer_left_start(void)
{
	TIMSK3 |= (1 << ICIE3);
	ICR3    = 0;
	TCNT3   = 0;
	TCCR3B |= (1 << CS32);
	TIFR3  |= (1 << ICF3);
}

void odometer_right_start(void)
{
	TIMSK1 |= (1 << ICIE1);
	ICR1    = 0;
	TCNT1   = 0;
	TCCR1B |= (1 << CS12);
	TIFR1  |= (1 << ICF1);
}

void odometer_left_stop(void)
{
	TCCR3B &= ~((1<<CS32) | (1<<CS31) | (1<<CS30));
}

void odometer_right_stop(void)
{
	TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
}

void drive(int8_t l, int8_t r)
{

    // left:

    if (l > 0)       // forward
	{
	    PORTD &= ~(1 << MOTOR_L); // Motor ein
        PORTF |=  (1 << MOT_L_F); PORTF &= ~(1 << MOT_L_B); //L->
    }
	else if (l < 0)  // backward
	{
	    PORTD &= ~(1 << MOTOR_L); // Motor ein
        PORTF &= ~(1 << MOT_L_F); PORTF |=  (1 << MOT_L_B); //L->

	}
	else             // stop
	{
		PORTD |=  (1 << MOTOR_L); // Motor aus
        PORTF |=  (1 << MOT_L_F); PORTF |=  (1 << MOT_L_B); //L->
	}

    // right:

	if (r > 0)       // forward
	{
	    PORTB &= ~(1 << MOTOR_R);
		PORTF |=  (1 << MOT_R_F); PORTC &= ~(1 << MOT_R_B); //R ->
    }
	else if (r < 0)  // backward
	{
	    PORTB &= ~(1 << MOTOR_R); // Motor ein
	    PORTF &= ~(1 << MOT_R_F); PORTC |=  (1 << MOT_R_B); //R<-

	}
	else             // stop
	{
	    PORTB |=  (1 << MOTOR_R); // Motor aus
	    PORTF |=  (1 << MOT_R_F); PORTC |=  (1 << MOT_R_B); //R ||
	}

	OCR4D = (unsigned int)speedL[l + 5]; // Left
	OCR4B = (unsigned int)speedR[r + 5]; // Right
}

unsigned char check_impulse(void)
{
		return 1;
}

// right controll
ISR(TIMER1_CAPT_vect)
{
	TIFR1 |= (1<<ICF1);
}

// left controll
ISR(TIMER3_CAPT_vect)
{
	TIFR3 |= (1<<ICF3);
}

ISR(TIMER0_OVF_vect)
{
//  TCNT0 = 200;
    tick++;

	if (tick > TICK_ONE_SEC)
	{
        tick = 0;
		ms++;
	}
}

