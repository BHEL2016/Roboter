/*
 * ELRobot.c
 *
 * Created: 21.11.2012 10:32:45
 *  Author: Martin Bürgmann
 */ 

#define F_CPU 8000000UL

volatile unsigned int status = 0;
	
#define STATUS_READY		0
#define STATUS_BUSY			1
#define STATUS_END			2
//#define STATUS_PROGRAMM		3
#define STATUS_MOT_R_READY	4
#define STATUS_MOT_L_READY	5
#define STATUS_PWM_TIMER	6

volatile unsigned int impulstime_right=0, impulstime_left=0;
volatile unsigned char check_impuls_right=0, check_impuls_left=0,impuls_right_set=0, impuls_left_set=0, motor_right_count=0,motor_left_count=0;
volatile unsigned char pwm_left=0, pwm_right=0;
//volatile unsigned char check_lock_left=0, check_lock_right=0;
char test[25] = {'A'};

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "ELRobot.h"



// rechter Motorgeber

ISR(TIMER1_CAPT_vect)
{
	_delay_us(10);
	if (status&(1<<STATUS_MOT_R_READY))
	{
	}
	else
	{
		if (!(PIND&(1<<WHEEL_RIGHT)))
		{
//			PORTB &=~(1<<DEBUG_PIN);
			if (motor_right_count == 0)
			{
				check_impuls_right++;
				motor_right_count=1;
				TCNT1 = 0;
			}
			else
			{
				impulstime_right = ICR1;
				check_impuls_right++;
				motor_right_count = 0;
				TCNT1 = 0;
			}
	
			if (check_impuls_right >= impuls_right_set)
			{
				status |= (1<<STATUS_MOT_R_READY);
				TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
				TIMSK1 = 0;
				OCR4B = PWM_STOPP;
			}
//			_delay_ms(5);
//			PORTB |= (1<<DEBUG_PIN);
		}	
	}
	TIFR1 |= (1<<ICF1);
}


// linker Motorgeber

ISR(TIMER3_CAPT_vect)
{
	unsigned int calc_pwm=0;
	
	_delay_us(10);
	if (status&(1<<STATUS_MOT_L_READY))
	{
	}
	else
	{
		if (!(PINC&(1<<WHEEL_LEFT)))
		{
//			PORTD &=~(1<<DEBUG_PIN2);
			if (motor_left_count == 0)
			{
				check_impuls_left++;
				motor_left_count=1;
				TCNT3 = 0;
			}
			else
			{
				impulstime_left = ICR3;
				check_impuls_left++;
				//// Berechnung PWM
				//calc_pwm = (pwm_left*DELTA_IMPULSTIME);			// SOLL wert
				//if (impulstime_left <= calc_pwm)
				//{
					//
					//
					//// Motor zu schnell
					//calc_pwm = (calc_pwm/impulstime_left) * 100;
					//pwm_left = pwm_left+((pwm_left/calc_pwm)*100);
					//
					//
					//
				//} 
				//else
				//{
					//// Motor zu langsam
				//}
				motor_left_count = 0;
				TCNT3 = 0;
			}
		
			if (check_impuls_left >= impuls_left_set)
			{
				status |= (1<<STATUS_MOT_L_READY);
				TCCR3B &= ~((1<<CS32) | (1<<CS31) | (1<<CS30));
				TIMSK3 = 0;
				OCR4D = PWM_STOPP;
			}
//			_delay_ms(5);
//			PORTD |= (1<<DEBUG_PIN2);
		}
	}	
	TIFR3 |= (1<<ICF3);
}

int main(void)
{	
	
	set_fuses();
	init();
	PORT_LED &=~ ((1 << LED_RV) | (1<< LED_RH) | ( 1 << LED_LH) | (1 << LED_LV));
/*
	_delay_ms(1000);
*/
	unsigned char programm_step = 1;
	beep(2);
	
	
	PORT_LED &=~ ((1<< LED_RH) | ( 1 << LED_LH) | (1 << LED_LV));
	PORT_LED |= (1 << LED_RV);
	_delay_ms(500);
	PORT_LED &=~ ((1 << LED_RV) | ( 1 << LED_LH) | (1 << LED_LV));
	PORT_LED |= (1 << LED_RH);
	_delay_ms(500);
	PORT_LED &=~ ((1 << LED_RV) | (1<< LED_RH) | (1 << LED_LV));
	PORT_LED |= (1 << LED_LH);
	_delay_ms(500);
	PORT_LED &=~ ((1 << LED_RV) | (1<< LED_RH) | ( 1 << LED_LH) );
	PORT_LED |= (1 << LED_LV);
	_delay_ms(500);
/*
	while(!(status&(1<<STATUS_END)))
    {

		switch (programm_step)
		{
			case 1:
			{	
				if (!(status&(1<<STATUS_BUSY)))
				{
					drive_Robot(MOT_FORWARD,242,MOT_BACKWARD,254,ONE_TURN,ONE_TURN);
				}
				else
				{
					if(check_impulse()) 
					{
						programm_step++;
					}				
				}
			}
			break;
			
			case 2:
			{
				_delay_ms(1000);
				programm_step++;
			}
			break;
			
			case 3:
			{
				if (!(status&(1<<STATUS_BUSY)))
				{
					drive_Robot(MOT_BACKWARD,242,MOT_FORWARD,254,ONE_TURN,ONE_TURN);
				}
				else
				{
					if(check_impulse())
					{
						programm_step++;
					}
				}
			}
			break;

			case 4:
			{
				if (!(status&(1<<STATUS_BUSY)))
				{
					drive_Robot(MOT_FORWARD,242,MOT_FORWARD,254, (5*ONE_TURN),(5*ONE_TURN));
				}
				else
				{
					if(check_impulse())
					{
						programm_step++;
					}
				}
			}
			break;

			case 5:
			{
				if (!(status&(1<<STATUS_BUSY)))
				{
					drive_Robot(MOT_BACKWARD,242,MOT_FORWARD,254,ONE_TURN,ONE_TURN);
				}
				else
				{
					if(check_impulse())
					{
						programm_step++;
					}
				}
			}
			break;
			
			case 6:
			{
				if (!(status&(1<<STATUS_BUSY)))
				{
					drive_Robot(MOT_FORWARD,242,MOT_FORWARD,254,(5*ONE_TURN),(5*ONE_TURN));
				}
				else
				{
					if(check_impulse())
					{
						status |= (1<<STATUS_END);
					}
				}
			}
			break;
		}		
	}
*/
	PORT_DUAL |= (1 << LED_ROT);
	PORT_LED &=~ ((1 << LED_RV) | (1<< LED_RH) | ( 1 << LED_LH) | (1 << LED_LV));

	pwm_left = 250;
	pwm_right = 250;/*
	drive_Robot(MOT_BACKWARD,pwm_right,MOT_BACKWARD,pwm_left,3*ONE_TURN,3*ONE_TURN);
	while (!check_impulse()) {
		beep(1);
		PORT_LED ^= (1 << LED_RH) | (1 << LED_LH);
	}
	PORT_LED &=~ ((1 << LED_RV) | (1<< LED_RH) | ( 1 << LED_LH) | (1 << LED_LV));
	
	drive_Robot(MOT_FORWARD, pwm_right, MOT_FORWARD, pwm_left, 0, ONE_TURN);
	while (!check_impulse()) {
		PORT_LED ^= (1 << LED_RH) | (1 << LED_RV);
		_delay_ms(200);
	}*/
	
	
	/*
	PORT_LED &=~ ((1 << LED_RV) | (1<< LED_RH) | ( 1 << LED_LH) | (1 << LED_LV));
	drive_Robot(MOT_FORWARD, 185, MOT_FORWARD, 185, 300, 300);
	for (int i = 20; i > 0; i--) {
		OCR4B = 240 - i * 5;
		OCR4D = 240 - i * 5;
		_delay_ms(200);
	}
	while (!check_impulse());
	*/
	int co = 4;
/*	while (co--){
	drive_Robot(MOT_FORWARD, 250, MOT_FORWARD, 250, 3*ONE_TURN-3, 3*ONE_TURN-3);
	while (!check_impulse());
	drive_Robot(MOT_FORWARD, 250, MOT_FORWARD, 0, ONE_TURN, 0);
	while (!check_impulse());
	}*/
	co = 0;
	while (co--) {
		drive_Robot(MOT_FORWARD, 250, MOT_BACKWARD, 250, 30,30);
		while (!check_impulse());
	}
//	_delay_ms(20);
/*
	pwm_left = 0;
	pwm_right = pwm_left-(pwm_left/RATIO_Left_Right);
	drive_Robot(MOT_FAST_STOPP,pwm_right,MOT_FAST_STOPP,pwm_left,STOPP,STOPP);
//	_delay_ms(200);

	pwm_left = 200;
	pwm_right = pwm_left-(pwm_left/RATIO_Left_Right);
	drive_Robot(MOT_FORWARD,pwm_right,MOT_BACKWARD,pwm_left,ROT_180,ROT_180);
while(!check_impulse()) {}
//	_delay_ms(850);

	pwm_left = 0;
	pwm_right = pwm_left-(pwm_left/RATIO_Left_Right);
	drive_Robot(MOT_FAST_STOPP,pwm_right,MOT_FAST_STOPP,pwm_left,ONE_TURN,ONE_TURN);
//	_delay_ms(200);

	pwm_left = 200;
	pwm_right = pwm_left-(pwm_left/RATIO_Left_Right);
	drive_Robot(MOT_FORWARD,pwm_right,MOT_FORWARD,pwm_left,5*ONE_TURN,5*ONE_TURN);
while(!check_impulse()) {}
//	_delay_ms(10000);

	pwm_left = 0;
	pwm_right = pwm_left-(pwm_left/RATIO_Left_Right);
	drive_Robot(MOT_FAST_STOPP,pwm_right,MOT_FAST_STOPP,pwm_left,ONE_TURN,ONE_TURN);
*/
	
	/*char dot = 3;
	while(dot)
	{
		if(!check_impulse()) {}
		else
		{
			PORTD ^= (1<<LED_GRUEN);
			PORTB ^= (1<<LED_RH);
			PORTB ^= (1<<LED_RV);
			beep(3);
			_delay_ms(200);
			dot--;
		}
	}
	*/
	timer_beep_melodie();
//	beep(4);
	PORT_DUAL &=~ (1 << LED_ROT);
	PORT_DUAL |= (1 << LED_GRUEN);
	while (1)
	{
		PORT_DUAL &=~ (1 << LED_GRUEN);
		_delay_ms(500);
		PORT_DUAL |= (1 << LED_GRUEN);
		_delay_ms(2000);
	}
}
