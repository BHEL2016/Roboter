//***********************************************************************************
// elRobot: state machine
// Version: 2014_02_05
// author:  W. Kuran
//***********************************************************************************
// sauber Trennung zwischen c und header ...
// transfer all to englisch
// to do List:
// delete all unneeded stuff of disk
// in post pre out for all functions
// test it
// warning transfer to the headerfile
// header - corrections
// 0 errors 0 warnings
// write in the eeprom some numbers
// read from the flip the eeprom infos
// repair beep
//***********************************************************************************


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include "elRobot.h"

#define  WAIT      0
#define  FORWARD   1
#define  ROTATE    2




int main(void)
{
    volatile unsigned int state;

	set_fuses();
	init();

//#	beep(5);

    start();

    state = WAIT;

    for(;;)
	{
	    switch (state)
		{
		    case WAIT:
			     if (ms >= 500)
				 {
				     ms = 0;
                     state = FORWARD;
                     drive(2,2);


  				 }
			break;

			case FORWARD:
			     if (ms >= 500)
				 {
				     ms = 0;
                     state = ROTATE;
                     drive(3,-3);
  				 }
			break;

		    case ROTATE:
			     if (ms >= 500)
				 {

				     ms = 0;
                     state = FORWARD;
                     drive(2,2);
  				 }
			break;
		} // end of switch
	}
    return 0;
}
