//***********************************************************************************
// elRobot: state machine header file
// Version: 2014_02_05
// author:  W. Kuran
//***********************************************************************************

void set_fuses(void);
// POST: this function corrects internal fuse bits - without this, left wheel dosn't work correct

void init(void);
// PRE: the function set_fuses must be called!
// POST: all direction register settings and interrupt preparations are done by this function!

void start(void);
// PRE: correct initialization
// POST: the elRobot hardware is ready to drive

void drive(int8_t l, int8_t r);
// IN: l the speed for the left wheel +5,+4,...0,-1,-2,-3,-4,-5;  r for the right; 0 means standing
// PRE: correct initialization, the function start must be called
// POST: elRobot moves

void ledGreenOn(void);
void ledRedOn(void);
void ledGreenOff(void);
void ledRedOff(void);

// global used variables:
volatile uint16_t ms;
// ms: to measure the milli seconds

// end
