/*
	OROCA BLDC PROJECT.



*/

#include "main.h"


class cTest
{
public:
	void print( void );
};


void cTest::print( void )
{
	return;
}


cTest test;


/*---------------------------------------------------------------------------
     TITLE   : main
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
int main(void)
{
//=================================
// hardware setup
	
	bldc_init();

	test.print();

	app_init();

//=================================
	for(;;)
	{
		chThdSleepMilliseconds(10);
	}

	return 0;
}

