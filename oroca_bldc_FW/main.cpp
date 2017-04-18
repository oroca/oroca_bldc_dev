/*
	OROCA BLDC PROJECT.



*/
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "timeout.h"


#include "main.h"

#include "uart3_print.h"
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

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

	app_init();

	timeout_init();
	timeout_configure(1000);


	test.print();

//=================================
	for(;;)
	{
		chThdSleepMilliseconds(10);
	}

	return 0;
}

