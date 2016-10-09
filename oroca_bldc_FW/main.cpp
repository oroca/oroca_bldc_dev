/*
	OROCA BLDC PROJECT.



*/

#include "main.h"



//-- 내부함수
//
void main_init(void);


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
	main_init();

	test.print();

//=================================
//chibios process start
	bldc_start();

	app_ppm_start();
	return 0;
}





/*---------------------------------------------------------------------------
     TITLE   : main_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void main_init(void)
{
	bldc_init();
}
