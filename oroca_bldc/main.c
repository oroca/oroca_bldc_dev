/*
	OROCA BLDC PROJECT.



*/

#include "main.h"



//-- 내부함수
//
void main_init(void);





/*---------------------------------------------------------------------------
     TITLE   : main
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
int main(void)
{
	main_init();


	//-- BLDC 시작
	//
	bldc_start();


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
