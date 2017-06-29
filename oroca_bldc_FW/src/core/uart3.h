/*
 * uart3.h
 *
 */

#ifndef __UART3_H__
#define __UART3_H__

#include <chstreams.h>
#include "chprintf.h"

void Uart3_init(void);
void Uart3_printf(BaseSequentialStream * chp,const char * fmt,...);

void Uart3_write(char *pbuf, uint8_t len);
uint8_t Uart3_getch( void );


#endif
