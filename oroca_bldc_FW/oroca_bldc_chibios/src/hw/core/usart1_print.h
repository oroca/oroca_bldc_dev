/*
 * uart1_print_initl.h
 *
 */

#ifndef __USART1_PRINT_H__
#define __USART1_PRINT_H__

#include "chprintf.h"

void Usart1_print_init(void);
void Usart1_SendString(SerialDriver *sdp, const char *string);  /* for example 3 */
void Usart1_print(char *p);
void Usart1_println(char *p);
void Usart1_printn(uint32_t n);
void Usart1_printf(BaseSequentialStream * chp,const char * fmt,...);


#endif
