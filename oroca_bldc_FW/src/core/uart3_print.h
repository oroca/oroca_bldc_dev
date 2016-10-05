/*
 * uart3_print_initl.h
 *
 */

#ifndef __UART3_PRINT_H__
#define __UART3_PRINT_H__

void Uart3_print_init(void);
void Uart3_SendString(SerialDriver *sdp, const char *string);  /* for example 3 */
void Uart3_print(char *p);
void Uart3_println(char *p);
void Uart3_printn(uint32_t n);
void Uart3_printf(BaseSequentialStream * chp,const char * fmt,...);


#endif
