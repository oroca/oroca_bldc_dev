/*
 * uart3_print_initl.h
 *
 */

#ifndef __UART3_PRINT_H__
#define __UART3_PRINT_H__

static void Uart3_print_init(void);
static void Uart3_SendString(SerialDriver *sdp, const char *string);  /* for example 3 */
static void Uart3_print(char *p);
static void Uart3_println(char *p);
static void Uart3_printn(uint32_t n);
static void Uart3_printf(BaseSequentialStream * chp,const char * fmt,...);


#endif
