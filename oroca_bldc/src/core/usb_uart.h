/*
 * comm_usb_serial.h
 *
 *  Created on: 8 okt 2015
 *      Author: benjamin
 */

#ifndef USB_UART_H_INCLUDED
#define USB_UART_H_INCLUDED

extern SerialUSBDriver SDU1;

void usb_uart_init(void);
int usb_uart_isActive(void);

int usb_uart_printf( const char *fmt, ...);
uint8_t usb_uart_getch( void );

#endif
