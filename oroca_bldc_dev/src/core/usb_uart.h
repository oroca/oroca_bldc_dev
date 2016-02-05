#ifndef MYUSB_H_INCLUDED
#define MYUSB_H_INCLUDED
extern SerialUSBDriver SDU1;

void usb_uart_init(void);
int isUsbActive(void);

#endif // MYUSB_H_INCLUDED
