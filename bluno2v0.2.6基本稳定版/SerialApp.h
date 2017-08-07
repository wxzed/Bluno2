/*
 * SerialApp.h
 *
 *  Created on: 2017Äê3ÔÂ14ÈÕ
 *      Author: Administrator
 */

#ifndef SERIALAPP_H_
#define SERIALAPP_H_
#include <ad_spi.h>
#include <ad_i2c.h>
#include <ad_uart.h>
#include <ad_gpadc.h>
#include <ad_temp_sens.h>
#include <Global.h>


uint16_t datalen;
extern bool have_usb;
extern bool open_usb;
extern void Rx_blink_hight();
extern void Tx_blink_hight();
void Serial_Reinit(U32 DTERate);
void Serial_App_init(void);
void Serial_App_stop(void);
#endif /* SERIALAPP_H_ */
