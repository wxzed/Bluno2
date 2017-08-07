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
extern bool baudrate_change;
extern uint8_t baudrate_change_data;
extern U32 baudrate_num;
extern void Merge_the_package(uint8_t *buf,uint8_t len,uint8_t *final_buf);
extern void Rx_blink_hight();
extern void Tx_blink_hight();
extern void Uno_rest();
void Serial_Reinit(U32 DTERate);
void Serial_Reinit1(U32 DTERate);
void Serial_App_init(void);
void Serial_App_stop(void);
#endif /* SERIALAPP_H_ */
