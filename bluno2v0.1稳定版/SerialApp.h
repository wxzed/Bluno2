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


uint16_t datalen;
void Serial_App_init(void);
void Serial_App_stop(void);
#endif /* SERIALAPP_H_ */
