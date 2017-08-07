/*
 * MyGlobal.h
 *
 *  Created on: 2017Äê6ÔÂ16ÈÕ
 *      Author: Administrator
 */

#ifndef MYGLOBAL_H_
#define MYGLOBAL_H_
#include "stdbool.h"
#include "Global.h"
#include "sdk_defs.h"
#define B_ADDR_LEN    6
bool have_usb;
bool open_usb;
bool baudrate_change;
uint8_t baudrate_change_data;
U32 baudrate_num;
bool attest;
uint8_t conn_net_work;
uint8_t conn_role;
uint8 *bdAddr2Str(uint8 *buf, uint8 *pAddr );
#endif /* MYGLOBAL_H_ */
