/*
 * Common.h
 *
 *  Created on: 2017��3��10��
 *      Author: Administrator
 */

#ifndef COMMON_H_
#define COMMON_H_
#define B_ADDR_LEN    6
#include "sdk_defs.h"

extern bool have_usb;
extern uint8 *bdAddr2Str(uint8 *buf, uint8 *pAddr );
#endif /* COMMON_H_ */
