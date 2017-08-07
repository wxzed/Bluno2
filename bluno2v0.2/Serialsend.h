/*
 * Serialsend.h
 *
 *  Created on: 2017Äê3ÔÂ22ÈÕ
 *      Author: Administrator
 */

#ifndef SERIALSEND_H_
#define SERIALSEND_H_
#include <osal.h>


extern OS_TASK Serial_send_task_handle;
uint16_t rdatalen;
uint16_t bledatalen;
//uint16_t rbledatalen;
void Serial_send_init(void);
void Serial_send_stop(void);
#endif /* SERIALSEND_H_ */
