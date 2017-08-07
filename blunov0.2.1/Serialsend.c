/*
 * Serialsend.c
 *
 *  Created on: 2017年3月22日
 *      Author: Administrator
 */


/*
 * SerialApp.c
 *
 *  Created on: 2017年3月14日
 *      Author: Administrator
 */


#include "SerialApp.h"
#include "osal.h"
#include "hw_uart.h"
#include "atParser.h"
#include "stateMachine.h"
#include "stdio.h"
#include "stdbool.h"
#include "ad_uart.h"
#include <platform_devices.h>
#include <USB_CDC.h>
#include "simpleGATTprofile.h"
#include "Devinfoservice.h"
#include "Serialsend.h"



__RETAINED  OS_TASK Serial_send_task_handle;
extern volatile int rbledatalen;
void Serial_send_task(){
        uart_device dev;
        dev = ad_uart_open(SERIAL2);
        while(1){


                OS_BASE_TYPE ret;
                uint32_t notif;

                ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
                OS_ASSERT(ret == OS_TASK_NOTIFY_SUCCESS);

                if(ble_connect == 1 ){
                        OS_TASK_YIELD();
                }else{
                        if(notif && datalen){
                                rdatalen+=notif;
                                if(notif && rdatalen){
                                        if(rdatalen){
                                                debugcuappSendData(dev);
                                        }
                                }
                        }
                }

        }

}
void Serial_send_init(){
        OS_BASE_TYPE status;
        /* Start the Serial_App application task. */
        status = OS_TASK_CREATE("Serial_send_app",
                Serial_send_task,
                NULL,
                512,
                OS_TASK_PRIORITY_LOWEST+1,
                Serial_send_task_handle);
        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);
}

void Serial_send_stop(){
        OS_TASK_DELETE(Serial_send_task_handle);
}
