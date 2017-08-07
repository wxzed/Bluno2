/*
 * SerialApp.c
 *
 *  Created on: 2017Äê3ÔÂ14ÈÕ
 *      Author: Administrator
 */


#include "SerialApp.h"
#include "osal.h"
#include "hw_uart.h"
#include "atParser.h"
#include "stateMachine.h"
#include "stdio.h"
#include "stdbool.h"
__RETAINED static OS_TASK Serial_App_task_handle;
void Serial_App_task(){
        uint8_t buflength = 0;
        static uint8_t buf[32];
        while(1){
                if(si.at_mode == 0){
                        buf[0] = hw_uart_read(HW_UART2);
                        if(buf[0] == '+'){
                                buf[1] = hw_uart_read(HW_UART2);
                                if(buf[1] == '+'){
                                        buf[2] = hw_uart_read(HW_UART2);
                                        if(buf[2] == '+'){
                                                judgmentAT(buf,3);
                                        }else{
                                                hw_uart_send(HW_UART2, buf, 3, NULL, NULL);
                                        }
                                }else{
                                        hw_uart_write(HW_UART2,buf[0]);
                                        hw_uart_write(HW_UART2,buf[1]);
                                }
                        }else{
                                hw_uart_write(HW_UART2,buf[0]);
                        }
                }else if(si.at_mode == 1){
                        buf[buflength] = hw_uart_read(HW_UART2);
                        buflength++;
                        if((buf[buflength-1] == 0x0A) && (buf[buflength-2] == 0x0D)){
                                atParser(myCOM, buf, buflength);
                                buf[buflength-1] = 0;
                                buf[buflength-2] = 0;
                                buflength = 0;
                        }
                }
        }

}
void Serial_App_init(){
        OS_BASE_TYPE status;
        /* Start the Serial_App application task. */
        status = OS_TASK_CREATE("Serial_app",
                Serial_App_task,
                NULL,
                512,
                OS_TASK_PRIORITY_LOWEST+1,
                Serial_App_task_handle);
        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);
        //OS_TASK_DELETE(OS_GET_CURRENT_TASK());
}

void Serial_App_stop(){
        OS_TASK_DELETE(Serial_App_task_handle);
}
