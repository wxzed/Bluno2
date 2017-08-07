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
#include "time.h"
#include "oled2864.h"



__RETAINED  OS_TASK Serial_App_task_handle;


volatile int rbledatalen=0;
void Serial_Reinit1(U32 DTERate){
        U32 myDTERate = 115200;
        if(DTERate == 1000000){
                myDTERate = HW_UART_BAUDRATE_1000000;
        }else if(DTERate == 500000){
                myDTERate = HW_UART_BAUDRATE_500000;
        }else if(DTERate == 230400){
                myDTERate = HW_UART_BAUDRATE_230400;
        }else if(DTERate == 115200){
                myDTERate = HW_UART_BAUDRATE_115200;
        }else if(DTERate == 57600){
                myDTERate = HW_UART_BAUDRATE_57600;
        }else if(DTERate == 38400){
                myDTERate = HW_UART_BAUDRATE_38400;
        }else if(DTERate == 28800){
                myDTERate = HW_UART_BAUDRATE_28800;
        }else if(DTERate == 19200){
                myDTERate = HW_UART_BAUDRATE_19200;
        }else if(DTERate == 14400){
                myDTERate = HW_UART_BAUDRATE_14400;
        }else if(DTERate == 9600){
                myDTERate = HW_UART_BAUDRATE_9600;
        }else if(DTERate == 4800){
                myDTERate = HW_UART_BAUDRATE_4800;
        }
        uart_config recfg = {
                .baud_rate      = myDTERate,
                .data           = HW_UART_DATABITS_8,
                .stop           = HW_UART_STOPBITS_1,
                .parity         = HW_UART_PARITY_NONE,
                .use_dma        = 1,
                .use_fifo       = 1,
                .rx_dma_channel = HW_DMA_CHANNEL_4,
                .tx_dma_channel = HW_DMA_CHANNEL_5,
        };
        hw_uart_init(HW_UART2, &recfg);
        hw_gpio_configure_pin(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                        HW_GPIO_FUNC_GPIO, 1);

        hw_gpio_set_pin_function(HW_GPIO_PORT_1, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                        HW_GPIO_FUNC_UART2_TX);
        hw_gpio_set_pin_function(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                        HW_GPIO_FUNC_UART2_RX);
}
void Serial_Reinit(U32 DTERate){
        hw_gpio_configure_pin(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                        HW_GPIO_FUNC_GPIO, 0);

        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                HW_GPIO_FUNC_GPIO,0);
        hw_gpio_configure_pin(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                HW_GPIO_FUNC_GPIO,0);
        OS_DELAY(2);
        U32 myDTERate = 115200;
        if(DTERate == 1000000){
                myDTERate = HW_UART_BAUDRATE_1000000;
        }else if(DTERate == 500000){
                myDTERate = HW_UART_BAUDRATE_500000;
        }else if(DTERate == 230400){
                myDTERate = HW_UART_BAUDRATE_230400;
        }else if(DTERate == 115200){
                myDTERate = HW_UART_BAUDRATE_115200;
        }else if(DTERate == 57600){
                myDTERate = HW_UART_BAUDRATE_57600;
        }else if(DTERate == 38400){
                myDTERate = HW_UART_BAUDRATE_38400;
        }else if(DTERate == 28800){
                myDTERate = HW_UART_BAUDRATE_28800;
        }else if(DTERate == 19200){
                myDTERate = HW_UART_BAUDRATE_19200;
        }else if(DTERate == 14400){
                myDTERate = HW_UART_BAUDRATE_14400;
        }else if(DTERate == 9600){
                myDTERate = HW_UART_BAUDRATE_9600;
        }else if(DTERate == 4800){
                myDTERate = HW_UART_BAUDRATE_4800;
        }
        uart_config recfg = {
                .baud_rate      = myDTERate,
                .data           = HW_UART_DATABITS_8,
                .stop           = HW_UART_STOPBITS_1,
                .parity         = HW_UART_PARITY_NONE,
                .use_dma        = 1,
                .use_fifo       = 1,
                .rx_dma_channel = HW_DMA_CHANNEL_4,
                .tx_dma_channel = HW_DMA_CHANNEL_5,
        };
        hw_uart_init(HW_UART2, &recfg);
        hw_gpio_configure_pin(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                        HW_GPIO_FUNC_GPIO, 1);

        hw_gpio_set_pin_function(HW_GPIO_PORT_1, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                        HW_GPIO_FUNC_UART2_TX);
        hw_gpio_set_pin_function(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                        HW_GPIO_FUNC_UART2_RX);
}
uint8_t pack_net_buf[30];
uint8_t pack_net_buf_len = 0;
uint8_t packlen = 24;
bool my_cnt = false;
uint8_t receive_cnt = 0;
bool yes = false;
static void Group_package_1(uint8_t *buf,uint8_t len){
#if usblog
        if(open_usb == true){
                myserialusb("Group_package1\r\n",16);
        }

#endif
        memcpy(pack_net_buf+pack_net_buf_len,buf,len);
        pack_net_buf_len += len;
        if((packlen+4) == pack_net_buf_len){
                //hw_uart_send(HW_UART2, "Group_package1\r\n", 16, NULL, NULL);
                cuappEnqueue((uint8_t *)pack_net_buf+4, pack_net_buf_len-4);
                datalen++;
                bledatalen++;
                OS_TASK_NOTIFY(Serial_send_task_handle, datalen, eSetBits);
                pack_net_buf_len = 0;
                yes = false;
        }
        if(pack_net_buf_len>24){
                pack_net_buf_len = 0;
                yes = false;
        }
}
static void Group_package(uint8_t *buf,uint8_t len){
#if usblog
        if(open_usb == true){
                myserialusb("Group_package\r\n",15);
        }

#endif
        memcpy(pack_net_buf+pack_net_buf_len,buf,len);
        pack_net_buf_len += len;
        if((packlen+4) == pack_net_buf_len){
                cuappEnqueue((uint8_t *)pack_net_buf+4, pack_net_buf_len-4);
                datalen++;
                bledatalen++;
                OS_TASK_NOTIFY(Serial_send_task_handle, datalen, eSetBits);
                pack_net_buf_len = 0;
        }
        if(pack_net_buf_len>24){
                pack_net_buf_len = 0;
                yes = false;
        }
}
void Serial_App_task(){
        uint8_t buflength = 0;
        static char buf1[100];
        static char net_buf[30];

        //uint8_t net_buf[20];
        uint8_t receive_len = 24;

        uint8_t sum_len = 0;
        uart_device dev;

        uint8_t cnt;
        bool cnt1 = false;
        bool cnt2 = false;
        uint8_t ifat = 0;
        uint8_t cs = 0;
        datalen = 0;
        bledatalen = 0;
        dev = ad_uart_open(SERIAL2);

        while(1){
                ad_uart_bus_acquire(dev);
                if(si.at_mode == 1){
                        cnt = (uint8_t)ad_uart_read(dev, buf1, 32, 20);
                }else{
                        if(ble_connect == 1){
                                if(si.sys_cfg.networkcmode == 's'){
                                        cnt = (uint8_t)ad_uart_read(dev, net_buf, receive_len, 20);//之前测试的是(20,10),为了测试组网通信改为的(20,20)
                                }else{
                                        cnt = (uint8_t)ad_uart_read(dev, buf1, 20, 20);//之前测试的是(20,10),为了测试组网通信改为的(20,20)
                                }

                        }else{
                                cnt = (uint8_t)ad_uart_read(dev, buf1, 5, 3);//5,3
                        }
                }
                if(cnt){
                        Rx_blink_hight();
                        if(si.at_mode == 1){
                                atParser(myCOM, (uint8_t *)buf1, cnt);
                        }else if(cnt == 3 && ifat == 2 && (buf1[0] == '+' || net_buf[0] == '+') && (buf1[1] == '+' || net_buf[1] == '+') && (buf1[2] == '+' || net_buf[2] == '+')){
                                if(ble_connect == 1){
                                        if(si.sys_cfg.networkcmode == 's'){
                                                judgmentAT(myCOM, (uint8_t *)net_buf, cnt);//进入AT模式
                                                ifat = 0;
                                        }else{
                                                judgmentAT(myCOM, (uint8_t *)buf1, cnt);//进入AT模式
                                                ifat = 0;
                                        }
                                }else{
                                        judgmentAT(myCOM, (uint8_t *)buf1, cnt);//进入AT模式
                                        ifat = 0;
                                }
                        }else{
                                if(si.sys_cfg.networkcmode == 's'){
                                        sum_len += cnt;
                                        if(receive_len == 24){//代表是每个包的第一次接收。
                                                if(net_buf[0] == 0x55){
                                                        if(cnt>2){
                                                                if(net_buf[2] == cnt-4){//说明一次就接收完了数据包。
                                                                        packlen = net_buf[2];
                                                                        cuappEnqueue((uint8_t *)net_buf+4,cnt-4);
                                                                        datalen++;
                                                                        bledatalen++;
                                                                        OS_TASK_NOTIFY(Serial_send_task_handle, datalen, eSetBits);
                                                                        receive_len = 24;
                                                                        sum_len = 0;
                                                                }else if((net_buf[2]+4) > cnt){//说明数据包的实际长度应该比接收的长度长。
                                                                        packlen = net_buf[2];
                                                                        Group_package((uint8_t *)net_buf,cnt);
                                                                        receive_len = (net_buf[2]+4)-cnt;//下一次接收包的长度为该包还有多长的数据没接收。
                                                                }else{//说明是错包
                                                                        receive_len = 24;
                                                                }
                                                        }else if(cnt == 1){//说明第一次接收到的数据长度小于2
                                                                my_cnt = true;
                                                                cnt1 = true;
                                                                cnt2 = false;
                                                                receive_cnt = cnt;
                                                                receive_len = 2;
                                                                Group_package_1((uint8_t *)net_buf,cnt);
                                                                //printf("cnt == 1");
                                                        }else if(cnt == 2){
                                                                my_cnt = true;
                                                                cnt1 = false;
                                                                cnt2 = true;
                                                                receive_cnt = cnt;
                                                                receive_len = 1;
                                                                Group_package_1((uint8_t *)net_buf,cnt);
                                                                //printf("cnt == 2");
                                                        }
                                                }else{//如果不是则重新接收
                                                        receive_len = 24;
                                                }
                                        }else if(receive_len < 24){//代表是第二次及以上次接收数据包。
                                                if(my_cnt == true){
                                                        if(cnt1 == true && cnt2 == false){
                                                                if(cnt == 2){//说明接收到了包长。
                                                                        packlen =  net_buf[1];
                                                                        receive_len = packlen+4 - 3;
                                                                        Group_package_1((uint8_t *)net_buf,cnt);
                                                                        my_cnt = false;
                                                                        cnt1 = false;
                                                                        cnt2 = false;
                                                                        yes = true;
                                                                }else if(cnt == 1){//说明还没接收到包长
                                                                        cnt1 = false;
                                                                        cnt2 = true;
                                                                        receive_len = 1;
                                                                        Group_package_1((uint8_t *)net_buf,cnt);
                                                                        my_cnt = true;
                                                                        yes = false;
                                                                }

                                                        }else if(cnt1 == false && cnt2 == true){
                                                                packlen =  net_buf[0];
                                                                receive_len = packlen+4 - 3;
                                                                Group_package_1((uint8_t *)net_buf,cnt);
                                                                my_cnt = false;
                                                                yes = true;
                                                        }

                                                }else if(yes == true){
                                                        receive_len = (packlen+4) - sum_len;
                                                        Group_package_1((uint8_t *)net_buf,cnt);
                                                        if((sum_len-4) == packlen){
                                                                receive_len = 24;
                                                                sum_len = 0;
                                                        }
                                                        if(sum_len>24){
                                                                receive_len = 24;
                                                                sum_len = 0;
                                                        }
                                                }else{
                                                        receive_len = (packlen+4) - sum_len;
                                                        Group_package((uint8_t *)net_buf,cnt);//把数据包继续放在组包函数中
                                                        if((sum_len-4) == packlen){//则说明接收完了。
                                                                receive_len = 24;
                                                                sum_len = 0;
                                                        }
                                                        if(sum_len>24){
                                                                receive_len = 24;
                                                                sum_len = 0;
                                                        }
                                                }

                                        }else{
                                                //printf("else");
                                        }

                                }else{
                                        cuappEnqueue((uint8_t *)buf1,cnt);
                                        datalen++;
                                        bledatalen++;
                                        OS_TASK_NOTIFY(Serial_send_task_handle, datalen, eSetBits);
                                }

                        }
                 }
                else{
                         if(ifat == 0){
                                 ifat++;
                         }else if(ifat == 1){
                                 ifat++;
                         }else{
                                 ifat=2;
                         }

                 }
                ad_uart_bus_release(dev);


#if 0
                if(cnt){
                        if(ble_connect == 1){
                                //hw_uart_send();
                                //hw_uart_send(HW_UART2, buf1, cnt, NULL, NULL);
                                ad_uart_write(dev, buf1, cnt);
                                //ucappEnqueue((uint8_t *)buf1,cnt);
                                //ucappSendData();
                        }else{
                                //ad_uart_write(dev, buf1, cnt);
                                cuappEnqueue((uint8_t *)buf1,cnt);
                                //ucappEnqueue((uint8_t *)buf1,cnt);
                                //myserialusb((uint8_t *)buf1, cnt);

                        }

                        //printf("cnt=%d\r\n",cnt);
                        //bleAppendNode((uint8_t *)buf1,cnt);
                        //cuappEnqueue((uint8_t *)buf1,cnt);
                        //ucappEnqueue((uint8_t *)buf1,cnt);
                        //myserialusb((uint8_t *)buf1, cnt);
                }
#endif
                //wx_write_notif();//串口转蓝牙

                //ucappSendData();//串口转usb或蓝牙
#if 0
                if(si.at_mode == 0){
                        buf[0] = hw_uart_read(HW_UART2);
                        if(buf[0] == '+'){
                                buf[1] = hw_uart_read(HW_UART2);
                                if(buf[1] == '+'){
                                        buf[2] = hw_uart_read(HW_UART2);
                                        if(buf[2] == '+'){
                                                judgmentAT(myCOM,buf,3);
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
#endif
        }
        ad_uart_close(dev);
}
void Serial_App_init(){
        OS_BASE_TYPE status;
        /* Start the Serial_App application task. */
        status = OS_TASK_CREATE("Serial_app",
                Serial_App_task,
                NULL,
                1024,
                OS_TASK_PRIORITY_LOWEST+2,
                Serial_App_task_handle);
        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);
}

void Serial_App_stop(){
        OS_TASK_DELETE(Serial_App_task_handle);
}

