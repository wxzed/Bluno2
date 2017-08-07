/**
 ****************************************************************************************
 *
 * @file usb_cdc.c
 *
 * @brief USB CDC app implementation
 *
 * Copyright (C) 2016. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
#include <stdarg.h>
#include "sys_charger.h"
#include "sys_power_mgr.h"
#include "hw_usb.h"
#include "sys_watchdog.h"
#include "USB_CDC.h"
#include "osal.h"


#include "ad_nvms.h"
#include "hw_uart.h"
#include "ble_gatts.h"
#include "simpleGATTprofile.h"
#include "wx_link.h"
#include "atParser.h"
#include "stateMachine.h"
#include "Common.h"
#include "Devinfoservice.h"
#include "SerialApp.h"
#include "Serialsend.h"
/*********************************************************************
 *
 *       Defines, configurable
 *
 **********************************************************************
 */

#define usb_main_TASK_PRIORITY              ( OS_TASK_PRIORITY_NORMAL )
__RETAINED static OS_TASK usb_cdc_task_handle;
__RETAINED static uint8 run_usb_task/* = 0*/;
static USB_HOOK UsbpHook;
//static char usb_cdc_buf[USB_MAX_PACKET_SIZE];
static uint8_t usb_cdc_buf[USB_MAX_PACKET_SIZE];
//
//  Information that is used during enumeration.
//
static const USB_DEVICE_INFO _DeviceInfo = {
        0x2341,         // VendorId
        0x0043,         // ProductId
        "Vendor",       // VendorName
        "CDC device",   // ProductName
        "12345678"      // SerialNumber.
};


USB_CDC_HANDLE mycdc;


USB_CDC_LINE_CODING  pLineCoding;
/*********************************************************************
 *
 *       _AddCDC
 *
 *  Function description
 *    Add communication device class to USB stack
 */
static USB_CDC_HANDLE _AddCDC(void)
{
        static U8 _abOutBuffer[USB_MAX_PACKET_SIZE];
        USB_CDC_INIT_DATA InitData;
        USB_CDC_HANDLE hInst;

        InitData.EPIn = USBD_AddEP(USB_DIR_IN, USB_TRANSFER_TYPE_BULK, 0, NULL, 0);
        InitData.EPOut = USBD_AddEP(USB_DIR_OUT, USB_TRANSFER_TYPE_BULK, 0, _abOutBuffer,
                USB_MAX_PACKET_SIZE);
        InitData.EPInt = USBD_AddEP(USB_DIR_IN, USB_TRANSFER_TYPE_INT, 8, NULL, 0);
        hInst = USBD_CDC_Add(&InitData);

        return hInst;
}

void usb_cdc_state_cb(void * pContext, U8 NewState)
{
        int OldState = USBD_GetState();

        if (((OldState & USB_STAT_ATTACHED) == 0) && (NewState & USB_STAT_ATTACHED)) {
                //Attached
        }

        if ((OldState & USB_STAT_ATTACHED) && ((NewState & USB_STAT_ATTACHED) == 0)) {
                //Detached
        }

        if (((OldState & USB_STAT_READY) == 0) && (NewState & USB_STAT_READY)) {
                //Ready
        }

        if ((OldState & USB_STAT_READY) && ((NewState & USB_STAT_READY) == 0)) {
                //Un-Ready
        }

        if (((OldState & USB_STAT_ADDRESSED) == 0) && (NewState & USB_STAT_ADDRESSED)) {
                //Addressed
        }

        if ((OldState & USB_STAT_ADDRESSED) && ((NewState & USB_STAT_ADDRESSED) == 0)) {
                //Un-Addressed
        }

        if (((OldState & USB_STAT_CONFIGURED) == 0) && (NewState & USB_STAT_CONFIGURED)) {
                //Configured
        }

        if ((OldState & USB_STAT_CONFIGURED) && ((NewState & USB_STAT_CONFIGURED) == 0)) {
                //Un-Configured
        }

        if (((OldState & USB_STAT_SUSPENDED) == 0) && (NewState & USB_STAT_SUSPENDED)) {
                // USB is going to be Suspended - DO NOT USE THIS POINT TO TRIGGER APP CODE!
                debug_print("USB Node State: Suspend (o:%d, n:%d)!\r\n", OldState, NewState);
        }

        if ((OldState & USB_STAT_SUSPENDED) && ((NewState & USB_STAT_SUSPENDED) == 0)) {
                // USB is going to be Resumed - DO NOT USE THIS POINT TO TRIGGER APP CODE!
                debug_print("USB Node State: Resume (o:%d, n:%d)!\r\n", OldState, NewState);
        }
}

/*********************************************************************
 *
 *       usb_is_suspended
 *
 *  Function description
 *    Callback to indicate that the USB Node is going to be suspended
 */
void usb_is_suspended(void)
{
        debug_print("App: USB Suspend!\r\n", 1);
}

/*********************************************************************
 *
 *       usb_is_resumed
 *
 *  Function description
 *    Callback to indicate that the USB Node is going to be resumed
 */
void usb_is_resumed(void)
{
        debug_print("App: USB Resume!\r\n", 1);
}
int myself_first = 1;
static void myself(USB_CDC_LINE_CODING * pLineCoding){
        if(myself_first == 1){
                myself_first = 0;
        }else{
                myself_first = 1;

                uint8_t bufdata[3]={0x50,0x70,0x60};
                if(pLineCoding->DTERate == 57600){
                        bufdata[0]=0x50;
                        bufdata[1]=0x70;
                        bufdata[2]=0x60;
                }else if(pLineCoding->DTERate == 115200){
                        bufdata[0]=0x10;
                        bufdata[1]=0x10;
                        bufdata[2]=0x50;
                }
                if(ble_connect == 1){
                        cuappEnqueue(bufdata,3);
                        bledatalen++;
                        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
                }
                Serial_Reinit1(pLineCoding->DTERate);

        }
}
static void my_scls(USB_CDC_CONTROL_LINE_STATE * pLineState){
        if(pLineState->DTR == 1 && pLineState->RTS == 0){//打开USB端口
                open_usb = true;
        }else if(pLineState->DTR == 0 && pLineState->RTS == 1){//关闭USB端口
                open_usb = false;
        }
}
/*********************************************************************
 *
 *       usb_cdc_task
 *
 */

void  myusbcdc(USB_CDC_HANDLE hInst){
        mycdc = hInst;
}
void  myserialusb(uint8_t* buf, uint8_t len){
        USBD_CDC_Write(mycdc, buf, len, 0);
}

void Merge_the_package(uint8_t *buf,uint8_t len,uint8_t *final_buf){
        uint16_t cs = 0;

        for(int i = 0;i < len; i++){
                cs +=buf[i];
        }
        cs=cs&0xff;
        final_buf[0] = 0x55;
        final_buf[1] = 0xaa;
        final_buf[2] = len;
        final_buf[3] = cs;
        memcpy(final_buf+4,buf,len*sizeof(uint8_t));
}

void usb_cdc_task(void *params)
{
        struct msgBuf *head =NULL;/*指向链表头*/
        struct blemsgBuf *blehead = NULL;

        static const uint8_t getFwversion[8]={0x55, 0x8F, 0x00, 0x01, 0x81, 0x30, 0x2E, 0x31};
        static const uint8_t dataconf[45]= {0x55, 0x78, 0x00, 0x26, 0x82, 0x7b, 0x22, 0x70, 0x61, 0x79, 0x6c, 0x6f, 0x61,
                                                0x64, 0x4c, 0x65, 0x6e, 0x22, 0x3a, 0x31, 0x30, 0x32, 0x34, 0x2c, 0x22,
                                                0x6d, 0x61, 0x78, 0x46, 0x69, 0x6c, 0x65, 0x53, 0x69, 0x7a, 0x65, 0x22,
                                                0x3a, 0x31, 0x30, 0x32, 0x34, 0x30, 0x30, 0x7d};
        static const uint8_t startUpgrade[7]={0x55, 0x9A, 0x00, 0x02, 0x83, 0x4F, 0x4B };
        static const uint8_t sendFile[7] = {0x55, 0x9A, 0x00, 0x02, 0x84, 0x4F, 0x4B};
        static const uint8_t sendSha256[7] = {0x55, 0x9A, 0x00, 0x02, 0x85, 0x4F, 0x4B};
        static const uint8_t endUpgrade[7] = {0x55, 0x9A, 0x00, 0x02, 0x86, 0x4F, 0x4B};
        uint8_t getFwversionflag = 0;
        uint8_t dataconfflag = 0;
        uint8_t startUpgradeflag = 0;
        uint8_t sendSha256flag = 0;
        uint16_t receivedlen = 0;
        uint16_t myreceivedlen = 0;
        uint8_t pack_cs = 0;
        uint8_t re_pack_data[20];
        uint32_t dst_offset;
        nvms_t update_part;


        USB_CDC_HANDLE hInst;
        int NumBytesReceived;
#if dg_configUSE_WDOG
        int8_t wdog_id;

        /* register usb_cdc_task task to be monitored by watchdog */
        wdog_id = sys_watchdog_register(false);
#endif

        USBD_Init();
        USBD_CDC_Init();
        USBD_RegisterSCHook(&UsbpHook, usb_cdc_state_cb, NULL);
        hInst = _AddCDC();
        USBD_CDC_SetOnLineCoding(hInst, myself);
        USBD_CDC_SetOnControlLineState(hInst,my_scls);
        //USBD_CDC_WaitForRX(0,0);
        //USBD_CDC_WaitForTX(0,0);
        USBD_SetDeviceInfo(&_DeviceInfo);
        USBD_Start();
        myusbcdc(hInst);
        dst_offset = 0;
        ble_evt_gatts_write_req_t *evt;
        evt->conn_idx = 0;
        evt->handle = 32;
        evt->offset = 0;
        evt->length = 0;
        uint8_t UNO_down = 0;
        bool arduino_down = false;
        while (1) {
#if dg_configUSE_WDOG
                /* notify watchdog on each loop */
                sys_watchdog_notify(wdog_id);
#endif

                //
                // Wait for configuration
                //
                while ((USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED))
                        != USB_STAT_CONFIGURED) {
                        OS_DELAY(50);
                }

#if dg_configUSE_WDOG
                /* suspend watchdog while blocking on USBD_CDC_Receive */
                sys_watchdog_suspend(wdog_id);
#endif
                //
                // Receive at maximum of sizeof(usb_cdc_buf) Bytes
                // If less data has been received,
                // this should be OK.
                //
                NumBytesReceived = USBD_CDC_Receive(hInst, usb_cdc_buf, sizeof(usb_cdc_buf), 0);
#if dg_configUSE_WDOG
                /* resume watchdog */
                sys_watchdog_notify_and_resume(wdog_id);
#endif


                if (NumBytesReceived > 0) {
#if 1
                        uint16_t n=0;
                        uint16_t cur_len = 0;
                        if(arduino_down == true){//mega1280
                                TX_blink_hight();
                                if(NumBytesReceived == 2 && usb_cdc_buf[0]== 0x51 && usb_cdc_buf[1]== 0x20){
                                        arduino_down = false;
                                        UNO_down = 0;

                                        hw_uart_send(HW_UART2, usb_cdc_buf, NumBytesReceived, NULL, NULL);
                                }else{

                                        hw_uart_send(HW_UART2, usb_cdc_buf, NumBytesReceived, NULL, NULL);
                                }
                        }else
                        if(si.at_mode == 1){//在AT模式下
                                if(si.sys_cfg.update == '1'){//判断是否处于更新固件模式

                                        if(getFwversionflag == 1 && startUpgradeflag == 1 && dataconfflag ==1){
                                                receivedlen += NumBytesReceived;
                                                if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x04) && (NumBytesReceived ==(usb_cdc_buf[3]*16*16+usb_cdc_buf[2]+5))){//如果收到的数据包和发过的整个包的数据相等
                                                        dst_offset = usb_cdc_buf[5]+(usb_cdc_buf[6]*16*16)+(usb_cdc_buf[7]*16*16*16*16)+(usb_cdc_buf[8]*16*16*16*16*16*16);
                                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                                        ad_nvms_write(update_part, dst_offset, usb_cdc_buf+9, NumBytesReceived-9);
                                                        dst_offset = 0;
                                                        USBD_CDC_Write(hInst, sendFile, 7, 0);
                                                }else if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x04) &&  (NumBytesReceived !=(usb_cdc_buf[3]*16*16+usb_cdc_buf[2]+5))){//每个包只会进来一次
                                                        dst_offset = usb_cdc_buf[5]+(usb_cdc_buf[6]*16*16)+(usb_cdc_buf[7]*16*16*16*16)+(usb_cdc_buf[8]*16*16*16*16*16*16);
                                                        myreceivedlen = usb_cdc_buf[3]*16*16+usb_cdc_buf[2]+5;
                                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                                        ad_nvms_write(update_part, dst_offset, usb_cdc_buf+9, NumBytesReceived-9);
                                                        dst_offset +=(NumBytesReceived-9);
                                                }else if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x05)){
                                                        sendSha256flag = 1;
                                                        USBD_CDC_Write(hInst, sendSha256, 7, 0);
                                                }else if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x06)){
                                                        getFwversionflag = 0;
                                                        startUpgradeflag = 0;
                                                        dataconfflag = 0;
                                                        USBD_CDC_Write(hInst, endUpgrade, 7, 0);
                                                }else{
                                                        if(sendSha256flag == 0){
                                                                if(receivedlen == myreceivedlen){
                                                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                                                        ad_nvms_write(update_part, dst_offset, usb_cdc_buf, NumBytesReceived);
                                                                        dst_offset = 0;
                                                                        receivedlen = 0;
                                                                        USBD_CDC_Write(hInst, sendFile, 7, 0);
                                                                }else{
                                                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                                                        ad_nvms_write(update_part, dst_offset, usb_cdc_buf, NumBytesReceived);
                                                                        dst_offset += (NumBytesReceived);
                                                                }
                                                        }else{
                                                                receivedlen = 0;
                                                        }
                                                }
                                        }else if(getFwversionflag == 0 && startUpgradeflag == 0 && dataconfflag == 0){
                                                if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x01)){//判断是否准备进入下载固件模式。
                                                        getFwversionflag = 1;
                                                        USBD_CDC_Write(hInst, getFwversion, 7, 0);
                                                }
                                        }else if(getFwversionflag == 1 && startUpgradeflag == 0 && dataconfflag ==0){
                                                if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x02)){
                                                        dataconfflag = 1;
                                                        USBD_CDC_Write(hInst, dataconf, 45, 0);
                                                }
                                        }else if(getFwversionflag == 1 && dataconfflag ==1 && startUpgradeflag == 0){
                                                if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x03)){
                                                        startUpgradeflag = 1;
                                                        //printf("Serial_App_stop\r\n");
                                                        Serial_App_stop();
                                                        USBD_CDC_Write(hInst, startUpgrade, 7, 0);
                                                }
                                        }
                                }else{//执行其他AT命令。
                                        atParser(myUSB, usb_cdc_buf, NumBytesReceived);
                                }

                        }else if(NumBytesReceived == 3 && (usb_cdc_buf[0]=='+') && (usb_cdc_buf[1]=='+') && (usb_cdc_buf[2]=='+')){//进入AT指令
                                judgmentAT(myUSB, usb_cdc_buf, NumBytesReceived);
                        }else if(ble_connect == 1){//蓝牙连接的情况下

                                if(si.sys_cfg.networkcmode == 'o'){//如果蓝牙连接且在关闭星型网络下的话，将数据放入队列中（以20个字节为一个包）////BUG要换

                                        do{
                                                cur_len = NumBytesReceived > MAX_SPP_PACKET_LEN ? MAX_SPP_PACKET_LEN : NumBytesReceived;
                                                cuappEnqueue(usb_cdc_buf+n,cur_len);
                                                bledatalen++;
                                                OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
                                                n  += cur_len;
                                                NumBytesReceived -=cur_len;
                                        }while(NumBytesReceived != 0);
                                }else if(si.sys_cfg.networkcmode == 's'){//在星型网络下
                                        if(NumBytesReceived>4){//如果收到的数据包大于4，就是说
                                                if(usb_cdc_buf[0] == 0x55 && (uint8_t)usb_cdc_buf[1] == 0xaa){//判断数据是不是有效包
                                                        if(usb_cdc_buf[2] == NumBytesReceived-4){//收到的长度-4等于包自带的数据长度。
                                                                memcpy(re_pack_data, (uint8_t*)usb_cdc_buf+4, NumBytesReceived-4);
                                                                pack_cs = 0;

                                                                for(int i=0; i<NumBytesReceived-4; i++){
                                                                        pack_cs += (uint8_t)usb_cdc_buf[i+4];
                                                                }
                                                                pack_cs = pack_cs&0xff;
                                                                if(pack_cs == (uint8_t)usb_cdc_buf[3]){//校验cs
#if usblog
                                                                        USBD_CDC_Write(hInst, "cuappEnqueue\r\n", 14, 0);
#endif
                                                                        cuappEnqueue(re_pack_data,NumBytesReceived-4);
                                                                        bledatalen++;
                                                                        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
                                                                }
                                                        }
                                                }
                                        }
                                }
                        }else{//蓝牙未连接
                                if(NumBytesReceived == 2){//arduino UNO下载程序时自动复位
                                        if(UNO_down == 0 && usb_cdc_buf[0] == 0x30 && usb_cdc_buf[1] == 0x20){
                                                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_6, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
                                                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
                                                OS_DELAY(3);
                                                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_6, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
                                                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
                                                UNO_down = 1;
                                                arduino_down = true;
                                        }else{
                                                UNO_down = 0;
                                                TX_blink_hight();
                                                hw_uart_send(HW_UART2, usb_cdc_buf, NumBytesReceived, NULL, NULL);
                                        }
                                }else{
                                        UNO_down = 0;
                                        TX_blink_hight();
                                        hw_uart_send(HW_UART2, usb_cdc_buf, NumBytesReceived, NULL, NULL);
                                }

                        }
#endif
#if 0
                        uint16_t n=0;
                        int cur_len;
                        if(arduino_down == true){//mega1280
                                TX_blink_hight();
                                if(NumBytesReceived == 2 && usb_cdc_buf[0]== 0x51 && usb_cdc_buf[1]== 0x20){
                                        arduino_down = false;
                                        UNO_down = 0;

                                        hw_uart_send(HW_UART2, usb_cdc_buf, NumBytesReceived, NULL, NULL);
                                }else{

                                        hw_uart_send(HW_UART2, usb_cdc_buf, NumBytesReceived, NULL, NULL);
                                }
                        }else
                        if(getFwversionflag == 1 && startUpgradeflag == 1 && dataconfflag ==1){
                                receivedlen += NumBytesReceived;
                                if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x04) && (NumBytesReceived ==(usb_cdc_buf[3]*16*16+usb_cdc_buf[2]+5))){//如果收到的数据包和发过的整个包的数据相等
                                        dst_offset = usb_cdc_buf[5]+(usb_cdc_buf[6]*16*16)+(usb_cdc_buf[7]*16*16*16*16)+(usb_cdc_buf[8]*16*16*16*16*16*16);
                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                        ad_nvms_write(update_part, dst_offset, usb_cdc_buf+9, NumBytesReceived-9);
                                        dst_offset = 0;
                                        USBD_CDC_Write(hInst, sendFile, 7, 0);
                                }else if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x04) &&  (NumBytesReceived !=(usb_cdc_buf[3]*16*16+usb_cdc_buf[2]+5))){//每个包只会进来一次
                                        dst_offset = usb_cdc_buf[5]+(usb_cdc_buf[6]*16*16)+(usb_cdc_buf[7]*16*16*16*16)+(usb_cdc_buf[8]*16*16*16*16*16*16);
                                        myreceivedlen = usb_cdc_buf[3]*16*16+usb_cdc_buf[2]+5;
                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                        ad_nvms_write(update_part, dst_offset, usb_cdc_buf+9, NumBytesReceived-9);
                                        dst_offset +=(NumBytesReceived-9);
                                }else if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x05)){
                                        sendSha256flag = 1;
                                        USBD_CDC_Write(hInst, sendSha256, 7, 0);
                                }else if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x06)){
                                        getFwversionflag = 0;
                                        startUpgradeflag = 0;
                                        dataconfflag = 0;
                                        USBD_CDC_Write(hInst, endUpgrade, 7, 0);
                                }else{
                                        if(sendSha256flag == 0){
                                                if(receivedlen == myreceivedlen){
                                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                                        ad_nvms_write(update_part, dst_offset, usb_cdc_buf, NumBytesReceived);
                                                        dst_offset = 0;
                                                        receivedlen = 0;
                                                        USBD_CDC_Write(hInst, sendFile, 7, 0);
                                                }else{
                                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                                        ad_nvms_write(update_part, dst_offset, usb_cdc_buf, NumBytesReceived);
                                                        dst_offset += (NumBytesReceived);
                                                }
                                        }else{
                                                receivedlen = 0;
                                        }
                                }
                        }else if(getFwversionflag == 0 && startUpgradeflag == 0 && dataconfflag == 0){
                                if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x01)){//判断是否准备进入下载固件模式。
                                        getFwversionflag = 1;
                                        USBD_CDC_Write(hInst, getFwversion, 7, 0);
                                }else{
                                        if(si.at_mode == 1){/*AT模式下*/
                                                atParser(myUSB, usb_cdc_buf, NumBytesReceived);
                                        }else if(NumBytesReceived == 3 && (usb_cdc_buf[0]=='+') && (usb_cdc_buf[1]=='+') && (usb_cdc_buf[2]=='+')){/*进入AT模式*/
                                                judgmentAT(myUSB, usb_cdc_buf, NumBytesReceived);
                                        }else if(ble_connect == 1){//如果蓝牙连接的话，将数据放入队列中（以20个字节为一个包）
                                                if(si.sys_cfg.networkcmode == 'o'){
                                                        do{
                                                                cur_len = NumBytesReceived > MAX_SPP_PACKET_LEN ? MAX_SPP_PACKET_LEN : NumBytesReceived;
                                                                cuappEnqueue(usb_cdc_buf+n,cur_len);
                                                                bledatalen++;
                                                                OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
                                                                n  += cur_len;
                                                                NumBytesReceived -=cur_len;
                                                        }while(NumBytesReceived != 0);

                                                }else if(si.sys_cfg.networkcmode == 's'){
                                                        if(NumBytesReceived>4){
                                                                if(usb_cdc_buf[0] == 0x55 && (uint8_t)usb_cdc_buf[1] == 0xaa){
                                                                        if(usb_cdc_buf[2] == NumBytesReceived-4){
                                                                                memcpy(re_pack_data, (uint8_t*)usb_cdc_buf+4, NumBytesReceived-4);
                                                                                pack_cs = 0;

                                                                                for(int i=0; i<NumBytesReceived-4; i++){
                                                                                        pack_cs += (uint8_t)usb_cdc_buf[i+4];
                                                                                }
                                                                                pack_cs = pack_cs&0xff;
                                                                                if(pack_cs == (uint8_t)usb_cdc_buf[3]){
#if usblog
                                                                                        USBD_CDC_Write(hInst, "cuappEnqueue\r\n", 14, 0);
#endif
                                                                                        cuappEnqueue(re_pack_data,NumBytesReceived-4);
                                                                                        bledatalen++;
                                                                                        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
                                                                                }
                                                                        }
                                                                }
                                                        }
                                                }
                                        }else{//蓝牙未连接，usb打开，则将USB数据传给串口
                                                //hw_uart_send(HW_UART2, usb_cdc_buf, NumBytesReceived, NULL, NULL);
#if 1
                                                if(NumBytesReceived == 2){

                                                        if(UNO_down == 0 && usb_cdc_buf[0] == 0x30 && usb_cdc_buf[1] == 0x20){//arduino UNO下载程序时主动进行复位
                                                                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_6, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
                                                                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
                                                                OS_DELAY(3);
                                                                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_6, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
                                                                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
                                                                UNO_down = 1;
                                                                arduino_down = true;
                                                        }else{
                                                                TX_blink_hight();
                                                                hw_uart_send(HW_UART2, usb_cdc_buf, NumBytesReceived, NULL, NULL);
                                                        }

                                                }else{
                                                        UNO_down = 0;
                                                        TX_blink_hight();
                                                        hw_uart_send(HW_UART2, usb_cdc_buf, NumBytesReceived, NULL, NULL);
                                                }
#endif
                                                //hw_uart_send(HW_UART2, usb_cdc_buf, NumBytesReceived, NULL, NULL);

                                        }
                                }
                        }else if(getFwversionflag == 1 && startUpgradeflag == 0 && dataconfflag ==0){
                                if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x02)){
                                        dataconfflag = 1;
                                        USBD_CDC_Write(hInst, dataconf, 45, 0);
                                }
                        }else if(getFwversionflag == 1 && dataconfflag ==1 && startUpgradeflag == 0){
                                if((usb_cdc_buf[0] == 0x55) && (usb_cdc_buf[4] == 0x03)){
                                        startUpgradeflag = 1;
                                        //printf("Serial_App_stop\r\n");
                                        Serial_App_stop();
                                        USBD_CDC_Write(hInst, startUpgrade, 7, 0);
                                }
                        }
#endif
                }else{
                        OS_TASK_YIELD();
                }
        }
}

void usb_cdc_start()
{
        OS_BASE_TYPE status;

        /* Start the USB CDC application task. */
        status = OS_TASK_CREATE("UsbCdcTask",   /* The text name assigned to the task, for
                                                   debug only; not used by the kernel. */
                        usb_cdc_task,           /* The function that implements the task. */
                        NULL,                   /* The parameter passed to the task. */
                        1024,                    /* The number of bytes to allocate to the
                                                   stack of the task. */
                        usb_main_TASK_PRIORITY, /* The priority assigned to the task. */
                        usb_cdc_task_handle);   /* The task handle. */

        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);
}

void usb_cdc_stop()
{
        USBD_UnregisterSCHook(&UsbpHook);
        USBD_DeInit();
        OS_TASK_DELETE(usb_cdc_task_handle);
}

/*********************************************************************
 *
 *       usb_start_enumeration_cb
 *
 *  Function description
 *    Event callback called from the usbcharger task to notify
 *    the application about to allow enumeration.
 *    Note: The USB charger task is started before the application task. Thus, these
 *          call-backs may be called before the application task is started.
 *          The application code should handle this case, if need be.
 */
void usb_start_enumeration_cb(void)
{
        if (run_usb_task == 0) {
                have_usb = true;
                pm_stay_alive();
                run_usb_task = 1;
                cm_sys_clk_set(sysclk_PLL96);
                hw_usb_init();
                hw_usb_bus_attach();
                usb_cdc_start();
        }

}
/*********************************************************************
 *
 *       usb_detach_cb
 *
 *  Function description
 *    Event callback called from the usbcharger task to notify
 *    the application that a detach of the USB cable was detected.
 *
 *    Note: The USB charger task is started before the application task. Thus, these
 *          call-backs may be called before the application task is started.
 *          The application code should handle this case, if need be.
 */
void usb_detach_cb(void)
{
        if (run_usb_task == 1) {
                have_usb = false;
                hw_usb_bus_detach();
                usb_cdc_stop();
                cm_sys_clk_set(sysclk_XTAL16M);
                run_usb_task = 0;
                pm_resume_sleep();
        }
}
