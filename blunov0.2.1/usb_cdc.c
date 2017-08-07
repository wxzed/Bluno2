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
#include "USB.h"


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
//static char sector_buffer[USB_MAX_PACKET_SIZE];

#define FLASH_SECTOR_SIZE       (512)
static uint8_t sector_buffer[FLASH_SECTOR_SIZE];


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







#if 0
struct Link *AppendNode(struct Link *head, uint8_t  data);
static void DispLink(struct Link *head);
static void DeleteMemory(struct Link *head);
static void wx_main( uint8_t data);

struct Link{
        char data;
        struct Link *next;
};
//struct Link *head =NULL;/*指向链表头*/
//int i = 0;
//void wx_main(uint8_t  data){
        //printf(" do you want to append a new node (Y/N)?\r\n");
       // head = AppendNode(head,data);
        //DispLink(head);   /*显示当前链表中的各节点信息*/
        //i++;
        //printf("%d new nodes have been apended!\r\n", i);

        //DeleteMemory(head);  /*释放所有已分配的内存*/
//}


/*
 * 函数功能 ： 新建一个节点，并将该节点添加到链表的末尾
 * 函数的参数：结构体指针变量head，表示原有链表的头节点指针
 * 函数的返回值：添加节点后的链表的头节点指针 */
struct Link *AppendNode(struct Link *head,uint8_t data){
        struct Link *p = NULL;
        struct Link *pr = head;
        //int data;
        /*为新添加的节点申请内存*/
        p = (struct Link*)OS_MALLOC(sizeof(struct Link));

        if(p == NULL){/*若申请内存失败，则打印错误信息，退出程序*/
                printf("No enough memory to alloc\r\n");
                exit(0);
        }
        if(head == NULL){/*若原链表为空表，则将新建节点置为首节点*/
                head = p;
        }else{/*若原链表为非空，则将新建节点添加到表尾*/
                /*若未到表尾，则继续移动指针pr,直到pr指向表尾*/
                while(pr->next != NULL){
                        pr = pr->next;
                }
                pr->next = p;/*将新建节点添加到链表的末尾*/
        }

        pr = p;/* 让pr指向新建节点*/
        //printf("Input node data:\r\n");
        //scanf("%d",&data);/*输入节点数据*/
        pr->data = data;
        pr->next = NULL;/*将新建节点置为表尾*/
        return head;/*返回添加节点后的链表的头节点指针*/
}



/*
 * 函数的功能：显示所有已经建立好的节点的节点号和该节点中的数据项内容
 * 函数的参数：结构体指针变量head，表示链表的头节点指针
 * 函数的返回值：无*/
void DispLink(struct Link *head){
        struct Link *p = head;
        int j = 1;
        while(p != NULL){/*若不是表尾，则循环打印*/
                printf("%c\n",p->data);/*打印第j个节点的数据*/
                p = p->next;/*让p指向下一个节点*/
                j++;
        }
}


/*
 * 函数功能：释放head指向的链表中所有节点占用的内存
 * 输入参数：结构体指针变量head,表示链表的头节点指针
 * 返回参数：无*/
void DeleteMemory(struct Link *head){
        struct Link *p= head, *pr = NULL;
        while(p != NULL){/*若不是表尾，则释放节点占用的内存*/
                pr = p;/*在pr中保存当前节点的指针*/
                p = p->next;/*让p指向下一节点*/
                OS_FREE(pr);/*释放pr指向的当前节点占用的内存*/

        }
}


#endif



















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

void  myusbcdc(USB_CDC_HANDLE hInst){
        mycdc = hInst;
}
void  myserialusb(uint8_t* buf, uint8_t len){
        USBD_CDC_Write(mycdc, buf, len, 0);
}

/*********************************************************************
 *
 *       usb_cdc_task
 *
 */
static uint8_t mistake = 0;
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
        size_t dst_offset;
        nvms_t update_part;

        USB_CDC_HANDLE hInst;
        size_t NumBytesReceived;
        bledatalen = 0;
#if dg_configUSE_WDOG
        int8_t wdog_id;

        /* register usb_cdc_task task to be monitored by watchdog */
        wdog_id = sys_watchdog_register(false);
#endif

        USBD_Init();
        USBD_CDC_Init();
        USBD_RegisterSCHook(&UsbpHook, usb_cdc_state_cb, NULL);
        hInst = _AddCDC();
        USBD_SetDeviceInfo(&_DeviceInfo);
        USBD_Start();
        myusbcdc(hInst);

        dst_offset = 0;
        ble_evt_gatts_write_req_t *evt;

        evt->conn_idx = 0;
        evt->handle = 32;
        evt->offset = 0;
        evt->length = 0;


        while (1) {
#if dg_configUSE_WDOG
                /* notify watchdog on each loop */
                sys_watchdog_notify(wdog_id);
#endif

                if(mistake == 1){
                        hw_usb_init();
                        hw_usb_bus_attach();
                        USBD_Init();
                        USBD_CDC_Init();
                        USBD_RegisterSCHook(&UsbpHook, usb_cdc_state_cb, NULL);
                        hInst = _AddCDC();
                        USBD_SetDeviceInfo(&_DeviceInfo);
                        USBD_Start();
                        myusbcdc(hInst);
                        mistake= 0;
                }
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
                //NumBytesReceived = (size_t)USBD_CDC_Receive(hInst, sector_buffer, sizeof(sector_buffer), 0);
                NumBytesReceived = USBD_CDC_Receive(hInst, sector_buffer, sizeof(sector_buffer), 0);
                //NumBytesReceived = USBD_CDC_Receive(hInst, usb_cdc_buf, sizeof(usb_cdc_buf), 0);

#if dg_configUSE_WDOG
                /* resume watchdog */
                sys_watchdog_notify_and_resume(wdog_id);
#endif
                if(NumBytesReceived == 0){
                        mistake = 1;
                        hw_usb_bus_detach();
                        USBD_UnregisterSCHook(&UsbpHook);
                        USBD_DeInit();
                }else
                /*
                uint16_t cs = 0;
                for(int i = 5;i<43;i++){
                        cs+=dataconf[i];

                }
                cs=cs&0xff;
                printf("cs1=%d\r\n",cs);
                                                */
                if (NumBytesReceived > 0) {

                        uint16_t n=0;
                        int cur_len;
                        if(getFwversionflag == 1 && startUpgradeflag == 1 && dataconfflag ==1){
                                receivedlen += NumBytesReceived;
                                if((sector_buffer[0] == 0x55) && (sector_buffer[4] == 0x04) && (NumBytesReceived ==(sector_buffer[3]*16*16+sector_buffer[2]+5))){//如果收到的数据包和发过的整个包的数据相等
                                        dst_offset = sector_buffer[5]+(sector_buffer[6]*16*16)+(sector_buffer[7]*16*16*16*16)+(sector_buffer[8]*16*16*16*16*16*16);
                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                        ad_nvms_write(update_part, dst_offset, sector_buffer+9, NumBytesReceived-9);
                                        dst_offset = 0;
                                        USBD_CDC_Write(hInst, sendFile, 7, 0);
                                }else if((sector_buffer[0] == 0x55) && (sector_buffer[4] == 0x04) &&  (NumBytesReceived !=(sector_buffer[3]*16*16+sector_buffer[2]+5))){//每个包只会进来一次
                                        dst_offset = sector_buffer[5]+(sector_buffer[6]*16*16)+(sector_buffer[7]*16*16*16*16)+(sector_buffer[8]*16*16*16*16*16*16);
                                        myreceivedlen = sector_buffer[3]*16*16+sector_buffer[2]+5;
                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                        ad_nvms_write(update_part, dst_offset, sector_buffer+9, NumBytesReceived-9);
                                        dst_offset +=(NumBytesReceived-9);
                                }else if((sector_buffer[0] == 0x55) && (sector_buffer[4] == 0x05)){
                                        sendSha256flag = 1;
                                        USBD_CDC_Write(hInst, sendSha256, 7, 0);
                                }else if((sector_buffer[0] == 0x55) && (sector_buffer[4] == 0x06)){
                                        getFwversionflag = 0;
                                        startUpgradeflag = 0;
                                        dataconfflag = 0;
                                        USBD_CDC_Write(hInst, endUpgrade, 7, 0);
                                }else{
                                        if(sendSha256flag == 0){
                                                if(receivedlen == myreceivedlen){
                                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                                        ad_nvms_write(update_part, dst_offset, sector_buffer, NumBytesReceived);
                                                        dst_offset = 0;
                                                        receivedlen = 0;
                                                        USBD_CDC_Write(hInst, sendFile, 7, 0);
                                                }else{
                                                        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
                                                        ad_nvms_write(update_part, dst_offset, sector_buffer, NumBytesReceived);
                                                        dst_offset += (NumBytesReceived);
                                                }
                                        }else{
                                                receivedlen = 0;
                                        }
                                }
                        }else if(getFwversionflag == 0 && startUpgradeflag == 0 && dataconfflag == 0){
                                if((sector_buffer[0] == 0x55) && (sector_buffer[4] == 0x01)){//判断是否准备进入下载固件模式。
                                        getFwversionflag = 1;
                                        USBD_CDC_Write(hInst, getFwversion, 7, 0);
                                }else{
                                        if(si.at_mode == 1){/*AT模式下*/
                                                atParser(myUSB, sector_buffer, NumBytesReceived);
                                        }else if(NumBytesReceived == 3 && (sector_buffer[0]=='+') && (sector_buffer[1]=='+') && (sector_buffer[2]=='+')){/*进入AT模式*/
                                                judgmentAT(myUSB, sector_buffer, NumBytesReceived);
                                        }
                                        else if(ble_connect == 1){//如果蓝牙连接的话，将数据放入队列中（以20个字节为一个包）
                                                do{
                                                        cur_len = NumBytesReceived > MAX_SPP_PACKET_LEN ? MAX_SPP_PACKET_LEN : NumBytesReceived;
                                                        cuappEnqueue(sector_buffer+n,cur_len);
                                                        bledatalen++;
                                                        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
                                                        n  += cur_len;
                                                        NumBytesReceived -=cur_len;
                                                }while(NumBytesReceived != 0);
                                        }else{//蓝牙未连接，usb打开，则将USB数据传给串口
                                                hw_uart_send(HW_UART2, sector_buffer, NumBytesReceived, NULL, NULL);
                                        }
#if 0
                                        else if(si.usb_serial_open == 1){/*如果usb转串口打开的话，则将USB收到的数据发送给串口*/
                                                hw_uart_send(HW_UART2, sector_buffer, NumBytesReceived, NULL, NULL);

                                        }else{//蓝牙没连接的且USB转UART没开启的话USB收到的直接发给USB
                                                USBD_CDC_Write(hInst, sector_buffer, NumBytesReceived, 0);
                                                //hw_uart_send(HW_UART2, sector_buffer, NumBytesReceived, NULL, NULL);
                                        }
#endif
                                }
                        }else if(getFwversionflag == 1 && startUpgradeflag == 0 && dataconfflag ==0){
                                if((sector_buffer[0] == 0x55) && (sector_buffer[4] == 0x02)){
                                        dataconfflag = 1;
                                        USBD_CDC_Write(hInst, dataconf, 45, 0);
                                }

                        }else if(getFwversionflag == 1 && dataconfflag ==1 && startUpgradeflag == 0){
                                if((sector_buffer[0] == 0x55) && (sector_buffer[4] == 0x03)){
                                        startUpgradeflag = 1;
                                        Serial_App_stop();
                                        USBD_CDC_Write(hInst, startUpgrade, 7, 0);
                                }
                        }

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
                        512,                    /* The number of bytes to allocate to the
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
                hw_usb_bus_detach();
                usb_cdc_stop();
                cm_sys_clk_set(sysclk_XTAL16M);
                run_usb_task = 0;
                pm_resume_sleep();
        }
}
