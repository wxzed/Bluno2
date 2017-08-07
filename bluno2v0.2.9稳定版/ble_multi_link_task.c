/**
 ****************************************************************************************
 *
 * @file ble_multi_link_task.c
 *
 * @brief Multi-Link Demo task
 *
 * Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include "util/queue.h"
#include "osal.h"
#include "timers.h"
#include "osal.h"
#include "sys_watchdog.h"
#include "ble_att.h"
#include "ble_common.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "ble_gatt.h"
#include "ble_gattc.h"
#include "ble_service.h"
#include "dlg_mls.h"
#include "Devinfoservice.h"
#include "simpleGATTprofile.h"

#include "stateMachine.h"
#include "atParser.h"
#include "hw_uart.h"
#include "hw_gpio.h"
#include "Serialsend.h"
#include "simpleGATTprofile.h"
#include <ble_mgr.h>
#include "oled2864.h"
#include "USB_CDC.h"
#include "hw_otpc.h"
#include "SerialApp.h"
#include "ad_nvms.h"
#include <math.h>


#define DISCOVER_NOTIF  (1 << 1)
#define RECONNECT_NOTIF (1 << 2)
#define BEGIN_TIMER (1<<3)

#define RX_PORT                 HW_GPIO_PORT_3
#define RX_PIN                  HW_GPIO_PIN_2
#define TX_PORT                 HW_GPIO_PORT_3
#define TX_PIN                  HW_GPIO_PIN_1
#define UNO_RESR_PORT           HW_GPIO_PORT_3
#define UNO_RESR_PIN            HW_GPIO_PIN_4
#define PAIR_PORT               HW_GPIO_PORT_3
#define PAIR_PIN                HW_GPIO_PIN_3
#define LINK_PORT               HW_GPIO_PORT_3
#define LINK_PIN                HW_GPIO_PIN_0

bool have_usb = false;
bool open_usb = false;
bool baudrate_change = false;
uint8_t baudrate_change_data = 0;
U32 baudrate_num = 115200;
typedef struct {
        void *next;
        uint16_t start_h;
        uint16_t end_h;
} service_t;

typedef struct {
        void *next;
        uint16_t handle;
        uint16_t val_h;
} characteristic_t;
#if 0
PRIVILEGED_DATA static queue_t services;
PRIVILEGED_DATA static queue_t characteristics;
PRIVILEGED_DATA static uint16_t devname_val_h;
#endif
PRIVILEGED_DATA static OS_TASK ble_multi_link_task_handle;


/*
 * Bluno2 demo advertising data
 */
uint8_t scanRsp_Data[] = {
        0x0C,
        GAP_DATA_TYPE_LOCAL_NAME,
        'B', 'l', 'u', 'n', 'o', '2','v', '1', ' ', ' ', ' ',
        0x05,
        GAP_DATA_TYPE_MANUFACTURER_SPEC,
        'd','f','n','o',
        0x05,
        GAP_DATA_TYPE_SLAVE_CONN_INTV,
        LO_UINT16( 8 ),
        HI_UINT16( 8 ),
        LO_UINT16( 8 ),
        HI_UINT16( 8 ),
        0x02,
        GAP_DATA_TYPE_TX_POWER_LEVEL,
        0
        /*
        0x05, GAP_DATA_TYPE_UUID16_LIST_INC,
        0xB0, 0xDF, // = 0x1803 (DFB0)
        0x12, 0x18, // = 0x1802 (Human Interface Device Service)
        0x02,
        GAP_DATA_TYPE_TX_POWER_LEVEL,
        0
        */
};

uint8_t scanRsp_Data_center[] = {
        0x0C,
        GAP_DATA_TYPE_LOCAL_NAME,
        'B', 'l', 'u', 'n', 'o', '2','v', '1', ' ', ' ', ' ',
        0x07,
        GAP_DATA_TYPE_MANUFACTURER_SPEC,
        'd','f','r','o','o','t',
        0x05,
        GAP_DATA_TYPE_SLAVE_CONN_INTV,
        LO_UINT16( 8 ),
        HI_UINT16( 8 ),
        LO_UINT16( 8 ),
        HI_UINT16( 8 )
        /*
        0x05, GAP_DATA_TYPE_UUID16_LIST_INC,
        0xB0, 0xDF, // = 0x1803 (DFB0)
        0x12, 0x18, // = 0x1802 (Human Interface Device Service)
        0x02,
        GAP_DATA_TYPE_TX_POWER_LEVEL,
        0
        */
};

static const uint8_t adv_Data[] = {

        //0x02,
        //GAP_DATA_TYPE_FLAGS,
        //GAP_DISC_MODE_GEN_DISCOVERABLE,
#if 1
        0x1A,
        GAP_DATA_TYPE_MANUFACTURER_SPEC,
        0x4C,
        0x00,
        0x02,
        0x15,
        /*Device UUID (16 Bytes)*/
        0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48,0xD2, 0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0,
        /*Major Value (2 Bytes)*/
        0x00,0x00,
        /*Minor Value (2 Bytes)*/
        0x00,0x00,
        /*Measured Power*/
        0xC5
#endif
};



typedef struct {
        void            *next;

        bd_address_t    addr;
        uint16_t        conn_idx;
} conn_dev_t;

typedef struct {
        void            *next;

        bd_address_t    addr;
        uint16_t        conn_idx;
} net_inf_t;

PRIVILEGED_DATA queue_t connections;
PRIVILEGED_DATA queue_t net_connections;

PRIVILEGED_DATA static uint16_t master_dev_conn_idx = BLE_CONN_IDX_INVALID;

PRIVILEGED_DATA static bool connecting = false;

/*
 * Debug functions
 */
volatile static int  first_time;
volatile static int  first_there;
static bool scanning_yes_or_no = true;
uint8_t  connect_dev_num;
volatile int  connect_first;
static uint16_t my_interval_min=0;
static uint16_t my_interval_max=0;
static bool phone_static = false;
static uint8_t phone_id=0xff;
static bool connect_full = false;
static bool come_connecd = false;
static bool now_adv_or_scan = false;

OS_TIMER scanning_timer;
OS_TIMER link_timer;
bool Always_bright = false;
static void scanning_timer_cb(void *p){//15s切换广播或搜索。
        if(scanning_yes_or_no ==false){//如果不开始搜索，则说明上一次是在搜索，接下来该广播

                ble_gap_scan_stop();
                ble_gap_adv_stop();
                if(phone_static == true){//如果手机已经连接上了，则不广播，根据设备数有没有连接满来判断是否继续搜索
                        if(connect_full == false){//如果连接数没满，则继续搜索
                                ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(50), defaultBLE_SCAN_WINDOW, false,false);
                                now_adv_or_scan = false;
                        }else if(connect_full == true){
                                OS_TASK_YIELD();
                        }

                }else if(phone_static == false){
                        ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
                        now_adv_or_scan = true;
                }
                scanning_yes_or_no = true;
        }else if(scanning_yes_or_no == true){//如果开始搜索，则说明上一次是广播，则接下来该搜索。
                ble_gap_adv_stop();
                ble_gap_scan_stop();
                if(connect_full == true){//如果连接数满了，则判断手机是否连接来决定是否广播
                        if(phone_static == false){//如果手机没连接
                                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
                                now_adv_or_scan = true;
                        }else if(phone_static == true){
                                OS_TASK_YIELD();
                        }
                }else if(connect_full == false){//连接数没满，则继续搜索
                        ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(50), defaultBLE_SCAN_WINDOW, false,false);
                        now_adv_or_scan = false;
                }
                scanning_yes_or_no = false;
        }
        fflush(stdout);

}

static void link_timer_cb(void *p){/*连接上设备后，连接指示灯的指示方式*/
        if(ble_connect == 1){
                if(Always_bright == true){//说明刚连接上，常亮3s。
                        hw_gpio_configure_pin(LINK_PORT, LINK_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
                        Always_bright = false;
                }else{
                        hw_gpio_configure_pin(LINK_PORT, LINK_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
                        OS_DELAY(100);
                        hw_gpio_configure_pin(LINK_PORT, LINK_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
                }
        }else{
                hw_gpio_configure_pin(LINK_PORT, LINK_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        }
        OS_TASK_YIELD();
        fflush(stdout);
}
void rx_tx_blink_cb(void * p){
        hw_gpio_configure_pin(RX_PORT, RX_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(TX_PORT, TX_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        OS_TASK_YIELD();
        fflush(stdout);
}
void Rx_blink_hight(){
        hw_gpio_configure_pin(RX_PORT, RX_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
}
void Tx_blink_hight(){
        hw_gpio_configure_pin(TX_PORT, TX_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
}
void Uno_rest(){
        hw_gpio_configure_pin(UNO_RESR_PORT, UNO_RESR_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        OS_DELAY(3);
        hw_gpio_configure_pin(UNO_RESR_PORT, UNO_RESR_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
}
static void LED_Low( void );
static void LED_Hight(void);

static void LED_Low(void)
{
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_0, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
}


static void LED_Hight(void)
{
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_0, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
}
static void PAIR_LED_Hight(void){
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
}
static void PAIR_LED_Low(void){
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
}
//static int a=0;
uint8_t b= 5;
//uint8_t buf[20]="012345678901234567\r\n";
//static int c=0;
void wx_tim_cb(void * p)
{
#if 0
       // printf("0123456789\r\n");
        if(c==0){
                hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_0, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_GPIO,false);
                hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_1, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_GPIO,true);
                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_GPIO,true);
                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_GPIO,false);
                c++;
        }else{
                hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_0, HW_GPIO_MODE_OUTPUT,
                                                                       HW_GPIO_FUNC_GPIO,true);
                hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_1, HW_GPIO_MODE_OUTPUT,
                                                                       HW_GPIO_FUNC_GPIO,false);
                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_GPIO,false);
                hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_GPIO,true);
                c=0;
        }
#endif
#if 0
        serial_blewrite(buf,20);
#endif
#if 1
        if(ble_connect == 1){
                /*
                if(a<100){
                        a++;
                      OS_DELAY(10);
                }else
                if(c<500){
                        serial_blewrite(buf,20);
                        c++;
                }else
                */
                if(bledatalen > 0){
                        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_GPIO,false);
                        ble_sendData();
                }else{
                        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,
                                                HW_GPIO_FUNC_GPIO,true);
                        OS_TASK_YIELD();
                }
        }
#endif

        fflush(stdout);

}


bool list_elem_match(const void *elem, const void *ud)
{
        conn_dev_t *conn_dev = (conn_dev_t *) elem;
        uint16_t *conn_idx = (uint16_t *) ud;

        return conn_dev->conn_idx == *conn_idx;
}
bool net_list_elem_match(const void *elem, const void *ud){
        net_inf_t *net_inf = (net_inf_t *) elem;
        uint16_t *conn_idx = (uint16_t *) ud;
        return net_inf->conn_idx == *conn_idx;
}
#if 0
/* return static buffer with formatted UUID */
static const char *format_uuid(const att_uuid_t *uuid)
{
        static char buf[37];

        if (uuid->type == ATT_UUID_16) {
                sprintf(buf, "0x%04x", uuid->uuid16);
        } else {
                int i;
                int idx = 0;

                for (i = ATT_UUID_LENGTH; i > 0; i--) {
                        if (i == 12 || i == 10 || i == 8 || i == 6) {
                                buf[idx++] = '-';
                        }

                        idx += sprintf(&buf[idx], "%02x", uuid->uuid128[i - 1]);
                }
        }

        return buf;
}
/* return static buffer with characteristics properties mask */
static const char *format_properties(uint8_t properties)
{
        static const char props_str[] = "BRXWNISE"; // each letter corresponds to single property
        static char buf[9];
        int i;

        // copy full properties mask
        memcpy(buf, props_str, sizeof(props_str));

        for (i = 0; i < 8; i++) {
                // clear letter from mask if property not present
                if ((properties & (1 << i)) == 0) {
                        buf[i] = '-';
                }
        }

        return buf;
}
#endif
static const char *format_bd_address(const bd_address_t *addr)
{
        static char buf[19];
        int i;

        for (i = 0; i < sizeof(addr->addr); i++) {
                int idx;

                // for printout, address should be reversed
                idx = sizeof(addr->addr) - i - 1;
                sprintf(&buf[i * 3], "%02X:", addr->addr[idx]);
        }

        buf[sizeof(buf) - 2] = '\0';

        return buf;
}
static uint8_t network_mess[4]={0xFD,0x01,0x00,0x00};
static void network_message_func(void *data, void *user_data){
        const conn_dev_t *conn_dev = data;
        int *num = user_data;
        (*num)++;
        network_mess[3]=conn_dev->conn_idx;
        serial_blewrite(network_mess,4);
}
static void update_network_message(void){
        int num = 0;
        queue_foreach(&connections, network_message_func, &num);
}
uint8_t *first_net_message_data;
static void first_net_message_func(void *data, void *user_data){
        const conn_dev_t *conn_dev = data;
        int *num = user_data;
        (*num)++;

        if(conn_dev->conn_idx == first_net_message_data[0]){//给他发送主机的的设备信息

                first_net_message_data[1] = first_net_message_data[2] =0xFF;
                first_net_message_data[3] = 0xfe;
                for(int i=0;i<6;i++){
                        first_net_message_data[i+4] = si.sys_cfg.my_mac[i];
                }
                cuappEnqueue(first_net_message_data,10);
                bledatalen++;
                OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);

        }else{
                first_net_message_data[1] = first_net_message_data[2] =0xFF;
                first_net_message_data[3] = conn_dev->conn_idx;
                for(int i=0;i<6;i++){
                        first_net_message_data[i+4] = conn_dev->addr.addr[i];
                }
                cuappEnqueue(first_net_message_data,10);
                bledatalen++;
                OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
        }
}
static void first_net_message(){//给第一次连接上的设备发送主机里所有的设备信息
        int num = 0;
        queue_foreach(&connections, first_net_message_func, &num);
}
uint16_t * idmessage0;
uint16_t * idmessage1;
uint16_t * idmessage2;
uint16_t * idmessage3;
uint16_t * idmessage4;
uint16_t * idmessage5;
uint16_t * idmessage6;
uint16_t * idmessage7;
uint16_t * idmessage8;
static void save_net_message(uint8_t *buf){//从机用来更新的组网信息
        if(buf[2] == 0xFF){//说明有设备连接，应该增加设备
                net_inf_t *net_inf;
                net_inf = OS_MALLOC(sizeof(net_inf_t));
                for(int i = 0; i<6; i++){
                        net_inf->addr.addr[i]=buf[i+4];
                }

                net_inf->conn_idx = buf[3];
                switch(net_inf->conn_idx){
                case 0:
                        idmessage0 = &net_inf->conn_idx;
                        break;
                case 1:
                        idmessage1 = &net_inf->conn_idx;
                        break;
                case 2:
                        idmessage2 = &net_inf->conn_idx;
                        break;
                case 3:
                        idmessage3 = &net_inf->conn_idx;
                        break;
                case 4:
                        idmessage4 = &net_inf->conn_idx;
                        break;
                case 5:
                        idmessage5 = &net_inf->conn_idx;
                        break;
                case 6:
                        idmessage6 = &net_inf->conn_idx;
                        break;
                case 7:
                        idmessage7 = &net_inf->conn_idx;
                        break;
                default:
                        idmessage8 = &net_inf->conn_idx;
                        break;
                }
                queue_push_front(&net_connections, (void *)net_inf);
        }else if(buf[2] == 0xFE){//说明有设备掉网，应该减去设备信息。
                uint16_t * idmessage;
                switch(buf[3]){
                case 0:
                        idmessage = idmessage0;
                        break;
                case 1:
                        idmessage = idmessage1;
                        break;
                case 2:
                        idmessage = idmessage2;
                        break;
                case 3:
                        idmessage = idmessage3;
                        break;
                case 4:
                        idmessage = idmessage4;
                        break;
                case 5:
                        idmessage = idmessage5;
                        break;
                case 6:
                        idmessage = idmessage6;
                        break;
                case 7:
                        idmessage = idmessage7;
                        break;
                default:
                        idmessage = idmessage8;
                        break;
                }
                net_inf_t *net_inf = queue_remove(&net_connections, net_list_elem_match, idmessage);
                if (net_inf) {
                        OS_FREE(net_inf);
                }
        }
}
void send_network(uint8_t *pbuf,uint8_t len){
        network_mess[2] = pbuf[3];
        update_network_message();
}
uint8_t center_send_mess_uart[100];
bool center_first_send_mess = true;
uint8_t center_offset_len=10;
static void print_connection_func(void *data, void *user_data)
{
        uint8_t headbuf1[5]="{idx:";
        uint8_t headbuf2[6]=",{idx:";
        uint8_t tailbuf[5]=",mac:";
        uint8_t idbuf[1] = "0";
        const conn_dev_t *conn_dev = data;
        int *num = user_data;

        (*num)++;
        if(conn_dev->conn_idx == 0){
                idbuf[0] = '0';
        }else if(conn_dev->conn_idx == 1){
                idbuf[0] = '1';
        }else if(conn_dev->conn_idx == 2){
                idbuf[0] = '2';
        }else if(conn_dev->conn_idx == 3){
                idbuf[0] = '3';
        }else{
                idbuf[0] = 'F';
        }
        uint8 addr_ascii[]={"0x000000000000}"};
        uint8 mac_addr_ascii[6];
        for(int i=0;i<6;i++){
                mac_addr_ascii[i]=conn_dev->addr.addr[i];
        }
        bdAddr2Str(addr_ascii, mac_addr_ascii );
        addr_ascii[14] = '}';
        if(center_first_send_mess == true){
                memcpy(center_send_mess_uart + center_offset_len,headbuf1,5);
                center_offset_len += 5;
        }else{
                memcpy(center_send_mess_uart + center_offset_len,headbuf2,6);
                center_offset_len += 6;
        }
        memcpy(center_send_mess_uart + center_offset_len,idbuf,1);
        center_offset_len += 1;
        memcpy(center_send_mess_uart + center_offset_len,tailbuf,5);
        center_offset_len += 5;
        memcpy(center_send_mess_uart + center_offset_len,addr_ascii,15);
        center_offset_len += 15;
        center_first_send_mess = false;

#if wx_UART_DEBUG
        printf("%2d | %5d | %s\r\n", *num, conn_dev->conn_idx,
                                                        format_bd_address(&conn_dev->addr));
#endif
}
static bool first_net = false;
static uint8_t network_infor_func_buf[100];
static uint8_t network_infor_func_offset_len = 1;
static void network_infor_func(void *data, void *user_data){
        const net_inf_t *net_inf = data;
        int *num = user_data;
        (*num)++;
        uint8_t headbuf1[5]="{idx:";
        uint8_t headbuf2[6]=",{idx:";
        uint8_t tailbuf[5]=",mac:";
        uint8_t idbuf[1] = "0";
        if(net_inf->conn_idx == 0){
                idbuf[0] = '0';
        }else if(net_inf->conn_idx == 1){
                idbuf[0] = '1';
        }else if(net_inf->conn_idx == 2){
                idbuf[0] = '2';
        }else if(net_inf->conn_idx == 3){
                idbuf[0] = '3';
        }else{
                idbuf[0] = 'F';
        }
        uint8 addr_ascii[]={"0x000000000000}"};
        uint8 mac_addr_ascii[6];
        for(int i=0;i<6;i++){
                mac_addr_ascii[i]=net_inf->addr.addr[i];
        }
        bdAddr2Str(addr_ascii, mac_addr_ascii );
        addr_ascii[14] = '}';
        if(first_net == true){
                memcpy(network_infor_func_buf + network_infor_func_offset_len,headbuf1,5);
                network_infor_func_offset_len += 5;
        }else if(first_net == false){
                memcpy(network_infor_func_buf + network_infor_func_offset_len,headbuf2,6);
                network_infor_func_offset_len += 6;
        }
        memcpy(network_infor_func_buf + network_infor_func_offset_len,idbuf,1);
        network_infor_func_offset_len += 1;
        memcpy(network_infor_func_buf + network_infor_func_offset_len,tailbuf,5);
        network_infor_func_offset_len += 5;
        memcpy(network_infor_func_buf + network_infor_func_offset_len,addr_ascii,15);
        network_infor_func_offset_len += 15;
        first_net = false;
        //printf("%2d | %5d\r\n", *num, conn_dev->conn_idx);
}
void network_infor(enum data_src src){
        int num = 0;
        uint8_t buf1[5]="[{},]";
        first_net = true;
        network_infor_func_buf[0]=buf1[0];
        queue_foreach(&net_connections, network_infor_func, &num);
        first_net = false;
        network_infor_func_buf[network_infor_func_offset_len]=buf1[4];
        myUARTWrite(src,0,network_infor_func_buf,network_infor_func_offset_len+1);
        network_infor_func_offset_len = 1;
}
static void print_connections(void)
{

        int num = 0;
        uint8_t buf1[5]="[{},]";
        uint8_t cs = 0;
        center_first_send_mess = true;
        center_send_mess_uart[0] = 0x55;
        center_send_mess_uart[1] = 0xaa;
        center_send_mess_uart[4] = 0x00;
        center_send_mess_uart[5] = 0xFF;
        center_send_mess_uart[6] = 0xFF;
        center_send_mess_uart[7] = 0x00;
        center_send_mess_uart[8] = 0x01;//event  0x01  代表组网信息
        center_send_mess_uart[9]=buf1[0];
        queue_foreach(&connections, print_connection_func, &num);
        center_first_send_mess = false;
        center_send_mess_uart[center_offset_len]=buf1[4];
        center_send_mess_uart[2] = center_offset_len+1-4;//len
        for(int i= 4; i<center_offset_len; i++){
                cs += center_send_mess_uart[i];
        }
        cs = cs&0xff;
        center_send_mess_uart[3] = cs;//cs
        cuappEnqueue(center_send_mess_uart,center_offset_len+1);
        bledatalen++;
        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);

        center_offset_len = 10;
}
uint8_t *data_buf;
uint8_t *data_buf1;
uint16_t data_buf_len = 20;
uint8_t *final_send_net_messag;
uint16_t final_send_net_messag_len = 20;
static uint8_t * data_to_data(uint8_t *buf,uint8_t *buf1,uint16 len){
        for(int i = 0; i<len;i++){
                buf[i]=buf1[i];
        }
        return buf;
}
/*
static void send_net_message_func(void *data, void *user_data){//发送组网信息
        const conn_dev_t *conn_dev = data;
        int *num = user_data;
        (*num)++;
        serial_blewrite(send_net_messag,9);
}
static void send_net_message(uint8_t *buf, uint16_t len){
        int num = 0;
        queue_foreach(&connections, send_net_message_func, &num);
}
*/
static void broadcast_net_message_func(void *data, void *user_data){//
        const conn_dev_t *conn_dev = data;
        int *num = user_data;
        (*num)++;
        final_send_net_messag[0] =conn_dev->conn_idx;
        cuappEnqueue(final_send_net_messag,10);
        bledatalen++;
        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
        //serial_blewrite(final_send_net_messag,data_buf_len);
}
void broadcast_net_message(uint8_t *buf, uint16_t len){//主机会调用这个来给已经连接的设备发送组网信息。
        int num = 0;
        data_to_data(final_send_net_messag,buf,len);
        final_send_net_messag_len = len;
        queue_foreach(&connections, broadcast_net_message_func, &num);
}

static void broadcast_data_func(void *data, void *user_data){
        const conn_dev_t *conn_dev = data;
        int *num = user_data;
        (*num)++;
        data_buf[0] =conn_dev->conn_idx;
        //serial_blewrite(data_buf,data_buf_len);
        cuappEnqueue(data_buf,data_buf_len);
        bledatalen++;
        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
}
void broadcast_data(uint8_t *buf, uint16_t len){
        int num = 0;
        data_buf = data_to_data(data_buf1,buf,len);
        data_buf_len = len;
        queue_foreach(&connections, broadcast_data_func, &num);
}
uint8_t send_mess_uart[300];
uint8_t new_mess_buf[300];
uint16_t new_mess_buf_len=0;
bool first_send_mess = true;
uint8_t offset_len=15;
bool master = false;
bool myself = false;
static void send_net_mess_for_uart_func(void *data, void *user_data){
        const net_inf_t *net_inf = data;
        int *num = user_data;
        (*num)++;
        uint8_t headbuf1[5]="{0i0:";
        uint8_t headbuf2[6]=",{0i0:";
        uint8_t tailbuf[5]=",0m0:";
        uint8_t stailbuf[5]=",0s0:";
        uint8_t stailbufvalue[2]="0}";
        uint8_t mhead[1]="0";
        mhead[0]='"';
        uint8_t idbuf[3] = "000";
        headbuf1[1] = headbuf1[3] ='"';
        headbuf2[2] = headbuf2[4] = '"';
        tailbuf[1] = tailbuf[3] = '"';
        stailbuf[1] = stailbuf[3] = '"';
        if(net_inf->conn_idx < 10){
                idbuf[0]=(uint8_t)(net_inf->conn_idx)+48;
        }else{
                master = true;
                idbuf[0]='2';
                idbuf[1]='5';
                idbuf[2]='4';
        }
        uint8 addr_ascii[]={"0x000000000000"};
        uint8 my_addr_ascii[]={"0x000000000000"};
        uint8 mac_addr_ascii[6];
        uint8 my_mac_addr_ascii[6];
        for(int i=0;i<6;i++){
                mac_addr_ascii[i]=net_inf->addr.addr[i];
                my_mac_addr_ascii[i] = si.sys_cfg.my_mac[i];
        }
        bdAddr2Str(addr_ascii, mac_addr_ascii );
        bdAddr2Str(my_addr_ascii, my_mac_addr_ascii );
        if(memcmp(addr_ascii,my_addr_ascii,14) == 0){
                myself = true;
                stailbufvalue[0]='1';
        }else{
                myself = false;
                stailbufvalue[0]='0';
        }
        /*{"i": || ,{"i":*/
        if(first_send_mess == true){
                memcpy(new_mess_buf,headbuf1,5);
                new_mess_buf_len += 5;
        }else{
                memcpy(new_mess_buf+new_mess_buf_len,headbuf2,6);
                new_mess_buf_len += 6;
        }
        /*0 || 1 || .....|| 254*/
        if(master == true){
                memcpy(new_mess_buf+new_mess_buf_len,idbuf,3);
                new_mess_buf_len += 3;
                master = false;
        }else{
                memcpy(new_mess_buf+new_mess_buf_len,idbuf,1);
                new_mess_buf_len += 1;
        }
        /*,"m":*/
        memcpy(new_mess_buf+new_mess_buf_len,tailbuf,5);
        new_mess_buf_len += 5;
        /*"00000000"*/
        memcpy(new_mess_buf+new_mess_buf_len,mhead,1);
        new_mess_buf_len += 1;
        memcpy(new_mess_buf+new_mess_buf_len,addr_ascii+10,4);//MAC地址后4位
        new_mess_buf_len += 4;
        memcpy(new_mess_buf+new_mess_buf_len,mhead,1);
        new_mess_buf_len += 1;

        if(myself == true){
                /*"s":*/
                memcpy(new_mess_buf+new_mess_buf_len,stailbuf,5);
                new_mess_buf_len += 5;
                /*0}||1}*/
                memcpy(new_mess_buf+new_mess_buf_len,stailbufvalue,2);
                new_mess_buf_len += 2;
        }else{
                memcpy(new_mess_buf+new_mess_buf_len,stailbufvalue+1,1);
                new_mess_buf_len+=1;
        }
        first_send_mess = false;
#if 0
        uint8_t headbuf1[5]="{0i0:";
        uint8_t headbuf2[6]=",{0i0:";
        uint8_t tailbuf[5]=",0m0:";
        uint8_t stailbuf[5]=",0s0:";
        uint8_t stailbufvalue[2]="0}";
        uint8_t mhead[1]="0";
        mhead[0]='"';
        headbuf1[1] = headbuf1[3] ='"';
        headbuf2[2] = headbuf2[4] = '"';
        tailbuf[1] = tailbuf[3] = '"';
        stailbuf[1] = stailbuf[3] = '"';
        const net_inf_t *net_inf = data;
        int *num = user_data;
        (*num)++;
        uint8_t idbuf[3] = "000";
        if(net_inf->conn_idx < 10){
                idbuf[0]=(uint8_t)(net_inf->conn_idx)+48;
        }else{
                master = true;
                idbuf[0]='2';
                idbuf[1]='5';
                idbuf[2]='4';
        }
        /*
        if(net_inf->conn_idx == 0){
                idbuf[0]='0';
        }else if(net_inf->conn_idx == 1){
                idbuf[0]='1';
        }else if(net_inf->conn_idx == 2){
                idbuf[0]='2';
        }else if(net_inf->conn_idx == 3){
                idbuf[0]='3';
        }else{

        }
        */
        uint8 addr_ascii[]={"0x000000000000"};
        uint8 my_addr_ascii[]={"0x000000000000"};
        uint8 mac_addr_ascii[6];
        uint8 my_mac_addr_ascii[6];
        for(int i=0;i<6;i++){
                mac_addr_ascii[i]=net_inf->addr.addr[i];
                my_mac_addr_ascii[i] = si.sys_cfg.my_mac[i];
        }
        bdAddr2Str(addr_ascii, mac_addr_ascii );
        bdAddr2Str(my_addr_ascii, my_mac_addr_ascii );
        if(memcmp(addr_ascii,my_addr_ascii,14) == 0){
                stailbufvalue[0]='1';
        }else{
                stailbufvalue[0]='0';
        }
        if(first_send_mess == true){
                memcpy(send_mess_uart+offset_len,headbuf1,5);
                offset_len += 5;
        }else{
                memcpy(send_mess_uart+offset_len,headbuf2,6);
                offset_len += 6;
        }
        if(master == true){
                memcpy(send_mess_uart+offset_len,idbuf,3);
                offset_len += 3;
                master = false;
        }else{
                memcpy(send_mess_uart+offset_len,idbuf,1);
                offset_len += 1;
        }
        memcpy(send_mess_uart+offset_len,tailbuf,5);
        offset_len += 5;
        memcpy(send_mess_uart+offset_len,mhead,1);
        offset_len += 1;
        memcpy(send_mess_uart+offset_len,addr_ascii+2,12);
        offset_len += 12;
        memcpy(send_mess_uart+offset_len,mhead,1);
        offset_len += 1;
        memcpy(send_mess_uart+offset_len,stailbuf,5);
        offset_len += 5;
        memcpy(send_mess_uart+offset_len,stailbufvalue,2);
        offset_len += 2;
        first_send_mess = false;
#endif
}
static void send_net_mess_for_uart(){
        int num = 0;
        uint8_t Lim_len = 80;//分包个数
        uint8_t Lim_buf[Lim_len+9];
        uint8_t frequency = 0;//需要分几次
        uint8_t remainder = 0;//最后一个包还剩多少个
        uint16_t frequency_len = 0;
        first_send_mess = true;
        new_mess_buf_len = 0;
        Lim_buf[0] = 0x55;
        Lim_buf[1] = 0xaa;
        Lim_buf[2] = 0x00;//len
        Lim_buf[3] = 0x00;//cs
        Lim_buf[4] = 0x00;
        Lim_buf[5] = 0xFF;//FF
        Lim_buf[6] = 0xFF;//FF
        Lim_buf[7] = 0x00;//ttl
        Lim_buf[8] = 0x01;//event
        queue_foreach(&net_connections, send_net_mess_for_uart_func, &num);
        /*{"r":[{},{},{}]}*/
        uint8_t bufhead[8]="{0r0:[]}";
        uint8_t cs = 0;
        bufhead[1] = bufhead[3] = '"';
        memcpy(send_mess_uart,bufhead,6);
        memcpy(send_mess_uart+6,new_mess_buf,new_mess_buf_len);
        memcpy(send_mess_uart+6+new_mess_buf_len,bufhead+6,2);
        frequency = (uint8_t)(new_mess_buf_len+8)/Lim_len;
        remainder = (uint8_t)(new_mess_buf_len+8)%Lim_len;
        if(Lim_len<(new_mess_buf_len+8)){
                for(int i = 0; i<frequency; i++){
                        cs = 0;
                        Lim_buf[2] = Lim_len+5;
                        if(remainder > 0){
                                Lim_buf[6] = i;
                        }else{
                                if(i == frequency-1){
                                        Lim_buf[6] = 0xff;
                                }else{
                                        Lim_buf[6] = i;
                                }
                        }
                        memcpy(Lim_buf+9,send_mess_uart+frequency_len,Lim_len);
                        frequency_len += Lim_len;
                        for(int j = 4; j < Lim_len; j++){
                                cs += Lim_buf[j];
                        }
                        cs = cs&0xff;
                        Lim_buf[3] = cs;
                        if(open_usb == true){
                                myserialusb(Lim_buf,Lim_len+9);
                        }
                        hw_uart_send(HW_UART2, Lim_buf,Lim_len+9, NULL, NULL);
                }
                if(remainder > 0){
                        cs = 0;
                        Lim_buf[2] = remainder+5;
                        Lim_buf[6] = 0xff;
                        memcpy(Lim_buf+9,(send_mess_uart+((new_mess_buf_len+8)-remainder)),remainder);
                        for(int j = 4; j < remainder;j++){
                                cs += Lim_buf[j];
                        }
                        cs = cs&0xff;
                        Lim_buf[3] = cs;
                        if(open_usb == true){
                                myserialusb(Lim_buf,remainder+9);
                        }
                        hw_uart_send(HW_UART2, Lim_buf,remainder+9, NULL, NULL);
                }
                frequency_len = 0;
        }else{
                Lim_buf[2] = new_mess_buf_len+8+5;
                Lim_buf[6] = 0xff;
                memcpy(Lim_buf+9,send_mess_uart,new_mess_buf_len+8+9);
                cs = 0;
                for(int j = 4; j < new_mess_buf_len+8+9;j++){
                        cs += Lim_buf[j];
                }
                cs = cs&0xff;
                Lim_buf[3] = cs;
                if(open_usb == true){
                        myserialusb(Lim_buf,new_mess_buf_len+8+9);
                }
                hw_uart_send(HW_UART2, Lim_buf,new_mess_buf_len+8+9, NULL, NULL);
        }

        //hw_uart_send(HW_UART2, send_mess_uart, new_mess_buf_len+8, NULL, NULL);
        new_mess_buf_len = 0;
#if 0
        int num = 0;
        uint8_t Lim_len = 80;
        uint8_t bufhead[6]="{0r0:}";
        uint8_t buf1[5]="[{},]";
        //uint8_t buf1[5]="{"r":[{},]}";
        uint8_t cs = 0;
        uint8_t Subcontracting[100];
        uint8_t headerbuf[15];
        first_send_mess = true;
        send_mess_uart[0] = 0x55;
        send_mess_uart[1] = 0xaa;
        send_mess_uart[4] = 0x00;
        send_mess_uart[5] = 0xFF;
        send_mess_uart[6] = 0xFF;
        send_mess_uart[7] = 0x00;
        send_mess_uart[8] = 0x01;
        send_mess_uart[9]=bufhead[0];
        send_mess_uart[10]='"';
        send_mess_uart[11]=bufhead[2];
        send_mess_uart[12]='"';
        send_mess_uart[13]=bufhead[4];
        send_mess_uart[14]=buf1[0];
        memcpy(headerbuf,send_mess_uart,14);
        queue_foreach(&net_connections, send_net_mess_for_uart_func, &num);
        first_send_mess = false;
        send_mess_uart[offset_len]=buf1[4];
        offset_len += 1;
        send_mess_uart[offset_len]=bufhead[5];
        send_mess_uart[2] = offset_len+1-4;//len
        for(int i= 4; i<offset_len; i++){
                cs += send_mess_uart[i];
        }
        cs = cs&0xff;
        send_mess_uart[3] = cs;//cs
        //cuappEnqueue(send_mess_uart,offset_len+1);
        //bledatalen++;
        //OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
        if(offset_len > (Lim_len-1)){

                uint8_t frequency = 0;
                uint8_t remainder = 0;
                uint8_t len = 0;
                frequency = (offset_len+1)/Lim_len;
                remainder = (offset_len+1)%Lim_len;
                for(int j = 0; j<frequency+1; j++){
                        if(j == 0){
                                cs = 0;
                                memcpy(Subcontracting,send_mess_uart,Lim_len);
                                Subcontracting[2] = Lim_len;
                                Subcontracting[6] = j;
                                for(int i= 4; i<(Lim_len); i++){
                                        cs += Subcontracting[i];
                                }
                                cs = cs&0xff;
                                Subcontracting[3] = cs;
                                len += Lim_len;
                                if(open_usb == true){
                                        myserialusb(Subcontracting,Lim_len);
                                }

                                hw_uart_send(HW_UART2, Subcontracting, Lim_len, NULL, NULL);

                        }else if(j < frequency){
                                cs = 0;
                                memcpy(Subcontracting,headerbuf,14);
                                memcpy(Subcontracting+9,send_mess_uart+len,Lim_len);
                                Subcontracting[2] = Lim_len+14;
                                Subcontracting[6] = j;
                                for(int i= 4; i<(Lim_len+14); i++){
                                        cs += Subcontracting[i];
                                }
                                cs = cs&0xff;
                                Subcontracting[3] = cs;
                                len += Lim_len;
                                if(open_usb == true){
                                        myserialusb(Subcontracting,Lim_len+14);
                                }
                                hw_uart_send(HW_UART2, Subcontracting, Lim_len+14, NULL, NULL);
                        }else{//最后一个包
                                cs = 0;
                                memcpy(Subcontracting,headerbuf,14);
                                memcpy(Subcontracting+9,send_mess_uart+len,remainder);
                                Subcontracting[2] = remainder+14;
                                Subcontracting[6] = 0xFF;
                                for(int i= 4; i<(remainder+14); i++){
                                        cs += Subcontracting[i];
                                }
                                cs = cs&0xff;
                                Subcontracting[3] = cs;
                                len = 0;
                                if(open_usb == true){
                                        myserialusb(Subcontracting,remainder+14);
                                }
                                hw_uart_send(HW_UART2, Subcontracting, remainder+14, NULL, NULL);

                        }
                }
        }else{
                if(open_usb == true){
                        myserialusb(send_mess_uart,offset_len+1);
                }
                hw_uart_send(HW_UART2, send_mess_uart, offset_len+1, NULL, NULL);
                offset_len = 15;
        }

#endif
}

#if 0
static void bd_addr_write_cb(ble_service_t *svc, uint16_t conn_idx, const bd_address_t *addr)
{
        ble_error_t status;

        status = ble_gap_connect(addr, &cp);

        if (status != BLE_STATUS_OK) {
                printf("%s: failed. Status=%d\r\n", __func__, status);
        }

        if (status == BLE_ERROR_BUSY) {
                /*
                 * If the device is not answered after connection, when you are trying to connect to
                 * another device, cancel the last connection to connect to with new one.
                 */
                ble_gap_connect_cancel();
                printf("%s: The last connection was canceled. Connecting to new device...\r\n",
                                                                                         __func__);

                status = ble_gap_connect(addr, &cp);
                printf("%s: Status=%d\r\n", __func__, status);
        }

        connecting = (status == BLE_STATUS_OK);
}
#endif
/*
static void devinfo_write_cb(uint16_t conn_idx, uint8_t level)
{
       printf("devinfo_write_cb");
}
*/

uint8_t now_addr=0x00;
static void handle_evt_gap_connected(ble_evt_gap_connected_t *evt)
{
        Always_bright = true;
        LED_Hight();
        //fill(0x00);//oled清屏
        ble_connect = 1;
        conn_dev_t *conn_dev;

        my_interval_min=evt->conn_params.interval_min;
        my_interval_max=evt->conn_params.interval_max;
#if wx_UART_DEBUG
        printf("%s: conn_idx=%d peer_address=%s\r\n", __func__, evt->conn_idx,
                                                        format_bd_address(&evt->peer_address));
#endif

        conn_dev = OS_MALLOC(sizeof(conn_dev_t));
        conn_dev->addr.addr_type = evt->peer_address.addr_type;
        memcpy(conn_dev->addr.addr, evt->peer_address.addr, sizeof(conn_dev->addr.addr));
        conn_dev->conn_idx = evt->conn_idx;
        myconn_idx = conn_dev->conn_idx;
        queue_push_front(&connections, (void *)conn_dev);

        if (master_dev_conn_idx == BLE_CONN_IDX_INVALID) {
                master_dev_conn_idx = evt->conn_idx;
#if wx_UART_DEBUG
                printf("master_dev_conn_idx\r\n");
#endif
        }
        connecting = false;

        //print_connections();
        if(phone_static == true){
                uint8_t new_network_mess[4]={0xFD,0x01,0x00,0x00};//new_network_mess[0]代表组网信息的头-----new_network_mess[1]代表连接着的-----new_network_mess[2]代表发送给谁，-----new_network_mess[3]代表断开连接的id号
                new_network_mess[2] = phone_id;
                new_network_mess[3] = evt->conn_idx;
                serial_blewrite(new_network_mess,4);
        }
        if(now_addr != evt->peer_address.addr[5]){//证明是手机连接
                phone_id = evt->conn_idx;
                phone_static = true;//证明手机连接上了。
        }else{
                connect_dev_num++;
        }
        /*
        if(evt->peer_address.addr[5] != 0x80){//证明是手机连接
                phone_id = evt->conn_idx;
                phone_static = true;//证明手机连接上了。
        }else{
                connect_dev_num++;
        }
        */
        if(si.sys_cfg.curr_role == ROLE_CENTRAL){
                PAIR_LED_Low();
                first_serial_blewrite(myconn_idx);                                                  /*给连接上的设备发送组网信息*/
                if(si.sys_cfg.networkcmode == 'o'){                                                 /*当组网方式关闭时即为一对一连接时，连接上一个设备则停止搜索。*/
                        connect_full = true;
                        ble_gap_scan_stop();
                }else if(si.sys_cfg.networkcmode == 's'){
                        OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(2000),10/ OS_PERIOD_MS);

                        if(connect_dev_num == si.sys_cfg.connectmax){
                                connect_full = true;
                        }else if(connect_dev_num < si.sys_cfg.connectmax){
                                //OS_DELAY(10);
                                //ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(50), defaultBLE_SCAN_WINDOW, false,false);
                                connect_full = false;
                        }

#if 0
                        if(connect_dev_num == 2){                                                   /*连接上的设备数满了之后停止搜索。*/
                                printf("The connection device is full and the search is stopped\r\n");
                                ble_gap_scan_stop();
                                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);//主机可以广播，允许被连接
                        }else if(connect_dev_num<2){                                                                      /*连接数没满时，继续搜索*/
                                printf("Connect the device is not full, continue to search\r\n");
                                ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(1000),
                                                                                        defaultBLE_SCAN_WINDOW, false,false);
                        }
#endif
                }
                //ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);//主机可以广播，允许被连接
#if OLED_PRINT
                oled_print(0, 0, "I'm central!");
                oled_print(0, 2, "connected dev:");
                oled_print_int(14, 2, connect_dev_num);
                oled_print(0, 4, "from:");
                oled_print(7, 4, "to:");
#endif

        }else{
#if OLED_PRINT
                oled_print(0, 0, "Att:per");
                oled_print(8, 0,"my_id:");
                oled_print(0, 2, "from:");
                oled_print(7, 2,"Frame:");
                oled_print(7, 4,"loss:");
                oled_print(0, 4,"sum:");
#endif
                ble_gap_adv_stop();
        }
        if(si.sys_cfg.networkcmode == 's'){
                if(si.sys_cfg.curr_role == ROLE_CENTRAL){

                        net_inf_t *net_inf;
                        net_inf = OS_MALLOC(sizeof(net_inf_t));
                        net_inf->addr.addr_type = evt->peer_address.addr_type;
                        memcpy(net_inf->addr.addr, evt->peer_address.addr, sizeof(net_inf->addr.addr));
                        net_inf->conn_idx = evt->conn_idx;
                        first_net_message_data[0]=evt->conn_idx;
                        first_net_message();
                        queue_push_front(&net_connections, (void *)net_inf);
                        uint8_t *send_net_messag;
                        send_net_messag[0] = 0xFA;
                        send_net_messag[1] = send_net_messag[2] = 0xFF;
                        send_net_messag[3] = evt->conn_idx;
                        for(int i=0;i<6;i++){
                                send_net_messag[i+4] = evt->peer_address.addr[i];
                        }


                        cuappEnqueue(send_net_messag,10);
                        bledatalen++;
                        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);

                        /*串口打印网络信息*/
                        send_net_mess_for_uart();
                }else{//从机先清掉信息。
                        queue_remove_all(&net_connections,OS_FREE_FUNC);
                }
        }
        come_connecd = true;
        OS_TASK_NOTIFY(ble_multi_link_task_handle, BEGIN_TIMER, eSetBits);
        OS_TASK_NOTIFY(ble_multi_link_task_handle, DISCOVER_NOTIF, eSetBits);
}

static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt)
{
        bool phone_now = false;//手机是不是刚刚掉的。
#if OLED_PRINT
        fill(0x00);//oled清屏
#endif
        si.rssi=0;
        conn_dev_t *conn_dev = queue_remove(&connections, list_elem_match, &evt->conn_idx);
        if(si.sys_cfg.networkcmode == 's'){
                if(si.sys_cfg.curr_role == ROLE_PERIPHERAL){
                        //清除所有的连接信息。
                        queue_remove_all(&net_connections,OS_FREE_FUNC);
                        /*掉网之后，将自己设备的信息上传到队列里去。*/
                        net_inf_t *net_inf;
                        net_inf = OS_MALLOC(sizeof(net_inf_t));
                        memcpy(net_inf->addr.addr, si.sys_cfg.my_mac, sizeof(net_inf->addr.addr));
                        net_inf->conn_idx = 0x00;
                        queue_push_front(&net_connections, (void *)net_inf);
                        /*用串口打印出该信息*/
                        send_net_mess_for_uart();



                }else if(si.sys_cfg.curr_role == ROLE_CENTRAL){
                        net_inf_t *net_inf = queue_remove(&net_connections, net_list_elem_match, &evt->conn_idx);
                        if (net_inf) {
                                OS_FREE(net_inf);
                        }
                        uint8_t *send_dis_net_messag;
                        send_dis_net_messag[0] = 0xFA;
                        send_dis_net_messag[1] =0xFF;
                        send_dis_net_messag[2] = 0xFE;//告诉从机掉网设备的conn_idx;
                        send_dis_net_messag[3] = evt->conn_idx;
                        for(int i=0;i<6;i++){
                                send_dis_net_messag[i+4] = evt->address.addr[i];
                        }

                        cuappEnqueue(send_dis_net_messag,10);
                        bledatalen++;
                        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);

                        //向从机发送掉网的信息。
                        send_net_mess_for_uart();
                }
        }
        if(evt->conn_idx == phone_id){
                phone_now = true;
                phone_static = false;//手机断开
        }else{
                if(connect_dev_num > 0){
                        connect_dev_num--;
                }else{
                        connect_dev_num = 0;
                }
        }

        if(connect_dev_num==0 && phone_static == false){
                ble_connect = 0;
                LED_Low();
        }
        if(phone_static == true){//如果手机是连接着的，则向手机更新组网信息。
#if wx_UART_DEBUG
                printf("发送组网信息\r\n");
#endif
                uint8_t new_network_mess[4]={0xFD,0x00,0x00,0x00};//new_network_mess[0]代表组网信息的头-----new_network_mess[1]代表断开连接-----new_network_mess[2]代表发送给谁，-----new_network_mess[3]代表断开连接的id号
                new_network_mess[2] = phone_id;
                new_network_mess[3] = evt->conn_idx;
                cuappEnqueue(new_network_mess,4);
                bledatalen++;
                OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
        }
        if (conn_dev) {
                OS_FREE(conn_dev);
        }
        //print_connections();

        if (master_dev_conn_idx == evt->conn_idx) {
                master_dev_conn_idx = BLE_CONN_IDX_INVALID;

                //ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
#if wx_UART_DEBUG
                printf("master_dev_conn_idx\r\n");
#endif
        }
        if(si.sys_cfg.curr_role == ROLE_CENTRAL){
                if(si.sys_cfg.networkcmode == 'o'){                                                                             /*关闭组网*/
                        if(si.sys_cfg.cmode == 'u'){                                                                            /*如果为指定连接*/
                                OS_TASK_NOTIFY(ble_multi_link_task_handle, RECONNECT_NOTIF, eSetBits);                          /*跳转到去连接蓝牙绑定设备即重连设备*/
                        }else{                                                                                                  /*如果为任意连接*/
                                ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(100),
                                                                             defaultBLE_SCAN_WINDOW, false,false);   /*重新开始搜索*/
                                //oled_print(0, 0, "Again scanning!");

                        }
                }else if(si.sys_cfg.networkcmode == 's'){
                        //ble_gap_adv_stop();
                        //ble_gap_scan_stop();
                        //OS_DELAY(10);
                        if(phone_now == true && phone_static == false){//如果是手机断开。
                                //ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
                                //printf("手机断开\r\n");
                        }else{
                                //ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(1000),
                               //                                            defaultBLE_SCAN_WINDOW, false,false);           /*重新开始搜索*/
#if wx_UART_DEBUG
                                printf("连接设备没满\r\n");
#endif
                                connect_full = false;
                                //oled_print(0, 0, "Again scanning!");
                        }



                }
        }else{
                connect_first=1;
                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
                //printf("Advertising is on again\r\n");
                //oled_print(0, 0, "I'm peripheral!");
                //oled_print(0, 2,"Start");
                //oled_print(0,4,"advertising!");
        }
        come_connecd = false;
        OS_TASK_NOTIFY(ble_multi_link_task_handle, BEGIN_TIMER, eSetBits);

}

static void handle_evt_gap_adv_completed(ble_evt_gap_adv_completed_t *evt)
{
#if wx_UART_DEBUG
        printf("%s: adv_type=%d status=%d\r\n", __func__, evt->adv_type, evt->status);
#endif
        ble_gap_adv_stop();
        //OS_TASK_NOTIFY(ble_multi_link_task_handle, RECONNECT_NOTIF, eSetBits);

        if (connecting && (evt->status == BLE_ERROR_NOT_ALLOWED)) {
                connecting = false;

                ble_gap_connect_cancel();
#if wx_UART_DEBUG
                printf("%s: cancel the connect\r\n", __func__);
#endif

                if (master_dev_conn_idx != BLE_CONN_IDX_INVALID) {
                        return;
                }

                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
#if wx_UART_DEBUG
                printf("Advertising is on again after error\r\n");
#endif
        }

}

static void handle_evt_gap_pair_req(ble_evt_gap_pair_req_t *evt)
{
        printf("%s: conn_idx=%d, bond=%d\r\n", __func__, evt->conn_idx, evt->bond);

        ble_gap_pair_reply(evt->conn_idx, true, evt->bond);
}

static void handle_evt_gap_security_request(ble_evt_gap_security_request_t *evt)
{
        ble_error_t status;

        status = ble_gap_pair(evt->conn_idx, evt->bond);

        if (status != BLE_STATUS_OK) {
                printf("%s: failed. Status=%d\r\n", __func__, status);
        }
}

#if 0
static void format_value(uint16_t length, const uint8_t *value)
{
        static char buf1[49]; // buffer for hexdump (16 * 3 chars + \0)
        static char buf2[17]; // buffer for ASCII (16 chars + \0)

        while (length) {
                int i;

                memset(buf1, 0, sizeof(buf1));
                memset(buf2, 0, sizeof(buf2));

                for (i = 0; i < 16 && length > 0; i++, length--, value++) {
                        sprintf(&buf1[i * 3], "%02x ", (int) *value);

                        // any character outside standard ASCII is presented as '.'
                        if (*value < 32 || *value > 127) {
                                buf2[i] = '.';
                        } else {
                                buf2[i] = *value;
                        }
                }

                printf("\t%-49s %-17s\r\n", buf1, buf2);
                //printf("\t%-49s %-17s\r\n", buf1, buf2);

        }
}
#endif
static void data_analysis(uint8_t * buf, uint16_t length){
        uint8_t from[]={"from:"};
        uint8_t from_id[]={"0"};
        uint8_t data[]={";data:"};
        uint8_t end[]={"\r\n"};
        if(buf[1] == 0x00){
                from_id[0] = '0';
        }else if(buf[1] == 0x01){
                from_id[0] = '1';
        }else if(buf[1] == 0x02){
                from_id[0] = '2';
        }else if(buf[1] == 0x03){
                from_id[0] = '3';
        }else if(buf[1] == 0x04){
                from_id[0] = '4';
        }else{
                from_id[0] = 'F';
        }
        if(open_usb == true){
                myserialusb(from, 5);
                myserialusb(from_id, 1);
                myserialusb(data, 6);
                myserialusb(buf+4, length-4);
                myserialusb(end, 2);
        }

}

uint8_t bbb[5];

static int my_pow(uint8_t a,uint8_t b){
        int sum = 1;
        if(b == 0){
                sum = 1;
        }else{
                for(int i=0;i<b;i++){
                        sum *= a;
                }
        }

        return sum;
}

static void ble_baudrate_num(U32 num){
        U32 temp = num;
        if(10000<num && num<100000){
                for(int i=0;i<5;i++){
                        bbb[i] = temp/(10000/(my_pow(10,i)));
                        temp = temp - (bbb[i]*(10000/(my_pow(10,i))));
                }
        }
}
static U32 com_ble_baudrate_num(uint8_t* buf,uint8_t len){
        U32 my_baudrate = 0;
        for(int i = 0; i<len;i++){
                my_baudrate +=buf[i]* (my_pow(10,len-i-1));
        }
        return my_baudrate;
}

volatile int sum;
/*蓝牙notif接收到数据的处理函数*/
bool UNO_DOWN = false;
uint8_t final_package[30];
uint8_t final_package_len = 30;
static void Merge_the_package_test(uint8_t *buf,uint8_t len,uint8_t *final_buf){
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
        final_package_len = len+4;
}
static void handle_evt_gattc_notification(ble_evt_gattc_notification_t *evt)
{
#if 1
        Merge_the_package_test(evt->value,evt->length,final_package);
        if(si.sys_cfg.curr_role == ROLE_CENTRAL){//主机收到notif数据时
                myconnid = 0xFE;
                if(si.sys_cfg.networkcmode == 'o'){//当模式不是组网状态时
                        if(open_usb == false){
                                if(evt->value[0] == 0x55 && evt->value[1] == 0xaa){
                                        if(evt->value[2] == 0x0b && evt->value[3] == 0x0d){//更新UART波特率
                                                Serial_Reinit(com_ble_baudrate_num(evt->value+5,evt->value[4]));
                                        }else{
                                                Tx_blink_hight();
                                                hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
                                        }

                                }else{
                                        Tx_blink_hight();
                                        hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
                                }
                        }else if(open_usb == true){
                                if(si.sys_cfg.usb_debug == '1'){//如果USB开启了监控，则将数据发送给串口和USB
                                        Tx_blink_hight();
                                        hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
                                        myserialusb(evt->value, evt->length);//给UNO下载程序
                                }else{
                                        myserialusb(evt->value, evt->length);//给UNO下载程序
                                }

                        }
                }else if(si.sys_cfg.networkcmode == 's'){//当主机模式为星型网络时
#if OLED_PRINT
                        oled_print_int(5, 4, evt->value[1]);
                        oled_print_int(11, 4, evt->value[0]);
#endif
#if 1
#if usblog
                        if(open_usb == true){
                                myserialusb("handle_evt_gattc_notification:\r\n",32);
                                myserialusb("receive->",9);
                                if(evt->value[1]==0x01){
                                        myserialusb("1",1);
                                }else if(evt->value[1]==0x00){
                                        myserialusb("0",1);
                                }
                                myserialusb("\r\nsend->",8);
                                if(evt->value[0]==0x01){
                                        myserialusb("1",1);
                                }else if(evt->value[0]==0x00){
                                        myserialusb("0",1);
                                }
                                myserialusb("\r\n",2);
                        }

#endif
                        if(evt->value[0] == 0xFE){//主机用来判断蓝牙收到的数据是不是发送给自己的，
                                Tx_blink_hight();
                                if(open_usb == false){//如果数据是给自己的，且USB没有打开的话。将数据打包后发送到串口。-------------------->不解析数据
                                        hw_uart_send(HW_UART2, final_package, final_package_len, NULL, NULL);
                                }else if(open_usb == true){//如果USB是打开的。
                                        if(si.sys_cfg.usb_debug == '1'){//如果是开启监控的，则将有效数据发送到串口和USB
                                                hw_uart_send(HW_UART2, final_package, final_package_len, NULL, NULL);
                                                //data_analysis(evt->value, evt->length);
                                                myserialusb(final_package,final_package_len);
                                        }else if(si.sys_cfg.usb_debug == '0'){//如果关闭了监控，则将有效数据打包成网络数据格式发送到USB
                                                myserialusb(final_package,final_package_len);
                                        }
                                }

                        }else{//如果不是发送给自己的，则将数据写入队列里，等待定时器发送到指定设备。
#if usblog
                                if(open_usb == true){
                                        myserialusb("cuappEnqueue\r\n",14);
                                }

#endif
                                cuappEnqueue(evt->value,evt->length);//根据收到的数据发送给指定的连接的从机
                                bledatalen++;
                                OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
                        }
#endif

                }
        }else{
                if(si.sys_cfg.networkcmode == 's'){//从机在星型网络下数据的流向。
#if 1
                        if(connect_first){//从机在被主机连接上且第一次收到notif时，将会收到它在主机中的id号
                                myconnid = evt->value[0];
#if OLED_PRINT
                                oled_print_int(14,0,evt->value[0]);//显示我的id
#endif
                                connect_first=0;
                        }else{
                                if(evt->value[1] == 0xFF && (evt->value[2] == 0xFF || evt->value[2] == 0xFE)){//说明我收到的是组网信息。

                                        save_net_message(evt->value);//处理组网信息
                                        send_net_mess_for_uart();//给串口下发更新的组网信息
                                }else if(open_usb == false){//没有打开USB的情况下把处理过的数据发送到串口。------------------->决定不处理数据，只有USB监控开启的时候才处理数据。
                                        Tx_blink_hight();
                                        hw_uart_send(HW_UART2, final_package, final_package_len, NULL, NULL);
                                }else if(open_usb == true){
                                        if(si.sys_cfg.usb_debug == '1'){//如果USB监控打开的话，则将有效数据打包成网络数据发送到USB和串口。
                                                Tx_blink_hight();
                                                hw_uart_send(HW_UART2, final_package, final_package_len, NULL, NULL);
                                                myserialusb(final_package,final_package_len);
                                                //data_analysis(evt->value, evt->length);//解析数据。
                                        }else if(si.sys_cfg.usb_debug == '0'){//如果关闭了USB监控的话，则将有效数据打包成网络数据格式，发送到USB
                                                myserialusb(final_package,final_package_len);
                                        }
                                }
#if 0
                                if(si.sys_cfg.usb_debug == '1'){
                                        data_analysis(evt->value, evt->length);
                                }
                                oled_print_int(5, 2, evt->value[1]);//显示来自哪个设备
                                oled_print_int(13, 2, evt->value[2]);//显示帧号信息
                                sum++;
                                if(evt->value[2] == 1){
                                        sum = 1;
                                        fill_point(12, 4, 0x00);
                                        fill_point(13, 4, 0x00);
                                        fill_point(14, 4, 0x00);
                                        oled_print_int(12, 4, 0);
                                }else if(sum == evt->value[2]){//如果收到的数据条数等于传过来的帧号，则说明没丢包
                                        fill_point(12, 4, 0x00);
                                        fill_point(13, 4, 0x00);
                                        fill_point(14, 4, 0x00);
                                        oled_print_int(12, 4, 0);
                                }else if(sum != evt->value[2]){//如果收到的数据条数不等于帧号，且帧号不为1则说明丢包了。
                                        fill_point(12, 4, 0x00);
                                        fill_point(13, 4, 0x00);
                                        fill_point(14, 4, 0x00);
                                        oled_print_int(12, 4, sum);//oled上显示的是losss的数据
                                }
                                fill_point(4, 4, 0x00);
                                fill_point(5, 4, 0x00);
                                fill_point(6, 4, 0x00);
                                oled_print_int(4, 4, sum);//显示的是收到的数据总条数。
                                oled_print(0, 6, (const char *)(evt->value+4));
                                hw_uart_send(HW_UART2, (evt->value)+4, (evt->length)-4, NULL, NULL);
#endif
                        }
#endif
                }else if(si.sys_cfg.networkcmode == 'o'){
#if 1
                        if(open_usb == false){
                                if(evt->value[0] == 0x55 && evt->value[1] == 0xaa){
                                        if(evt->value[2] == 0x0b && evt->value[3] == 0x0d){//更新UART波特率
                                                Serial_Reinit(com_ble_baudrate_num(evt->value+5,evt->value[4]));
                                        }else{
                                                Tx_blink_hight();
                                                hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
                                        }
                                }else{
                                        Tx_blink_hight();
                                        hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
                                }
                        }else if(open_usb == true){//在USB打开的情况下，USB监控开启，则将数据发送到串口和USB，否则则将数据只发送到usb
                                if(si.sys_cfg.usb_debug == '1'){
                                        hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
                                }
                                myserialusb(evt->value, evt->length);
                        }
#endif
                }
        }
#endif
}

#if 0
static void handle_evt_gattc_discover_svc(ble_evt_gattc_discover_svc_t *evt){
        service_t *service;
        printf("%s :conn_idx=%04x uuid=%s start_h=%04x end_h=%04\r\n",__func__, evt->conn_idx,
                                                 format_uuid(&evt->uuid),evt->start_h,evt->end_h);
        service = OS_MALLOC(sizeof(*service));
        service->start_h = evt->start_h;
        service->end_h = evt->end_h;

        queue_push_back(&services, service);

}

static void handle_evt_gattc_discover_char(ble_evt_gattc_discover_char_t *evt)
{
        characteristic_t *characteristic;
        att_uuid_t uuid;

        printf("%s: conn_idx=%04x uuid=%s handle=%04x value_handle=%04x properties=%02x (%s)\r\n",
                __func__, evt->conn_idx, format_uuid(&evt->uuid), evt->handle, evt->value_handle,
                evt->properties, format_properties(evt->properties));

        ble_uuid_create16(0x2a00, &uuid); // Device Name
        if (ble_uuid_equal(&uuid, &evt->uuid)) {
                // store handle if Device Name is writable - once read is completed we'll write new
                // value there and read again
                if (evt->properties & GATT_PROP_WRITE) {
                        devname_val_h = evt->value_handle;
                }

                ble_gattc_read(evt->conn_idx, evt->value_handle, 0);
        }

        characteristic = OS_MALLOC(sizeof(*characteristic));
        characteristic->handle = evt->handle;
        characteristic->val_h = evt->value_handle;

        queue_push_back(&characteristics, characteristic);
}

static void handle_evt_gattc_discover_desc(ble_evt_gattc_discover_desc_t *evt)
{
        printf("%s: conn_idx=%04x uuid=%s handle=%04x\r\n", __func__, evt->conn_idx,
                                                        format_uuid(&evt->uuid), evt->handle);
}

static void handle_evt_gattc_discover_completed(ble_evt_gattc_discover_completed_t *evt)
{
        service_t *service;

        printf("%s: conn_idx=%04x type=%d status=%d\r\n", __func__, evt->conn_idx, evt->type,
                                                                                        evt->status);


        service = queue_peek_front(&services);

        if (evt->type == GATTC_DISCOVERY_TYPE_SVC && service) {
                ble_gattc_discover_char(evt->conn_idx, service->start_h, service->end_h, NULL);
        } else if (evt->type == GATTC_DISCOVERY_TYPE_CHARACTERISTICS && service) {
                characteristic_t *charac, *next = NULL;

                for (charac = queue_peek_front(&characteristics); charac; charac = next) {
                        next = charac->next;

                        /*
                         * Check if there is enough room for at least one descriptor.
                         * Range start from next handle after characteristic value handle,
                         * ends before next characteristic or service's end handle
                         */
                        if (charac->val_h + 1 < (next ? next->handle : service->end_h)) {
                                ble_gattc_discover_desc(evt->conn_idx, charac->val_h + 1, next ?
                                                        next->handle - 1 : service->end_h);
                        }
                }

                queue_remove_all(&characteristics, OS_FREE_FUNC);
                queue_pop_front(&services);
                OS_FREE(service);

                service = queue_peek_front(&services);
                if (service) {
                        ble_gattc_discover_char(evt->conn_idx, service->start_h,
                                                                        service->end_h, NULL);
                }
        }
}
#endif
static void handle_evt_gap_scan_completed(ble_evt_gap_scan_completed_t *evt){
#if wx_UART_DEBUG
        printf("handle_evt_gap_scan_completed\r\n");
#endif

}
volatile int  equipment_num;

static void handle_evt_gap_adv_report(ble_evt_gap_adv_report_t *evt){
        if(si.sys_cfg.networkcmode == 's'){
                uint8_t my_mac[6];
                memcpy(my_mac,evt->address.addr,6);
                if(hw_gpio_get_pin_status(HW_GPIO_PORT_1,HW_GPIO_PIN_7) == false){//准备录入白名单
                        if((evt->rssi)>220){//说明设备离的很近
                                PAIR_LED_Low();
                                if(evt->data[15] == 0x64 && evt->data[16] == 0x66 && evt->data[17] == 0x6e){///'dfn'就是说主机只有看见广播里的信息这几位是dfn时才会主动连接。
                                        for(int j = 0; j<10; j++){
                                                if(si.sys_cfg.Whitelist_mac[j][5] > 0 && si.sys_cfg.Whitelist_mac[j][5] < 0xff ){
                                                        if(memcmp(my_mac,si.sys_cfg.Whitelist_mac[j],6) == 0){
                                                                break;
                                                        }
                                                }else{//则将新的mac地址保存进去
                                                        PAIR_LED_Hight();
                                                        memcpy(si.sys_cfg.Whitelist_mac[j],my_mac,6);
                                                        nvms_t NVID_SYS_CONFIG;
                                                        NVID_SYS_CONFIG = ad_nvms_open(NVMS_PARAM_PART);
                                                        ad_nvms_write(NVID_SYS_CONFIG, 0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                                                        break;
                                                }
                                        }
                                        OS_DELAY(2);
                                        if(connect_dev_num == 0){
                                                OS_DELAY(10);
                                                OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(5000),10/ OS_PERIOD_MS);
                                        }else if(connect_dev_num == 1){
                                                OS_DELAY(10);
                                                OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(7000),10/ OS_PERIOD_MS);
                                        }else if(connect_dev_num == 2){
                                                OS_DELAY(10);
                                                OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(9000),10/ OS_PERIOD_MS);
                                        }else if(connect_dev_num == 3){
                                                OS_DELAY(10);
                                                OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(11000),10/ OS_PERIOD_MS);
                                        }else if(connect_dev_num == 4){
                                                OS_DELAY(10);
                                                OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(13000),10/ OS_PERIOD_MS);
                                        }

                                        ble_gap_scan_stop();
                                        const bd_address_t equipment_addr ={
                                                .addr_type = PUBLIC_ADDRESS,
                                                .addr = { evt->address.addr[0],
                                                          evt->address.addr[1],
                                                          evt->address.addr[2],
                                                          evt->address.addr[3],
                                                          evt->address.addr[4],
                                                          evt->address.addr[5] },
                                        };
                                        gap_conn_params_t equipment_cp = {
                                                //.interval_min  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.min_interval_ms),     // 50.00 ms
                                                //.interval_max  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.max_interval_ms),     // 70.00 ms
                                                .interval_min  = BLE_CONN_INTERVAL_FROM_MS(10),     // 50.00 ms
                                                .interval_max  = BLE_CONN_INTERVAL_FROM_MS(40),     // 70.00 ms
                                                .slave_latency = 0,
                                                .sup_timeout   = BLE_SUPERVISION_TMO_FROM_MS(1000),  // 420.00 ms
                                        };
                                        now_addr = evt->address.addr[5];
                                        ble_gap_connect(&equipment_addr, &equipment_cp);
                                }
                        }
                }else{//根据白名单来连接(没按boot按钮)
                        PAIR_LED_Low();
                        bool Allow_connection = false;
                        for(int i= 0;i<10;i++){
                                if(si.sys_cfg.Whitelist_mac[i][5] > 0 && si.sys_cfg.Whitelist_mac[i][5] < 0xff ){
                                        if(memcmp(my_mac,si.sys_cfg.Whitelist_mac[i],6) == 0){//说明是白名单内的设备。允许连接
                                                Allow_connection = true;
                                                break;
                                        }
                                }else{//说明白名单中没有保存mac地址
                                        break;
                                }
                        }
                        if(Allow_connection == true){//说明设备是白名单中的设备
                                if(evt->data[15] == 0x64 && evt->data[16] == 0x66 && evt->data[17] == 0x6e){///'dfn'就是说主机只有看见广播里的信息这几位是dfn时才会主动连接。
                                        OS_DELAY(2);
                                        if(connect_dev_num == 0){
                                                OS_DELAY(10);
                                                OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(5000),10/ OS_PERIOD_MS);
                                        }else if(connect_dev_num == 1){
                                                OS_DELAY(10);
                                                OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(7000),10/ OS_PERIOD_MS);
                                        }else if(connect_dev_num == 2){
                                                OS_DELAY(10);
                                                OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(9000),10/ OS_PERIOD_MS);
                                        }else if(connect_dev_num == 3){
                                                OS_DELAY(10);
                                                OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(11000),10/ OS_PERIOD_MS);
                                        }else if(connect_dev_num == 4){
                                                OS_DELAY(10);
                                                OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(13000),10/ OS_PERIOD_MS);
                                        }

                                        ble_gap_scan_stop();
                                        const bd_address_t equipment_addr ={
                                                .addr_type = PUBLIC_ADDRESS,
                                                .addr = { evt->address.addr[0],
                                                          evt->address.addr[1],
                                                          evt->address.addr[2],
                                                          evt->address.addr[3],
                                                          evt->address.addr[4],
                                                          evt->address.addr[5] },
                                        };
                                        gap_conn_params_t equipment_cp = {
                                                //.interval_min  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.min_interval_ms),     // 50.00 ms
                                                //.interval_max  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.max_interval_ms),     // 70.00 ms
                                                .interval_min  = BLE_CONN_INTERVAL_FROM_MS(10),     // 50.00 ms
                                                .interval_max  = BLE_CONN_INTERVAL_FROM_MS(40),     // 70.00 ms
                                                .slave_latency = 0,
                                                .sup_timeout   = BLE_SUPERVISION_TMO_FROM_MS(1000),  // 420.00 ms
                                        };
                                        now_addr = evt->address.addr[5];
                                        ble_gap_connect(&equipment_addr, &equipment_cp);
                                }
                        }
                }
        }else{//设备在从机下
                uint8_t my_mac[6];
                memcpy(my_mac,evt->address.addr,6);
                if(hw_gpio_get_pin_status(HW_GPIO_PORT_1,HW_GPIO_PIN_7) == false){//录入设备
                        if((evt->rssi)>220){
                                PAIR_LED_Low();
                                for(int j = 0; j<10; j++){
                                        if(si.sys_cfg.Whitelist_mac[j][5] > 0 && si.sys_cfg.Whitelist_mac[j][5] < 0xff ){
                                                if(memcmp(my_mac,si.sys_cfg.Whitelist_mac[j],6) == 0){
                                                        PAIR_LED_Hight();
                                                        break;
                                                }
                                        }else{//则将新的mac地址保存进去
                                                PAIR_LED_Hight();
                                                memcpy(si.sys_cfg.Whitelist_mac[j],my_mac,6);
                                                nvms_t NVID_SYS_CONFIG;
                                                NVID_SYS_CONFIG = ad_nvms_open(NVMS_PARAM_PART);
                                                ad_nvms_write(NVID_SYS_CONFIG, 0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                                                break;
                                        }
                                }
                                if(evt->data[15] == 0x64 && evt->data[16] == 0x66 && evt->data[17] == 0x6e){///'dfn'就是说主机只有看见广播里的信息这几位是dfn时才会主动连接。
                                        PAIR_LED_Hight();
                                        bluno2 = true;
                                        ble_gap_scan_stop();
                                        OS_DELAY(5);
                                        const bd_address_t equipment_addr ={
                                                .addr_type = PUBLIC_ADDRESS,
                                                .addr = { evt->address.addr[0],
                                                          evt->address.addr[1],
                                                          evt->address.addr[2],
                                                          evt->address.addr[3],
                                                          evt->address.addr[4],
                                                          evt->address.addr[5] },
                                        };
                                        gap_conn_params_t equipment_cp = {
                                                .interval_min  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.min_interval_ms),     // 50.00 ms
                                                .interval_max  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.max_interval_ms),     // 70.00 ms
                                                .slave_latency = 0,
                                                .sup_timeout   = BLE_SUPERVISION_TMO_FROM_MS(420),  // 420.00 ms
                                        };
                                        now_addr = evt->address.addr[5];
                                        ble_gap_connect(&equipment_addr, &equipment_cp);
                                }else if(evt->address.addr[5] == 0xc4){////////////用于连接bluno1代(临时用的)
                                        bluno2 = false;
                                        ble_gap_scan_stop();
                                        OS_DELAY(5);
                                        const bd_address_t equipment_addr ={
                                                .addr_type = PUBLIC_ADDRESS,
                                                .addr = { evt->address.addr[0],
                                                          evt->address.addr[1],
                                                          evt->address.addr[2],
                                                          evt->address.addr[3],
                                                          evt->address.addr[4],
                                                          evt->address.addr[5] },
                                        };
                                        gap_conn_params_t equipment_cp = {
                                                .interval_min  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.min_interval_ms),     // 50.00 ms
                                                .interval_max  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.max_interval_ms),     // 70.00 ms
                                                .slave_latency = 0,
                                                .sup_timeout   = BLE_SUPERVISION_TMO_FROM_MS(420),  // 420.00 ms
                                        };
                                        now_addr = evt->address.addr[5];
                                        ble_gap_connect(&equipment_addr, &equipment_cp);
                                }
                        }
                }else{//根据白名单的mac地址选择连接
                        PAIR_LED_Low();
                        bool Allow_connection = false;
                        for(int i= 0;i<10;i++){
                                if(si.sys_cfg.Whitelist_mac[i][5] > 0 && si.sys_cfg.Whitelist_mac[i][5] < 0xff ){
                                        if(memcmp(my_mac,si.sys_cfg.Whitelist_mac[i],6) == 0){//说明是白名单内的设备。允许连接
                                                Allow_connection = true;
                                                break;
                                        }
                                }else{//说明白名单中没有保存mac地址
                                        break;
                                }
                        }
                        if(Allow_connection == true){
                                if(evt->data[15] == 0x64 && evt->data[16] == 0x66 && evt->data[17] == 0x6e){///'dfn'就是说主机只有看见广播里的信息这几位是dfn时才会主动连接。
                                        bluno2 = true;
                                        ble_gap_scan_stop();
                                        OS_DELAY(5);
                                        const bd_address_t equipment_addr ={
                                                .addr_type = PUBLIC_ADDRESS,
                                                .addr = { evt->address.addr[0],
                                                          evt->address.addr[1],
                                                          evt->address.addr[2],
                                                          evt->address.addr[3],
                                                          evt->address.addr[4],
                                                          evt->address.addr[5] },
                                        };
                                        gap_conn_params_t equipment_cp = {
                                                .interval_min  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.min_interval_ms),     // 50.00 ms
                                                .interval_max  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.max_interval_ms),     // 70.00 ms
                                                .slave_latency = 0,
                                                .sup_timeout   = BLE_SUPERVISION_TMO_FROM_MS(420),  // 420.00 ms
                                        };
                                        now_addr = evt->address.addr[5];
                                        ble_gap_connect(&equipment_addr, &equipment_cp);
                                }else if(evt->address.addr[5] == 0xc4){////////////用于连接bluno1代
                                        bluno2 = false;
                                        ble_gap_scan_stop();
                                        OS_DELAY(5);
                                        const bd_address_t equipment_addr ={
                                                .addr_type = PUBLIC_ADDRESS,
                                                .addr = { evt->address.addr[0],
                                                          evt->address.addr[1],
                                                          evt->address.addr[2],
                                                          evt->address.addr[3],
                                                          evt->address.addr[4],
                                                          evt->address.addr[5] },
                                        };
                                        gap_conn_params_t equipment_cp = {
                                                .interval_min  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.min_interval_ms),     // 50.00 ms
                                                .interval_max  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.max_interval_ms),     // 70.00 ms
                                                .slave_latency = 0,
                                                .sup_timeout   = BLE_SUPERVISION_TMO_FROM_MS(420),  // 420.00 ms
                                        };
                                        now_addr = evt->address.addr[5];
                                        ble_gap_connect(&equipment_addr, &equipment_cp);
                                }
                        }
                }

        }
        //OS_DELAY(5);
}

#if 0
static void handle_evt_gap_write_completed(ble_evt_gattc_write_completed_t *evt){
        printf("handle_evt_gap_write_completed\r\n");
        printf("evt->conn_idx=%d\r\n",evt->conn_idx);
        printf("evt->handle=%d\r\n",evt->handle);
}
#endif
static uint8_t *transfer_address(uint8 *buf){
        uint8_t i,j,z=0;
        uint8_t address[30];
        static uint8_t last_address[30];
        for(i=0; i<12; i++){
                if(buf[i]<='9' && buf[i]>='0'){
                        address[i]=buf[i]-48;
                }else if(buf[i]<='F' && buf[i]>='A'){
                        address[i]=buf[i]-55;
                }else if(buf[i]<='f' && buf[i]>='a'){
                        address[i]=buf[i]-87;
                }
        }
        for(j=0; j<12; j++){
              if(j%2==0){
                      last_address[z]=address[j]*16+address[j+1];
                      z++;
              }
        }
        return last_address;
}


int i =0;
void HexToStr(BYTE *pbDest, BYTE *pbSrc, int nLen)
{
char    ddl,ddh;
int i;

for (i=0; i<nLen; i++)
{
ddh = 48 + pbSrc[i] / 16;
ddl = 48 + pbSrc[i] % 16;
if (ddh > 57) ddh = ddh + 7;
if (ddl > 57) ddl = ddl + 7;
pbDest[i*2] = ddh;
pbDest[i*2+1] = ddl;
}

pbDest[nLen*2] = '\0';
}
static char * inttohex(int aa,char *buffer)
{
    if (aa < 16)            //递归结束条件
    {
        if (aa < 10)        //当前数转换成字符放入字符串
            buffer[i] = aa + '0';
        else
            buffer[i] = aa - 10 + 'a';
        buffer[i+1] = '\0'; //字符串结束标志
    }
    else
    {
        inttohex(aa / 16,buffer);  //递归调用
        i++;                //字符串索引+1
        aa %= 16;           //计算当前值
        if (aa < 10)        //当前数转换成字符放入字符串
            buffer[i] = aa + '0';
        else
            buffer[i] = aa - 10 + 'a';
    }


    return (buffer);
}
uint32_t cifang(uint8_t a,uint8_t n){
        uint32_t sum=a;
        for(int i=1;i<n;i++){
                sum=sum*16;
        }
        return sum;
}
uint8_t data2[4];
uint8_t* my_data(uint32_t data){
        uint8_t data1[8];

        for(int i = 0 ;i<8; i++){
                if(i == 0){
                        data1[i]=(data)%16;
                }else{
                        data1[i]=(data/cifang(16,i))%16;
                }
        }
        for(int i = 0; i<4; i++){
                data2[i]=data1[i*2]+data1[i*2+1]*10;
        }
}


static uint8_t interval_min_now = 8;
bool bluno2 = true;

void ble_multi_link_task(void *params)
{

        ble_baudrate_num(57600);

        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        connect_first=1;
        first_time=0;
        first_there=1;
        sum=0;
        oled_init();
        bool time_begin = false;
        bool time1_begin = false;
        ble_service_t *dlg_mls;
        ble_service_t *devinfo;
        ble_error_t status;
        int8_t wdog_id;
        uint8_t *name_buf;
        myconnid = 0;
        const bd_address_t addr ={
                .addr_type = PUBLIC_ADDRESS,
                //.addr = { 0x6E, 0xB1, 0x28, 0x84, 0xBE, 0xC4 },
                //.addr = { 0x05, 0x00, 0x80, 0xCA, 0xEA, 0x80 },
                .addr = {transfer_address(si.sys_cfg.peerAddr+2)[5],
                        transfer_address(si.sys_cfg.peerAddr+2)[4],
                        transfer_address(si.sys_cfg.peerAddr+2)[3],
                        transfer_address(si.sys_cfg.peerAddr+2)[2],
                        transfer_address(si.sys_cfg.peerAddr+2)[1],
                        transfer_address(si.sys_cfg.peerAddr+2)[0],
                         },

        };
        const bd_address_t addr1 ={
                .addr_type = PUBLIC_ADDRESS,
                //.addr = { 0x6E, 0xB1, 0x28, 0x84, 0xBE, 0xC4 },
                .addr = { 0x05, 0x00, 0x80, 0xCA, 0xEA, 0x80 },
        };
        gap_conn_params_t cp = {
                .interval_min  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.min_interval_ms),     // 50.00 ms
                .interval_max  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.max_interval_ms),     // 70.00 ms
                .slave_latency = 0,
                .sup_timeout   = BLE_SUPERVISION_TMO_FROM_MS(420),  // 420.00 ms
        };
        ble_multi_link_task_handle = OS_GET_CURRENT_TASK();

        OS_TIMER wx_timer = OS_TIMER_CREATE("wx_timer", OS_MS_2_TICKS(10), true,
                                                                       (void *) OS_GET_CURRENT_TASK(), wx_tim_cb);
        OS_TIMER rx_tx_blink = OS_TIMER_CREATE("rx_tx_blink", OS_MS_2_TICKS(30), true,
                                                                       (void *) OS_GET_CURRENT_TASK(), rx_tx_blink_cb);
        scanning_timer = OS_TIMER_CREATE("scanning_timer", OS_MS_2_TICKS(2000), true,
                                       (void *) OS_GET_CURRENT_TASK(), scanning_timer_cb);
        link_timer = OS_TIMER_CREATE("link_timer", OS_MS_2_TICKS(3000), true,
                                        (void *) OS_GET_CURRENT_TASK(),link_timer_cb);
        OS_TIMER_START(link_timer, 10/ OS_PERIOD_MS);//10ms是指超时。
        OS_TIMER wx_timer1;
        /*
        OS_TIMER scanning_timer = OS_TIMER_CREATE("scanning_timer", OS_MS_2_TICKS(5000), true,
                                        (void *) OS_GET_CURRENT_TASK(), scanning_timer_cb);
*/
        /* register ble_multi_link task to be monitored by watchdog */
        wdog_id = sys_watchdog_register(false);

        status = ble_enable();
        if (status == BLE_STATUS_OK) {
                ble_gap_role_set(GAP_CENTRAL_ROLE | GAP_PERIPHERAL_ROLE);
        } else {
                printf("%s: failed. Status=%d\r\n", __func__, status);
        }


        ble_register_app();
        ble_gap_device_name_set((const char *)si.sys_cfg.name, ATT_PERM_READ);
        name_buf = si.sys_cfg.name;
        uint8_t i,j;
        for(j = 0; j < 10; j++){
                scanRsp_Data[j+2] = ' ';
        }
        for(i = 0; i < (strlen((const char *)si.sys_cfg.name)); i++){
                scanRsp_Data[i+2] = name_buf[i];
        }

        //dlg_mls = dlg_mls_init(bd_addr_write_cb);
        //ble_service_add(dlg_mls);

        devinfo = devinfo_init(NULL);
        ble_service_add(devinfo);

        dlg_mls = simpleGATT_init(NULL);
        ble_service_add(dlg_mls);
        if(si.sys_cfg.curr_role == ROLE_CENTRAL){
                ble_gap_adv_data_set(sizeof(adv_Data), adv_Data, sizeof(scanRsp_Data_center), scanRsp_Data_center);
        }else{
                ble_gap_adv_data_set(sizeof(adv_Data), adv_Data, sizeof(scanRsp_Data), scanRsp_Data);
        }
        if(si.sys_cfg.networkcmode == 's'){
                Serial_Reinit(9600);
        }else{
                Serial_Reinit(115200);
        }
#if 0
        hw_otpc_init();
        hw_otpc_set_speed(5);
        uint32_t mydata[8]={943076401,541409585,0,0,0,0,0,0};
        my_data(mydata[0]);
        for(int i =0;i<4;i++){
                printf("data2[%d]=%d\r\n",i,data2[i]);
        }
        hw_otpc_dma_read(mydata,7492,0,8,false);
        printf("mydata[0]=%02x\r\n",mydata[0]);
        printf("mydata[1]=%02x\r\n",mydata[1]);
        printf("mydata[2]=%02x\r\n",mydata[2]);
        printf("mydata[3]=%02x\r\n",mydata[3]);
        printf("mydata[4]=%02x\r\n",mydata[4]);
        printf("mydata[5]=%02x\r\n",mydata[5]);
        printf("mydata[6]=%02x\r\n",mydata[6]);
        printf("mydata[7]=%02x\r\n",mydata[7]);

#endif
        if(si.sys_cfg.curr_role == ROLE_CENTRAL){

                if(si.sys_cfg.cmode == 'u'){//连接主板绑定的蓝牙设备
                        oled_print(0,0,"I'm central!");
#if wx_UART_DEBUG
                        printf("I'm central\r\n");
#endif
                        ble_gap_connect(&addr, &cp);
                        //ble_gap_connect(&addr1, &cp);
                }else{//为任意连接
                        equipment_num = 0;
                        connect_dev_num = 0;
                        //oled_print(0, 0, "Start scanning!");
                        if(si.sys_cfg.networkcmode == 'o'){
                                //OS_TIMER_START(wx_timer, 10/ OS_PERIOD_MS);
                                ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(100), defaultBLE_SCAN_WINDOW, false,false);
                        }else{
                                net_inf_t *net_inf;
                                net_inf = OS_MALLOC(sizeof(net_inf_t));
                                memcpy(net_inf->addr.addr, si.sys_cfg.my_mac, sizeof(net_inf->addr.addr));
                                net_inf->conn_idx = 0xfe;
                                queue_push_front(&net_connections, (void *)net_inf);
                                //ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(50), defaultBLE_SCAN_WINDOW, false,false);
                                scanning_yes_or_no=true;
                                //OS_TIMER_START(wx_timer, 10/ OS_PERIOD_MS);
                                OS_TIMER_START(scanning_timer, 10/ OS_PERIOD_MS);//10ms是指延时
                        }

#if wx_UART_DEBUG
                        printf("Start scanning\r\n");
#endif

                        //ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(1000), defaultBLE_SCAN_WINDOW, false,false);
                }
        }else{
                //oled_print(0,0,"I'm peripheral!");
                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
                //oled_print(0, 2,"Start");
                //oled_print(0,4,"advertising!");
#if wx_UART_DEBUG
                printf("Start advertising\r\n");
#endif
        }

        OS_TIMER_START(rx_tx_blink, 10/ OS_PERIOD_MS);
        for (;;) {

                OS_BASE_TYPE ret;
                uint32_t notif;

                /* notify watchdog on each loop */
                sys_watchdog_notify(wdog_id);

                /* suspend watchdog while blocking on OS_TASK_NOTIFY_WAIT() */
                sys_watchdog_suspend(wdog_id);

                /*
                 * Wait on any of the notification bits, then clear them all
                 */
                ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
                /* This must block forever, until a task notification is received. So, the
                   return value must be OS_TASK_NOTIFY_SUCCESS */
                OS_ASSERT(ret == OS_TASK_NOTIFY_SUCCESS);

                /* resume watchdog */
                sys_watchdog_notify_and_resume(wdog_id);

                /* notified from BLE manager, can get event */
                if (notif & BLE_APP_NOTIFY_MASK) {
                        ble_evt_hdr_t *hdr;

                        hdr = ble_get_event(false);
                        if (!hdr) {

                                goto no_event;
                        }

                        if (!ble_service_handle_event(hdr)) {
                                switch (hdr->evt_code) {
                                case BLE_EVT_GAP_CONNECTED:
                                        handle_evt_gap_connected((ble_evt_gap_connected_t *) hdr);
                                        break;
                                case BLE_EVT_GAP_DISCONNECTED:
                                        handle_evt_gap_disconnected(
                                                               (ble_evt_gap_disconnected_t *) hdr);
                                        break;
                                case BLE_EVT_GAP_ADV_COMPLETED:
                                        handle_evt_gap_adv_completed(
                                                              (ble_evt_gap_adv_completed_t *) hdr);
                                        break;
                                case BLE_EVT_GAP_PAIR_REQ:
                                        handle_evt_gap_pair_req((ble_evt_gap_pair_req_t *) hdr);
                                        break;
                                case BLE_EVT_GAP_SECURITY_REQUEST:
                                        handle_evt_gap_security_request(
                                                           (ble_evt_gap_security_request_t *) hdr);
                                        break;
                                case BLE_EVT_GATTC_NOTIFICATION:
                                        handle_evt_gattc_notification(
                                                            (ble_evt_gattc_notification_t *) hdr);
                                        break;
#if 0
                                case BLE_EVT_GATTC_DISCOVER_SVC:
                                        handle_evt_gattc_discover_svc((ble_evt_gattc_discover_svc_t *) hdr);
                                        break;
                                case BLE_EVT_GATTC_DISCOVER_CHAR:
                                        handle_evt_gattc_discover_char((ble_evt_gattc_discover_char_t *) hdr);
                                        break;
                                case BLE_EVT_GATTC_DISCOVER_DESC:
                                        handle_evt_gattc_discover_desc((ble_evt_gattc_discover_desc_t *) hdr);
                                        break;
                                case BLE_EVT_GATTC_DISCOVER_COMPLETED:
                                        handle_evt_gattc_discover_completed((ble_evt_gattc_discover_completed_t *) hdr);
                                        break;
#endif
                                case BLE_EVT_GAP_SCAN_COMPLETED:
                                        handle_evt_gap_scan_completed(
                                                (ble_evt_gap_scan_completed_t *) hdr);
                                        break;
#if 0
                                case BLE_EVT_GATTC_WRITE_COMPLETED:
                                        handle_evt_gap_write_completed((ble_evt_gattc_write_completed_t*)hdr);
                                        break;
#endif
                                case BLE_EVT_GAP_ADV_REPORT:
                                        handle_evt_gap_adv_report((ble_evt_gap_adv_report_t*)hdr);/*用来获取扫描得到的信息*/
                                        break;
                                default:
                                        ble_handle_event_default(hdr);
                                        break;
                                }
                        }

                        OS_FREE(hdr);

no_event:
                        // notify again if there are more events to process in queue
                        if (ble_has_event()) {
                                OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(), BLE_APP_NOTIFY_MASK,
                                                                               OS_NOTIFY_SET_BITS);
                        }
                }
                if(notif & BEGIN_TIMER){
                        if(my_interval_min%2 != 0){//判断是不是偶数
                                my_interval_min += 1;
                        }
                        if(come_connecd == true){//如果是连接设备跳过来的
                                if(my_interval_min > interval_min_now){
                                        interval_min_now = my_interval_min;
                                        if(time1_begin == false){
                                                wx_timer1 = OS_TIMER_CREATE("wx_timer1", OS_MS_2_TICKS(my_interval_min*1.25), true,
                                                                                                     (void *) OS_GET_CURRENT_TASK(), wx_tim_cb);
                                                if(time_begin == true){
                                                        OS_TIMER_STOP(wx_timer, 10/ OS_PERIOD_MS);
                                                        time_begin = false;
                                                }
                                                OS_TIMER_START(wx_timer1, 10/ OS_PERIOD_MS);//10ms是指超时。
                                                time1_begin = true;
                                        }
                                }else{
                                        if(time1_begin == false && time_begin == false ){
                                                OS_TIMER_START(wx_timer, 10/ OS_PERIOD_MS);
                                                time_begin = true;
                                        }
                                }
                        }else if(come_connecd == false){
                                if(time1_begin == true){//说明time1肯定是跑着的
                                        if(phone_static == false){
                                                OS_TIMER_STOP(wx_timer1, 10/ OS_PERIOD_MS);
                                                OS_TIMER_DELETE(wx_timer1,10/ OS_PERIOD_MS);
                                                time1_begin = false;
                                                OS_TIMER_START(wx_timer, 10/ OS_PERIOD_MS);
                                                time_begin = true;
                                        }
                                }else if(time1_begin == false){
                                        if(connect_dev_num > 0){

                                        }else{
                                                OS_TIMER_STOP(wx_timer, 10/ OS_PERIOD_MS);
                                                time_begin = false;
                                        }
                                }

                        }

                }
                if(notif & DISCOVER_NOTIF){
                        ble_gattc_browse(0, NULL);
                        //ble_gattc_browse(1, NULL);
                        //ble_gattc_discover_svc(0, NULL);
                }
                if(notif & RECONNECT_NOTIF){
                        if(si.sys_cfg.curr_role == ROLE_CENTRAL){
                                ble_gap_connect(&addr, &cp);
                                //ble_gap_connect(&addr1, &cp);
                        }


                }
        }
}
