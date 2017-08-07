/**
 ****************************************************************************************
 *
 * @file pxp_reporter_task.c
 *
 * @brief PXP profile app implementation
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
#include "USB_CDC.h"
#include "hw_otpc.h"
#include "SerialApp.h"
#include "ad_nvms.h"
#include <math.h>
#include "hw_led.h"
#include "hw_breath.h"
#include "MyGlobal.h"
#include "platform_nvparam.h"
#ifdef dg_LOWPOWER
#include "sys_power_mgr.h"
#endif

#if dg_configSUOTA_SUPPORT
#include "dis.h"
#include "dlg_suota.h"
#include "sw_version.h"
#endif

#if dg_configSUOTA_SUPPORT
/*
 * Store information about ongoing SUOTA.
 */
INITIALISED_PRIVILEGED_DATA static bool suota_ongoing = false;
#endif
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

#ifdef dg_LOWPOWER
extern uint16_t low_time;
extern bool low_status;
#endif
bool attest = false;
bool have_usb = false;
bool open_usb = false;
bool baudrate_change = false;
uint8_t baudrate_change_data = 0;
U32 baudrate_num = 115200;
uint8_t conn_net_work = 'o';
uint8_t conn_role = ROLE_CENTRAL;
extern uint8 *bdAddr2Str(uint8 *buf, uint8 *pAddr );
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

/*
 * Notification bits reservation
 *
 * Bit #0 is always assigned to BLE event queue notification.
 */
#define ALERT_TMO_NOTIF (1 << 1)
#define ADV_TMO_NOTIF   (1 << 2)
#define BAS_TMO_NOTIF   (1 << 3)

/*
 * PXP Update connection parameters notif mask
 */
#define PXP_UPDATE_CONN_PARAM_NOTIF     (1 << 4)

/*
 * The maximum length of name in scan response
 */
#define MAX_NAME_LEN    (BLE_SCAN_RSP_LEN_MAX - 2)

/*
 * PXP advertising and scan response data
 *
 * While not required, PXP specification states that PX reporter device using peripheral role can
 * advertise support for LLS. Device name is set in scan response to make it easily recognizable.
 */
/*
static const uint8_t adv_data[] = {
#if dg_configSUOTA_SUPPORT
        0x07, GAP_DATA_TYPE_UUID16_LIST_INC,
        0x03, 0x18, // = 0x1803 (LLS UUID)
        0x02, 0x18, // = 0x1802 (IAS UUID)
        0xF5, 0xFE, // = 0xFEF5 (DIALOG SUOTA UUID)
#else
        0x05, GAP_DATA_TYPE_UUID16_LIST_INC,
        0x03, 0x18, // = 0x1803 (LLS UUID)
        0x02, 0x18, // = 0x1802 (IAS UUID)
#endif
};
*/

PRIVILEGED_DATA static OS_TIMER scanning_timer;
PRIVILEGED_DATA static OS_TIMER link_timer;
PRIVILEGED_DATA static OS_TIMER wx_timer;
PRIVILEGED_DATA static OS_TIMER rx_tx_blink;
PRIVILEGED_DATA static OS_TASK ble_multi_link_task_handle;
/*
 * Bluno2 demo advertising data
 */
uint8_t scanRsp_Data[] = {
        0x0C,
        GAP_DATA_TYPE_LOCAL_NAME,
        'B', 'l', 'u', 'n', 'o', '2','v', '1', ' ', ' ', ' ',
        /*
        0x05,
        GAP_DATA_TYPE_MANUFACTURER_SPEC,
        'd','f','n','o',
        */
        0x05,
        GAP_DATA_TYPE_SLAVE_CONN_INTV,
        LO_UINT16( 8 ),
        HI_UINT16( 8 ),
        LO_UINT16( 8 ),
        HI_UINT16( 8 ),
        0x05, GAP_DATA_TYPE_UUID16_LIST_INC,
        0xB0, 0xDF, // = 0x1803 (DFB0)
        0x12, 0x18, // = 0x1802 (Human Interface Device Service)
        0x02,
        GAP_DATA_TYPE_TX_POWER_LEVEL,
        0
};

uint8_t scanRsp_Data_center[] = {
        0x0C,
        GAP_DATA_TYPE_LOCAL_NAME,
        'B', 'l', 'u', 'n', 'o', '2','v', '1', ' ', ' ', ' ',
        0x05,
        GAP_DATA_TYPE_SLAVE_CONN_INTV,
        LO_UINT16( 8 ),
        HI_UINT16( 8 ),
        LO_UINT16( 8 ),
        HI_UINT16( 8 ),
        0x05, GAP_DATA_TYPE_UUID16_LIST_INC,
        0xB0, 0xDF, // = 0x1803 (DFB0)
        0x12, 0x18, // = 0x1802 (Human Interface Device Service)
        0x02,
        GAP_DATA_TYPE_MANUFACTURER_SPEC,
        'r'

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

volatile static uint8_t  first_time;
volatile static uint8_t  first_there;
static bool scanning_yes_or_no = true;
uint8_t  connect_dev_num;
volatile uint8_t  connect_first;
static uint16_t my_interval_min=0;
static uint16_t my_interval_max=0;
static bool phone_static = false;
static uint8_t phone_id=0xff;
static bool connect_full = false;
static bool come_connecd = false;
static bool now_adv_or_scan = false;
#ifdef TalkBT_Charger_Test
void usb_charging(void){
        hw_gpio_configure_pin(PAIR_PORT, PAIR_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
}
void usb_charging_stopped(void){
        hw_gpio_configure_pin(PAIR_PORT, PAIR_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
}
void usb_charging_paused(void){
        hw_gpio_configure_pin(PAIR_PORT, PAIR_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
}
void usb_charged(void){
        hw_gpio_configure_pin(PAIR_PORT, PAIR_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
}

#endif

void Open_ble_data_timer(){
        if(ble_connect == 1){
                if(!OS_TIMER_IS_ACTIVE(wx_timer)){//����ڵ͹���ģʽ�£�����timeû�п����Ļ�,����time
                        OS_TIMER_START(wx_timer, 10/ OS_PERIOD_MS);
                }
        }
}
void Close_ble_data_timer(){
        if(OS_TIMER_IS_ACTIVE(wx_timer)){
                OS_TIMER_STOP(wx_timer, OS_TIMER_FOREVER);
        }
}

void Open_rx_tx_timer(){
        if(!OS_TIMER_IS_ACTIVE(rx_tx_blink)){//����ڵ͹���ģʽ�£�����timeû�п����Ļ�,����time
                OS_TIMER_START(rx_tx_blink, 10/ OS_PERIOD_MS);
        }
}
void Close_rx_tx_timer(){
        if(OS_TIMER_IS_ACTIVE(rx_tx_blink)){
                OS_TIMER_STOP(rx_tx_blink, OS_TIMER_FOREVER);
        }
}

void Open_link_timer(){
        if(!OS_TIMER_IS_ACTIVE(link_timer)){//����ڵ͹���ģʽ�£�����timeû�п����Ļ�,����time
                OS_TIMER_START(link_timer, 10/ OS_PERIOD_MS);
        }
}
void Close_link_timer(){
        if(ble_connect != 1){
                if(OS_TIMER_IS_ACTIVE(link_timer)){
                        OS_TIMER_STOP(link_timer, OS_TIMER_FOREVER);
                }
        }
}
bool Always_bright = false;
static void scanning_timer_cb(void *p){//15s�л��㲥��������
        if(scanning_yes_or_no ==false){//�������ʼ��������˵����һ�������������������ù㲥

                ble_gap_scan_stop();
                ble_gap_adv_stop();
                if(phone_static == true){//����ֻ��Ѿ��������ˣ��򲻹㲥�������豸����û�����������ж��Ƿ��������
                        if(connect_full == false){//���������û�������������
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
        }else if(scanning_yes_or_no == true){//�����ʼ��������˵����һ���ǹ㲥�����������������
                ble_gap_adv_stop();
                ble_gap_scan_stop();
                if(connect_full == true){//������������ˣ����ж��ֻ��Ƿ������������Ƿ�㲥
                        if(phone_static == false){//����ֻ�û����
                                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
                                now_adv_or_scan = true;
                        }else if(phone_static == true){
                                OS_TASK_YIELD();
                        }
                }else if(connect_full == false){//������û�������������
                        ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(50), defaultBLE_SCAN_WINDOW, false,false);
                        now_adv_or_scan = false;
                }
                scanning_yes_or_no = false;
        }
        fflush(stdout);

}

static void link_timer_cb(void *p){/*�������豸������ָʾ�Ƶ�ָʾ��ʽ*/
        if(ble_connect == 1){
                if(Always_bright == true){//˵���������ϣ�����3s��
                        hw_gpio_configure_pin(LINK_PORT, LINK_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
                        Always_bright = false;
                }else{
                        hw_gpio_configure_pin(LINK_PORT, LINK_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
                        OS_DELAY(100);
                        hw_gpio_configure_pin(LINK_PORT, LINK_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
                }
        }else{
                if(attest == false)
                hw_gpio_configure_pin(LINK_PORT, LINK_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        }
        OS_TASK_YIELD();
        fflush(stdout);
}
void rx_tx_blink_cb(void * p){
        if(attest == false){
                hw_gpio_configure_pin(RX_PORT, RX_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
                hw_gpio_configure_pin(TX_PORT, TX_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        }
        OS_TASK_YIELD();
        fflush(stdout);
}
void Rx_blink_hight(){
        hw_gpio_configure_pin(RX_PORT, RX_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
}
void Tx_blink_hight(){
#ifdef dg_LOWPOWER
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_2, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
#endif
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
        hw_gpio_configure_pin(LINK_PORT, LINK_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
}


static void LED_Hight(void)
{
        hw_gpio_configure_pin(LINK_PORT, LINK_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
}
static void PAIR_LED_Hight(void){
        hw_gpio_configure_pin(PAIR_PORT, PAIR_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
}
static void PAIR_LED_Low(void){
        hw_gpio_configure_pin(PAIR_PORT, PAIR_PIN, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
}
void wx_tim_cb(void * p)
{
#if 1
        if(ble_connect == 1){
                if(bledatalen > 0){
                        ble_sendData();
                }else{
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

        if(conn_dev->conn_idx == first_net_message_data[0]){//�������������ĵ��豸��Ϣ

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
static void first_net_message(){//����һ�������ϵ��豸�������������е��豸��Ϣ
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
static void save_net_message(uint8_t *buf){//�ӻ��������µ�������Ϣ
        if(buf[2] == 0xFF){//˵�����豸���ӣ�Ӧ�������豸
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
        }else if(buf[2] == 0xFE){//˵�����豸������Ӧ�ü�ȥ�豸��Ϣ��
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
/*
static uint8_t center_send_mess_uart[100];
static bool center_first_send_mess = true;
static uint8_t center_offset_len=10;

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
*/
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
        memcpy(mac_addr_ascii,net_inf->addr.addr,6);
        /*
        for(int i=0;i<6;i++){
                mac_addr_ascii[i]=net_inf->addr.addr[i];
        }
        */
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
/*
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
        center_send_mess_uart[8] = 0x01;//event  0x01  ����������Ϣ
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
*/
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
static void send_net_message_func(void *data, void *user_data){//����������Ϣ
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
void broadcast_net_message(uint8_t *buf, uint16_t len){//�����������������Ѿ����ӵ��豸����������Ϣ��
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
static uint8_t send_mess_uart[300];
static uint8_t new_mess_buf[300];
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
        memcpy(new_mess_buf+new_mess_buf_len,addr_ascii+10,4);//MAC��ַ��4λ
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
}
static void send_net_mess_for_uart(){
        int num = 0;
        uint8_t Lim_len = 80;//�ְ�����
        uint8_t Lim_buf[Lim_len+9];
        uint8_t frequency = 0;//��Ҫ�ּ���
        uint8_t remainder = 0;//���һ������ʣ���ٸ�
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
}

uint8_t now_addr=0x00;
static void handle_evt_gap_connected(ble_evt_gap_connected_t *evt)
{

#if 1
        if(conn_net_work == 's'){
                switch(hw_uart_baudrate_get(HW_UART2)){
                case HW_UART_BAUDRATE_9600:
                        break;
                default:
                        hw_uart_baudrate_set(HW_UART2,HW_UART_BAUDRATE_9600);
                        break;
                }
        }
        Always_bright = true;
        LED_Hight();
        ble_connect = 1;
#ifdef dg_LOWPOWER
        low_time = 0;
        if(low_status){//�������ǵ͹���
                low_status = false;
                pm_set_sleep_mode(pm_mode_active);
                Serial_Reinit1(baudrate_num);
                Serial_App_Resume();
                Open_ble_data_timer();
                Open_rx_tx_timer();
                Open_link_timer();
        }
#endif

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
                uint8_t new_network_mess[4]={0xFD,0x01,0x00,0x00};//new_network_mess[0]����������Ϣ��ͷ-----new_network_mess[1]���������ŵ�-----new_network_mess[2]�����͸�˭��-----new_network_mess[3]����Ͽ����ӵ�id��
                new_network_mess[2] = phone_id;
                new_network_mess[3] = evt->conn_idx;
                serial_blewrite(new_network_mess,4);
        }
        if(now_addr != evt->peer_address.addr[5]){//֤�����ֻ�����
                phone_id = evt->conn_idx;
                phone_static = true;//֤���ֻ��������ˡ�
        }else{
                connect_dev_num++;
        }
        /*
        if(evt->peer_address.addr[5] != 0x80){//֤�����ֻ�����
                phone_id = evt->conn_idx;
                phone_static = true;//֤���ֻ��������ˡ�
        }else{
                connect_dev_num++;
        }
        */
        if(conn_role == ROLE_CENTRAL){
                //PAIR_LED_Low();
                first_serial_blewrite(myconn_idx);                                                  /*�������ϵ��豸����������Ϣ*/
                if(conn_net_work == 'o'){                                                 /*��������ʽ�ر�ʱ��Ϊһ��һ����ʱ��������һ���豸��ֹͣ������*/
                        connect_full = true;
                        ble_gap_scan_stop();
                }else if(conn_net_work == 's'){
                        OS_TIMER_CHANGE_PERIOD(scanning_timer,OS_MS_2_TICKS(2000),10/ OS_PERIOD_MS);

                        if(connect_dev_num == si.sys_cfg.connectmax){
                                connect_full = true;
                        }else if(connect_dev_num < si.sys_cfg.connectmax){
                                //OS_DELAY(10);
                                //ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(50), defaultBLE_SCAN_WINDOW, false,false);
                                connect_full = false;
                        }

#if 0
                        if(connect_dev_num == 2){                                                   /*�����ϵ��豸������֮��ֹͣ������*/
                                printf("The connection device is full and the search is stopped\r\n");
                                ble_gap_scan_stop();
                                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);//�������Թ㲥����������
                        }else if(connect_dev_num<2){                                                                      /*������û��ʱ����������*/
                                printf("Connect the device is not full, continue to search\r\n");
                                ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(1000),
                                                                                        defaultBLE_SCAN_WINDOW, false,false);
                        }
#endif
                }
                //ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);//�������Թ㲥����������
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
        if(conn_net_work == 's'){
                if(conn_role == ROLE_CENTRAL){

                        net_inf_t *net_inf;
                        net_inf = OS_MALLOC(sizeof(net_inf_t));
                        net_inf->addr.addr_type = evt->peer_address.addr_type;
                        memcpy(net_inf->addr.addr, evt->peer_address.addr, sizeof(net_inf->addr.addr));
                        net_inf->conn_idx = evt->conn_idx;
                        first_net_message_data[0]=evt->conn_idx;
                        first_net_message();
                        queue_push_front(&net_connections, (void *)net_inf);
                        uint8_t send_net_messag[10];
                        send_net_messag[0] = 0xFA;
                        send_net_messag[1] = send_net_messag[2] = 0xFF;
                        send_net_messag[3] = evt->conn_idx;
                        for(int i=0;i<6;i++){
                                send_net_messag[i+4] = evt->peer_address.addr[i];
                        }


                        cuappEnqueue(send_net_messag,10);
                        bledatalen++;
                        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);

                        /*���ڴ�ӡ������Ϣ*/
                        send_net_mess_for_uart();
                }else{//�ӻ��������Ϣ��
                        queue_remove_all(&net_connections,OS_FREE_FUNC);
                }
        }
        come_connecd = true;
        OS_TASK_NOTIFY(ble_multi_link_task_handle, BEGIN_TIMER, eSetBits);
        OS_TASK_NOTIFY(ble_multi_link_task_handle, DISCOVER_NOTIF, eSetBits);
#endif
}

static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt)
{
#if 1
        bool phone_now = false;//�ֻ��ǲ��Ǹոյ��ġ�
#if OLED_PRINT
        fill(0x00);//oled����
#endif
        si.rssi=0;
        conn_dev_t *conn_dev = queue_remove(&connections, list_elem_match, &evt->conn_idx);
        if(conn_net_work == 's'){
                if(conn_role == ROLE_PERIPHERAL){
                        //������е�������Ϣ��
                        queue_remove_all(&net_connections,OS_FREE_FUNC);
                        /*����֮�󣬽��Լ��豸����Ϣ�ϴ���������ȥ��*/
                        net_inf_t *net_inf;
                        net_inf = OS_MALLOC(sizeof(net_inf_t));
                        memcpy(net_inf->addr.addr, si.sys_cfg.my_mac, sizeof(net_inf->addr.addr));
                        net_inf->conn_idx = 0x00;
                        queue_push_front(&net_connections, (void *)net_inf);
                        /*�ô��ڴ�ӡ������Ϣ*/
                        send_net_mess_for_uart();



                }else if(conn_role == ROLE_CENTRAL){
                        net_inf_t *net_inf = queue_remove(&net_connections, net_list_elem_match, &evt->conn_idx);
                        if (net_inf) {
                                OS_FREE(net_inf);
                        }
                        uint8_t send_dis_net_messag[10];
                        send_dis_net_messag[0] = 0xFA;
                        send_dis_net_messag[1] = 0xFF;
                        send_dis_net_messag[2] = 0xFE;//���ߴӻ������豸��conn_idx;
                        send_dis_net_messag[3] = evt->conn_idx;
                        for(int i=0;i<6;i++){
                                send_dis_net_messag[i+4] = evt->address.addr[i];
                        }

                        cuappEnqueue(send_dis_net_messag,10);
                        bledatalen++;
                        OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);

                        //��ӻ����͵�������Ϣ��
                        send_net_mess_for_uart();
                }
        }
        if(evt->conn_idx == phone_id){
                phone_now = true;
                phone_static = false;//�ֻ��Ͽ�
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
        if(phone_static == true){//����ֻ��������ŵģ������ֻ�����������Ϣ��
#if wx_UART_DEBUG
                printf("����������Ϣ\r\n");
#endif
                uint8_t new_network_mess[4]={0xFD,0x00,0x00,0x00};//new_network_mess[0]����������Ϣ��ͷ-----new_network_mess[1]����Ͽ�����-----new_network_mess[2]�����͸�˭��-----new_network_mess[3]����Ͽ����ӵ�id��
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
        if(conn_role == ROLE_CENTRAL){
                if(conn_net_work == 'o'){                                                                             /*�ر�����*/
                        if(si.sys_cfg.cmode == 'u'){                                                                            /*���Ϊָ������*/
                                OS_TASK_NOTIFY(ble_multi_link_task_handle, RECONNECT_NOTIF, eSetBits);                          /*��ת��ȥ�����������豸�������豸*/
                        }else{                                                                                                  /*���Ϊ��������*/
                                ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(100),
                                                                             defaultBLE_SCAN_WINDOW, false,false);   /*���¿�ʼ����*/
                                //oled_print(0, 0, "Again scanning!");

                        }
                }else if(conn_net_work == 's'){
                        //ble_gap_adv_stop();
                        //ble_gap_scan_stop();
                        //OS_DELAY(10);
                        if(phone_now == true && phone_static == false){//������ֻ��Ͽ���
                                //ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
                                //printf("�ֻ��Ͽ�\r\n");
                        }else{
                                //ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(1000),
                               //                                            defaultBLE_SCAN_WINDOW, false,false);           /*���¿�ʼ����*/
#if wx_UART_DEBUG
                                printf("�����豸û��\r\n");
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
#endif

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
/*
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
*/


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
/*
 * uint8_t bbb[5];
static void ble_baudrate_num(U32 num){
        U32 temp = num;
        if(10000<num && num<100000){
                for(int i=0;i<5;i++){
                        bbb[i] = temp/(10000/(my_pow(10,i)));
                        temp = temp - (bbb[i]*(10000/(my_pow(10,i))));
                }
        }
}
*/
static U32 com_ble_baudrate_num(uint8_t* buf,uint8_t len){
        U32 my_baudrate = 0;
        for(int i = 0; i<len;i++){
                my_baudrate +=buf[i]* (my_pow(10,len-i-1));
        }
        return my_baudrate;
}

volatile int sum;
/*����notif���յ����ݵĴ�����*/
bool UNO_DOWN = false;
static uint8_t final_package[30];
static uint8_t final_package_len = 30;
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
#ifdef dg_LOWPOWER
        low_time = 0;
        if(low_status){
                low_status = false;
                pm_set_sleep_mode(pm_mode_active);
                Serial_Reinit1(115200);
                Serial_App_Resume();
                Open_ble_data_timer();
                Open_rx_tx_timer();
                Open_link_timer();
        }

#endif
#if 1
        Merge_the_package_test(evt->value,evt->length,final_package);
        if(conn_role == ROLE_CENTRAL){//�����յ�notif����ʱ
                myconnid = 0xFE;
                if(conn_net_work == 'o'){//��ģʽ��������״̬ʱ
                        if(open_usb == false){
                                if(evt->value[0] == 0x55 && evt->value[1] == 0xaa){
                                        if(evt->value[2] == 0x0b && evt->value[3] == 0x0d){//����UART������
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
                                if(si.sys_cfg.usb_debug == '1'){//���USB�����˼�أ������ݷ��͸����ں�USB
                                        Tx_blink_hight();
                                        hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
                                        myserialusb(evt->value, evt->length);//��UNO���س���
                                }else{
                                        myserialusb(evt->value, evt->length);//��UNO���س���
                                }

                        }
                }else if(conn_net_work == 's'){//������ģʽΪ��������ʱ
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
                        if(evt->value[0] == 0xFE){//���������ж������յ��������ǲ��Ƿ��͸��Լ��ģ�
                                Tx_blink_hight();
                                if(open_usb == false){//��������Ǹ��Լ��ģ���USBû�д򿪵Ļ��������ݴ�����͵����ڡ�-------------------->����������
                                        hw_uart_send(HW_UART2, final_package, final_package_len, NULL, NULL);
                                }else if(open_usb == true){//���USB�Ǵ򿪵ġ�
                                        if(si.sys_cfg.usb_debug == '1'){//����ǿ�����صģ�����Ч���ݷ��͵����ں�USB
                                                hw_uart_send(HW_UART2, final_package, final_package_len, NULL, NULL);
                                                //data_analysis(evt->value, evt->length);
                                                myserialusb(final_package,final_package_len);
                                        }else if(si.sys_cfg.usb_debug == '0'){//����ر��˼�أ�����Ч���ݴ�����������ݸ�ʽ���͵�USB
                                                myserialusb(final_package,final_package_len);
                                        }
                                }

                        }else{//������Ƿ��͸��Լ��ģ�������д�������ȴ���ʱ�����͵�ָ���豸��
#if usblog
                                if(open_usb == true){
                                        myserialusb("cuappEnqueue\r\n",14);
                                }

#endif
                                cuappEnqueue(evt->value,evt->length);//�����յ������ݷ��͸�ָ�������ӵĴӻ�
                                bledatalen++;
                                OS_TASK_NOTIFY(Serial_send_task_handle, bledatalen, eSetBits);
                        }
#endif

                }
        }else{
                if(conn_net_work == 's'){//�ӻ����������������ݵ�����
#if 1
                        if(connect_first){//�ӻ��ڱ������������ҵ�һ���յ�notifʱ�������յ����������е�id��
                                myconnid = evt->value[0];
#if OLED_PRINT
                                oled_print_int(14,0,evt->value[0]);//��ʾ�ҵ�id
#endif
                                connect_first=0;
                        }else{
                                if(evt->value[1] == 0xFF && (evt->value[2] == 0xFF || evt->value[2] == 0xFE)){//˵�����յ�����������Ϣ��

                                        save_net_message(evt->value);//����������Ϣ
                                        send_net_mess_for_uart();//�������·����µ�������Ϣ
                                }else if(open_usb == false){//û�д�USB������°Ѵ���������ݷ��͵����ڡ�------------------->�������������ݣ�ֻ��USB��ؿ�����ʱ��Ŵ������ݡ�
                                        Tx_blink_hight();
                                        hw_uart_send(HW_UART2, final_package, final_package_len, NULL, NULL);
                                }else if(open_usb == true){
                                        if(si.sys_cfg.usb_debug == '1'){//���USB��ش򿪵Ļ�������Ч���ݴ�����������ݷ��͵�USB�ʹ��ڡ�
                                                Tx_blink_hight();
                                                hw_uart_send(HW_UART2, final_package, final_package_len, NULL, NULL);
                                                myserialusb(final_package,final_package_len);
                                                //data_analysis(evt->value, evt->length);//�������ݡ�
                                        }else if(si.sys_cfg.usb_debug == '0'){//����ر���USB��صĻ�������Ч���ݴ�����������ݸ�ʽ�����͵�USB
                                                myserialusb(final_package,final_package_len);
                                        }
                                }
#if 0
                                if(si.sys_cfg.usb_debug == '1'){
                                        data_analysis(evt->value, evt->length);
                                }
                                oled_print_int(5, 2, evt->value[1]);//��ʾ�����ĸ��豸
                                oled_print_int(13, 2, evt->value[2]);//��ʾ֡����Ϣ
                                sum++;
                                if(evt->value[2] == 1){
                                        sum = 1;
                                        fill_point(12, 4, 0x00);
                                        fill_point(13, 4, 0x00);
                                        fill_point(14, 4, 0x00);
                                        oled_print_int(12, 4, 0);
                                }else if(sum == evt->value[2]){//����յ��������������ڴ�������֡�ţ���˵��û����
                                        fill_point(12, 4, 0x00);
                                        fill_point(13, 4, 0x00);
                                        fill_point(14, 4, 0x00);
                                        oled_print_int(12, 4, 0);
                                }else if(sum != evt->value[2]){//����յ�����������������֡�ţ���֡�Ų�Ϊ1��˵�������ˡ�
                                        fill_point(12, 4, 0x00);
                                        fill_point(13, 4, 0x00);
                                        fill_point(14, 4, 0x00);
                                        oled_print_int(12, 4, sum);//oled����ʾ����losss������
                                }
                                fill_point(4, 4, 0x00);
                                fill_point(5, 4, 0x00);
                                fill_point(6, 4, 0x00);
                                oled_print_int(4, 4, sum);//��ʾ�����յ���������������
                                oled_print(0, 6, (const char *)(evt->value+4));
                                hw_uart_send(HW_UART2, (evt->value)+4, (evt->length)-4, NULL, NULL);
#endif
                        }
#endif
                }else if(conn_net_work == 'o'){
#if 1
                        if(open_usb == false){
                                if(evt->value[0] == 0x55 && evt->value[1] == 0xaa){
                                        if(evt->value[2] == 0x0b && evt->value[3] == 0x0d){//����UART������
                                                Serial_Reinit(com_ble_baudrate_num(evt->value+5,evt->value[4]));
                                        }else{
                                                Tx_blink_hight();
                                                hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
                                        }
                                }else{
                                        Tx_blink_hight();
                                        hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
                                }
                        }else if(open_usb == true){//��USB�򿪵�����£�USB��ؿ����������ݷ��͵����ں�USB������������ֻ���͵�usb
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
static void handle_evt_gap_scan_completed(ble_evt_gap_scan_completed_t *evt){
#if wx_UART_DEBUG
        printf("handle_evt_gap_scan_completed\r\n");
#endif

}
volatile int  equipment_num;

static uint16_t getserverid(ble_evt_gap_adv_report_t *evt){
        uint16_t ret = 0;
        uint8_t length = 0;
        uint8_t type = evt->data[1];
        while(length < evt->length){
                type = evt->data[length+1];
                if(type == GAP_DATA_TYPE_UUID16_LIST_INC){
                        ret = (evt->data[length+3]<<8)+evt->data[length+2];
                }
                length += evt->data[length]+1;
        }
        return ret;
}
static uint8_t getdfnode(ble_evt_gap_adv_report_t *evt){
        uint8_t ret = 0;
        uint8_t length = 0;
        uint8_t type = evt->data[1];
        while(length < evt->length){
                type = evt->data[length+1];
                if(type == GAP_DATA_TYPE_MANUFACTURER_SPEC){
                        ret = evt->data[length+2];
                }
                length += evt->data[length]+1;
        }
        return ret;
}
static void handle_evt_gap_adv_report(ble_evt_gap_adv_report_t *evt){
#if 1
        if(conn_net_work == 's'){
                uint8_t my_mac[6];
                memcpy(my_mac,evt->address.addr,6);
                if(hw_gpio_get_pin_status(HW_GPIO_PORT_1,HW_GPIO_PIN_7) == false){//׼��¼�������
                        if((evt->rssi)>220){//˵���豸��ĺܽ�
                                //PAIR_LED_Low();
                                if(getserverid(evt) == 0xdfb0 && getdfnode(evt) == 'n'){///'dfn'����˵����ֻ�п����㲥�����Ϣ�⼸λ��dfnʱ�Ż��������ӡ�
                                        for(int j = 0; j<10; j++){
                                                if(si.sys_cfg.Whitelist_mac[j][5] > 0 && si.sys_cfg.Whitelist_mac[j][5] < 0xff ){
                                                        if(memcmp(my_mac,si.sys_cfg.Whitelist_mac[j],6) == 0){
                                                                break;
                                                        }
                                                }else{//���µ�mac��ַ�����ȥ
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
                }else{//���ݰ�����������(û��boot��ť)
                        //PAIR_LED_Low();
                        bool Allow_connection = false;
                        for(int i= 0;i<10;i++){
                                if(si.sys_cfg.Whitelist_mac[i][5] > 0 && si.sys_cfg.Whitelist_mac[i][5] < 0xff ){
                                        if(memcmp(my_mac,si.sys_cfg.Whitelist_mac[i],6) == 0){//˵���ǰ������ڵ��豸����������
                                                Allow_connection = true;
                                                break;
                                        }
                                }else{//˵����������û�б���mac��ַ
                                        break;
                                }
                        }
                        if(Allow_connection == true){//˵���豸�ǰ������е��豸
                                if(getserverid(evt) == 0xdfb0 && getdfnode(evt) == 'n'){///'dfn'����˵����ֻ�п����㲥�����Ϣ�⼸λ��dfnʱ�Ż��������ӡ�
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
        }else{//�豸��P2P����ʱ
                uint8_t my_mac[6];
                memcpy(my_mac,evt->address.addr,6);
                if(hw_gpio_get_pin_status(HW_GPIO_PORT_1,HW_GPIO_PIN_7) == false){//¼���豸
                        if((evt->rssi)>220){
                                //PAIR_LED_Low();
                                for(int j = 0; j<10; j++){
                                        if(si.sys_cfg.Whitelist_mac[j][5] > 0 && si.sys_cfg.Whitelist_mac[j][5] < 0xff ){
                                                if(memcmp(my_mac,si.sys_cfg.Whitelist_mac[j],6) == 0){
                                                        PAIR_LED_Hight();
                                                        break;
                                                }
                                        }else{//���µ�mac��ַ�����ȥ
                                                PAIR_LED_Hight();
                                                memcpy(si.sys_cfg.Whitelist_mac[j],my_mac,6);
                                                nvms_t NVID_SYS_CONFIG;
                                                NVID_SYS_CONFIG = ad_nvms_open(NVMS_PARAM_PART);
                                                ad_nvms_write(NVID_SYS_CONFIG, 0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                                                break;
                                        }
                                }
                                if(getserverid(evt) == 0xdfb0 && getdfnode(evt) == 'n'){///'dfn'����˵����ֻ�п����㲥�����Ϣ�⼸λ��dfnʱ�Ż��������ӡ�
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
                                }else if(getserverid(evt) == 0xdfb0 && getdfnode(evt) != 'n' && getdfnode(evt) != 'r'){////////////��������bluno1��(��ʱ�õ�)
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
                }else{//���ݰ�������mac��ַѡ������
                        //PAIR_LED_Low();
                        bool Allow_connection = false;
                        for(int i= 0;i<10;i++){
                                if(si.sys_cfg.Whitelist_mac[i][5] > 0 && si.sys_cfg.Whitelist_mac[i][5] < 0xff ){
                                        if(memcmp(my_mac,si.sys_cfg.Whitelist_mac[i],6) == 0){//˵���ǰ������ڵ��豸����������
                                                Allow_connection = true;
                                                break;
                                        }
                                }else{//˵����������û�б���mac��ַ
                                        break;
                                }
                        }
                        if(Allow_connection == true){
                                if(getserverid(evt) == 0xdfb0 && getdfnode(evt) == 'n'){///'dfn'����˵����ֻ�п����㲥�����Ϣ�⼸λ��dfnʱ�Ż��������ӡ�
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
                                }else if(getserverid(evt) == 0xdfb0 && getdfnode(evt) != 'n' && getdfnode(evt) != 'r'){////////////��������bluno1��
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
#endif
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

#if 0
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
#endif
uint32_t cifang(uint8_t a,uint8_t n){
        uint32_t sum=a;
        for(int i=1;i<n;i++){
                sum=sum*16;
        }
        return sum;
}

static void AT_init(){
        conn_net_work = si.sys_cfg.networkcmode;
        switch(si.sys_cfg.curr_role){
        case ROLE_CENTRAL:
                conn_role = ROLE_CENTRAL;
                break;
        case ROLE_PERIPHERAL:
                conn_role = ROLE_PERIPHERAL;
                break;
        default:
                conn_role = ROLE_PERIPHERAL;
                break;
        }

}

static uint8_t scan_data2[] = {
        0x05,
        GAP_DATA_TYPE_SLAVE_CONN_INTV,
        LO_UINT16( 8 ),
        HI_UINT16( 8 ),
        LO_UINT16( 8 ),
        HI_UINT16( 8 ),
        0x05, GAP_DATA_TYPE_UUID16_LIST_INC,
        0xB0, 0xDF, // = 0x1803 (DFB0)
        0x12, 0x18, // = 0x1802 (Human Interface Device Service)
        0x02,
        GAP_DATA_TYPE_TX_POWER_LEVEL,
        0,
        0x02,
        GAP_DATA_TYPE_MANUFACTURER_SPEC,
        'n'
};
static uint8_t *myscan_data;
static uint8_t myscan_data_len = 0;
static void init_scandata(){
        uint8_t *name_buf;
        uint8_t conn_intv = 0;
        name_buf = si.sys_cfg.name;
        myscan_data[0] = (strlen((const char *)si.sys_cfg.name))+1;
        myscan_data[1] = GAP_DATA_TYPE_LOCAL_NAME;
        conn_intv = myscan_data[0] + 1;
        for(int i = 0; i < (strlen((const char *)si.sys_cfg.name)); i++){
                myscan_data[i+2] = ' ';
                myscan_data[i+2] = name_buf[i];
        }
        memcpy(myscan_data+conn_intv, scan_data2, 18);
        myscan_data_len = conn_intv+18;
}
static uint8_t interval_min_now = 8;
bool bluno2 = true;

void pxp_reporter_task(void *params)
{
         AT_init();

 #if defined ENCRYPTION
 #else
         uint8_t my_mac[6]=defaultBLE_STATIC_ADDRESS;
         memcpy(si.sys_cfg.my_mac,my_mac,6);
         nvms_t NVID_SYS_CONFIG;
         NVID_SYS_CONFIG = ad_nvms_open(NVMS_PARAM_PART);
         ad_nvms_write(NVID_SYS_CONFIG, 0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
 #endif
         //ble_baudrate_num(57600);
         hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
         connect_first=1;
         first_time=0;
         first_there=1;
         sum=0;
         //oled_init();
         ble_service_t *dlg_mls;
         ble_service_t *devinfo;
         ble_error_t status;
         int8_t wdog_id;
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
         /*
         const bd_address_t addr1 ={
                 .addr_type = PUBLIC_ADDRESS,
                 .addr = { 0x05, 0x00, 0x80, 0xCA, 0xEA, 0x80 },
         };
         */
         gap_conn_params_t cp = {
                 .interval_min  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.min_interval_ms),     // 50.00 ms
                 .interval_max  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.max_interval_ms),     // 70.00 ms
                 .slave_latency = 0,
                 .sup_timeout   = BLE_SUPERVISION_TMO_FROM_MS(420),  // 420.00 ms
         };
         ble_multi_link_task_handle = OS_GET_CURRENT_TASK();

         wx_timer = OS_TIMER_CREATE("wx_timer", OS_MS_2_TICKS(10), true,
                                                                        (void *) OS_GET_CURRENT_TASK(), wx_tim_cb);
         rx_tx_blink = OS_TIMER_CREATE("rx_tx_blink", OS_MS_2_TICKS(30), true,
                                                                        (void *) OS_GET_CURRENT_TASK(), rx_tx_blink_cb);
         scanning_timer = OS_TIMER_CREATE("scanning_timer", OS_MS_2_TICKS(2000), true,
                                        (void *) OS_GET_CURRENT_TASK(), scanning_timer_cb);
         link_timer = OS_TIMER_CREATE("link_timer", OS_MS_2_TICKS(3000), true,
                                         (void *) OS_GET_CURRENT_TASK(),link_timer_cb);
         OS_TIMER_START(link_timer, 10/ OS_PERIOD_MS);//10ms��ָ��ʱ��
         //OS_TIMER wx_timer1;
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
         init_scandata();

         //dlg_mls = dlg_mls_init(bd_addr_write_cb);
         //ble_service_add(dlg_mls);

         devinfo = devinfo_init(NULL);
         ble_service_add(devinfo);

         dlg_mls = simpleGATT_init(NULL);
         ble_service_add(dlg_mls);
         if(conn_role == ROLE_CENTRAL){
                 ble_gap_adv_data_set(sizeof(adv_Data), adv_Data, sizeof(scanRsp_Data_center), scanRsp_Data_center);
         }else{
                 if(myscan_data[0] > 0x0C){
                         ble_gap_adv_data_set(sizeof(adv_Data), adv_Data, sizeof(scanRsp_Data), scanRsp_Data);
                 }else{
                         ble_gap_adv_data_set(sizeof(adv_Data), adv_Data, myscan_data_len, myscan_data);
                 }

         }
         if(conn_net_work == 's'){
                 Serial_Reinit(9600);
         }else{
                 Serial_Reinit(115200);
         }
         if(conn_role == ROLE_CENTRAL){

                 if(si.sys_cfg.cmode == 'u'){//��������󶨵������豸
                         //oled_print(0,0,"I'm central!");
 #if wx_UART_DEBUG
                         printf("I'm central\r\n");
 #endif
                         ble_gap_connect(&addr, &cp);
                 }else{//Ϊ��������
                         equipment_num = 0;
                         connect_dev_num = 0;
                         //oled_print(0, 0, "Start scanning!");
                         if(conn_net_work == 'o'){
                                 ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(100), defaultBLE_SCAN_WINDOW, false,false);
                         }else{
                                 net_inf_t *net_inf;
                                 net_inf = OS_MALLOC(sizeof(net_inf_t));
                                 memcpy(net_inf->addr.addr, si.sys_cfg.my_mac, sizeof(net_inf->addr.addr));
                                 net_inf->conn_idx = 0xfe;
                                 queue_push_front(&net_connections, (void *)net_inf);
                                 scanning_yes_or_no=true;
                                 OS_TIMER_START(scanning_timer, 10/ OS_PERIOD_MS);//10ms��ָ��ʱ
                         }
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
#ifdef dg_LOWPOWER

#endif
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
                                         handle_evt_gap_adv_report((ble_evt_gap_adv_report_t*)hdr);/*������ȡɨ��õ�����Ϣ*/
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
                         if(my_interval_min%2 != 0){//�ж��ǲ���ż��
                                 my_interval_min += 1;
                         }
                         if(come_connecd == true){
                                 if(my_interval_min > interval_min_now){
                                         interval_min_now = my_interval_min;
                                         if(OS_TIMER_IS_ACTIVE(wx_timer)){
                                                 OS_TIMER_CHANGE_PERIOD(wx_timer,OS_MS_2_TICKS(my_interval_min*1.25),10/ OS_PERIOD_MS);
                                         }else{
                                                 OS_TIMER_START(wx_timer, 10/ OS_PERIOD_MS);
                                         }

                                 }else{
                                         if(!OS_TIMER_IS_ACTIVE(wx_timer)){
                                                 OS_TIMER_START(wx_timer, 10/ OS_PERIOD_MS);
                                         }
                                 }
                         }else if(come_connecd == false){
                                 if(ble_connect == 0){
                                         if(OS_TIMER_IS_ACTIVE(wx_timer)){
                                                 OS_TIMER_STOP(wx_timer ,OS_TIMER_FOREVER);
                                         }
                                 }
                         }
#if 0
                         if(come_connecd == true){//����������豸��������
                                 if(my_interval_min > interval_min_now){
                                         interval_min_now = my_interval_min;
                                         if(time1_begin == false){
                                                 wx_timer1 = OS_TIMER_CREATE("wx_timer1", OS_MS_2_TICKS(my_interval_min*1.25), true,
                                                                                                      (void *) OS_GET_CURRENT_TASK(), wx_tim_cb);
                                                 if(time_begin == true){
                                                         OS_TIMER_STOP(wx_timer, 10/ OS_PERIOD_MS);
                                                         time_begin = false;
                                                 }
                                                 OS_TIMER_START(wx_timer1, 10/ OS_PERIOD_MS);//10ms��ָ��ʱ��
                                                 time1_begin = true;
                                         }
                                 }else{
                                         if(time1_begin == false && time_begin == false ){
                                                 OS_TIMER_START(wx_timer, 10/ OS_PERIOD_MS);
                                                 time_begin = true;
                                         }
                                 }
                         }else if(come_connecd == false){
                                 if(time1_begin == true){//˵��time1�϶������ŵ�
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
#endif

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

