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

#define DISCOVER_NOTIF  (1 << 1)
#define RECONNECT_NOTIF (1 << 2)

PRIVILEGED_DATA static OS_TASK ble_multi_link_task_handle;

/*
 * Bluno2 demo advertising data
 */
uint8_t scanRsp_Data[] = {
        0x0C,
        GAP_DATA_TYPE_LOCAL_NAME,
        'B', 'l', 'u', 'n', 'o', '2',' ', ' ', ' ', ' ', ' ',

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

PRIVILEGED_DATA static queue_t connections;

PRIVILEGED_DATA static uint16_t master_dev_conn_idx = BLE_CONN_IDX_INVALID;

PRIVILEGED_DATA static bool connecting = false;

/*
 * Debug functions
 */

static void LED_Low( void );
static void LED_Hight(void);

static void LED_Low(void)
{
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        /*
        hw_gpio_configure_pin(HW_GPIO_PORT_0, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_0, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_1, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_2, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_6, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_2, HW_GPIO_PIN_2, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_0, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_1, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_2, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_6, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_6, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_2, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_1, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_0, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,false);
        */
        //hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,
        //                HW_GPIO_FUNC_GPIO,false);

}


static void LED_Hight(void)
{
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        /*
        hw_gpio_configure_pin(HW_GPIO_PORT_0, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_0, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_1, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_2, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_6, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_2, HW_GPIO_PIN_2, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_0, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_1, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_2, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_6, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_3, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_6, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_4, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_2, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_1, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_0, HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,true);
        */
        //hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_7, HW_GPIO_MODE_OUTPUT,
        //                HW_GPIO_FUNC_GPIO,true);

}

static int a=0;
uint8_t b= 5;
//uint8_t buf[20]="01234567890123456789";
//static int c=0;
void wx_tim_cb(void * p)
{
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

static void print_connection_func(void *data, void *user_data)
{
        const conn_dev_t *conn_dev = data;
        int *num = user_data;

        (*num)++;

        printf("%2d | %5d | %s\r\n", *num, conn_dev->conn_idx,
                                                        format_bd_address(&conn_dev->addr));
}

static void print_connections(void)
{
        int num = 0;

        printf("\r\n");
        printf("Nr | Index | Address\r\n");
        queue_foreach(&connections, print_connection_func, &num);

        if (!num) {
                printf("(no active connections)\r\n");
        }

        printf("\r\n");
}

bool list_elem_match(const void *elem, const void *ud)
{
        conn_dev_t *conn_dev = (conn_dev_t *) elem;
        uint16_t *conn_idx = (uint16_t *) ud;

        return conn_dev->conn_idx == *conn_idx;
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
static void handle_evt_gap_connected(ble_evt_gap_connected_t *evt)
{
        LED_Hight();
        ble_connect = 1;
        conn_dev_t *conn_dev;

        printf("%s: conn_idx=%d peer_address=%s\r\n", __func__, evt->conn_idx,
                                                        format_bd_address(&evt->peer_address));

        conn_dev = OS_MALLOC(sizeof(conn_dev_t));

        conn_dev->addr.addr_type = evt->peer_address.addr_type;
        memcpy(conn_dev->addr.addr, evt->peer_address.addr, sizeof(conn_dev->addr.addr));
        conn_dev->conn_idx = evt->conn_idx;
        myconn_idx = conn_dev->conn_idx;

        queue_push_front(&connections, (void *)conn_dev);

        if (master_dev_conn_idx == BLE_CONN_IDX_INVALID) {
                master_dev_conn_idx = evt->conn_idx;
        }
        connecting = false;

        print_connections();
        ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
        OS_TASK_NOTIFY(ble_multi_link_task_handle, DISCOVER_NOTIF, eSetBits);
}

static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt)
{
        printf("%s: conn_idx=%d address=%s\r\n", __func__, evt->conn_idx,
                                                        format_bd_address(&evt->address));
        LED_Low();
        ble_connect = 0;
        si.rssi=0;
        conn_dev_t *conn_dev = queue_remove(&connections, list_elem_match, &evt->conn_idx);

        if (conn_dev) {
                OS_FREE(conn_dev);
        }

        print_connections();

        if (master_dev_conn_idx == evt->conn_idx) {
                master_dev_conn_idx = BLE_CONN_IDX_INVALID;

                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
                printf("Advertising is on again\r\n");
        }
        if(si.sys_cfg.curr_role == ROLE_CENTRAL){
                OS_TASK_NOTIFY(ble_multi_link_task_handle, RECONNECT_NOTIF, eSetBits);
        }

}

static void handle_evt_gap_adv_completed(ble_evt_gap_adv_completed_t *evt)
{
        printf("%s: adv_type=%d status=%d\r\n", __func__, evt->adv_type, evt->status);
        OS_TASK_NOTIFY(ble_multi_link_task_handle, RECONNECT_NOTIF, eSetBits);

        if (connecting && (evt->status == BLE_ERROR_NOT_ALLOWED)) {
                connecting = false;

                ble_gap_connect_cancel();
                printf("%s: cancel the connect\r\n", __func__);

                if (master_dev_conn_idx != BLE_CONN_IDX_INVALID) {
                        return;
                }

                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
                printf("Advertising is on again after error\r\n");
        }
}

static void handle_evt_gap_pair_req(ble_evt_gap_pair_req_t *evt)
{
       // printf("%s: conn_idx=%d, bond=%d\r\n", __func__, evt->conn_idx, evt->bond);

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
static void handle_evt_gattc_notification(ble_evt_gattc_notification_t *evt)
{
        //format_value(evt->length, evt->value);
        if(si.sys_cfg.curr_role == ROLE_CENTRAL){
                myserialusb(evt->value, evt->length);//¸øUNOÏÂÔØ³ÌÐò
                if(si.sys_cfg.usb_debug == '1'){
                        //hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
                }
                hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
        }else{
                if(si.sys_cfg.usb_debug == '1'){
                        myserialusb(evt->value, evt->length);
                }

                hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);
        }
}
static void handle_evt_gap_scan_completed(ble_evt_gap_scan_completed_t *evt){
        ble_dev_params_t *params = ble_mgr_dev_params_acquire();

        //printf("params->dev_name=%c\r\n",&params->dev_name);
        hw_uart_send(HW_UART2, params->dev_name, 20, NULL, NULL);
        ble_mgr_dev_params_release();
        printf("handle_evt_gap_scan_completed\r\n");
        printf("scan_type=%d\r\n",evt->scan_type);
        printf("status=%d\r\n",evt->status);

}
static void handle_evt_gap_write_completed(ble_evt_gattc_write_completed_t *evt){
        printf("handle_evt_gap_write_completed\r\n");
        printf("evt->conn_idx=%d\r\n",evt->conn_idx);
        printf("evt->handle=%d\r\n",evt->handle);
}
static uint8_t *transfer_address(uint8 *buf){
        uint8_t i,j,z=0;
        uint8_t address[30];
        static uint8_t last_address[30];
        for(i=0; i<12; i++){
                if(buf[i]<='9' && buf[i]>='0'){
                        address[i]=buf[i]-48;
                }else{
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
void ble_multi_link_task(void *params)
{

        ble_service_t *dlg_mls;
        ble_service_t *devinfo;
        ble_error_t status;
        int8_t wdog_id;
        uint8_t *name_buf;
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
        gap_conn_params_t cp = {
                .interval_min  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.min_interval_ms),     // 50.00 ms
                .interval_max  = BLE_CONN_INTERVAL_FROM_MS(si.sys_cfg.max_interval_ms),     // 70.00 ms
                .slave_latency = 0,
                .sup_timeout   = BLE_SUPERVISION_TMO_FROM_MS(420),  // 420.00 ms
        };
        ble_multi_link_task_handle = OS_GET_CURRENT_TASK();


        //LED_Hight();
        //OS_DELAY(1000);
        OS_TIMER wx_timer = OS_TIMER_CREATE("wx_timer", OS_MS_2_TICKS(10), true,
                                                                       (void *) OS_GET_CURRENT_TASK(), wx_tim_cb);
                               OS_TIMER_START(wx_timer, 10 / OS_PERIOD_MS);

        /* register ble_multi_link task to be monitored by watchdog */
        wdog_id = sys_watchdog_register(false);

        status = ble_enable();

        if (status == BLE_STATUS_OK) {
                ble_gap_role_set(si.sys_cfg.curr_role);
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

        ble_gap_adv_data_set(sizeof(adv_Data), adv_Data, sizeof(scanRsp_Data), scanRsp_Data);
        if(si.sys_cfg.curr_role == ROLE_CENTRAL){
                if(si.sys_cfg.cmode == 'u'){
                        printf("I'm central\r\n");
                        ble_gap_connect(&addr, &cp);
                }else{
                        printf("begin scanning");
                        ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE, BLE_SCAN_INTERVAL_FROM_MS(10000), defaultBLE_SCAN_WINDOW, false,false);
                }
        }else{
                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
                printf("Advertising is on11\r\n");
        }
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
                                case BLE_EVT_GAP_SCAN_COMPLETED:
                                        handle_evt_gap_scan_completed(
                                                (ble_evt_gap_scan_completed_t *) hdr);
                                        break;
#if 0
                                case BLE_EVT_GATTC_WRITE_COMPLETED:
                                        handle_evt_gap_write_completed((ble_evt_gattc_write_completed_t*)hdr);
                                        break;
#endif
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
                if(notif & DISCOVER_NOTIF){
                        ble_gattc_browse(0, NULL);
                }
                if(notif & RECONNECT_NOTIF){
                        ble_gap_connect(&addr, &cp);
                }
        }
}
