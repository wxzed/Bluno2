/*
 * stateMachine.h
 *
 *  Created on: 2017年3月7日
 *      Author: Administrator
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sdk_defs.h>
#include <ble_gap.h>
#include <USB_CDC.h>
#include <hw_uart.h>

//enum sys_sm
//{
//初始化状态，上电后进入此状态
//bit  7    6    5    4    3    2     1    0
//    BLE   COM  USB                     TRANS/AT
#define BLE_OPEN (1<<7)
#define COM_OPEN (1<<6)
#define USB_OPEN (1<<5)
#define HID_OPEN (1<<4)
#define USB_AT_OPEN  (1<<1)
#define COM_AT_OPEN  (1<<0)
#define RESTART  0xFF
#define   FSM_INIT  0
#define  FSM_BLE_AT    (BLE_OPEN)
//USB<->COM
#define  FSM_USB_COM   (USB_OPEN | COM_OPEN)
//BLE<->COM
#define  FSM_COM_BLE_AT (BLE_OPEN | COM_OPEN | COM_AT_OPEN)
//(ATUSB)<->(BLE/CPU)<->(ATCOM)    USB上电并且配置开关开后进入这个状态
#define  FSM_USB_COM_BLE_AT  ( USB_OPEN | BLE_OPEN | COM_OPEN | COM_AT_OPEN | USB_AT_OPEN )
//透明串口模式USB<->BLE
#define  FSM_TRANS_USB_BLE ( USB_OPEN | BLE_OPEN )
//透明串口模式COM<->BLE     //如果连接了USB，USB将把上拉电阻关闭，只供电
#define  FSM_TRANS_COM_BLE ( COM_OPEN | BLE_OPEN )
//USB<->  <BLE>  <->COM  如果插入了usb，那么就是usb<->com 如果没有插入usb，就是ble<->com
#define  FSM_TRANS_USB_COM_BLE   (USB_OPEN | COM_OPEN | BLE_OPEN)
#define  FSM_HID_USB_COM_BLE_AT  ( USB_OPEN | BLE_OPEN | COM_OPEN | COM_AT_OPEN | HID_OPEN)
//};

#define B_ADDR_STR_LEN 15

enum role
{
  ROLE_PERIPHERAL = 0x08,
  ROLE_CENTRAL = 0x04,
  ROLE_UNKNOWN
};

enum triggerSource
{
    TRIGGER_SOURCE_UNKNOWN,
    TRIGGER_SOURCE_COM,
    TRIGGER_SOURCE_USB
};
enum data_src
{
  myUSB,
  myCOM,
  myBLE,
  myBLE_AT
};

typedef struct sys_config
{
    USB_CDC_LINE_CODING lineCoding;
    uint16         final_sm;
    uint16_t       min_interval;
    enum           role    curr_role;
    uint8_t        cmode;
    uint8_t        networkcmode;
    uint8 peerAddr[B_ADDR_STR_LEN];
    uint8        name[20];
    uint8_t        md5[17];
    uint16_t       min_interval_ms;
    uint16_t       max_interval;
    uint16_t       max_interval_ms;

    uint8_t        usb_debug;
    uint8_t        bluno_debug;
    uint8_t        reliable_transfer;
    uint8_t        setting;
    uint16_t       major;
    uint16_t       minor;
    int8           txpower;
    uint8        version[6];
    uint8_t        ibeacons;
    uint8_t        password[8];
    uint8_t        reserve[19];
}sys_config_t;

typedef struct sys_info
{
    enum           role      curr_role;
    uint8_t        prev_sm;
    uint8_t        curr_sm;
    uint8_t        final_sm;
    uint8_t        ble_open:1;
    uint8_t        usb_open:1;
    uint8_t        usb_serial_open:1;
    uint8_t        restart:1;
    uint8_t        bleAtCmdResponse:1;
    uint8_t        at_mode:1;
    uint8_t        pass:1;
    uint8_t        bleSendEnable:1;
    uint8_t        mac[6];
    uint8_t        rssi;
    enum           triggerSource  plusTriggerSource;
    uint8_t        idleTimeElapseCom,idleTimeElapseUSB;
    uint8_t        plusTimeElapseCom,plusTimeElapseUSB;
    uint8_t        plusCounterCom,plusCounterUSB;
    uint8_t        ble_at_enable;
    uint8_t        ble_peri_at_ctl;
    uint16_t       conn_interval_ms;
    sys_config_t sys_cfg;
}sys_info_t;

extern struct sys_info si;
extern uint8 deviceRole;
extern bool initDone;
extern bool initCount;

void fsm_init();
void fsm_updata(uint8 curr);


extern uint8_t scanRspData[];
extern uint8_t advertData[];
extern USB_CDC_LINE_CODING currentLineCoding;

#endif /* STATEMACHINE_H_ */
