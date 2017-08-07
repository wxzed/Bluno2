/*
 * stateMachine.c
 *
 *  Created on: 2017Äê3ÔÂ8ÈÕ
 *      Author: Administrator
 */

#include "stateMachine.h"
#include "atParser.h"
#include "ad_nvms.h"
#include "Devinfoservice.h"
#include <string.h>
#include <stdio.h>
#include <USB_CDC.h>
#include <time.h>

USB_CDC_LINE_CODING currentLineCoding;

struct sys_info si={
    .restart = 0,
};

void fsm_init(){
        nvms_t NVID_SYS_CONFIG;
        NVID_SYS_CONFIG = ad_nvms_open(NVMS_PARAM_PART);
        si.prev_sm = FSM_INIT;
        si.curr_sm = FSM_INIT;

        si.sys_cfg.curr_role =GAP_PERIPHERAL_ROLE;
        si.sys_cfg.final_sm = FSM_TRANS_USB_COM_BLE;
        si.sys_cfg.cmode = 'a';
        si.ble_open = 0;
        si.usb_open = 0;
        si.restart = 0;
        si.pass = 1;
        si.ble_at_enable = 0;
        si.ble_peri_at_ctl = 0;
        si.bleAtCmdResponse = 1;
        si.sys_cfg.min_interval_ms = 10;
        si.sys_cfg.max_interval_ms = 10;
        ad_nvms_read(NVID_SYS_CONFIG, 0, ( uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
        if(si.sys_cfg.update != '0'){
                si.sys_cfg.update = '0';
        }
        if((si.sys_cfg.txpower) < '0' || (si.sys_cfg.txpower > '4')){
                si.sys_cfg.txpower = '2';
        }
        if(si.sys_cfg.connectmax<1 || si.sys_cfg.connectmax>4){
                si.sys_cfg.connectmax =4;
        }
#if 0
        if((si.sys_cfg.min_interval < 8) || (si.sys_cfg.min_interval > 10000)){
                si.sys_cfg.min_interval = 16;
        }
        if((si.sys_cfg.max_interval < 8) || (si.sys_cfg.max_interval > 10000)){
                si.sys_cfg.max_interval = 32;
        }
#endif
        if((si.sys_cfg.min_interval_ms < 8) || (si.sys_cfg.min_interval_ms > 10000)){
                si.sys_cfg.min_interval = 8;
                si.sys_cfg.min_interval_ms = 10;
        }
        if((si.sys_cfg.max_interval_ms < 8) || (si.sys_cfg.max_interval_ms > 10000)){
                si.sys_cfg.max_interval_ms = 10;
                si.sys_cfg.max_interval = 8;
        }
        if((si.sys_cfg.usb_debug != '0') && (si.sys_cfg.usb_debug != '1')){
                si.sys_cfg.usb_debug = '0';
        }
        if((si.sys_cfg.reliable_transfer != 0) && (si.sys_cfg.reliable_transfer != '1')){
                si.sys_cfg.reliable_transfer = '0';
        }
        if((si.sys_cfg.bluno_debug != '0') && (si.sys_cfg.bluno_debug != '1')){
                si.sys_cfg.bluno_debug = '1';
        }
        if((si.sys_cfg.ibeacons != '0') && (si.sys_cfg.ibeacons != '1')){
                si.sys_cfg.ibeacons = '1';
        }
        if((si.sys_cfg.setting != 'u') && (si.sys_cfg.setting != 'd') && (si.sys_cfg.setting != 'd'+'c')){
                si.sys_cfg.setting = 'd';
        }
        if((si.sys_cfg.cmode != 'u') && (si.sys_cfg.cmode != 'a')){
                si.sys_cfg.cmode = 'a';
        }
        if((si.sys_cfg.networkcmode != 'o') && (si.sys_cfg.networkcmode != 's') && (si.sys_cfg.networkcmode != 't')){
                si.sys_cfg.networkcmode = 'o';
        }
        if(si.sys_cfg.password[0] == 0){
                strncpy((char *)si.sys_cfg.password,DEFPASSWORD,strlen(DEFPASSWORD)+1);
        }
        si.idleTimeElapseCom = 0;
        si.plusTimeElapseCom = 0;
        si.plusCounterCom = 0;
        si.idleTimeElapseUSB = 0;
        si.plusTimeElapseUSB = 0;
        si.plusCounterUSB = 0;
        si.plusTriggerSource = TRIGGER_SOURCE_UNKNOWN;
        if(memcmp(BLE_VERSION,si.sys_cfg.version,strlen(BLE_VERSION)) != 0){
                memcpy(si.sys_cfg.version,BLE_VERSION,strlen(BLE_VERSION));
        }
        if((si.sys_cfg.name[0] == 255) || (si.sys_cfg.name[0] == 0)){
                osal_memcpy(si.sys_cfg.name,BLE_NAME,strlen(BLE_NAME)+1);
        }
        /*
        {

                uint8 len = strlen((char const *)si.sys_cfg.name);
                osal_memcpy(&scanRspData[2],si.sys_cfg.name,len>14?14:len);
                if(len < 14){
                        for(;len<14;len++){
                                scanRspData[2+len] = 0;
                        }
                }
                scanRspData[18] = LO_UINT16( si.sys_cfg.min_interval );
                scanRspData[19] = HI_UINT16( si.sys_cfg.min_interval );
                scanRspData[20] = LO_UINT16( si.sys_cfg.max_interval );
                scanRspData[21] = HI_UINT16( si.sys_cfg.max_interval );

                if(si.sys_cfg.ibeacons == '1'){
                        advertData[25] = HI_UINT16( si.sys_cfg.major );
                        advertData[26] = LO_UINT16( si.sys_cfg.major );
                        advertData[27] = HI_UINT16( si.sys_cfg.minor );
                        advertData[28] = LO_UINT16( si.sys_cfg.minor );
                }else{
                        uint8 i;
                        for(i=3; i<30; i++){
                                advertData[i] = 0;
                        }
                }
        }
        osal_memcpy(&currentLineCoding,&si.sys_cfg.lineCoding,sizeof(USB_CDC_LINE_CODING));
        if(si.sys_cfg.final_sm == FSM_INIT){
                si.sys_cfg.final_sm = FSM_TRANS_USB_COM_BLE;
        }
        if(si.sys_cfg.final_sm == FSM_HID_USB_COM_BLE_AT){
                si.at_mode = 1;
        }
        si.final_sm = si.sys_cfg.final_sm;
        si.curr_role = si.sys_cfg.curr_role;
        si.sys_cfg.min_interval = 16;
        si.sys_cfg.max_interval = 32;

        si.sys_cfg.min_interval_ms = (uint16)(si.sys_cfg.min_interval*1.25);
        si.sys_cfg.max_interval_ms = (uint16)(si.sys_cfg.max_interval*1.25);
        si.conn_interval_ms = si.sys_cfg.max_interval_ms;
*/
}





