/*
 * atParser.c
 *
 *  Created on: 2017年3月6日
 *      Author: Administrator
 */

#include "atParser.h"
#include "comdef.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "hw_uart.h"
#include "stateMachine.h"
#include "USB_CDC.h"
#include "sdk_defs.h"
#include "comdef.h"
#include "ad_nvms.h"
#include "ble_common.h"
#include "Common.h"
#include "Devinfoservice.h"


uint8 local_addr[B_ADDR_STR_LEN];
uint8 atComCmd[AT_BUF_LEN];
uint8 atBLECmd[AT_BUF_LEN];
uint8 atUSBCmd[AT_BUF_LEN];
uint8 * long_buf_addr = NULL;

const uint8 atResult[AT_RESULT_NUM][25]={"?","ROLE_PERIPHERAL","ROLE_CENTRAL","FSM_USB_COM_BLE_AT","FSM_COM_BLE_AT",
                      "FSM_USB_COM","FSM_TRANS_USB_BLE","FSM_TRANS_COM_BLE","FSM_TRANS_USB_COM_BLE","FSM_HID_USB_COM_BLE_AT"};
const uint8 atPinMode[AT_PINMODE_NUM][4]={"OUT","IN","PWM","ADC"};
const uint8 resource[]=
{"[\
{\
\"id\": \"D001\",\
\"name\": \"SWITCH01\",\
\"mode\": \"OUTPUT\",\
\"pin\": \"P07\"\
},\
{\
\"id\": \"D002\",\
\"name\": \"SWITCH02\",\
\"mode\": \"OUTPUT\",\
\"pin\": \"P10\"\
}\
]\r\n"};
dataTransferT transArg;
void * selfFuncArgs;
selfFunc selfFuncPtr;
typedef uint8 (*do_cmd_t)(enum data_src src,uint8 *atCmd);
typedef struct cmdExecMap{
        char *cmdStr;
        do_cmd_t do_cmd;
}cmdExecMap_t;

const struct cmdExecMap atCmdArray[]=
{
        {"RESTART",doRestart},
        {"ROLE",doRole},
        {"FSM",doFsm},
        {"VERSION",doVersion},
        {"TXPOWER",doTxpower},
        {"IBEACONS",doIbeacons},
        {"UART",doUart},
        {"CURRUART",doCurrUart},
        {"CMODE",doCmode},
        {"BIND",doBind},
        {"VERSION",doVersion},
        {"LADDR",doLaddr},
        {"SERIAL",doSerial},
        {"KEY",doKey},
        {"MAC",doMac},
        {"SN",doSn},
        {"MIN_INTERVAL",doMinInterval},
        {"MAX_INTERVAL",doMaxInterval},
        {"NAME",doName},
        {"RESOURCE",doResource},
        {"USB_MONITOR",doUsbdebug},
        {"RTRANS",doRtrans},
        {"BLUNODEBUG",doBlunodebug},
        {"SETTING",doSetting},
        {"RSSI",doRssi},
        {"MAJOR",doMajor},
        {"MINOR",doMinor},
        {"PASSWORD",doPassword},
        {"EXIT",doExit},
        {"CONMAX",doConMax},
        {"NETWORK",doNetwork},
        {"P",doPort}
};


uint8 osal_memcmp(const void GENERIC *src1, const void GENERIC *src2, unsigned int len){
        const uint8 GENERIC *pStrc1;
        const uint8 GENERIC *pStrc2;

        pStrc1 = src1;
        pStrc2 = src2;

        while(len--){
                if(*pStrc1++ != *pStrc2++){
                        return FALSE;
                }
        }
        return TRUE;
}

void *osal_memcpy( void *dst, const void GENERIC *src, unsigned int len )
{
  uint8 *pDst;
  const uint8 GENERIC *pSrc;

  pSrc = src;
  pDst = dst;

  while ( len-- )
    *pDst++ = *pSrc++;

  return ( pDst );
}

static void my_ad_nvms_write(uint32_t addr, const uint8_t *buf, uint32_t size){
        nvms_t NVID_SYS_CONFIG;
        NVID_SYS_CONFIG = ad_nvms_open(NVMS_PARAM_PART);
        ad_nvms_write(NVID_SYS_CONFIG, addr, buf, size);
}

void judgmentAT(enum data_src src, uint8 *buf,uint16_t len){
        if(buf[0] == '+' && buf[1] == '+' && buf[2] == '+'){
                si.at_mode = 1;
                myUARTWrite(src, 0, "Enter AT Mode\r\n", strlen("Enter AT Mode\r\n"));
        }
}
void myUARTWrite(enum data_src src,uint8 port,const void *buf,uint16 len){
        if(src == myUSB){
                myserialusb((uint8_t*)buf, len);
        }
        else if(src == myCOM){
                hw_uart_send(HW_UART2, buf, len, NULL, NULL);
        }
        else if(src == myBLE){
            //返回给DFB1  已经废弃
           // bleSerialWrite(0,buf,len);
        }else if(src == myBLE_AT){
            //返回给DFB2
           // bleAtSerialWrite(0,buf,len);
        }
}

uint8 sendLongString(void *arg)
{
        dataTransferT *pTransArgs = (dataTransferT *)arg;
        enum data_src src = (enum data_src)pTransArgs->priv; //BLE USB COM
        uint8 *srcAddr = (uint8 *)pTransArgs->startAddr; //src addr
        uint8 *startAddr = srcAddr + pTransArgs->offset;
        uint16_t len;
        if(pTransArgs->offset < strlen((char const *)srcAddr)){
                len = strlen((char const *)startAddr)>20?20:strlen((char const *)startAddr);
                myUARTWrite(src,0,(unsigned char *)startAddr,len);
                pTransArgs->offset += len;
        }
        if(pTransArgs->offset == strlen((char const *)srcAddr)){
                return FINISH;
        }
        return CONTINUE;
}

uint8 BLETriggerTransfer(void *arg){
        return SUCCESS;
}

uint8 getResult(uint8 *buf){
      uint8 i;
      for(i = 0; i<AT_RESULT_NUM; i++){
              if(osal_memcmp((char *)atResult[i],(char *)buf,(int)strlen((char *)atResult[i]))){
                      break;
              }
      }
      return i;
}

void getAtCmdStr(uint8 *src,uint8 *cmdBuf)
{
    int len=0;
    while((*src != ' ') && (*src != '=') && (*src != '+') && (*src != '\r') && (*src != '\n') && (len < 20)){
        *cmdBuf = *src;
        src++;cmdBuf++;
        len++;
    }
    *cmdBuf = 0;
}

uint8 execAtCmd(enum data_src src,uint8 *atCmd)
{
        uint8 *p=atCmd,i;
        uint8 cmd[20];
    /*if(si.sys_cfg.bluno_debug == '1'){
        myUARTWrite(src,0,at_cmd,strlen((char const *)at_cmd));
    }*/
    if(p[0]!='A' || p[1]!='T'){
        return FAILURE;
    }
    if((p[2] == 0x0D) && (p[3] == 0x0A)){
        return SUCCESS;
    }else if(p[2] != '+'){
        return FAILURE;
    }
    //获取param
    p+=3;
    getAtCmdStr(p,cmd);
    for(i=0;i<(sizeof(atCmdArray)/sizeof(cmdExecMap_t));i++){

        if(memcmp((char *)atCmdArray[i].cmdStr,cmd,(int)strlen(atCmdArray[i].cmdStr)) == 0){
            if(atCmdArray[i].do_cmd != NULL){
                p += (int)strlen(atCmdArray[i].cmdStr);
                return atCmdArray[i].do_cmd(src,p);
            }
        }

    }
    return FAILURE;
}
/*
src 0 usb
src 1 com
src 2 ble
src 3 ble_at
*/
uint8 atParser(enum data_src src, uint8 *buf, uint8 len){
        static uint8 comIndex = 0;
        uint8 i = 0,comFinish = 0;
        if(src == myCOM || src == myUSB){
                while(len--){
                        atComCmd[comIndex] = buf[i];
                        if((atComCmd[(comIndex-1)%AT_BUF_LEN] == 0x0D) &&
                                (atComCmd[comIndex%AT_BUF_LEN]) == 0x0A){
                                if(comIndex < AT_BUF_LEN-1){
                                        atComCmd[(comIndex+1)%AT_BUF_LEN] = 0;
                                        comFinish = 1;
                                        comIndex = 0;
                                        break;
                                }
                        }
                        if(++comIndex == AT_BUF_LEN){
                                comIndex = 0;
                        }
                        i++;
                }
                if(comFinish){
                        uint8 ret;
                        ret = execAtCmd(src,atComCmd);
                        if(ret == SUCCESS){
                                myUARTWrite(src,0,"OK\r\n",strlen("OK\r\n"));
                        }else if(ret == FAILURE){
                                myUARTWrite(src,0,"ERROR CMD\r\n",strlen("ERROR CMD\r\n"));
                        }
                        memset(atComCmd,0,AT_BUF_LEN);
                }

        }
        return SUCCESS;
}



#if 0
uint8_t atParser(enum data_src src, uint8_t *buf, uint8_t len){
        static uint8_t comIndex=0, bleIndex=0, usbIndex=0;
        uint8_t i=0, comFinish=0, bleFinish=0, usbFinish=0;
        if(src == myCOM){
                printf("src == myCOM\r\n");
                while(len--){
                        atComCmd[comIndex] = buf[i];
                        if((atComCmd[(comIndex-1)%AT_BUF_LEN] == 0x0D) &&
                                (atComCmd[comIndex%AT_BUF_LEN]) == 0x0A){
                                printf("comIndex<AT_BUF_LEN\r\n");
                                if(comIndex<AT_BUF_LEN-1){
                                        printf("comIndex<AT_BUF_LEN-1\r\n");
                                        atComCmd[(comIndex+1)%AT_BUF_LEN]=0;
                                        comFinish = 1;
                                        printf("comFinish\r\n");
                                        comIndex = 0;
                                        break;
                                }
                        }
                        if(++comIndex == AT_BUF_LEN){
                                comIndex = 0;
                        }
                        i++;
                }
                if(comFinish){
                        uint8_t ret;
                        ret = execAtCmd(src,atComCmd);
                        if(ret == SUCCESS){
                                myUARTWrite(src,0,"OK\r\n",strlen("OK\r\n"));
                        }else if(ret == FAILURE){
                                myUARTWrite(src,0,"ERROR CMD\r\n",strlen("ERROR CMD\r\n"));
                        }
                        memset(atComCmd,0,AT_BUF_LEN);
                }
                return SUCCESS;
        }else if(src == myUSB){
                while(len--){
                        atUSBCmd[usbIndex] = buf[i];
                        if((atUSBCmd[(usbIndex-1)%AT_BUF_LEN] == 0x0D) &&
                                (atUSBCmd[usbIndex%AT_BUF_LEN] == 0x0A)){
                                if(usbIndex<AT_BUF_LEN-1){
                                        atUSBCmd[(usbIndex+1)%AT_BUF_LEN]=0;
                                        usbFinish = 1;
                                        usbIndex = 0;
                                        break;
                                }
                        }
                        if(++usbIndex == AT_BUF_LEN){
                                usbIndex = 0;
                        }
                        i++;
                }
                if(usbFinish){
                        uint8_t ret;
                        ret = execAtCmd(src,atUSBCmd);
                        if(ret == SUCCESS){
                                myUARTWrite(src,0,"OK\r\n",strlen("OK\r\n"));
                        }else if(ret == FAILURE){
                                myUARTWrite(src,0,"ERROR CMD\r\n",strlen("ERROR CMD\r\n"));
                        }
                        memset(atUSBCmd,0,AT_BUF_LEN);
                }
                return SUCCESS;
        }else{
                while(len--){
                        atBLECmd[bleIndex] = buf[i];
                        if((atBLECmd[(bleIndex-1)%AT_BUF_LEN] == 0x0D) &&
                                (atBLECmd[bleIndex%AT_BUF_LEN] == 0x0A)){
                                if(bleIndex<AT_BUF_LEN-1){
                                        atBLECmd[(bleIndex+1)%AT_BUF_LEN]=0;
                                        bleFinish = 1;
                                        bleIndex = 0;
                                        break;
                                }
                        }
                        if(++bleIndex == AT_BUF_LEN){
                                bleIndex = 0;
                        }
                        i++;
                }
                if(bleFinish){
                        uint8_t ret;
                        ret = execAtCmd(src,atBLECmd);
                        if(ret == SUCCESS){
#ifdef DBGBLEAT
                                if(si.usb_serial_open && (si.sys_cfg.usb_debug=='1')){
                                         myUARTWrite(USB,0,atBLECmd,strlen((const char *)atBLECmd));
                                }
#endif
                                myUARTWrite(src,0,"OK\r\n",strlen("OK\r\n"));
                        }else if(ret == FAILURE){
#ifdef DBGBLEAT
                                if(si.usb_serial_open && (si.sys_cfg.usb_debug=='1')){
                                         myUARTWrite(USB,0,atBLECmd,strlen((const char *)atBLECmd));
                                }
#endif
                                myUARTWrite(src,0,"ERROR CMD\r\n",strlen("ERROR CMD\r\n"));
                        }
                        memset(atBLECmd,0,AT_BUF_LEN);
                }
                return SUCCESS;
        }
}

#endif

uint8_t doRestart(enum data_src src,uint8_t *atCmd)
{
    return SUCCESS;
}
uint8_t doRole(enum data_src src,uint8_t *atCmd)
{
    uint8_t *p = atCmd;
    if(*p != '='){
            return FAILURE;
    }else{
            p++;
            if(*p == '?'){
                    if(si.sys_cfg.curr_role == ROLE_PERIPHERAL){
                            myUARTWrite(src,0,"ROLE_PERIPHERAL\r\n",strlen("ROLE_PERIPHERAL\r\n"));

                    }else if (si.sys_cfg.curr_role == ROLE_CENTRAL){
                            myUARTWrite(src,0,"ROLE_CENTRAL\r\n",strlen("ROLE_CENTRAL\r\n"));
                    }else{
                            myUARTWrite(src,0,"ROLE_CENTRAL | ROLE_PERIPHERAL\r\n",strlen("ROLE_CENTRAL | ROLE_PERIPHERAL\r\n"));
                    }
                    return FINISH;
            }else{
                    uint8 result_index;
                    result_index = getResult(p);
                    if(si.pass == 0){
                            return FAILURE;
                    }
                    if(result_index == AT_RESULT_NUM){
                            return -1;
                    }else{
                            if(result_index == 1){
                                    si.sys_cfg.curr_role = ROLE_PERIPHERAL | ROLE_PERIPHERAL;
                            }else if(result_index == 2){
                                    si.sys_cfg.curr_role = ROLE_CENTRAL;
                            }else{
                                    return FAILURE;
                            }
                            si.sys_cfg.setting = 'u';
                            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                            return SUCCESS;
                    }
            }
    }
    return SUCCESS;
}
uint8_t doFsm(enum data_src src,uint8_t *atCmd)
{
        uint8_t *p =atCmd;
        if(*p != '='){
                return FAILURE;
        }else{
                p++;
                if(*p == '?'){
                        if(si.sys_cfg.final_sm == FSM_INIT){
                                myUARTWrite(src, 0, "FSM_INIT\r\n",strlen("FSM_INIT\r\n"));
                        }else if(si.sys_cfg.final_sm == FSM_USB_COM){
                                myUARTWrite(src, 0, "FSM_USB_COM\r\n",strlen("FSM_USB_COM\r\n"));
                        }else if(si.sys_cfg.final_sm == FSM_COM_BLE_AT){
                                myUARTWrite(src, 0, "FSM_COM_BLE_AT\r\n", strlen("FSM_COM_BLE_AT\r\n"));
                        }else if(si.sys_cfg.final_sm == FSM_USB_COM_BLE_AT){
                                myUARTWrite(src, 0, "FSM_USB_COM_BLE_AT\r\n", strlen("FSM_USB_COM_BLE_AT\r\n"));
                        }else if(si.sys_cfg.final_sm == FSM_TRANS_USB_BLE){
                                myUARTWrite(src, 0, "FSM_TRANS_USB_BLE\r\n", strlen("FSM_TRANS_USB_BLE\r\n"));
                        }else if(si.sys_cfg.final_sm == FSM_TRANS_COM_BLE){
                                myUARTWrite(src, 0, "FSM_TRANS_COM_BLE\r\n", strlen("FSM_TRANS_COM_BLE\r\n"));
                        }else if(si.sys_cfg.final_sm == FSM_TRANS_USB_COM_BLE){
                                myUARTWrite(src, 0, "FSM_TRANS_USB_COM_BLE\r\n", strlen("FSM_TRANS_USB_COM_BLE\r\n"));
                        }else if(si.sys_cfg.final_sm == FSM_HID_USB_COM_BLE_AT){
                                myUARTWrite(src, 0, "FSM_HID_USB_COM_BLE_AT\r\n", strlen("FSM_HID_USB_COM_BLE_AT\r\n"));
                        }else{
                                return FAILURE;
                        }
                        return FINISH;
                }else{
                        uint8 result_index;
                        result_index = getResult(p);
                        p+=(int)strlen((char *)atResult[result_index]);
                        if((*p != 0x0d) || (*(p+1) != 0x0a)){
                                printf("(*p != 0x0d) || (*(p+1) != 0x0a)\r\n");
                                return FAILURE;
                        }
                        if(result_index == AT_RESULT_NUM){
                                return FAILURE;
                        }else if(result_index == 3){
                                si.sys_cfg.final_sm = FSM_USB_COM_BLE_AT;
                        }else if(result_index == 4){
                                si.sys_cfg.final_sm = FSM_COM_BLE_AT;
                        }else if(result_index == 5){
                                si.sys_cfg.final_sm = FSM_USB_COM;
                        }else if(result_index == 6){
                                si.sys_cfg.final_sm = FSM_TRANS_USB_BLE;
                        }else if(result_index == 7){
                                si.sys_cfg.final_sm = FSM_TRANS_COM_BLE;
                        }else if(result_index == 8){
                                si.sys_cfg.final_sm = FSM_TRANS_USB_COM_BLE;
                        }else if(result_index == 9){
                                si.sys_cfg.final_sm = FSM_HID_USB_COM_BLE_AT;
                        }else{
                                return FAILURE;
                        }
                        si.sys_cfg.setting = 'u';
                        my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                        return SUCCESS;
                }
        }

}
uint8_t doVersion(enum data_src src,uint8_t *atCmd)
{
        uint8 *p = atCmd;
        if(*p != '='){
                return FAILURE;
        }
        p++;
        if(*p == '?'){
                myUARTWrite(src, 0, si.sys_cfg.version,sizeof(si.sys_cfg.version));
                myUARTWrite(src, 0, "\r\n", 2);
                return FINISH;
        }else{
                return FAILURE;
        }
}
uint8_t doTxpower(enum data_src src,uint8_t *atCmd)
{
        uint8 *p = atCmd;
        if(*p != '='){
                return FAILURE;
        }
        p++;
        if(*p == '?'){
                if(si.sys_cfg.txpower-'0' == TX_POWER_LEVEL_MAX ){
                        myUARTWrite(src, 0, "MAX\r\n",strlen("MAX\r\n"));
                }else if(si.sys_cfg.txpower-'0' == TX_POWER_LEVEL_CURRENT){
                        myUARTWrite(src, 0, "LOW\r\n",strlen("LOW\r\n"));
                }
        }else{
                if(strcmp((char const *)p,"MAX\r\n") == 0){
                        si.sys_cfg.txpower = TX_POWER_LEVEL_MAX + '0';
                        return SUCCESS;
                }else if(strcmp((char const *)p,"LOW\r\n") == 0){
                        si.sys_cfg.txpower = TX_POWER_LEVEL_CURRENT + '0';
                        return SUCCESS;
                }else{
                        return FAILURE;
                }
        }
        return FINISH;
}
uint8 doIbeacons(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
            printf("*p != '='\r\n");
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        if(si.sys_cfg.ibeacons == '0'){
            myUARTWrite(src,0,"OFF",strlen("OFF"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }else if(si.sys_cfg.ibeacons == '1'){
            myUARTWrite(src,0,"ON",strlen("ON"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }
    }else{
        if(strcmp((char const *)p,"ON\r\n") == 0){
            si.sys_cfg.ibeacons = '1';
            si.sys_cfg.setting = 'u';
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
            return SUCCESS;
        }else if(strcmp((char const *)p,"OFF\r\n") == 0){
            si.sys_cfg.ibeacons = '0';
            si.sys_cfg.setting = 'u';
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
            return SUCCESS;
        }else{
            return FAILURE;
        }
    }
    return SUCCESS;
}
uint8 doUart(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        //返回 currentLineCoding
        uint8 buf[20],len=0;
        if(si.sys_cfg.lineCoding.DTERate == 9600){
          osal_memcpy(buf,"9600",4);
          len = 4;
        }else if(si.sys_cfg.lineCoding.DTERate == 19200){
          osal_memcpy(buf,"19200",5);
          len = 5;
        }else if(si.sys_cfg.lineCoding.DTERate == 38400){
          osal_memcpy(buf,"38400",5);
          len = 5;
        }else if(si.sys_cfg.lineCoding.DTERate == 57600){
          osal_memcpy(buf,"57600",5);
          len = 5;
        }else if(si.sys_cfg.lineCoding.DTERate == 115200){
          osal_memcpy(buf,"115200",6);
          len = 6;
        }
        if(len == 0){
            return FAILURE;
        }

        buf[len++] = ',';
        buf[len++] = currentLineCoding.CharFormat+'8';
        buf[len++] = ',';
        buf[len++] = currentLineCoding.ParityType+'N';
        buf[len++] = ',';
        buf[len++] = currentLineCoding.ParityType+'1';
        buf[len++] = 0x0d;
        buf[len++]   = 0x0a;
        myUARTWrite(src,0,buf,len);
        return FINISH;
    }else{
        uint32 dteRate=0;
        uint8  charFormat=0,parityType=0;
        while((*p>='0') && (*p<='9')){
            dteRate = dteRate*10 + (*p - '0');
            p++;
        }
        if((*p != '\r') || (*(p+1) != '\n')){
            if(*p!=','){
                return FAILURE;
            }
            p++;
            if((*p>='0') && (*p<='2'))
            {
                charFormat = *p-'0';
                p++;
            }
            if(*p!=',')
            {
                return FAILURE;
            }
            p++;
            if((*p>='0') && (*p<='2'))
            {
                parityType = *p-'0';
                p++;
            }
        }
        if((*p == 0x0d) && (*(p+1)==0x0a)){
                /*
                if(dteRate == 9600){
                        currentLineCoding.DTERate = HW_UART_BAUDRATE_9600;
                }else if(dteRate == 19200){
                        currentLineCoding.DTERate = HW_UART_BAUDRATE_19200;
                }
                osal_memcpy(&si.sys_cfg.lineCoding,&currentLineCoding,sizeof(USB_CDC_LINE_CODING));
                si.sys_cfg.setting = 'u';
                my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                */
            if((dteRate == 9600) ||
              (dteRate == 19200) ||
              (dteRate == 38400) ||
              (dteRate == 57600) ||
              (dteRate == 115200)){
                  currentLineCoding.DTERate = dteRate;
                  currentLineCoding.CharFormat = charFormat;
                  currentLineCoding.ParityType = parityType;
                  currentLineCoding.DataBits = 8;
                  osal_memcpy(&si.sys_cfg.lineCoding,&currentLineCoding,sizeof(USB_CDC_LINE_CODING));
                  si.sys_cfg.setting = 'u';
                  my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
              }else{
                  return FAILURE;
              }

        }else{
            return FAILURE;
        }
        //设置currentLineCoding
    }
    return SUCCESS;
}

uint8 doCurrUart(enum data_src src,uint8 *atCmd)
{
#if 0
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        //返回 currentLineCoding
        uint8 buf[20],len=0;
        if(currentLineCoding.dteRate == 9600){
          osal_memcpy(buf,"9600",4);
          len = 4;
        }else if(currentLineCoding.dteRate == 19200){
          osal_memcpy(buf,"19200",5);
          len = 5;
        }else if(currentLineCoding.dteRate == 38400){
          osal_memcpy(buf,"38400",5);
          len = 5;
        }else if(currentLineCoding.dteRate == 57600){
          osal_memcpy(buf,"57600",5);
          len = 5;
        }else if(currentLineCoding.dteRate == 115200){
          osal_memcpy(buf,"115200",6);
          len = 6;
        }
        if(len == 0){
            return FAILURE;
        }

        buf[len++] = ',';
        buf[len++] = currentLineCoding.charFormat+'8';
        buf[len++] = ',';
        buf[len++] = currentLineCoding.parityType+'N';
        buf[len++] = ',';
        buf[len++] = currentLineCoding.parityType+'1';
        buf[len++] = 0x0d;
        buf[len++]   = 0x0a;
        myUARTWrite(src,0,buf,len);
        return FINISH;
    }else{
        uint32 dteRate=0;
        uint8  charFormat=0,parityType=0;
        while((*p>='0') && (*p<='9')){
            dteRate = dteRate*10 + (*p - '0');
            p++;
        }
        if((*p != '\r') || (*(p+1) != '\n')){
            if(*p!=','){
                return FAILURE;
            }
            p++;
            if((*p>='0') && (*p<='2')){
                charFormat = *p-'0';
                p++;
            }
            if(*p!=','){
                return FAILURE;
            }
            p++;
            if((*p>='0') && (*p<='2')){
                parityType = *p-'0';
                p++;
            }
        }
        if((*p == 0x0d) && (*(p+1)==0x0a)){
            if((dteRate == 9600) ||
              (dteRate == 19200) ||
              (dteRate == 38400) ||
              (dteRate == 57600) ||
              (dteRate == 115200)){
                  currentLineCoding.dteRate = dteRate;
                  currentLineCoding.charFormat = charFormat;
                  currentLineCoding.parityType = parityType;
                  currentLineCoding.dataBits = 8;
              }else{
                  return FAILURE;
              }

        }else{
            return FAILURE;
        }
        //设置currentLineCoding
        serialAppInitTransport();
    }
#endif
    return SUCCESS;
}
uint8 doCmode(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        //返回 cmode
        uint8 buf[15];
        if(si.sys_cfg.cmode == 'u'){
            strcpy((char *)buf,"UNIQUE\r\n");
        }else{
            strcpy((char *)buf,"ANYONE\r\n");
        }
        myUARTWrite(src,0,buf,strlen((char const*)buf));
        return FINISH;
    }else{
        uint8 cmode_tmp=-1;
        if(osal_memcmp(p,"ANYONE",strlen("ANYONE"))){
            cmode_tmp = 'a';
            p+=strlen("ANYONE");
        }else if(osal_memcmp(p,"UNIQUE",strlen("UNIQUE"))){
            cmode_tmp = 'u';
            p+=strlen("UNIQUE");
        }else{
            return FAILURE;
        }
        if((p[0] == '\r') && (p[1] == '\n')){
            si.sys_cfg.cmode = cmode_tmp;
            si.sys_cfg.setting = 'u';
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
            return SUCCESS;
        }else{
            return FAILURE;
        }

        //设置currentLineCoding
    }
}

uint8 doBind(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        if((int)strlen((char *)si.sys_cfg.peerAddr) == B_ADDR_STR_LEN-1){
            myUARTWrite(src,0,si.sys_cfg.peerAddr,B_ADDR_STR_LEN-1);
        }else{
            myUARTWrite(src,0,"0x000000000000",B_ADDR_STR_LEN-1);
        }
        myUARTWrite(src,0,"\r\n",2);
        return FINISH;
        //返回 bind addr
    }else{//BIND
        if((*p=='0')&& (*(p+1) == 'x') && (*(p+14) == 0x0D) && (*(p+15) == 0x0A)){
            osal_memcpy(si.sys_cfg.peerAddr,p,B_ADDR_STR_LEN-1);
            si.sys_cfg.peerAddr[B_ADDR_STR_LEN-1] = 0;
            si.sys_cfg.setting = 'u';
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
            return SUCCESS;
        }else{
            return FAILURE;
        }
        //设置 bind addr
    }
}
uint8 doLaddr(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        myUARTWrite(src,0,local_addr,B_ADDR_STR_LEN-1);
        myUARTWrite(src,0,"\r\n",2);
        return FINISH;
    }else{  //ERROR
        return FAILURE;
    }
}
uint8 doSerial(enum data_src src,uint8 *atCmd)
{
#if 0
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }else{
        uint8 len=0;
        p++;
        while((p[len]!=0x0d) || (p[len+1]!=0x0a))
        {
          len++;
        }
        if((len>2) && (p[len-2] == '\n') && (p[len-1] == '\r')){
            p[len-2] = '\r';
            p[len-1] = '\n';
        }
        #ifndef SLAVE_ONLY
        if((si.curr_role == ROLE_CENTRAL) && (simpleBLEConnHandle != GAP_CONNHANDLE_INIT)){
            bleSerialWrite(0,p,len);
        }else
        #endif
        if((si.curr_role == ROLE_PERIPHERAL) && (gapRole_state == GAPROLE_CONNECTED)){
            bleSerialWrite(0,p,len);
        }else{
            if(src == USB){
                if(HalPrivUARTWrite(0,p,len)==0){
                    ucappEnqueue(p,len);
                }
            }else if(src == COM){
                cuappEnqueue(p,len);
            }
            return SUCCESS;
        }
    }
#endif
    return SUCCESS;
}
uint8 doKey(enum data_src src,uint8 *atCmd)
{
#if 0
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }else{
        uint8  key_value1=0;
        uint8  key_value2=0;
        uint8  key_value3=0;
        p++;
        while(*p>='0' && *p<='9'){
            key_value1 = key_value1*10 + (*p++ -'0');
        }
        if(*p == '+'){
            p++;
            while(*p>='0' && *p<='9'){
                key_value2 = key_value2*10 + (*p++ -'0');
            }
        }
        if(*p == '+'){
            p++;
            while(*p>='0' && *p<='9'){
                key_value3 = key_value3*10 + (*p++ -'0');
            }
        }
        if(si.curr_role == ROLE_PERIPHERAL){
            if(si.curr_sm & HID_OPEN){
                if(key_value2 == 0 ){
                    hidEmuKbdSendReport( key_value1 );
                }else if(key_value3 == 0){
                    hidEmuKbdSendReport2( key_value1,key_value2 );
                }else{
                    hidEmuKbdSendReport3( key_value1,key_value2,key_value3 );
                }
          }else{
              attHandleValueInd_t attValInd;
              attValInd.handle = 0;
              attValInd.len = 4;
              osal_memcpy(attValInd.value,&key_value1,sizeof(uint16));
              GATT_Indication( 0, &attValInd, FALSE, simpleBLERole_TaskID );
          }
        }/*else if(si.curr_role == ROLE_CENTRAL){
            attWriteReq_t keyReport;
            keyReport.handle = simpleBLEKeyHdl;               //!< Handle of the attribute to be written (must be first field)
            osal_memcpy(keyReport.value,&key_value1,sizeof(uint16));
            keyReport.len = sizeof(uint16);                   //!< Length of value
            keyReport.sig = 0;                   //!< Authentication Signature status (not included (0), valid (1), invalid (2))
            keyReport.cmd = 0;                   //!< Command Flag
            currValue = NULL;
            GATT_WriteCharValue(simpleBLEConnHandle, &keyReport, simpleBLERole_TaskID);
        }*/
    }
#endif
    return SUCCESS;
}
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
                /*
                                else if(buf[i]<='f' && buf[i]>='a'){
                                        address[i]=buf[i]-87;
                                }
                                */
        }
        for(j=0; j<12; j++){
              if(j%2==0){
                      last_address[z]=address[j]*16+address[j+1];
                      z++;
              }
        }
        return last_address;
}
uint8 doMac(enum data_src src,uint8 *atCmd)
{

    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
            uint8 addr_ascii[]={"0x000000000000\r\n"};
            uint8 mac_addr_ascii[B_ADDR_LEN];

            for(int i=0;i<6;i++){
                    mac_addr_ascii[i]=si.sys_cfg.my_mac[i];
            }

            //uint8 mac_addr_ascii[B_ADDR_LEN] = transfer_address(si.sys_cfg.peerAddr+2);
            bdAddr2Str(addr_ascii, mac_addr_ascii );
            addr_ascii[14]='\r';addr_ascii[15]='\n';
            myUARTWrite(src,0,addr_ascii,16);
            return FINISH;
    }else{
            if(first_mac == 0){//
                    if((*p=='0')&& (*(p+1) == 'x') && (*(p+14) == 0x0D) && (*(p+15) == 0x0A)){
                            static uint8 myaddr[15];
                            osal_memcpy(myaddr,p,B_ADDR_STR_LEN-1);
                            myaddr[B_ADDR_STR_LEN-1] = 0;
                            si.sys_cfg.my_mac[0]=transfer_address(myaddr+2)[5];
                            si.sys_cfg.my_mac[1]=transfer_address(myaddr+2)[4];
                            si.sys_cfg.my_mac[2]=transfer_address(myaddr+2)[3];
                            si.sys_cfg.my_mac[3]=transfer_address(myaddr+2)[2];
                            si.sys_cfg.my_mac[4]=transfer_address(myaddr+2)[1];
                            si.sys_cfg.my_mac[5]=transfer_address(myaddr+2)[0];

                            /*
                            for(int i=0;i<6;i++){
                                    si.sys_cfg.my_mac[i]=transfer_address(myaddr+2)[5-i];
                            }
                            */
                            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                            return SUCCESS;
                    }
            }else{
                    return FAILURE;
            }
    }

}

uint8 doSn(enum data_src src,uint8 *atCmd)
{
#if 0
    uint8 *p = atCmd;
    uint8 len=0,i,tmp;
    uint8 *sn = df_osal_mem_alloc(34);
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
      if(si.pass == 0){
        myUARTWrite(src,0,"OFF",3);
      }else{
        myUARTWrite(src,0,"ON",2);
      }
      myUARTWrite(src,0,"\r\n",2);
      return FINISH;
    }
    while(*p != '\r' && len < 34){
        sn[len++] = *p++;
    }
    sn[len] = 0;
    #define ascii2num(x) (((x)>='0'&& (x)<='9')?(x)-'0':(((x)>='a'&& (x)<='f')?(x)-'a'+0xa:0xf))
    for(i = 0; i<9; i++){
        tmp = (ascii2num(sn[i*2+1])) | (ascii2num(sn[i*2])<<4);
        si.sys_cfg.md5[i] = tmp;
    }
    osal_mem_free(sn);
    osal_snv_write(NVID_SYS_CONFIG,sizeof(sys_config_t),&si.sys_cfg);
#endif
    return SUCCESS;
}

uint8 doMinInterval(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }else{
        p++;
        if(*p == '?'){//get MIN_INTERVAL
          uint8 ascii_interval[7]={0};
            uint16 min_interval = (uint16)(si.sys_cfg.min_interval*1.25);
            if(min_interval>=10000){
              ascii_interval[0] = (min_interval/10000)%10+'0';
              ascii_interval[1] = (min_interval/1000)%10+'0';
              ascii_interval[2] = (min_interval/100)%10+'0';
              ascii_interval[3] = (min_interval/10)%10+'0';
              ascii_interval[4] = min_interval%10+'0';
              ascii_interval[5] = '\r';
              ascii_interval[6] = '\n';
            }else if(min_interval>=1000){
              ascii_interval[0] = (min_interval/1000)%10+'0';
              ascii_interval[1] = (min_interval/100)%10+'0';
              ascii_interval[2] = (min_interval/10)%10+'0';
              ascii_interval[3] = min_interval%10+'0';
              ascii_interval[4] = '\r';
              ascii_interval[5] = '\n';
            }else if(min_interval>=100){
              ascii_interval[0] = min_interval/100+'0';
              ascii_interval[1] = (min_interval/10)%10+'0';
              ascii_interval[2] = min_interval%10+'0';
              ascii_interval[3] = '\r';
              ascii_interval[4] = '\n';
            }else if(min_interval>=10){
              ascii_interval[0] = (min_interval/10)%10+'0';
              ascii_interval[1] = min_interval%10+'0';
              ascii_interval[2] = '\r';
              ascii_interval[3] = '\n';
            }else{
              ascii_interval[0] = min_interval%10+'0';
              ascii_interval[1] = '\r';
              ascii_interval[2] = '\n';
            }
            myUARTWrite(src,0,ascii_interval,strlen((char const*)ascii_interval));
            return FINISH;
        }else{//set MIN_INTERVAL
            uint16 min_interval = 0;

            while(*p>='0' && *p<='9'){
                min_interval = min_interval*10 + (*p++ -'0');
            }
            if((min_interval<10) || (min_interval>60000)){
                return FAILURE;
            }
            if((*p != '\r') || (*(p+1) != '\n')){
                return FAILURE;
            }
            si.sys_cfg.min_interval_ms = min_interval;
            si.sys_cfg.min_interval = (uint16)(min_interval/1.25);
            si.sys_cfg.setting = 'u';
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
        }
    }
    return SUCCESS;
}

uint8 doMaxInterval(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }else{
        p++;
        if(*p == '?'){//get CONN_INTERVAL
          uint8 ascii_interval[7]={0};
            uint16 max_interval = (uint16)(si.sys_cfg.max_interval*1.25);
            if(max_interval>=10000){
              ascii_interval[0] = (max_interval/10000)%10+'0';
              ascii_interval[1] = (max_interval/1000)%10+'0';
              ascii_interval[2] = (max_interval/100)%10+'0';
              ascii_interval[3] = (max_interval/10)%10+'0';
              ascii_interval[4] = max_interval%10+'0';
              ascii_interval[5] = '\r';
              ascii_interval[6] = '\n';
            }else if(max_interval>=1000){
              ascii_interval[0] = (max_interval/1000)%10+'0';
              ascii_interval[1] = (max_interval/100)%10+'0';
              ascii_interval[2] = (max_interval/10)%10+'0';
              ascii_interval[3] = max_interval%10+'0';
              ascii_interval[4] = '\r';
              ascii_interval[5] = '\n';
            }else if(max_interval>=100){
              ascii_interval[0] = (max_interval/100)+'0';
              ascii_interval[1] = (max_interval/10)%10+'0';
              ascii_interval[2] = max_interval%10+'0';
              ascii_interval[3] = '\r';
              ascii_interval[4] = '\n';
            }else if(max_interval>=10){
              ascii_interval[0] = (max_interval/10)%10+'0';
              ascii_interval[1] = max_interval%10+'0';
              ascii_interval[2] = '\r';
              ascii_interval[3] = '\n';
            }else{
              ascii_interval[0] = max_interval%10+'0';
              ascii_interval[1] = '\r';
              ascii_interval[2] = '\n';
            }
            myUARTWrite(src,0,ascii_interval,strlen((char const*)ascii_interval));
            return FINISH;
        }else{//set MAX_INTERVAL
            uint16 max_interval = 0;

            while(*p>='0' && *p<='9'){
                max_interval = max_interval*10 + (*p++ -'0');
            }
            if((max_interval<10) || (max_interval>10000)){
                return FAILURE;
            }
            if((*p != '\r') || (*(p+1) != '\n')){
                return FAILURE;
            }
            si.sys_cfg.max_interval_ms = max_interval;
            si.sys_cfg.max_interval = (uint16)(max_interval/1.25);
            si.sys_cfg.setting = 'u';
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
        }
    }
    return SUCCESS;
}

uint8 doName(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
      }
      p++;
      if(*p == '?'){
          myUARTWrite(src,0,si.sys_cfg.name,strlen((char const*)si.sys_cfg.name));
          myUARTWrite(src,0,"\r\n",2);
          return FINISH;
      }else{
          uint8 len=0;
          while((len<14) && (*p != '\r')){
              si.sys_cfg.name[len++] = *p++;
          }
          si.sys_cfg.name[len] = 0;
          si.sys_cfg.setting = 'u';
          my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
          return SUCCESS;
      }
}

uint8 doResource(enum data_src src,uint8 *atCmd)
{
#if 0
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        transArg.startAddr = (void *)resource;
        transArg.offset = 0;
        selfFuncPtr = sendLongString;
        selfFuncArgs = (void *)&transArg;
        osal_start_timerEx( simpleBLERole_TaskID, SBP_SELF_FUNCTION, si.conn_interval_ms);
        return FINISH;
    }
#endif
    return SUCCESS;
}

uint8 doUsbdebug(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        if(si.sys_cfg.usb_debug == '0'){
            myUARTWrite(src,0,"OFF",strlen("OFF"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }else if(si.sys_cfg.usb_debug == '1'){
            myUARTWrite(src,0,"ON",strlen("ON"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }
    }else{
        if(strcmp((char const *)p,"ON\r\n") == 0){
            si.sys_cfg.usb_debug = '1';
            si.sys_cfg.setting = 'u';
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
            return SUCCESS;
        }else if(strcmp((char const *)p,"OFF\r\n") == 0){
            si.sys_cfg.usb_debug = '0';
            si.sys_cfg.setting = 'u';
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
            return SUCCESS;
        }else{
                p = 0;
            return FAILURE;
        }
    }
    return SUCCESS;
}

uint8 doSetting(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        if(si.sys_cfg.setting == 'u'){
            myUARTWrite(src,0,"UNKNOWN",strlen("UNKNOWN"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }else if(si.sys_cfg.setting == 'd'){
            myUARTWrite(src,0,"DEFPERIPHERAL",strlen("DEFPERIPHERAL"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }else if(si.sys_cfg.setting == 'h'){
            myUARTWrite(src,0,"DEFHID",strlen("DEFHID"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }else if(si.sys_cfg.setting == 'd'+'c'){
            myUARTWrite(src,0,"DEFCENTRAL",strlen("DEFCENTRAL"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }
    }else{
            if((strcmp((char const *)p,"DEFAULT\r\n") == 0)){
                    si.sys_cfg.curr_role = GAP_CENTRAL_ROLE | GAP_PERIPHERAL_ROLE;
                    my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                    return SUCCESS;
            }else{
                    return FAILURE;
            }
#if 0
        if((strcmp((char const *)p,"DEFAULT\r\n") == 0) || (strcmp((char const *)p,"DEFPERIPHERAL\r\n") == 0)){
             si.sys_cfg.lineCoding.DTERate = 115200;
             si.sys_cfg.final_sm           = FSM_TRANS_USB_COM_BLE;
             si.sys_cfg.min_interval       = 16;
             si.sys_cfg.min_interval_ms    = 20;
             si.sys_cfg.max_interval       = 32;
             si.sys_cfg.max_interval_ms    = 40;
             si.sys_cfg.password[0]        = 0;

             si.sys_cfg.curr_role = ROLE_PERIPHERAL;
             si.sys_cfg.cmode = 'a';//ANYONE
             si.sys_cfg.usb_debug = '0';
             si.sys_cfg.reliable_transfer = '0';
             si.sys_cfg.txpower = TX_POWER_LEVEL_MAX + '0';

             si.sys_cfg.ibeacons = '1'; //ibeacons open
             si.sys_cfg.major = 0;
             si.sys_cfg.minor = 0;
             si.sys_cfg.bluno_debug = '1';
             si.sys_cfg.setting = 'd';//DEFAULT
             memcpy(si.sys_cfg.name,BLE_NAME,strlen((char const *)BLE_NAME)+1);
             memcpy(si.sys_cfg.password,DEFPASSWORD,strlen((char const *)DEFPASSWORD)+1);
             my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
             return SUCCESS;
        }else if(strcmp((char const *)p,"DEFHID\r\n") == 0){
             si.sys_cfg.lineCoding.DTERate = 115200;
             si.sys_cfg.final_sm           = FSM_HID_USB_COM_BLE_AT;
             si.sys_cfg.min_interval       = 16;
             si.sys_cfg.min_interval_ms    = 20;
             si.sys_cfg.max_interval       = 32;
             si.sys_cfg.max_interval_ms    = 40;
             si.sys_cfg.password[0]        = 0;

             si.sys_cfg.curr_role = ROLE_PERIPHERAL;
             si.sys_cfg.cmode = 'a';//ANYONE
             si.sys_cfg.usb_debug = '0';
             si.sys_cfg.reliable_transfer = '0';
             si.sys_cfg.txpower = TX_POWER_LEVEL_MAX + '0';

             si.sys_cfg.ibeacons = '1'; //ibeacons open
             si.sys_cfg.major = 0;
             si.sys_cfg.minor = 0;
             si.sys_cfg.bluno_debug = '1';
             si.sys_cfg.setting = 'h';//DEFHID
             memcpy(si.sys_cfg.name,BLE_NAME,strlen((char const *)BLE_NAME)+1);
             memcpy(si.sys_cfg.password,DEFPASSWORD,strlen((char const *)DEFPASSWORD)+1);
             my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
             return SUCCESS;
        }else if(strcmp((char const *)p,"DEFCENTRAL\r\n") == 0){
             si.sys_cfg.lineCoding.DTERate = 115200;
             si.sys_cfg.final_sm           = FSM_TRANS_USB_COM_BLE;
             si.sys_cfg.min_interval       = 16;
             si.sys_cfg.min_interval_ms    = 20;
             si.sys_cfg.max_interval       = 32;
             si.sys_cfg.max_interval_ms    = 40;
             si.sys_cfg.password[0]        = 0;

             si.sys_cfg.curr_role = ROLE_CENTRAL;
             si.sys_cfg.cmode = 'a';
             si.sys_cfg.usb_debug = '0';
             si.sys_cfg.reliable_transfer = '0';
             si.sys_cfg.txpower = TX_POWER_LEVEL_MAX + '0';

             si.sys_cfg.ibeacons = '1'; //ibeacons open
             si.sys_cfg.major = 0;
             si.sys_cfg.minor = 0;
             si.sys_cfg.bluno_debug = '1';
             si.sys_cfg.setting = 'd'+'c';//DEFAULT CENTRAL
             memcpy(si.sys_cfg.name,BLE_NAME,strlen((char const *)BLE_NAME)+1);
             my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
             return SUCCESS;
        }else{
            return FAILURE;
        }
#endif
    }
    return SUCCESS;
}

uint8 doRtrans(enum data_src src,uint8 *atCmd)
{
#if 0
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        if(si.sys_cfg.reliable_transfer == '0'){
            myUARTWrite(src,0,"OFF",strlen("OFF"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }else if(si.sys_cfg.reliable_transfer == '1'){
            myUARTWrite(src,0,"ON",strlen("ON"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }
    }else{
        if(strcmp((char const *)p,"ON\r\n") == 0){
            si.sys_cfg.reliable_transfer = '1';
            si.sys_cfg.setting = 'u';
            osal_snv_write(NVID_SYS_CONFIG,sizeof(sys_config_t),&si.sys_cfg);
            return SUCCESS;
        }else if(strcmp((char const *)p,"OFF\r\n") == 0){
            si.sys_cfg.reliable_transfer = '0';
            si.sys_cfg.setting = 'u';
            osal_snv_write(NVID_SYS_CONFIG,sizeof(sys_config_t),&si.sys_cfg);
            return SUCCESS;
        }else{
            return FAILURE;
        }
    }
#endif
    return SUCCESS;
}

uint8 doBlunodebug(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        if(si.sys_cfg.bluno_debug == '0'){
            myUARTWrite(src,0,"OFF",strlen("OFF"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }else if(si.sys_cfg.bluno_debug == '1'){
            myUARTWrite(src,0,"ON",strlen("ON"));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }
    }else{
        if(strcmp((char const *)p,"ON\r\n") == 0){
            si.sys_cfg.bluno_debug = '1';
            si.sys_cfg.setting = 'u';
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
            return SUCCESS;
        }else if(strcmp((char const *)p,"OFF\r\n") == 0){
            si.sys_cfg.bluno_debug = '0';
            si.sys_cfg.setting = 'u';
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
            return SUCCESS;
        }else{
            return FAILURE;
        }
    }
    return SUCCESS;
}

uint8 doRssi(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?' && *(p+1) == '\r' && *(p+2) == '\n'){
        uint8 buf[6]={"-000\r\n"};
        if(ble_connect == 1){
                int8_t mybuf;
                int8_t *rssi;
                rssi = &mybuf;
                ble_gap_conn_rssi_get(0,rssi);
                si.rssi = -rssi[0];
                buf[1] = (si.rssi/100)%10 + '0';
                buf[2] = (si.rssi/10)%10 + '0';
                buf[3] = si.rssi%10 + '0';
        }
        myUARTWrite(src,0,buf,6);
        return FINISH;
    }else{
        return FAILURE;
    }
}

uint8 doMajor(enum data_src src,uint8 *atCmd){
    uint8 *p = atCmd;
    if(*p != '='){
            return FAILURE;
    }
    p++;
    if(*p == '?' && *(p+1) == '\r' && *(p+2) == '\n'){
        uint8 buf[7]={"00000\r\n"};
        buf[0] = (si.sys_cfg.major/10000)%10 + '0';
        buf[1] = (si.sys_cfg.major/1000)%10 + '0';
        buf[2] = (si.sys_cfg.major/100)%10 + '0';
        buf[3] = (si.sys_cfg.major/10)%10 + '0';
        buf[4] = si.sys_cfg.major%10 + '0';
        myUARTWrite(src,0,buf,7);
        return FINISH;
    }else if((*p >= '0') && (*p <= '9')){
        uint16 major=0;
        while(*p>='0' && *p<='9'){
            major = major*10 + (*p++ -'0');
        }
        if(*p=='\r' && *(p+1)=='\n'){
            si.sys_cfg.setting = 'u';
            si.sys_cfg.major = major;
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
            return SUCCESS;
        }else{
            return FAILURE;
        }
    }else{
        return FAILURE;
    }
}

uint8 doMinor(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
        if(*p != '='){
        return FAILURE;
    }
    p++;
    if(*p == '?' && *(p+1) == '\r' && *(p+2) == '\n'){
        uint8 buf[7]={"00000\r\n"};
        buf[0] = (si.sys_cfg.minor/10000)%10 + '0';
        buf[1] = (si.sys_cfg.minor/1000)%10 + '0';
        buf[2] = (si.sys_cfg.minor/100)%10 + '0';
        buf[3] = (si.sys_cfg.minor/10)%10 + '0';
        buf[4] = si.sys_cfg.minor%10 + '0';
        myUARTWrite(src,0,buf,7);
        return FINISH;
    }else if((*p >= '0') && (*p <= '9')){
        uint16 minor=0;
        while(*p>='0' && *p<='9'){
            minor = minor*10 + (*p++ -'0');
        }
        if(*p=='\r' && *(p+1)=='\n'){
            si.sys_cfg.setting = 'u';
            si.sys_cfg.minor = minor;
            my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
            return SUCCESS;
        }else{
            return FAILURE;
        }
    }else{
        return FAILURE;
    }
}

uint8 doPassword(enum data_src src,uint8 *atCmd)
{
    uint8 *p = atCmd;
    uint8 pass[9]={0},i;
    #ifdef DBGBLEAT
    char dbg[10]={0};
    uint8 value[2]={0};
        #endif
    if(*p != '='){
        #ifdef DBGBLEAT
        if((si.sys_cfg.usb_debug=='1') && si.usb_serial_open){
            strcat(dbg,"1*p=");value[0]=*p;strcat(dbg,(char const *)value);strcat(dbg,"\r\n");
            myUARTWrite(USB,0,(uint8*)dbg,strlen((char const*)dbg));
        }
        #endif
        return FAILURE;
    }
    p++;
    if(*p == '?'){
        if((src == myBLE) || (src == myBLE_AT)){
            myUARTWrite(src,0,"OPERATE FORBID\r\n",strlen("OPERATE FORBID\r\n"));
            return FINISH;
        }else{
            myUARTWrite(src,0,si.sys_cfg.password,strlen((char const*)si.sys_cfg.password));
            myUARTWrite(src,0,"\r\n",2);
            return FINISH;
        }
    }
    for(i=0;i<8 && *p!= 0x0d;i++){
        pass[i] = *p;
        p++;
    }
    if(i == 0){
        #ifdef DBGBLEAT
        if((si.sys_cfg.usb_debug=='1') && si.usb_serial_open){
            strcat(dbg,"i==0");strcat(dbg,"\r\n");
            myUARTWrite(USB,0,(uint8*)dbg,strlen((char const*)dbg));
        }
        #endif
        return FAILURE;
    }
    if(*p!=0x0d){
        return FAILURE;
    }
    if((src == myBLE) || (src == myBLE_AT)){
        if(strcmp((char const*)pass,(char const*)si.sys_cfg.password)==0){
            si.ble_at_enable = 1;
            return SUCCESS;
        }else{
            myUARTWrite(src,0,"PASSWORD ERROR\r\n",strlen("PASSWORD ERROR\r\n"));
            return FINISH;
        }
    }else{
        strcpy((char *)si.sys_cfg.password, (char const*)pass);
        my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
        return SUCCESS;
    }
}

uint8 doExit(enum data_src src,uint8 *atCmd)
{

    uint8 *p = atCmd;
    if((p[0] != '\r') ||(p[1]!= '\n')){
        return FAILURE;
    }
    si.at_mode = 0;
    if(src == myCOM){
        si.curr_sm &= ~COM_AT_OPEN;
        //P0_7 = 1;  // BUG P0_7 test led off
    }else if(src == myUSB){
        si.curr_sm &= ~USB_AT_OPEN;
    }
    return SUCCESS;
}

uint8 doConMax(enum data_src src,uint8 *atCmd){//设置星型网络下，允许连接的最大从机数。
        uint8 *p = atCmd;
        if(*p != '='){
                return FAILURE;
        }
        p++;
        if(*p == '?' && *(p+1) == '\r' && *(p+2) == '\n'){
                uint8 buf[3];
                if(si.sys_cfg.connectmax == 1){
                        strcpy((char *)buf,"1\r\n");
                }else if(si.sys_cfg.connectmax == 2){
                        strcpy((char *)buf,"2\r\n");
                }else if(si.sys_cfg.connectmax == 3){
                        strcpy((char *)buf,"3\r\n");
                }else if(si.sys_cfg.connectmax == 4){
                        strcpy((char *)buf,"4\r\n");
                }else{
                        strcpy((char *)buf,"4\r\n");
                }
                myUARTWrite(src,0,buf,strlen((char const*)buf));
                return FINISH;
        }else{
                if(strcmp((char const *)p,"1\r\n") == 0){
                        si.sys_cfg.connectmax = 1;
                        my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                        return SUCCESS;
                }else if(strcmp((char const *)p,"2\r\n") == 0){
                        si.sys_cfg.connectmax = 2;
                        my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                        return SUCCESS;
                }else if(strcmp((char const *)p,"3\r\n") == 0){
                        si.sys_cfg.connectmax = 3;
                        my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                        return SUCCESS;
                }else if(strcmp((char const *)p,"4\r\n") == 0){
                        si.sys_cfg.connectmax = 4;
                        my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                        return SUCCESS;
                }else{
                        return FAILURE;
                }

        }
}
uint8 doNetwork(enum data_src src,uint8 *atCmd){
        uint8 *p = atCmd;
        if(*p != '='){
            return FAILURE;
        }
        p++;
        if(*p == '?'){
                uint8 buf[15];
                if(si.sys_cfg.networkcmode == 'o'){
                        strcpy((char *)buf,"OFF\r\n");
                }else if(si.sys_cfg.networkcmode == 's'){
                        strcpy((char *)buf,"STAR\r\n");
                }else{
                        strcpy((char *)buf,"TREE\r\n");
                }
                myUARTWrite(src,0,buf,strlen((char const*)buf));
                return FINISH;
        }else{
                if(strcmp((char const *)p,"OFF\r\n") == 0){
                        si.sys_cfg.networkcmode = 'o';
                        my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                        return SUCCESS;
                }else if(strcmp((char const *)p,"STAR\r\n") == 0){
                        si.sys_cfg.networkcmode = 's';
                        my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                        return SUCCESS;
                }else if(strcmp((char const *)p,"TREE\r\n") == 0){
                        si.sys_cfg.networkcmode = 't';
                        my_ad_nvms_write(0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                        return SUCCESS;
                }else{
                        return FAILURE;
                }
        }
        return SUCCESS;
}

uint8 doPort(enum data_src src,uint8 *atCmd){
#if 0
    uint8 *p=atCmd;
    int pin=0;
    int index=-1;
    int group=0;
    int sub_pin=0;
    #ifdef DBGBLEAT
    char dbg[10]={0};
    char value[2]={0};
    #endif
        if(p[0]=='F' && p[1]=='F'){
        group = 3;
    }else{
        pin += (p[0]-'0')*10 + (p[1]-'0');
        group = pin/10;
        sub_pin = pin%10;
    }
    p+=2;
    if(((group < 3) && (sub_pin > 7)) || ((group == 2) && (sub_pin > 4))){
                #ifdef DBGBLEAT
        strcat(dbg,"G"); value[0] = group+'0'; strcat(dbg,value);
        strcat(dbg," S");value[0] = sub_pin+'0'; strcat(dbg,value);
        strcat(dbg,"\r\n");
        myUARTWrite(USB,0,(uint8 *)dbg,strlen((char const *)dbg));
        myUARTWrite(USB,0,"1\r\n",3);
                #endif
        return FAILURE;
    }
    if(*p != '+'){
        #ifdef DBGBLEAT
        strcat(dbg,"P!+ =");
        value[0] = *p; strcat(dbg,value);
        strcat(dbg,"\r\n");
        myUARTWrite(USB,0,(uint8 *)dbg,strlen((char const *)dbg));
        myUARTWrite(USB,0,"2\r\n",3);
                #endif
        return FAILURE;
    }
    p++;
    index = getPinMode(p);
    if(index == AT_PINMODE_NUM){
        #ifdef DBGBLEAT
        strcat(dbg,"index=4");
        strcat(dbg,"\r\n");
        myUARTWrite(USB,0,(uint8 *)dbg,strlen((char const *)dbg));
        myUARTWrite(USB,0,"3\r\n",3);
        #endif
        return FAILURE;
    }
    p+=strlen((char const *)atPinMode[index]);
    if(*p != '='){
        #ifdef DBGBLEAT
        strcat(dbg,"P!= =");
        value[0] = *p;strcat(dbg,value);
        strcat(dbg,"\r\n");
        myUARTWrite(USB,0,(uint8 *)dbg,strlen((char const *)dbg));
        myUARTWrite(USB,0,"4\r\n",3);
                #endif
        return FAILURE;
    }else{
        //unsigned char volatile __xdata *psel = &X_P0SEL;
        //unsigned char volatile __xdata *pdir = &X_P0DIR;
        //unsigned char volatile __xdata *pdat = &X_P0;
        int result;
        p++;
        /*先处理返回所有值得情况*/
        if(group == 3){
            uint8 *array = df_osal_mem_alloc(50);
            uint8 tmp;
            if(array == NULL){
                #ifdef DBGBLEAT
                strcat(dbg,"malloc");
                strcat(dbg,"\r\n");
                myUARTWrite(USB,0,"5\r\n",3);
                #endif
                return FAILURE;
            }
            if(index == 0){
                memcpy(array,"P0+OUT=0x00,P1+OUT=0x00,P2+OUT=0x00\r\n",strlen("P0+OUT=0x00,P1+OUT=0x00,P2+OUT=0x00\r\n\0")+1);
                tmp = P0>>4;
                array[9] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');
                tmp = P0&0x0f;
                array[10] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');

                tmp = P1>>4;
                array[21] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');
                tmp = P1&0x0f;
                array[22] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');

                tmp = P2>>4;
                array[33] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');
                tmp = P2&0x0f;
                array[34] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');
                myUARTWrite(src,0,array,strlen((char const *)array));
                osal_mem_free(array);
                return FINISH;
            }else if(index == 1){
                memcpy(array,"P0+IN=0x00,P1+IN=0x00,P2+IN=0x00\r\n",strlen("P0+IN=0x00,P1+IN=0x00,P2+IN=0x00\r\n\0")+1);
                tmp = P0>>4;
                array[8] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');
                tmp = P0&0x0f;
                array[9] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');

                tmp = P1>>4;
                array[19] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');
                tmp = P1&0x0f;
                array[20] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');

                tmp = P2>>4;
                array[30] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');
                tmp = P2&0x0f;
                array[31] = (tmp>=10)?(tmp-10+'A'):(tmp+'0');
                myUARTWrite(src,0,array,strlen((char const *)array));
                osal_mem_free(array);
                return FINISH;
            }else{
                return FAILURE;
            }
        }
        /*处理单个数据*/
        result = *p;
        if(index == 0){//"OUT"
            uint8 array[]={"P00+OUT=1\r\n"};
            array[1] = group + '0';
            array[2] = sub_pin + '0';
            if(group == 0){
                P0SEL &= ~(1<<sub_pin);
                P0DIR |= (1<<sub_pin);
                if(result == '?'){
                    array[8]=P0&(1<<sub_pin)?'1':'0';
                    myUARTWrite(src,0,array,strlen((char const *)array));
                    return FINISH;
                }
                result -='0';
                if(sub_pin == 5){
                    if(result == 0) flushAllBuf();
                    #if RST_REVERSE
                    result = !result;
                    #endif
                }
                if(result){
                    P0 |= (1<<sub_pin);
                }else{
                    P0 &= ~(1<<sub_pin);
                }
            }else if(group == 1){
                P1SEL &= ~(1<<sub_pin);
                P1DIR |= (1<<sub_pin);
                if(result == '?'){
                    array[8]=P1&(1<<sub_pin)?'1':'0';
                    myUARTWrite(src,0,array,strlen((char const *)array));
                    return FINISH;
                }
                result -='0';
                if(result){
                    P1 |= (1<<sub_pin);
                }else{
                    P1 &= ~(1<<sub_pin);
                }
            }else if(group == 2){
                P2SEL &= ~(1<<sub_pin);
                P2DIR |= (1<<sub_pin);
                if(result == '?'){
                    array[8]=P1&(1<<sub_pin)?'1':'0';
                    myUARTWrite(src,0,array,strlen((char const *)array));
                    return FINISH;
                }
                result -='0';
                if(result){
                    P2 |= (1<<sub_pin);
                }else{
                    P2 &= ~(1<<sub_pin);
                }
            }
            return SUCCESS;
        }else if(index == 1){//"IN"
            uint8 array[]={"P00+IN=1\r\n"};
            array[1] = group + '0';
            array[2] = sub_pin + '0';
            if(group == 0){
                P0SEL &= ~(1<<sub_pin);
                P0DIR &= ~(1<<sub_pin);
                delay_us(3);
                array[7] = (P0 & (1<<sub_pin))?'1':'0';
            }else if(group == 1){
                P1SEL &= ~(1<<sub_pin);
                P1DIR &= ~(1<<sub_pin);
                delay_us(3);
                array[7] = (P1 & (1<<sub_pin))?'1':'0';
            }else if(group == 2){
                P2SEL &= ~(1<<sub_pin);
                P2DIR &= ~(1<<sub_pin);
                delay_us(3);
                array[7] = (P2 & (1<<sub_pin))?'1':'0';
            }
             myUARTWrite(src,0,array,strlen((char const *)array));
             return FINISH;
        }else if(index == 2){//"PWM"
            if(group == 0){

            }else if(group == 1){

            }else if(group == 2){

            }
        }else if(index == 3){//"ADC"
            uint8 array[]={"0000\r\n"};
            uint16 adc;
            if(result != '?'){
                #ifdef DBGBLEAT
                myUARTWrite(USB,0,"6\r\n",3);
                #endif
                return FAILURE;
            }
            if(group == 0){
                adc=HalAdcRead(sub_pin,HAL_ADC_RESOLUTION_12);
                array[0] = (adc/1000)%10 + '0';
                array[1] = (adc/100)%10 + '0';
                array[2] = (adc/10)%10 + '0';
                array[3] = (adc/1)%10 + '0';
                myUARTWrite(src,0,array,strlen((char const *)array));
                return FINISH;
            }else if(group == 1){
                #ifdef DBGBLEAT
                myUARTWrite(USB,0,"7\r\n",3);
                #endif
                return FAILURE;
            }else if(group == 2){
                #ifdef DBGBLEAT
                myUARTWrite(USB,0,"8\r\n",3);
                #endif
                return FAILURE;
            }
        }
    }
#endif
    return SUCCESS;
}


#if 0
#pragma optimize=none
void delay_us(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 3 NOPs == 1 usecs */
        asm("NOP");
        asm("NOP");
        asm("NOP");
  }
}
#endif

