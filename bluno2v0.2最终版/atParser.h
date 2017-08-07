/*
 * atParser.h
 *
 *  Created on: 2017Äê3ÔÂ6ÈÕ
 *      Author: Administrator
 */



#ifndef ATPARSER_H_
#define ATPARSER_H_



#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sdk_defs.h>
#include "comdef.h"



#define AT_BUF_LEN 32
#define AT_PARAM_NUM 28
#define AT_RESULT_NUM 10
#define AT_PINMODE_NUM 4

#define AT_RST_SET  "AT+P05+OUT=1\r\n"
#define AT_RST_CLR  "AT+P05+OUT=0\r\n"
#define AT_SET_UART9600 "AT+CURRUART=9600\r\n"
#define AT_SET_UART19200 "AT+CURRUART=19200\r\n"
#define AT_SET_UART38400 "AT+CURRUART=38400\r\n"
#define AT_SET_UART57600 "AT+CURRUART=57600\r\n"
#define AT_SET_UART115200 "AT+CURRUART=115200\r\n"

#define FINISH                  0x02
#define CONTINUE                0x03


typedef struct {
  void *startAddr;
  uint16_t offset;
  void *priv;
}dataTransferT;

typedef uint8 (*selfFunc)(void *arg);
extern void *selfFuncArgs;
extern selfFunc selfFuncPtr;



uint8_t BLETriggerTransfer(void *arg);

uint16_t myconnid;
extern uint8 * long_buf_addr;
extern uint16_t long_buf_offset;
extern uint16_t long_buf_len;
extern enum data_src long_buf_src;
extern uint8 at_cmd[];
extern uint8 atParser(enum data_src src,uint8 *buf,uint8 len);
extern void myUARTWrite(enum data_src src,uint8 port,const void *buf,uint16 len);
extern void delay_us(uint16_t microSecs);
extern void network_infor(enum data_src src);

extern void judgmentAT(enum data_src src, uint8_t *buf, uint16_t len);
extern uint8 doRestart(enum data_src src,uint8 *atCmd);
extern uint8 doRole(enum data_src src,uint8 *atCmd);
extern uint8 doFsm(enum data_src src,uint8 *atCmd);
extern uint8 doVersion(enum data_src src,uint8 *atCmd);
extern uint8 doTxpower(enum data_src src,uint8 *atCmd);
extern uint8 doIbeacons(enum data_src src,uint8 *atCmd);
extern uint8 doUart(enum data_src src,uint8 *atCmd);
extern uint8 doCurrUart(enum data_src src,uint8 *atCmd);
extern uint8 doCmode(enum data_src src,uint8 *atCmd);
extern uint8 doBind(enum data_src src,uint8 *atCmd);
extern uint8 doVersion(enum data_src src,uint8 *atCmd);
extern uint8 doLaddr(enum data_src src,uint8 *atCmd);
extern uint8 doSerial(enum data_src src,uint8 *atCmd);
extern uint8 doKey(enum data_src src,uint8 *atCmd);
extern uint8 doMac(enum data_src src,uint8 *atCmd);
extern uint8 doSn(enum data_src src,uint8 *atCmd);
extern uint8 doMaxInterval(enum data_src src,uint8 *atCmd);
extern uint8 doMinInterval(enum data_src src,uint8 *atCmd);
extern uint8 doName(enum data_src src,uint8 *atCmd);
extern uint8 doResource(enum data_src src,uint8 *atCmd);
extern uint8 doUsbdebug(enum data_src src,uint8 *atCmd);
extern uint8 doSetting(enum data_src src,uint8 *atCmd);
extern uint8 doRtrans(enum data_src src,uint8 *atCmd);
extern uint8 doBlunodebug(enum data_src src,uint8 *atCmd);
extern uint8 doRssi(enum data_src src,uint8 *atCmd);
extern uint8 doMajor(enum data_src src,uint8 *atCmd);
extern uint8 doMinor(enum data_src src,uint8 *atCmd);
extern uint8 doPassword(enum data_src src,uint8 *atCmd);
extern uint8 doExit(enum data_src src,uint8 *atCmd);
extern uint8 doConnMax(enum data_src src,uint8 *atCmd);
extern uint8 doNetwork(enum data_src src,uint8 *atCmd);
extern uint8 domyid(enum data_src src,uint8 *atCmd);
extern uint8 donetInf(enum data_src src,uint8 *atCmd);
extern uint8 doPort(enum data_src src,uint8 *atCmd);
extern uint8 doupdate(enum data_src src,uint8 *atCmd);
extern uint8 doclearwhitelist(enum data_src src,uint8 *atCmd);
extern uint8 dotest(enum data_src src,uint8 *atCmd);
extern uint8 dohelp(enum data_src src,uint8 *atCmd);

extern void ble_App_stop();

extern uint8 osal_memcmp( const void GENERIC *src1, const void GENERIC *src2, unsigned int len );
extern void *osal_memcpy( void*, const void GENERIC *, unsigned int );


#endif /* ATPARSER_H_ */
