/**
 ****************************************************************************************
 *
 * @file lls.c
 *
 * @brief Link Loss Service sample implementation
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
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "util/queue.h"
#include "ble_att.h"
#include "ble_bufops.h"
#include "ble_common.h"
#include "ble_gatts.h"
#include "ble_uuid.h"
#include "simpleGATTprofile.h"

#include "hw_uart.h"
#include "wx_link.h"


#define simpleProfileChar1Props  GATT_PROP_NOTIFY | GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RESP

static uint8_t dfbvalue1 = 1;
static uint8_t dfbvalue2 = 2;
static uint8_t dfbvalue3 = 3;

static const char DFB1name[] = "Serial";
static const char DFB2name[] = "AT Ctrl";
typedef struct {
        ble_service_t svc;

        // handles
        uint16_t DFBV1;
        uint16_t DFBV2;
        uint16_t DFBV3;

        // callbacks
        simpleGATT_alert_level_cb_t cb;
        queue_t levels;
} simpleGATT_service_t;

typedef struct {
        void *next;

        uint16_t conn_idx;
        uint8_t level;
} conn_dev_t;


struct blemsgBuf *blemsgBufHead=NULL;
struct blemsgBuf *myblemsgBufHead=NULL;
struct blemsgBuf *blemsgBufTail=NULL;
ble_service_t *wxsvc;



#if 0

void blesappEnqueue(uint8_t *pbuf, uint8_t len){
        struct blemsgBuf *p;
        uint8_t nextPacketLen = len;
        uint8_t appendlen = 0;
        if(blemsgBufHead != NULL){
                appendlen = MAX_SPP_PACKET_LEN-blemsgBufTail->len;
                appendlen = appendlen>len?len:appendlen;
                if(appendlen != 0){
                        memcpy(blemsgBufTail->data + blemsgBufTail->len, pbuf, appendlen);
                        blemsgBufTail->len += appendlen;
                }
                if(nextPacketLen !=0){
                        p = (struct blemsgBuf*)OS_MALLOC(sizeof(struct blemsgBuf)+MAX_SPP_PACKET_LEN);
                        if(p == NULL){
                                return;
                        }
                        p->next = NULL;
                        blemsgBufTail->next = p;
                        blemsgBufTail = p;
                        p->len = nextPacketLen;
                        p->handle = 0;
                        memcpy(p->data,pbuf+appendlen,nextPacketLen);
                }
        }else{
                p = (struct blemsgBuf*)OS_MALLOC(sizeof(struct blemsgBuf)+MAX_SPP_PACKET_LEN);
                if(p == NULL){
                        return;
                }
                p->next = NULL;
                blemsgBufHead = p;
                blemsgBufTail = p;
                p->len = len;
                p->handle = 0;
                memcpy(p->data,pbuf,len);
        }
}
#elif 0
struct blemsgBuf *bleAppendNode(struct blemsgBuf *head, uint8_t *data, uint16_t len){
        struct blemsgBuf *p = NULL;
        struct blemsgBuf *pr = head;
        //int data;
        /*为新添加的节点申请内存*/
        p = (struct blemsgBuf*)OS_MALLOC(sizeof(struct blemsgBuf)+len);

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
        //pr->data = data;
        pr->len = len;
        memcpy(pr->data,data,len);
        pr->next = NULL;/*将新建节点置为表尾*/
        blemsgBufHead = head;
        return head;/*返回添加节点后的链表的头节点指针*/
}

#elif 1
void bleAppendNode (uint8_t *data, uint16_t len){
        struct blemsgBuf *p = NULL;
        struct blemsgBuf *pr = blemsgBufHead;
        //int data;
        /*为新添加的节点申请内存*/
        p = (struct blemsgBuf*)OS_MALLOC(sizeof(struct blemsgBuf)+len);

        if(p == NULL){/*若申请内存失败，则打印错误信息，退出程序*/
                exit(0);
        }
        if(blemsgBufHead == NULL){/*若原链表为空表，则将新建节点置为首节点*/
                blemsgBufHead = p;
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
        //pr->data = data;
        pr->len = len;
        memcpy(pr->data,data,len);
        pr->next = NULL;/*将新建节点置为表尾*/
        myblemsgBufHead = blemsgBufHead;
        //return head;/*返回添加节点后的链表的头节点指针*/
}
#endif
void bleDispLink(struct blemsgBuf *head){
        struct blemsgBuf *p = head;
        int j = 1;
        while(p != NULL){/*若不是表尾，则循环打印*/
                //printf("%s\r\n",p->data);/*打印第j个节点的数据*/

                hw_uart_send(HW_UART2, p->data, p->len, NULL, NULL);
                p = p->next;/*让p指向下一个节点*/
                j++;
        }
}

void bleDeleteMemory(struct blemsgBuf *head){
        struct blemsgBuf *p= head, *pr = NULL;
        while(p != NULL){/*若不是表尾，则释放节点占用的内存*/
                pr = p;/*在pr中保存当前节点的指针*/
                p = p->next;/*让p指向下一节点*/
                OS_FREE(pr);/*释放pr指向的当前节点占用的内存*/
        }
}

void wx_bleshow(){
        bleDispLink(blemsgBufHead);
        bleDeleteMemory(blemsgBufHead);
        blemsgBufHead = NULL;
}









static bool conn_dev_conn_idx_match(const void *data, const void *match_data)
{
        conn_dev_t *conn_dev = (conn_dev_t *) data;
        uint16_t conn_idx = (*(uint16_t *) match_data);

        return conn_dev->conn_idx == conn_idx;
}

static void handle_disconnected_evt(ble_service_t *svc, const ble_evt_gap_disconnected_t *evt)
{
        simpleGATT_service_t *spG = (simpleGATT_service_t *) svc;

        if (evt->reason == 0x00 || evt->reason == 0x13 || evt->reason == 0x16) {
                // do not fire callback if disconnection was triggered by either side
                return;
        }

        // fire callback with current Alert Level - app should trigger an alarm
        if (spG->cb) {
                uint8_t level = 0;
                conn_dev_t *conn_dev;

                conn_dev = queue_remove(&spG->levels, conn_dev_conn_idx_match, &evt->conn_idx);

                if (conn_dev) {
                        level = conn_dev->level;
                        OS_FREE(conn_dev);
                }

                spG->cb(evt->conn_idx, &evt->address, level);
        }
}



#if 1
static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt)
{
        wxx * wxc;

        ble_evt_gatts_write_req_t *abc;
        simpleGATT_service_t *spG = (simpleGATT_service_t *) svc;
        //printf("abc->length=%d\r\n",abc->length);
        //hw_uart_send(HW_UART2, abc->value, abc->length, NULL, NULL);
        //memcpy(wxc->buffer + wxc->buffer_len, abc->value, abc->length);
        //printf("wxc->buffer_len=%d\r\n",wxc->buffer_len);
                //wxc->buffer_len += abc->length;

        if (evt->handle == spG->DFBV1) {

                //ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, blemsgBufHead->len, blemsgBufHead->data);
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(dfbvalue1), &dfbvalue1);
                //hw_uart_send(HW_UART2, blemsgBufHead->data, blemsgBufHead->len, NULL, NULL);

                //bleDispLink(blemsgBufHead);
#if 0
                struct blemsgBuf *p = blemsgBufHead, *pr = NULL;
                        if(p != NULL){/*若不是表尾，则循环打印*/
                                //printf("%s\r\n",p->data);/*打印第j个节点的数据*/
                                pr = p;
                                hw_uart_send(HW_UART2, p->data, p->len, NULL, NULL);
                                p = p->next;/*让p指向下一个节点*/
                                OS_FREE(pr);
                                pr = NULL;
                                blemsgBufHead = p;
                        }
#endif
                //bleDeleteMemory(blemsgBufHead);
                //blemsgBufHead = NULL;
                //ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(dfbvalue1), &dfbvalue1);

        }else if(evt->handle == spG->DFBV2){
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(dfbvalue2), &dfbvalue2);
        }else if(evt->handle == spG->DFBV3){
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(dfbvalue3), &dfbvalue3);
        }
}
#endif


#if 0
static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt)
{
        simpleGATT_service_t *spG = (simpleGATT_service_t *) svc;
        printf("handle_read_req\r\n");
        wx_bleshow();
}
#endif

void wx_write_notif(){
        simpleGATT_service_t *spG = (simpleGATT_service_t *) wxsvc;
        att_error_t err = ATT_ERROR_OK;
        struct blemsgBuf *p = blemsgBufHead, *pr = NULL;
        while(p != NULL){
                pr = p;
                ble_gatts_send_event(0, spG->DFBV1, GATT_EVENT_NOTIFICATION, p->len, p->data);
                ble_gatts_write_cfm(0, 32, err);
                hw_uart_send(HW_UART2, p->data, p->len, NULL, NULL);
                p = p->next;
                OS_FREE(pr);
                pr = NULL;
                blemsgBufHead = p;
        }
}
static void handle_write_req(ble_service_t *svc, const ble_evt_gatts_write_req_t *evt)
{

        /*
        wxx * wxc = (wxx *) svc;
        wxc->conn_idx = evt->conn_idx;
        wxc->handle = evt->handle;
        wxc->offset = evt->offset;
        wxc->length = evt->length;
        memcpy(wxc->value, evt->value, evt->length);
*/


        printf("handle_write_req\r\n");

        simpleGATT_service_t *spG = (simpleGATT_service_t *) svc;
        att_error_t err = ATT_ERROR_OK;
        //uint8_t level = get_u8(evt->value);
        uint16_t va = 0;
        uint16_t length = evt->length;
        //const uint8_t *value = evt->value;
        //printf("length=%d\r\n",length);
        //printf("wxc->length=%d\r\n",wxc->length);

        if(evt->handle == spG->DFBV1){
                /*
                while(length > 0){
                        va = get_u8_inc(&value);
                        printf("%c",va);
                        length--;
                }
                printf("\r\n");
                dfbvalue1 = va;
                if(va == 1){
                        uint8_t c = hw_uart_read(HW_UART2);
                        while(c){printf("%c\r\n",c);c=hw_uart_read(HW_UART2);}
                }
                */



                struct blemsgBuf *p = blemsgBufHead, *pr = NULL;
                                        while(p != NULL){/*若不是表尾，则循环打印*/
                                                //printf("%s\r\n",p->data);/*打印第j个节点的数据*/
                                                pr = p;
                                                ble_gatts_send_event(0, spG->DFBV1, GATT_EVENT_NOTIFICATION, p->len, p->data);/*Notif的触发事件*/
                                                ble_gatts_write_cfm(0, 32, err);
                                                hw_uart_send(HW_UART2, p->data, p->len, NULL, NULL);
                                                p = p->next;/*让p指向下一个节点*/
                                                OS_FREE(pr);
                                                pr = NULL;
                                                blemsgBufHead = p;
                                        }
                //hw_uart_send(HW_UART2, evt->value, evt->length, NULL, NULL);

        }else if(evt->handle == spG->DFBV2){
                va = get_u8(evt->value);
                dfbvalue2 = va;
        }else if(evt->handle == spG->DFBV2){
                va = get_u8(evt->value);
                dfbvalue3 = va;
        }
        //printf("evt->conn_idx=%d\r\n",evt->conn_idx);
        //printf("evt->handlex=%d\r\n",(uint16_t)evt->handle);
        ble_gatts_write_cfm(evt->conn_idx, evt->handle, err);
}

ble_service_t *simpleGATT_init(simpleGATT_alert_level_cb_t alert_cb)
{
        simpleGATT_service_t *spG;
        uint16_t num_attr, DFB1name_h, DFB2name_h;
        att_uuid_t uuid;
        wxx *wx;

        spG = OS_MALLOC(sizeof(*spG));
        memset(spG, 0, sizeof(*spG));
        queue_init(&spG->levels);




        spG->svc.disconnected_evt = handle_disconnected_evt;
        spG->svc.read_req = handle_read_req;
        spG->svc.write_req = handle_write_req;
        spG->cb = alert_cb;

        num_attr = ble_gatts_get_num_attr(0, 3, 0);

        ble_uuid_create16(SIMPLEPROFILE_SERV_UUID, &uuid);
        ble_gatts_add_service(&uuid, GATT_SERVICE_PRIMARY, num_attr);

        ble_uuid_create16(SIMPLEPROFILE_CHAR1_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, simpleProfileChar1Props, ATT_PERM_RW, 160, 1, NULL, &spG->DFBV1);

        ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);
        ble_gatts_add_descriptor(&uuid, ATT_PERM_READ, sizeof(DFB1name), 0, &DFB1name_h);

        ble_uuid_create16(SIMPLEPROFILE_CHAR2_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, simpleProfileChar1Props, ATT_PERM_RW, sizeof(uint8_t), 1, NULL, &spG->DFBV2);

        ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);
        ble_gatts_add_descriptor(&uuid, ATT_PERM_READ, sizeof(DFB2name), 0, &DFB2name_h);

        ble_uuid_create16(SIMPLEPROFILE_CHAR3_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, simpleProfileChar1Props, ATT_PERM_RW, sizeof(uint8_t), 1, NULL, &spG->DFBV3);

        ble_gatts_register_service(&spG->svc.start_h, &spG->DFBV1, &DFB1name_h, &spG->DFBV2, &DFB2name_h,&spG->DFBV3, 0);

        ble_gatts_set_value(DFB1name_h, sizeof(DFB1name), DFB1name);
        ble_gatts_set_value(DFB2name_h, sizeof(DFB2name), DFB2name);

        spG->svc.end_h = spG->svc.start_h + num_attr;
        wxsvc = &spG->svc;
        return &spG->svc;
}
