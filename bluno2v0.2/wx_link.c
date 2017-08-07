/*
 * wx_link.c
 *
 *  Created on: 2017年3月1日
 *      Author: Administrator
 */

#include "osal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wx_link.h"
#include "hw_uart.h"



struct msgBuf *msgBufHead=NULL;
struct msgBuf *msgBufTail=NULL;
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



struct msgBuf * mysappEnqueue(struct msgBuf *head,uint8_t *pbuf, uint8_t len){
        struct msgBuf *p;
        struct msgBuf *pr = head;
        uint8_t nextPacketLen = len;
        uint8_t appendlen = 0;
        if(msgBufHead != NULL){
                appendlen = MAX_SPP_PACKET_LEN-msgBufTail->len;
                appendlen = appendlen>len?len:appendlen;
                if(appendlen != 0){
                        memcpy(msgBufTail->data + msgBufTail->len, pbuf, appendlen);
                        msgBufTail->len += appendlen;
                }
                if(nextPacketLen !=0){
                        p = (struct msgBuf*)OS_MALLOC(sizeof(struct msgBuf)+MAX_SPP_PACKET_LEN);
                        if(p == NULL){
                                exit(0);
                        }
                        p->next = NULL;
                        msgBufTail->next = p;
                        msgBufTail = p;
                        p->len = nextPacketLen;
                        p->handle = 0;
                        memcpy(p->data,pbuf+appendlen,nextPacketLen);
                }
        }else{
                p = (struct msgBuf*)OS_MALLOC(sizeof(struct msgBuf)+MAX_SPP_PACKET_LEN);
                if(p == NULL){
                        exit(0);
                }
                p->next = NULL;
                msgBufHead = p;
                head = p;
                msgBufTail = p;
                p->len = len;
                p->handle = 0;
                memcpy(p->data,pbuf,len);
        }
        return msgBufHead;
}

#if 0
void sappEnqueue(uint8_t *pbuf, uint8_t len){
        struct msgBuf *p;
        uint8_t nextPacketLen = len;
        uint8_t appendlen = 0;
        if(msgBufHead != NULL){
                appendlen = MAX_SPP_PACKET_LEN-msgBufTail->len;
                appendlen = appendlen>len?len:appendlen;
                if(appendlen != 0){
                        memcpy(msgBufTail->data + msgBufTail->len, pbuf, appendlen);
                        msgBufTail->len += appendlen;
                }
                if(nextPacketLen !=0){
                        p = (struct msgBuf*)OS_MALLOC(sizeof(struct msgBuf)+MAX_SPP_PACKET_LEN);
                        if(p == NULL){
                                return;
                        }
                        p->next = NULL;
                        msgBufTail->next = p;
                        msgBufTail = p;
                        p->len = nextPacketLen;
                        p->handle = 0;
                        memcpy(p->data,pbuf+appendlen,nextPacketLen);
                }
        }else{
                p = (struct msgBuf*)OS_MALLOC(sizeof(struct msgBuf)+MAX_SPP_PACKET_LEN);
                if(p == NULL){
                        return;
                }
                p->next = NULL;
                msgBufHead = p;
                msgBufTail = p;
                p->len = len;
                p->handle = 0;
                memcpy(p->data,pbuf,len);
        }
}


#elif 0
/*
 * 函数功能 ： 新建一个节点，并将该节点添加到链表的末尾
 * 函数的参数：结构体指针变量head，表示原有链表的头节点指针
 * 函数的返回值：添加节点后的链表的头节点指针 */
struct msgBuf *AppendNode(struct msgBuf *head, uint8_t *data, uint16_t len){
        struct msgBuf *p = NULL;
        struct msgBuf *pr = head;
        //int data;
        /*为新添加的节点申请内存*/
        p = (struct msgBuf*)OS_MALLOC(sizeof(struct msgBuf)+len);

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
        msgBufHead = head;
        return head;/*返回添加节点后的链表的头节点指针*/
}
#elif 1
struct msgBuf *myAppendNode(struct msgBuf *head, uint8_t *data, uint16_t len){
        struct msgBuf *p = NULL;
        struct msgBuf *pr = head;
        uint8_t nextPacketLen = len;
        uint8_t appendlen = 0;
        appendlen = MAX_SPP_PACKET_LEN-len;
        if(appendlen>0){
                p = (struct msgBuf*)OS_MALLOC(sizeof(struct msgBuf)+MAX_SPP_PACKET_LEN);
                if(head == NULL){
                        head = p;
                        p->len = len;
                        pr = p;/* 让pr指向新建节点*/
                        msgBufTail = p;
                        msgBufTail->len = len;
                        memcpy(pr->data,data,len);
                }else{
                        appendlen=MAX_SPP_PACKET_LEN-msgBufTail->len;
                        if(appendlen>0){
                                memcpy(msgBufTail->data+msgBufTail->len,data,appendlen);
                                pr->next = NULL;

                        }
                }
        }else{
                p = (struct msgBuf*)OS_MALLOC(sizeof(struct msgBuf)+MAX_SPP_PACKET_LEN);
                if(head == NULL){
                        head = p;
                        p->len = len;
                        pr = p;
                        memcpy(pr->data,data,len);
                        pr->next = NULL;
                }
        }
        //int data;
        /*为新添加的节点申请内存*/
        p = (struct msgBuf*)OS_MALLOC(sizeof(struct msgBuf)+MAX_SPP_PACKET_LEN);

        if(p == NULL){/*若申请内存失败，则打印错误信息，退出程序*/
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
        memcpy(pr->data,data,len);
        pr->next = NULL;/*将新建节点置为表尾*/
        return head;/*返回添加节点后的链表的头节点指针*/
}

#endif


#if 1
/*
 * 函数的功能：显示所有已经建立好的节点的节点号和该节点中的数据项内容
 * 函数的参数：结构体指针变量head，表示链表的头节点指针
 * 函数的返回值：无*/
void DispLink(struct msgBuf *head){
        struct msgBuf *p = head;
        int j = 1;
        while(p != NULL){/*若不是表尾，则循环打印*/
                //printf("%s\r\n",p->data);/*打印第j个节点的数据*/
                hw_uart_send(HW_UART2, p->data, p->len, NULL, NULL);
                p = p->next;/*让p指向下一个节点*/
                j++;
        }
}


/*
 * 函数功能：释放head指向的链表中所有节点占用的内存
 * 输入参数：结构体指针变量head,表示链表的头节点指针
 * 返回参数：无*/
void DeleteMemory(struct msgBuf *head){
        struct msgBuf *p= head, *pr = NULL;
        while(p != NULL){/*若不是表尾，则释放节点占用的内存*/
                pr = p;/*在pr中保存当前节点的指针*/
                p = p->next;/*让p指向下一节点*/
                OS_FREE(pr);/*释放pr指向的当前节点占用的内存*/
        }
}

void wx_show(){
        DispLink(msgBufHead);
        DeleteMemory(msgBufHead);
        msgBufHead = NULL;
}
#endif
