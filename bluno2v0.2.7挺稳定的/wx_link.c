/*
 * wx_link.c
 *
 *  Created on: 2017��3��1��
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
//struct Link *head =NULL;/*ָ������ͷ*/
//int i = 0;
//void wx_main(uint8_t  data){
        //printf(" do you want to append a new node (Y/N)?\r\n");
       // head = AppendNode(head,data);
        //DispLink(head);   /*��ʾ��ǰ�����еĸ��ڵ���Ϣ*/
        //i++;
        //printf("%d new nodes have been apended!\r\n", i);

        //DeleteMemory(head);  /*�ͷ������ѷ�����ڴ�*/
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
 * �������� �� �½�һ���ڵ㣬�����ýڵ���ӵ������ĩβ
 * �����Ĳ������ṹ��ָ�����head����ʾԭ�������ͷ�ڵ�ָ��
 * �����ķ���ֵ����ӽڵ��������ͷ�ڵ�ָ�� */
struct msgBuf *AppendNode(struct msgBuf *head, uint8_t *data, uint16_t len){
        struct msgBuf *p = NULL;
        struct msgBuf *pr = head;
        //int data;
        /*Ϊ����ӵĽڵ������ڴ�*/
        p = (struct msgBuf*)OS_MALLOC(sizeof(struct msgBuf)+len);

        if(p == NULL){/*�������ڴ�ʧ�ܣ����ӡ������Ϣ���˳�����*/
                printf("No enough memory to alloc\r\n");
                exit(0);
        }
        if(head == NULL){/*��ԭ����Ϊ�ձ����½��ڵ���Ϊ�׽ڵ�*/
                head = p;
        }else{/*��ԭ����Ϊ�ǿգ����½��ڵ���ӵ���β*/
                /*��δ����β��������ƶ�ָ��pr,ֱ��prָ���β*/
                while(pr->next != NULL){
                        pr = pr->next;
                }
                pr->next = p;/*���½��ڵ���ӵ������ĩβ*/
        }

        pr = p;/* ��prָ���½��ڵ�*/
        //printf("Input node data:\r\n");
        //scanf("%d",&data);/*����ڵ�����*/
        //pr->data = data;
        pr->len = len;
        memcpy(pr->data,data,len);
        pr->next = NULL;/*���½��ڵ���Ϊ��β*/
        msgBufHead = head;
        return head;/*������ӽڵ��������ͷ�ڵ�ָ��*/
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
                        pr = p;/* ��prָ���½��ڵ�*/
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
        /*Ϊ����ӵĽڵ������ڴ�*/
        p = (struct msgBuf*)OS_MALLOC(sizeof(struct msgBuf)+MAX_SPP_PACKET_LEN);

        if(p == NULL){/*�������ڴ�ʧ�ܣ����ӡ������Ϣ���˳�����*/
                exit(0);
        }
        if(head == NULL){/*��ԭ����Ϊ�ձ����½��ڵ���Ϊ�׽ڵ�*/
                head = p;
        }else{/*��ԭ����Ϊ�ǿգ����½��ڵ���ӵ���β*/
                /*��δ����β��������ƶ�ָ��pr,ֱ��prָ���β*/
                while(pr->next != NULL){
                        pr = pr->next;
                }
                pr->next = p;/*���½��ڵ���ӵ������ĩβ*/
        }

        pr = p;/* ��prָ���½��ڵ�*/
        memcpy(pr->data,data,len);
        pr->next = NULL;/*���½��ڵ���Ϊ��β*/
        return head;/*������ӽڵ��������ͷ�ڵ�ָ��*/
}

#endif


#if 1
/*
 * �����Ĺ��ܣ���ʾ�����Ѿ������õĽڵ�Ľڵ�ź͸ýڵ��е�����������
 * �����Ĳ������ṹ��ָ�����head����ʾ�����ͷ�ڵ�ָ��
 * �����ķ���ֵ����*/
void DispLink(struct msgBuf *head){
        struct msgBuf *p = head;
        int j = 1;
        while(p != NULL){/*�����Ǳ�β����ѭ����ӡ*/
                //printf("%s\r\n",p->data);/*��ӡ��j���ڵ������*/
                hw_uart_send(HW_UART2, p->data, p->len, NULL, NULL);
                p = p->next;/*��pָ����һ���ڵ�*/
                j++;
        }
}


/*
 * �������ܣ��ͷ�headָ������������нڵ�ռ�õ��ڴ�
 * ����������ṹ��ָ�����head,��ʾ�����ͷ�ڵ�ָ��
 * ���ز�������*/
void DeleteMemory(struct msgBuf *head){
        struct msgBuf *p= head, *pr = NULL;
        while(p != NULL){/*�����Ǳ�β�����ͷŽڵ�ռ�õ��ڴ�*/
                pr = p;/*��pr�б��浱ǰ�ڵ��ָ��*/
                p = p->next;/*��pָ����һ�ڵ�*/
                OS_FREE(pr);/*�ͷ�prָ��ĵ�ǰ�ڵ�ռ�õ��ڴ�*/
        }
}

void wx_show(){
        DispLink(msgBufHead);
        DeleteMemory(msgBufHead);
        msgBufHead = NULL;
}
#endif
