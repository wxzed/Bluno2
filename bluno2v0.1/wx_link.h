/*
 * wx_link.h
 *
 *  Created on: 2017Äê3ÔÂ1ÈÕ
 *      Author: Administrator
 */

#ifndef WX_LINK_H_
#define WX_LINK_H_
#include <stdio.h>
#include <stdlib.h>

#define MAX_SPP_PACKET_LEN        20
struct msgBuf{
        struct msgBuf *next;
        uint16_t handle;
        uint16_t len;
        uint8_t data[0];

};
struct msgBuf * sappEnqueue(struct msgBuf *head,uint8_t *pbuf, uint8_t len);
struct msgBuf *AppendNode(struct msgBuf *head, uint8_t *data, uint16_t len);
//void AppendNode(uint8_t *pbuf, uint8_t len);
void DispLink(struct msgBuf *head);
void DeleteMemory(struct msgBuf *head);
void wx_show( void );
//void sappEnqueue( uint8_t *pbuf, uint8_t len );

#endif /* WX_LINK_H_ */
