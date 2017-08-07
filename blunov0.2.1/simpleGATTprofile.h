/*
 * simpleGATTprofile.h
 *
 *  Created on: 2017Äê2ÔÂ21ÈÕ
 *      Author: Administrator
 */

#ifndef SIMPLEGATTPROFILE_H_
#define SIMPLEGATTPROFILE_H_


#include <stdbool.h>
#include <stdint.h>
#include <ble_service.h>
#include <ad_uart.h>



#define SIMPLEPROFILE_CHAR1                   0  // RW uint8 - Profile Characteristic 1 value
#define SIMPLEPROFILE_CHAR2                   1  // RW uint8 - Profile Characteristic 2 value
#define SIMPLEPROFILE_CHAR3                   2  // RW uint8 - Profile Characteristic 3 value
#define SIMPLEPROFILE_CHAR4                   3  // RW uint8 - Profile Characteristic 4 value
#define SIMPLEPROFILE_CHAR5                   4  // RW uint8 - Profile Characteristic 4 value

// Simple Profile Service UUID
#define SIMPLEPROFILE_SERV_UUID               0xDFB0

// Key Pressed UUID
#define SIMPLEPROFILE_CHAR1_UUID            0xDFB1
#define SIMPLEPROFILE_CHAR2_UUID            0xDFB2
#define SIMPLEPROFILE_CHAR3_UUID            0xDFB3
#define SIMPLEPROFILE_CHAR4_UUID            0xDFB4
#define SIMPLEPROFILE_CHAR5_UUID            0xDFB5

// Simple Keys Profile Services bit fields
#define SIMPLEPROFILE_SERVICE               0x00000001

// Length of Characteristic 5 in bytes
#define SIMPLEPROFILE_CHAR5_LEN           5



#define bleMAX_SPP_PACKET_LEN        20
struct blemsgBuf{
        struct blemsgBuf *next;
        uint16_t handle;
        uint16_t len;
        uint8_t data[0];

};
uint16_t myconn_idx;
uint16_t EnqueueData;
//struct blemsgBuf * blesappEnqueue(struct blemsgBuf *head,uint8_t *pbuf, uint8_t len);
//struct blemsgBuf *bleAppendNode(struct blemsgBuf *head, uint8_t *data, uint16_t len);
void bleAppendNode(uint8_t *pbuf, uint16_t len);
void blesappEnqueue(uint8_t *pbuf, uint8_t len);
void bleDispLink(struct blemsgBuf *head);
void bleDeleteMemory(struct blemsgBuf *head);
void wx_bleshow( void );
void serial_blewrite(uint8_t *pbuf, uint8_t len);
void first_serial_blewrite(uint8_t uuid);
void wx_write_notif( void );
void debugcuappSendData(uart_device dev);
void debug2cuappSendData(uart_device dev);
void ble_sendData(void);
typedef struct {
        uint16_t        buffer_len;
        uint8_t         *buffer;
} wxx;

extern void ucappEnqueue(uint8_t *pbuf,uint16_t len);
extern struct blemsgBuf *ucappDequeue( void );
extern void ucappSendData(void);

extern void cuappEnqueue(uint8_t *pbuf,uint16_t len);
extern struct blemsgBuf *cuappDequeue( void );
extern void cuappSendData(void);
extern uint8_t cuapp_next_packet_len(void);

void cmdPacketEnqueue(uint8_t *pbuf, uint8_t len);

void flushAllBuf( void );

struct blemsgBuf *cmdPacketDequeue( void );

/**
 * \brief IAS alert level callback
 *
 * \param [in] conn_idx         connection index
 * \param [in] level            devinfo level
 *
 */
typedef void (* simpleGATT_alert_level_cb_t) (uint16_t conn_idx, const bd_address_t *address, uint8_t level);




ble_service_t *simpleGATT_init(simpleGATT_alert_level_cb_t simpleGATT_level_cb);



#endif /* SIMPLEGATTPROFILE_H_ */
