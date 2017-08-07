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
#include <string.h>
#include "osal.h"
#include "util/queue.h"
#include "ble_att.h"
#include "ble_bufops.h"
#include "ble_common.h"
#include "ble_gatts.h"
#include "ble_uuid.h"
#include "Devinfoservice.h"
#include "simpleGATTprofile.h"


// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     (8)

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     (8)


static uint8 devInfoSystemId[DEVINFO_SYSTEM_ID_LEN] = {0, 0, 0, 0, 0, 0, 0, 0};

static uint8 devInfoModelNumber[] = "DF Bluno";

static uint8 devInfoSerialNumber[] = "0123456789";


static uint8 devInfoFirmwareRev[] = "FW "BLE_VERSION;

static uint8 devInfoHardwareRev[] = "HW V1.7";

static uint8 devInfoSoftwareRev[] = "SW "BLE_VERSION;

static uint8 devInfoMfrName[] = "DFRobot";

static uint8 devInfo11073Cert[] =
{
  DEVINFO_11073_BODY_EXP,      // authoritative body type
  0x00,                       // authoritative body structure type
                              // authoritative body data follows below:
  'e', 'x', 'p', 'e', 'r', 'i', 'm', 'e', 'n', 't', 'a', 'l'
};


  // PnP ID length
#define DEVINFO_PNP_ID_LEN                7
static uint8 devInfoPnpId[DEVINFO_PNP_ID_LEN] =
{
  1,                                      // Vendor ID source (1=Bluetooth SIG)
  LO_UINT16(0x000D), HI_UINT16(0x000D),   // Vendor ID (Texas Instruments)
  LO_UINT16(0x0000), HI_UINT16(0x0000),   // Product ID (vendor-specific)
  LO_UINT16(0x0110), HI_UINT16(0x0110)    // Product version (JJ.M.N)
};


uint8_t scanRspData[] =
{
  // complete name
  0x0F,   // length of this data
  GAP_DATA_TYPE_LOCAL_NAME,
  'D',
  'F',
  'B',
  'L',
  'E',
  'd',
  'u',
  'i',
  'n',
  'o',
  'V',
  '1',
  '.',
  '0',

  // connection interval range
  0x05,   // length of this data
  GAP_DATA_TYPE_SLAVE_CONN_INTV,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_DATA_TYPE_TX_POWER_LEVEL,
  0,       // 0dBm



  // service UUID, to notify central devices what services are included
  // in this peripheral
  #if 1
  0x05,   // length of this data
  GAP_DATA_TYPE_UUID16_LIST_INC,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),
  LO_UINT16(UUID_SERVICE_LLS),
  HI_UINT16(UUID_SERVICE_LLS)
  #else
  0x03,   // length of this data
  GAP_DATA_TYPE_UUID16_LIST_INC,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID )
  #endif

};

uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_DATA_TYPE_FLAGS,
  GAP_DISC_MODE_GEN_DISCOVERABLE | GAP_DISC_MODE_BROADCASTER,

  // in this peripheral
  0x1A,   // length of this data 26byte
  GAP_DATA_TYPE_MANUFACTURER_SPEC,
  /*Apple Pre-Amble*/
  0x4C,
  0x00,
  0x02,
  0x15,
  /*Device UUID (16 Bytes)*/
  0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48,0xD2, 0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0,
  /*Major Value (2 Bytes)*/
  0x00, 0x00,

  /*Minor Value (2 Bytes)*/
  0x00,0x00,

  /*Measured Power*/
  0xC5
};



typedef struct {
        ble_service_t svc;

        // handles
        uint16_t SystemId;
        uint16_t ModelNumber;
        uint16_t SerialNumber;
        uint16_t FirmwareRev;
        uint16_t HardwareRev;
        uint16_t SoftwareRev;
        uint16_t MfrName;
        uint16_t devInfoCert;
        uint16_t PnpId;

        // callbacks
        devinfo_alert_level_cb_t cb;
        queue_t levels;
} devinfo_service_t;

typedef struct {
        void *next;

        uint16_t conn_idx;
        uint8_t level;
} conn_dev_t;

static bool conn_dev_conn_idx_match(const void *data, const void *match_data)
{
        conn_dev_t *conn_dev = (conn_dev_t *) data;
        uint16_t conn_idx = (*(uint16_t *) match_data);

        return conn_dev->conn_idx == conn_idx;
}

static void handle_disconnected_evt(ble_service_t *svc, const ble_evt_gap_disconnected_t *evt)
{
        devinfo_service_t *dev = (devinfo_service_t *) svc;

        if (evt->reason == 0x00 || evt->reason == 0x13 || evt->reason == 0x16) {
                // do not fire callback if disconnection was triggered by either side
                return;
        }

        // fire callback with current Alert Level - app should trigger an alarm
        if (dev->cb) {
                uint8_t level = 0;
                conn_dev_t *conn_dev;

                conn_dev = queue_remove(&dev->levels, conn_dev_conn_idx_match, &evt->conn_idx);

                if (conn_dev) {
                        level = conn_dev->level;
                        OS_FREE(conn_dev);
                }

                dev->cb(evt->conn_idx, &evt->address, level);
        }
}

static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt)
{
        devinfo_service_t *dev = (devinfo_service_t *) svc;
        conn_dev_t *conn_dev;

        if (evt->handle == dev->SystemId) {
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(devInfoSystemId), &devInfoSystemId);
        }else if(evt->handle == dev->ModelNumber){
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(devInfoModelNumber)-1, devInfoModelNumber);
        }else if(evt->handle == dev->SerialNumber){
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(devInfoSerialNumber)-1, devInfoSerialNumber);
        }else if(evt->handle == dev->FirmwareRev){
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(devInfoFirmwareRev)-1, devInfoFirmwareRev);
        }else if(evt->handle == dev->HardwareRev){
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(devInfoHardwareRev)-1, devInfoHardwareRev);
        }else if(evt->handle == dev->SoftwareRev){
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(devInfoSoftwareRev)-1, devInfoSoftwareRev);
        }else if(evt->handle == dev->MfrName){
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(devInfoMfrName)-1, devInfoMfrName);
        }else if(evt->handle == dev->devInfoCert){
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(devInfo11073Cert)-1, devInfo11073Cert);
        }else if(evt->handle == dev->PnpId){
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(devInfoPnpId), &devInfoPnpId);
        }
}

static void handle_write_req(ble_service_t *svc, const ble_evt_gatts_write_req_t *evt)
{
        devinfo_service_t *dev = (devinfo_service_t *) svc;
        att_error_t err = ATT_ERROR_OK;
        conn_dev_t *conn_dev;
        uint8_t level = get_u8(evt->value);

        if (evt->length == 1) {
                if (level > 2) {
                        err = ATT_ERROR_APPLICATION_ERROR;
                } else {
                        conn_dev = queue_find(&dev->levels, conn_dev_conn_idx_match, &evt->conn_idx);

                        if (!conn_dev) {
                                conn_dev = OS_MALLOC(sizeof(*conn_dev));
                                conn_dev->conn_idx = evt->conn_idx;
                                queue_push_front(&dev->levels, conn_dev);
                        }

                        conn_dev->level = level;
                }
        }

        ble_gatts_write_cfm(evt->conn_idx, evt->handle, err);
}

ble_service_t *devinfo_init(devinfo_alert_level_cb_t alert_cb)
{
        devinfo_service_t *dev;
        uint16_t num_attr;
        att_uuid_t uuid;

        dev = OS_MALLOC(sizeof(*dev));
        memset(dev, 0, sizeof(*dev));
        queue_init(&dev->levels);

        dev->svc.disconnected_evt = handle_disconnected_evt;
        dev->svc.read_req = handle_read_req;
        dev->svc.write_req = handle_write_req;
        dev->cb = alert_cb;

        num_attr = ble_gatts_get_num_attr(0, 9, 0);

        ble_uuid_create16(UUID_SERVICE_DIS, &uuid);
        ble_gatts_add_service(&uuid, GATT_SERVICE_PRIMARY, num_attr);

        ble_uuid_create16(DEVINFO_SYSTEM_ID_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, GATT_PROP_READ, ATT_PERM_RW, sizeof(uint8_t), 1,NULL, &dev->SystemId);

        ble_uuid_create16(DEVINFO_MODEL_NUMBER_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, GATT_PROP_READ, ATT_PERM_RW, sizeof(uint8_t), 1, NULL, &dev->ModelNumber);

        ble_uuid_create16(DEVINFO_SERIAL_NUMBER_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, GATT_PROP_READ, ATT_PERM_RW, sizeof(uint8_t), 1, NULL, &dev->SerialNumber);

        ble_uuid_create16(DEVINFO_FIRMWARE_REV_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, GATT_PROP_READ, ATT_PERM_READ, sizeof(uint8_t), 1, NULL, &dev->FirmwareRev);

        ble_uuid_create16(DEVINFO_HARDWARE_REV_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, GATT_PROP_READ, ATT_PERM_READ, sizeof(uint8_t), 1, NULL, &dev->HardwareRev);

        ble_uuid_create16(DEVINFO_SOFTWARE_REV_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, GATT_PROP_READ, ATT_PERM_READ, sizeof(uint8_t), 1, NULL, &dev->SoftwareRev);


        ble_uuid_create16(DEVINFO_MANUFACTURER_NAME_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, GATT_PROP_READ, ATT_PERM_READ, sizeof(uint8_t), 1, NULL, &dev->MfrName);

        ble_uuid_create16(DEVINFO_11073_CERT_DATA_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, GATT_PROP_READ, ATT_PERM_READ, sizeof(uint8_t), 1, NULL, &dev->devInfoCert);

        ble_uuid_create16(DEVINFO_PNP_ID_UUID, &uuid);
        ble_gatts_add_characteristic(&uuid, GATT_PROP_READ, ATT_PERM_READ, sizeof(uint8_t), 1, NULL, &dev->PnpId);

        ble_gatts_register_service(&dev->svc.start_h, &dev->SystemId, &dev->ModelNumber, &dev->SerialNumber, &dev->FirmwareRev,
                                        &dev->HardwareRev, &dev->SoftwareRev, &dev->MfrName, &dev->devInfoCert, &dev->PnpId, 0);

        dev->svc.end_h = dev->svc.start_h + num_attr;

        return &dev->svc;
}
