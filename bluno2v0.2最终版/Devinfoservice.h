
#ifndef Devinfoservice_H_
#define Devinfoservice_H_

#include <stdbool.h>
#include <stdint.h>
#include <ble_service.h>


#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
#define BLE_VERSION "V0.20"
#define DEFPASSWORD "DFRobot"
#define BLE_NAME "Bluno2"

// Device Information Service Parameters
#define DEVINFO_SYSTEM_ID                 0
#define DEVINFO_MODEL_NUMBER              1
#define DEVINFO_SERIAL_NUMBER             2
#define DEVINFO_FIRMWARE_REV              3
#define DEVINFO_HARDWARE_REV              4
#define DEVINFO_SOFTWARE_REV              5
#define DEVINFO_MANUFACTURER_NAME         6
#define DEVINFO_11073_CERT_DATA           7
#define DEVINFO_PNP_ID                    8

// Device Information Service UUIDs
#define DEVINFO_SERV_UUID                 0x180A    // Service UUID
#define DEVINFO_SYSTEM_ID_UUID            0x2A23    // System ID
#define DEVINFO_MODEL_NUMBER_UUID         0x2A24    // Model Number String
#define DEVINFO_SERIAL_NUMBER_UUID        0x2A25    // Serial Number String
#define DEVINFO_FIRMWARE_REV_UUID         0x2A26    // Firmware Revision String
#define DEVINFO_HARDWARE_REV_UUID         0x2A27    // Hardware Revision String
#define DEVINFO_SOFTWARE_REV_UUID         0x2A28    // Software Revision String
#define DEVINFO_MANUFACTURER_NAME_UUID    0x2A29    // Manufacturer Name String
#define DEVINFO_11073_CERT_DATA_UUID      0x2A2A    // IEEE 11073-20601 Regulatory Certification Data List
#define DEVINFO_PNP_ID_UUID               0x2A50    // PnP ID

// IEEE 11073 authoritative body values
#define DEVINFO_11073_BODY_EMPTY          0
#define DEVINFO_11073_BODY_IEEE           1
#define DEVINFO_11073_BODY_CONTINUA       2
#define DEVINFO_11073_BODY_EXP            254

// System ID length
#define DEVINFO_SYSTEM_ID_LEN             8

  // PnP ID length
#define DEVINFO_PNP_ID_LEN                7

uint8_t ble_connect;

/**
 * \brief IAS alert level callback
 *
 * \param [in] conn_idx         connection index
 * \param [in] level            devinfo level
 *
 */
typedef void (* devinfo_alert_level_cb_t) (uint16_t conn_idx, const bd_address_t *address, uint8_t level);




ble_service_t *devinfo_init(devinfo_alert_level_cb_t devinfo_level_cb);

#endif /* IAS_H_ */
