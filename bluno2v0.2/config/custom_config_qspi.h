/**
\addtogroup BSP
\{
\addtogroup CONFIG
\{
\addtogroup CUSTOM
\{
*/

/**
****************************************************************************************
*
* @file custom_config_qspi.h
*
* @brief Board Support Package. User Configuration file for cached QSPI mode.
*
* Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
* program includes Confidential, Proprietary Information and is a Trade Secret of
* Dialog Semiconductor. All use, disclosure, and/or reproduction is prohibited
* unless authorized in writing. All Rights Reserved.
*
* <black.orca.support@diasemi.com> and contributors.
*
****************************************************************************************
*/

#ifndef CUSTOM_CONFIG_QSPI_H_
#define CUSTOM_CONFIG_QSPI_H_

#include "bsp_definitions.h"

#define CONFIG_USE_BLE

#define CONFIG_RETARGET
#define CONFIG_RETARGET_UART    HW_UART2



#define dg_configUSE_LP_CLK                     LP_CLK_32768
#define dg_configEXEC_MODE                      MODE_IS_CACHED
#define dg_configCODE_LOCATION                  NON_VOLATILE_IS_FLASH
#define dg_configEXT_CRYSTAL_FREQ               EXT_CRYSTAL_IS_16M

#define dg_configIMAGE_SETUP                    DEVELOPMENT_MODE
#define dg_configEMULATE_OTP_COPY               (0)

#define dg_configUSER_CAN_USE_TIMER1            (0)

/*
 * Controls the retRAM size used by the project.
 * 0: all RAM is retained
 * 1: retention memory size is optimal
 */
#define proj_configOPTIMAL_RETRAM                (0)

#if !defined(RELEASE_BUILD) && (proj_configOPTIMAL_RETRAM == 1)
        /* WARNING: retRAM optimizations are disabled in DEBUG builds! */
        #undef proj_configOPTIMAL_RETRAM
        #define proj_configOPTIMAL_RETRAM       (0)
#elif (dg_configEXEC_MODE != MODE_IS_CACHED)
        /* WARNING: retRAM optimizations are not applicable in MIRRORED mode! */
        #undef proj_configOPTIMAL_RETRAM
        #define proj_configOPTIMAL_RETRAM       (0)
#endif

#if (proj_configOPTIMAL_RETRAM == 0)
        #define dg_configMEM_RETENTION_MODE     (0x1F)
        #define dg_configSHUFFLING_MODE         (0x3)
#else
                #define dg_configMEM_RETENTION_MODE             (0x12)
                #define dg_configSHUFFLING_MODE                 (0x1)
#endif

#define dg_configUSE_WDOG                       (0)


#define dg_configFLASH_CONNECTED_TO             (FLASH_CONNECTED_TO_1V8)
#define dg_configFLASH_POWER_DOWN               (0)

#define dg_configPOWER_1V8_ACTIVE               (1)
#define dg_configPOWER_1V8_SLEEP                (1)

#define dg_configPOWER_1V8P                     (1)

//#define dg_configBATTERY_TYPE                   (BATTERY_TYPE_LIMN2O4)
#define dg_configBATTERY_TYPE                   (BATTERY_TYPE_CUSTOM)
#define dg_configBATTERY_CHARGE_VOLTAGE         0xA     // 4.2V
#define dg_configBATTERY_TYPE_CUSTOM_ADC_VOLTAGE        (3439)
#define dg_configPRECHARGING_THRESHOLD          (2462)  // 3.006V
#define dg_configCHARGING_THRESHOLD             (2498)  // 3.05V
#define dg_configBATTERY_CHARGE_CURRENT         2       // 30mA
#define dg_configBATTERY_PRECHARGE_CURRENT      20      // 2.1mA



#define dg_configBATTERY_CHARGE_NTC             1       // disabled
#define dg_configPRECHARGING_TIMEOUT            (30 * 60 * 100)  // N x 10msec


#define dg_configUSE_USB                        1
#define dg_configUSB_SUSPEND_MODE               1
#define dg_configUSE_USB_CHARGER                1
#define dg_configALLOW_CHARGING_NOT_ENUM        1
#define dg_configUSE_NOT_ENUM_CHARGING_TIMEOUT  0

#define dg_configUSE_ProDK                      (1)

#define dg_configUSE_SW_CURSOR                  (1)
#define dg_configUSE_HW_TRNG                    (1)

/*************************************************************************************************\
 * FreeRTOS specific config
 */
#define OS_FREERTOS                              /* Define this to use FreeRTOS */
#define configTOTAL_HEAP_SIZE                    14000   /* This is the FreeRTOS Total Heap Size */

/*************************************************************************************************\
 * Peripheral specific config
 */
#define usbopen (0)
#define dg_configUSE_HW_I2C                     (1)
#define dg_configUSE_HW_SPI                     (0)

#define dg_configUART_ADAPTER                   (1)
#define dg_configI2C_ADAPTER                    (1)
#define dg_configSPI_ADAPTER                    (0)
#define dg_configFLASH_ADAPTER                  (1)
#define dg_configNVMS_ADAPTER                   (1)
#define dg_configNVMS_VES                       (1)
#define dg_configGPADC_ADAPTER                  (1)
#define dg_configCACHEABLE_QSPI_AREA_LEN        (NVMS_PARAM_PART_start - MEMORY_QSPIF_BASE)

/*************************************************************************************************\
 * BLE device config
 */
#define dg_configBLE_GATT_CLIENT                (1)
#define dg_configBLE_OBSERVER                   (0)
#define dg_configBLE_BROADCASTER                (0)
#define dg_configBLE_L2CAP_COC                  (0)
#define dg_configUSE_HW_USB                     1
#define dg_configUSE_USB_ENUMERATION            1

/* Include bsp default values */
#include "bsp_defaults.h"


/*************************************************************************************************\
 * BLE specific config
 */
//#define BLE_MAX_MISSES_ALLOWED                  (3)
//#define BLE_MAX_DELAYS_ALLOWED                  (3)

#define dg_configDEBUG_TRACE                    0

/*
 * SUOTA loader configuration:
 * - To enable SUOTA over GATT only, set SUOTA_VERSION to any version >= SUOTA_VERSION_1_1
 *      and leave SUOTA_PSM undefined.
 * - To enable SUOTA over GATT and L2CAP CoC, set SUOTA_VERSION to any version >= SUOTA_VERSION_1_2
 *      and also define SUOTA_PSM to match the desired PSM. In this case the central device
 *      can use either of both according to its preference.
 */
#define SUOTA_VERSION                           SUOTA_VERSION_1_3
#define SUOTA_PSM                               0x81

/*
 * When enabled, a special button is detected as having been pressed or not during boot.
 * If the button is pressed, SUOTA service will be started without booting any flashed
 * application. This allows the user to force SUOTA service in certain circumstances.
 */
#define CFG_FORCE_SUOTA_GPIO                    (0)

#define USE_PARTITION_TABLE_1MB_WITH_SUOTA


/*************************************************************************************************\
 * Memory layout configuration
 */
#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
        #define CODE_SIZE     (128 * 1024)

        #if (dg_configEXEC_MODE == MODE_IS_CACHED)
                        /* DA14681-01
                         * RAM goes first, RetRAM0 follows. RetRAM1 is added at the beginning when
                         * optimized RetRAM configuration is used (so that the IVT is preserved).
                         * RAM size should be defined such that it covers the whole empty space
                         * between RetRAM1, if it exists, and RetRAM0.
                         */
                        #define RETRAM_FIRST    0

                        #if (proj_configOPTIMAL_RETRAM == 0)
                                #define RAM_SIZE        ( 32 * 1024)
                                #define RETRAM_0_SIZE   ( 64 * 1024)
                                #define RETRAM_1_SIZE   ( 32 * 1024)
                        #else
                                #define RAM_SIZE        ( 72 * 1024)
                                #define RETRAM_0_SIZE   ( 32 * 1024)
                                #define RETRAM_1_SIZE   ( 24 * 1024)
                        #endif
        #else // MIRRORED
                #error "QSPI mirrored mode is not supported!"
        #endif

#else
        #error "Unknown configuration..."
#endif

#endif /* CUSTOM_CONFIG_QSPI_H_ */

/**
\}
\}
\}
*/
