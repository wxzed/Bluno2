/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief BLE multi-link demo application
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

#include <string.h>
#include <stdbool.h>

#include "osal.h"
#include "resmgmt.h"
#include "ad_ble.h"
#include "ad_nvms.h"
#include "ble_mgr.h"
#include "hw_gpio.h"
#include "sys_clock_mgr.h"
#include "sys_power_mgr.h"
#include "sys_watchdog.h"
#include "platform_devices.h"


#include "suota.h"
#include "hw_timer1.h"
#include "stateMachine.h"
//#include "dlg_suota.h"
#include "SerialApp.h"
#include "Serialsend.h"
#include "hw_uart.h"
#include "USB_CDC.h"
#include "gpio_setup.h"
#include "Md5.h"
#include "Common.h"
#include "hw_otpc.h"


/* Task priorities */
#define mainBLE_MULTI_LINK_TASK_PRIORITY              ( OS_TASK_PRIORITY_NORMAL )

#if (dg_configTRACK_OS_HEAP == 1)
/*
 * ConstantsVariables used for Tasks Stack and OS Heap tracking
 * Declared global to avoid IDLE stack Overflows
 */
#define mainMAX_NB_OF_TASKS           10
#define mainMIN_STACK_GUARD_SIZE      8 /* words */
#define mainTOTAL_HEAP_SIZE_GUARD     64 /*bytes */

TaskStatus_t pxTaskStatusArray[mainMAX_NB_OF_TASKS];
uint32_t ulTotalRunTime;
#endif /* (dg_configTRACK_OS_HEAP == 1) */

/* The configCHECK_FOR_STACK_OVERFLOW setting in FreeRTOSConifg can be used to
check task stacks for overflows.  It does not however check the stack used by
interrupts.  This demo has a simple addition that will also check the stack used
by interrupts if mainCHECK_INTERRUPT_STACK is set to 1.  Note that this check is
only performed from the tick hook function (which runs in an interrupt context).
It is a good debugging aid - but won't catch interrupt stack problems until the
tick interrupt next executes. */
//#define mainCHECK_INTERRUPT_STACK			1
#if mainCHECK_INTERRUPT_STACK == 1
const unsigned char ucExpectedInterruptStackValues[] = { 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC };
#endif

#if dg_configUSE_WDOG
INITIALISED_PRIVILEGED_DATA int8_t idle_task_wdog_id = -1;
#endif

/*
 * Perform any application specific hardware configuration.  The clocks,
 * memory, etc. are configured before main() is called.
 */




#define FLASH_SECTOR_SIZE       (4096)
static uint8_t sector_buffer[FLASH_SECTOR_SIZE];
/* Task priorities */
#define mainBLE_SUOTA_TASK_PRIORITY              ( tskIDLE_PRIORITY + 1 )

#define NVMS_MINIMUM_STACK      768

static bool force_suota = false;
/*
 * Offset of image header inside partition.
 */
#define SUOTA_IMAGE_HEADER_OFFSET       0






static void prvSetupHardware( void );
/*
 * Task functions .
 */
void ble_multi_link_task(void *params);

static OS_TASK handle = NULL;

/**
 * @brief System Initialization and creation of the BLE task
 */





#if dg_configDEBUG_TRACE
#define TRACE(...) printf(__VA_ARGS__)
#else
#define TRACE(...)
#endif


static const uint32_t wx_crc32_tab[] = {
        0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
        0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
        0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
        0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
        0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
        0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
        0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
        0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
        0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
        0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
        0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
        0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
        0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
        0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
        0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
        0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
        0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
        0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
        0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
        0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
        0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
        0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
        0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
        0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
        0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
        0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
        0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
        0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
        0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
        0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
        0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
        0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
        0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
        0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
        0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
        0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
        0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
        0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
        0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
        0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
        0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
        0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
        0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

uint32_t wx_suota_update_crc(uint32_t crc, const uint8_t *data, size_t len)
{
        while (len--) {
                crc = wx_crc32_tab[(crc ^ *data++) & 0xff] ^ (crc >> 8);
        }
        return crc;
}

static void periph_deinit(void)
{
#if LOADER_UART
        while (!hw_uart_is_tx_fifo_empty(CONFIG_RETARGET_UART)) {
        }
        /* Configure pins used for UART as input, since UART can be used on other pins in app */
        hw_gpio_set_pin_function(UART_TX_PORT, UART_TX_PIN, HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
        hw_gpio_set_pin_function(UART_RX_PORT, UART_RX_PIN, HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
#endif

#if (CFG_FORCE_SUOTA_GPIO == 1)
        hw_gpio_set_pin_function(CFG_FORCE_SUOTA_GPIO_PORT, CFG_FORCE_SUOTA_GPIO_PIN,
                                                                HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
#endif /* (CFG_FORCE_SUOTA_GPIO == 1) */

        /* Timer1 is used for ticks in OS disable it for now */
        hw_timer1_disable();
        /* Restore initial clock settings */
        cm_sys_clk_set(sysclk_XTAL16M);
        hw_cpm_pll_sys_off();
}


static void reboot(void)
{
        /*
         * Reset platform
         */
        __disable_irq();
        REG_SETF(CRG_TOP, SYS_CTRL_REG, SW_RESET, 1);
}

static bool image_ready(suota_1_1_image_header_t *header)
{
        /* Is header ready for update */
        if ((header->flags & SUOTA_1_1_IMAGE_FLAG_VALID) &&
                                header->signature[0] == SUOTA_1_1_IMAGE_HEADER_SIGNATURE_B1 &&
                                header->signature[1] == SUOTA_1_1_IMAGE_HEADER_SIGNATURE_B2) {
                return true;
        }
        return false;
}

static bool valid_image(suota_1_1_image_header_t *header, nvms_t exec_part, uint32_t exec_location,
                                                                        bool force_crc_check)
{
        const uint8_t *mapped_ptr;
        uint32_t crc;

        if (header == NULL || exec_part == NULL) {
                return false;
        }

        /*
         * Check CRC can be forced by image (then on every start CRC will be checked)
         * If it is not forced it will be checked anyway before image is copied to executable
         * partition.
         */
        if (!force_crc_check && (0 == (header->flags & SUOTA_1_1_IMAGE_FLAG_FORCE_CRC))) {
                return true;
        }

        crc = ~0; /* Initial value of CRC prepared by mkimage */
        /*
         * Utilize QSPI memory mapping for CRC check, this way no additional buffer is needed.
         */
        if (header->code_size != ad_nvms_get_pointer(exec_part, exec_location, header->code_size,
                                                                (const void **) &mapped_ptr)) {
                return false;
        }
        TRACE("Checking image CRC.\r\n");
        crc = wx_suota_update_crc(crc, mapped_ptr, header->code_size);
        crc ^= ~0; /* Final XOR */

        return crc == header->crc;
}

static bool image_sanity_check(int32_t *image_address)
{
        /*
         * Test reset vector for sanity:
         * - greater then image address
         * - address is odd for THUMB instruction
         */
        if (image_address[1] < (int32_t) image_address || (image_address[1] & 1) == 0) {
                return false;
        }
        return true;
}

static inline bool read_image_header(nvms_t part, size_t offset,
                                                                suota_1_1_image_header_t *header)
{
        if (!part) {
                return false;
        }

        ad_nvms_read(part, offset, (uint8_t *) header, sizeof(*header));
        return true;
}

static bool update_image(suota_1_1_image_header_t *new_header, nvms_t update_part,
                                                        nvms_t exec_part, nvms_t header_part)
{
        size_t left;
        size_t src_offset;
        size_t dst_offset;
        suota_1_1_image_header_t header;
        bool exec_image_valid = false;

        /*
         * Erase header partition. New header will be written after executable is copied.
         */
        if (!ad_nvms_erase_region(header_part, 0, sizeof(suota_1_1_image_header_t))) {
                return false;
        }

        /*
         * Erase executable partition.
         */
        if (!ad_nvms_erase_region(exec_part, 0, new_header->code_size)) {
                return false;
        }

        left = new_header->code_size;   /* Whole image to copy */
        dst_offset = 0;                 /* Write from the beginning of executable partition */
        src_offset = SUOTA_IMAGE_HEADER_OFFSET + new_header->exec_location;

        while (left > 0) {
                size_t chunk = left > FLASH_SECTOR_SIZE ? FLASH_SECTOR_SIZE : left;

                ad_nvms_read(update_part, src_offset, sector_buffer, chunk);
                ad_nvms_write(exec_part, dst_offset, sector_buffer, chunk);

                left -= chunk;
                src_offset += chunk;
                dst_offset += chunk;
        }

        /*
         * Header is in different partition than executable.
         * Executable is a the beginning of partition, change location to 0.
         */
        header = *new_header;

        if (new_header->flags & SUOTA_1_1_IMAGE_FLAG_RETRY2) {
                new_header->flags ^= SUOTA_1_1_IMAGE_FLAG_RETRY2;
        } else if (new_header->flags & SUOTA_1_1_IMAGE_FLAG_RETRY1) {
                new_header->flags ^= SUOTA_1_1_IMAGE_FLAG_RETRY1;
        } else {
                new_header->signature[0] = 0;
                new_header->signature[0] = 1;
                new_header->flags &= ~SUOTA_1_1_IMAGE_FLAG_VALID;
        }

        exec_image_valid = valid_image(&header, exec_part, 0, true);
        if (exec_image_valid) {
                /*
                 * Write image header, so it can be used later and in subsequent reboots.
                 */
                ad_nvms_write(header_part, 0, (uint8_t *) &header, sizeof(header));
                /*
                 * Mark header from update partition as invalid since it will not be used any more.
                 */
                new_header->signature[0] = 0;
                new_header->signature[0] = 1;
                new_header->flags &= ~SUOTA_1_1_IMAGE_FLAG_VALID;
        }
        /*
         * Write header to update partition. It can be invalid header if update was ok.
         * If number of retries run out, this header will also be written so no further tries
         * with image in update partition will be performed.
         */
        ad_nvms_write(update_part, SUOTA_IMAGE_HEADER_OFFSET, (uint8_t *) new_header,
                                                                        sizeof(*new_header));

        if (!exec_image_valid) {
                /*
                 * New image is not valid. Reboot it can result in yet another try or with SUOTA.
                 */
                reboot();
        }
        return true;
}


void boot_application(void)
{
        nvms_t update_part;
        nvms_t exec_part;
        nvms_t header_part;
        int32_t *int_vector_table = (int32_t *) 0;
        int32_t *image_address;
        suota_1_1_image_header_t new_header = { {0} };
        suota_1_1_image_header_t current_header = { {0} };

        TRACE("\r\nBootloader started.\r\n");

        if (force_suota) {
                return;
        }

#if (CFG_FORCE_SUOTA_GPIO == 1)
        TRACE("Checking status of K1 Button..\r\n");
        if (!hw_gpio_get_pin_status(CFG_FORCE_SUOTA_GPIO_PORT, CFG_FORCE_SUOTA_GPIO_PIN)) {
                TRACE("K1 Button is pressed, starting SUOTA service without booting application.\r\n");
                return;
        }
#endif /* (CFG_FORCE_SUOTA_GPIO == 1) */

        update_part = ad_nvms_open(NVMS_FW_UPDATE_PART);
        exec_part = ad_nvms_open(NVMS_FW_EXEC_PART);
        header_part = ad_nvms_open(NVMS_IMAGE_HEADER_PART);

        TRACE("Checking for update image.\r\n");
        read_image_header(update_part, SUOTA_IMAGE_HEADER_OFFSET, &new_header);

        if (image_ready(&new_header)) {
                /* Check if there is valid image for update, check CRC */
                if (valid_image(&new_header, update_part, new_header.exec_location, true)) {
                        TRACE("Updating image.\r\n");
                        update_image(&new_header, update_part, exec_part, header_part);
                } else {
                        TRACE("New image invalid, erasing.\r\n");
                        /* Update image not good, just erase it and start whatever is there */
                        new_header.signature[0] = 0;
                        new_header.signature[1] = 0;
                        ad_nvms_write(update_part, SUOTA_IMAGE_HEADER_OFFSET,
                                                (uint8_t *) &new_header, sizeof(new_header));
                }
        }

        /*
         * Check if current image is valid, CRC can be forced by image header but it is not
         * forced here.
         */
        read_image_header(header_part, 0, &current_header);
        TRACE("Validating current image.\r\n");
        if (!valid_image(&current_header, exec_part, 0, false)) {
                TRACE("Current image invalid, starting SUOTA.\r\n");
                return;
        }

        /*
         * The following code assumes that code will be executed in QSPI cached mode.
         *
         * The binary image that is stored in the QSPI flash must be compiled for a specific
         * address, other than address 0x0 (or 0x8000000) since this is where the boot loader is
         * stored.
         * The binary images that are stored in the QSPI Flash, except for the boot loader image,
         * must not be modified in any way before flashed. No image header must be preceded. The
         * images start with the initial stack pointer and the reset handler and the rest of the
         * vector table and image code and data follow.
         * The complete vector table of the application image is copied from the image location
         * to the RAM.
         */
        if (256 != ad_nvms_get_pointer(exec_part, 0, 256, (const void **) &image_address)) {
                return;
        }

        /* Check sanity of image */
        if (!image_sanity_check(image_address)) {
                TRACE("Current executable insane, starting SUOTA.\r\n");
                return;
        }

        TRACE("Starting image at 0x%X, reset vector 0x%X.\r\n", (unsigned int) image_address,
                                                                (unsigned int) image_address[1]);
        /*
         * In OS environment some interrupt could already be enabled, disable all before
         * interrupt vectors are changed.
         */
        __disable_irq();

        /*
         * Copy interrupt vector table from image. We only care to copy the
         * address of the reset handler here, to perform software reset properly.
         * The actual VT copy using the correct shuffling value will be done
         * by the reset handler.
         * */
        memcpy(int_vector_table, image_address, 0x100);

        /*
         * If bootloader changed any configuration (GPIO, clocks) it should be uninitialized here
         */
        periph_deinit();

        /*
         * Reset platform
         */
        reboot();
        for (;;) {
        }
}

uint32_t cifang1(uint8_t a,uint8_t n){
        uint32_t sum=a;
        for(int i=1;i<n;i++){
                sum=sum*16;
        }
        return sum;
}
uint8_t* my_data_1(uint32_t data){
        uint8_t data1[8];
        uint8_t data_2[4];

        for(int i = 0 ;i<8; i++){
                if(i == 0){
                        data1[i]=(data)%16;
                }else{
                        data1[i]=(data/cifang1(16,i))%16;
                }
        }
        for(int i = 0; i<4; i++){
                data_2[i]=data1[i*2]+data1[i*2+1]*10;
        }
        return data_2;
}
static void test_otp_mac_flash(uint32_t first,uint32_t next){
        uint8_t first_8[8];
        uint8_t next_8[8];
        uint8_t first_4[4];
        uint8_t next_4[4];
        uint8_t otp_mac[6];
        uint8 decrypt[17];       //存放加密后的结果
        uint8 addr_ascii[]={"0x000000000000"};
        MD5_CTX *md5;
        md5 = (MD5_CTX *)OS_MALLOC(sizeof(MD5_CTX));
        MD5Init(md5);                          //初始化用于md5加密的结构

        first_8[0] = first%16;
        for(int i=1; i<8; i++){
                first_8[i] = (first/cifang1(16,i))%16;
        }
        for(int i = 0; i<4; i++){
                first_4[i] = first_8[2*i]+first_8[2*i+1]*16;
                printf("first_4[%d]=0x%02x\r\n",i,first_4[i]);
        }
        for(int i = 0; i<4; i++){//把读出来的MAC地址解析后放在flash中前4位flash;
                otp_mac[i+2] = first_4[i];
        }
        next_8[0] = next%16;
        for(int i=1; i<8; i++){
                next_8[i] = (next/cifang1(16,i))%16;
        }
        for(int i = 0; i<4; i++){
                next_4[i] = next_8[2*i]+next_8[2*i+1]*16;
                printf("next_4[%d]=0x%02x\r\n",i,next_4[i]);
        }
        for(int i = 0; i<2; i++){///后两位
                otp_mac[i] = next_4[i];
        }
        for(int i = 0;i<6;i++){
                si.sys_cfg.my_mac[i] = otp_mac[i];
        };
        bdAddr2Str(addr_ascii, otp_mac);
        MD5Update(md5,(const uint8*)addr_ascii,14);   //对欲加密的字符进行加密
        MD5Final(decrypt,md5);
        OS_FREE(md5);
        for(int i = 0; i<17;i++){
              printf("decrypt[%d] = 0x%02x\r\n",i,decrypt[i]);
        }
        if(memcmp(decrypt,si.sys_cfg.md5,9) == 0){
                si.pass = 1;
                if(memcmp(decrypt,si.sys_cfg.md5,16) != 0){
                    memcpy(si.sys_cfg.md5,decrypt,16);
                }
        }else{
                si.pass = 0;
        }
        nvms_t NVID_SYS_CONFIG;
        NVID_SYS_CONFIG = ad_nvms_open(NVMS_PARAM_PART);
        ad_nvms_write(NVID_SYS_CONFIG, 0, (const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));

}
static void read_otp_mac(uint8_t *buf){
        hw_otpc_init();
        hw_otpc_set_speed(5);
        uint32_t mydata[8]={943076401,541409585,2863311530,0,0,0,0,0};
        uint8_t* otp_mac0;
        uint8_t* otp_mac1;
        uint8_t* otp_mac2;
        //hw_otpc_dma_read(mydata,7492,0,8,false);
        otp_mac0 = my_data_1(mydata[0]);
        otp_mac1 = my_data_1(mydata[1]);
        otp_mac2 = my_data_1(mydata[2]);
        printf("otp_mac2=%02x\r\n",mydata[2]);
        for(int i =0;i<4;i++){
                buf[i]=otp_mac0[i];
                //printf("otp_mac0[%d]=%d\r\n",i,otp_mac0[i]);
        }
        for(int i =0;i<2;i++){
                buf[i+4]=otp_mac1[i];
                //printf("otp_mac1[%d]=%d\r\n",i,otp_mac1[i]);
        }
        for(int i = 0;i<4;i++){
                printf("otp_mac2[%d]=%d\r\n",i,otp_mac2[i]);
        }
}
uint8_t *data_change(uint8_t *buf,uint8_t len){
        uint8_t *data_buf;
        for(int i = 0; i<len; i++){
                data_buf[i]=(buf[i]%10)+(buf[i]/10)*16;
        }
        return data_buf;
}
static void pass_word(){
        uint8_t mac[6]={31,32,33,34,35,41};
        uint8_t mybuf[6];
        for(int i = 0; i<6; i++){
                mybuf[i]=(mac[i]%10)+(mac[i]/10)*16+(mac[i]/100)*16*16;
        }
        //read_otp_mac(mac);
        nvms_t NVID_SYS_CONFIG;
        NVID_SYS_CONFIG = ad_nvms_open(NVMS_PARAM_PART);
        uint8 decrypt[17];       //存放加密后的结果
        uint8 addr_ascii[]={"0x000000000000"};
        MD5_CTX *md5;
        md5 = (MD5_CTX *)OS_MALLOC(sizeof(MD5_CTX));
        MD5Init(md5);                          //初始化用于md5加密的结构
        bdAddr2Str(addr_ascii, mybuf);
        //bdAddr2Str(addr_ascii, si.mac );
        hw_uart_send(HW_UART2, addr_ascii, 14, NULL, NULL);
        MD5Update(md5,(const uint8*)addr_ascii,14);   //对欲加密的字符进行加密
        MD5Final(decrypt,md5);
        OS_FREE(md5);
        if(memcmp(decrypt,si.sys_cfg.md5,9) == 0){
                si.pass = 1;
                if(memcmp(decrypt,si.sys_cfg.md5,16) != 0){
                    memcpy(si.sys_cfg.md5,decrypt,16);
                    //ad_nvms_write(NVID_SYS_CONFIG, 0,(const uint8_t *)&si.sys_cfg, sizeof(sys_config_t));
                }
        }
}
void ble_App_stop(){
        OS_TASK_DELETE(ble_multi_link_task);
}
static void system_init( void *pvParameters )
{

#if defined CONFIG_RETARGET
        extern void retarget_init(void);
#endif

        /* Prepare clocks. Note: cm_cpu_clk_set() and cm_sys_clk_set() can be called only from a
         * task since they will suspend the task until the XTAL16M has settled and, maybe, the PLL
         * is locked.
         */
        cm_sys_clk_init(sysclk_XTAL16M);
        cm_apb_set_clock_divider(apb_div1);
        cm_ahb_set_clock_divider(ahb_div1);
        cm_lp_clk_init();

        /*
         * Initialize platform watchdog
         */
        sys_watchdog_init();

#if dg_configUSE_WDOG
        // Register the Idle task first.
        idle_task_wdog_id = sys_watchdog_register(false);
        ASSERT_WARNING(idle_task_wdog_id != -1);
        sys_watchdog_configure_idle_id(idle_task_wdog_id);
#endif

        /* Set system clock */
        cm_sys_clk_set(sysclk_PLL96);

        /* Prepare the hardware to run this demo. */
        prvSetupHardware();

        /* init resources */
        resource_init();


        boot_application();/*写*。img*/

        /* Set the desired sleep mode. */
        //pm_set_wakeup_mode(true);
        //pm_set_sleep_mode(pm_mode_hibernation);
        fsm_init();

#if defined CONFIG_RETARGET
        retarget_init();
#endif

        /* Initialize BLE Manager */
        ble_mgr_init();

        Serial_App_init();
        Serial_send_init();
        /*
        uint32_t otp_mac[8];
        hw_otpc_init();
        hw_otpc_set_speed(5);
        hw_otpc_dma_read(otp_mac,7492,0,8,false);
        test_otp_mac_flash(otp_mac[0],otp_mac[1]);
        */
        //test_otp_mac_flash(2162836184,45616);
        //cm_sys_clk_set(sysclk_PLL96);

        //pass_word();
        //uint8_t *buf;
        //read_otp_mac(buf);
        //cm_sys_clk_set(sysclk_PLL96);
        /* Start the Multi-Link application task. */
        //if(si.pass == 1){
                OS_TASK_CREATE("Multi-Link",                    /* The text name assigned to the task, for
                                                                   debug only; not used by the kernel. */
                               ble_multi_link_task,             /* The function that implements the task. */
                               NULL,                            /* The parameter passed to the task. */
                               500 * OS_STACK_WORD_SIZE,        /* The number of bytes to allocate to the
                                                                   stack of the task. */
                               mainBLE_MULTI_LINK_TASK_PRIORITY,/* The priority assigned to the task. */
                               handle);                         /* The task handle. */
                OS_ASSERT(handle);
        //}
        /* the work of the SysInit task is done */
        OS_TASK_DELETE(OS_GET_CURRENT_TASK());

}
/*-----------------------------------------------------------*/

/**
 * @brief Basic initialization and creation of the system initialization task.
 */
int main( void )
{
        OS_BASE_TYPE status;

        cm_clk_init_low_level();                            /* Basic clock initializations. */

        /* Start the two tasks as described in the comments at the top of this
        file. */
        status = OS_TASK_CREATE("SysInit",                /* The text name assigned to the task, for
                                                             debug only; not used by the kernel. */
                                system_init,              /* The System Initialization task. */
                                ( void * ) 0,             /* The parameter passed to the task. */
                                1024,                     /* The number of bytes to allocate to the
                                                             stack of the task. */
                                OS_TASK_PRIORITY_HIGHEST, /* The priority assigned to the task. */
                                handle);                  /* The task handle */
        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);

        /* Start the tasks and timer running. */
        vTaskStartScheduler();

        /* If all is well, the scheduler will now be running, and the following
        line will never be reached.  If the following line does execute, then
        there was insufficient FreeRTOS heap memory available for the idle and/or
        timer tasks     to be created.  See the memory management section on the
        FreeRTOS web site for more details. */
        for( ;; );
}



/* GPIO pins configuration array, see periph_setup() for details */
static const gpio_config gpio_cfg[] = {

#if CFG_DEMO_HW_TIMER0
        /* Timer0 timer */
        HW_GPIO_PINCONFIG(CFG_GPIO_TIMER0_PORT,  CFG_GPIO_TIMER0_PIN,      OUTPUT,       PWM0,      true),
#endif // CFG_DEMO_HW_TIMER0

#if CFG_DEMO_HW_GPADC || CFG_DEMO_AD_GPADC
        /* General Purpose ADC */
        //HW_GPIO_PINCONFIG(0,                     6,                         INPUT,        ADC,       true),
        HW_GPIO_PINCONFIG(0,                     7,                         INPUT,        ADC,       true),
        //HW_GPIO_PINCONFIG(1,                     0,                         INPUT,        ADC,       true),
        HW_GPIO_PINCONFIG(1,                     2,                         INPUT,        ADC,       true),
#if dg_configBLACK_ORCA_MB_REV != BLACK_ORCA_MB_REV_D
        HW_GPIO_PINCONFIG(1,                     3,                         INPUT,        ADC,       true),
#endif
        HW_GPIO_PINCONFIG(1,                     4,                         INPUT,        ADC,       true),
        //HW_GPIO_PINCONFIG(1,                     5,                         INPUT,        ADC,       true),
        //HW_GPIO_PINCONFIG(2,                     4,                         INPUT,        ADC,       true),
#endif // CFG_DEMO_HW_GPADC

#if CFG_DEMO_HW_IRGEN
        /* IR generator */
        HW_GPIO_PINCONFIG(CFG_GPIO_IR_PORT,      CFG_GPIO_IR_PIN,          OUTPUT,       IR_OUT,    true),
#endif // CFG_DEMP_HW_IRGEN

#if CFG_DEMO_HW_QUAD
        /* Quadrature decoder */
        HW_GPIO_PINCONFIG(CFG_GPIO_QUAD_XA_PORT, CFG_GPIO_QUAD_XA_PIN,     INPUT,        QUADEC_XA, true),
        HW_GPIO_PINCONFIG(CFG_GPIO_QUAD_XB_PORT, CFG_GPIO_QUAD_XB_PIN,     INPUT,        QUADEC_XB, true),
        HW_GPIO_PINCONFIG(CFG_GPIO_QUAD_YA_PORT, CFG_GPIO_QUAD_YA_PIN,     INPUT,        QUADEC_YA, true),
        HW_GPIO_PINCONFIG(CFG_GPIO_QUAD_YB_PORT, CFG_GPIO_QUAD_YB_PIN,     INPUT,        QUADEC_YB, true),
        HW_GPIO_PINCONFIG(CFG_GPIO_QUAD_ZA_PORT, CFG_GPIO_QUAD_ZA_PIN,     INPUT,        QUADEC_ZA, true),
        HW_GPIO_PINCONFIG(CFG_GPIO_QUAD_ZB_PORT, CFG_GPIO_QUAD_ZB_PIN,     INPUT,        QUADEC_ZB, true),
#endif // CFG_DEMO_HW_QUAD

#if CFG_DEMO_HW_TIMER1
        /* Timer1 */
        HW_GPIO_PINCONFIG(CFG_GPIO_TIMER1_PWM_PORT,
                                                 CFG_GPIO_TIMER1_PWM_PIN,  OUTPUT,       PWM5,      true),
#endif // CFG_DEMO_HW_TIMER1

#if CFG_DEMO_HW_TIMER2
        /* Timer2 */
        HW_GPIO_PINCONFIG(CFG_GPIO_TIMER2_PWM2_PORT,
                                                 CFG_GPIO_TIMER2_PWM2_PIN, OUTPUT,       PWM2,      true),
        HW_GPIO_PINCONFIG(CFG_GPIO_TIMER2_PWM3_PORT,
                                                 CFG_GPIO_TIMER2_PWM3_PIN, OUTPUT,       PWM3,      true),
        HW_GPIO_PINCONFIG(CFG_GPIO_TIMER2_PWM4_PORT,
                                                 CFG_GPIO_TIMER2_PWM4_PIN, OUTPUT,       PWM4,      true),
#endif // CFG_DEMO_HW_WKUP

#if CFG_DEMO_HW_WKUP
        /* Wakeup timer */
        HW_GPIO_PINCONFIG(CFG_GPIO_WKUP_1_PORT,  CFG_GPIO_WKUP_1_PIN,      INPUT_PULLUP, GPIO,      true),
        HW_GPIO_PINCONFIG(CFG_GPIO_WKUP_2_PORT,  CFG_GPIO_WKUP_2_PIN,      INPUT_PULLUP, GPIO,      true),
        HW_GPIO_PINCONFIG(CFG_GPIO_WKUP_3_PORT,  CFG_GPIO_WKUP_3_PIN,      INPUT_PULLUP, GPIO,      true),
#endif // CFG_DEMO_HW_WKUP


//#if CFG_DEMO_HW_I2C || CFG_AD_I2C_1
        /* I2C */
        HW_GPIO_PINCONFIG(CFG_GPIO_I2C1_SCL_PORT,
                                                 CFG_GPIO_I2C1_SCL_PIN,    OUTPUT,       I2C_SCL,   true),
        HW_GPIO_PINCONFIG(CFG_GPIO_I2C1_SDA_PORT,
                                                 CFG_GPIO_I2C1_SDA_PIN,    INPUT,        I2C_SDA,   true),
//#endif // CFG_DEMO_HW_I2C || CFG_DEMO_HW_I2C_ASYNC || CFG_DEMO_AD_SPI_I2C

#if CFG_AD_SPI_1
        /* SPI1 */
        HW_GPIO_PINCONFIG(CFG_GPIO_SPI1_CLK_PORT,
                                                 CFG_GPIO_SPI1_CLK_PIN,    OUTPUT,       SPI_CLK,   true),
        HW_GPIO_PINCONFIG(CFG_GPIO_SPI1_DO_PORT, CFG_GPIO_SPI1_DO_PIN,     OUTPUT,       SPI_DO,    true),
        HW_GPIO_PINCONFIG(CFG_GPIO_SPI1_DI_PORT, CFG_GPIO_SPI1_DI_PIN,     INPUT,        SPI_DI,    true),
        HW_GPIO_PINCONFIG(CFG_GPIO_SPI1_CS_PORT, CFG_GPIO_SPI1_CS_PIN,     OUTPUT,       GPIO,      true),
#if CFG_DEMO_SENSOR_ADXL362
        /* Chip select for ADXL362 */
        HW_GPIO_PINCONFIG(CFG_GPIO_ADXL362_CS_PORT,
                                                 CFG_GPIO_ADXL362_CS_PIN,  OUTPUT,       GPIO,      true),
#endif // CFG_DEMO_SENSOR_ADXL362
#endif // CFG_AD_SPI_1

        HW_GPIO_PINCONFIG_END // important!!!
};

static void periph_init(void)
{
        /*
         * Workaround for JLink emulated serial port.
         *
         * JLink serial port does not set its output UART pin high (UART idle state) unless
         * there is something transmitted from PC to board.
         * Pin state is kept by level shifter low after board reset.
         * Configuring pin as UART_RX does not turn on pull up resistor, hence RX line stays low
         * and this state is usually detected as break condition.
         * With low state on UART RX, configuration of UART is unsuccessful: as soon as baud rate
         * is set up UART goes into busy state, all other settings are ignored.
         *
         * Workaround sets up pin that will be used as UART RX as output with high state.
         * This will result in level shifter holding this state until JLink starts to drive this
         * line for transmission.
         */

        hw_gpio_configure_pin(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_GPIO, 1);

        hw_gpio_set_pin_function(HW_GPIO_PORT_1, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                HW_GPIO_FUNC_UART2_TX);
        hw_gpio_set_pin_function(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                HW_GPIO_FUNC_UART2_RX);
        /*i2c*/
        hw_gpio_configure(gpio_cfg);

}

static void prvSetupHardware( void )
{
#if mainCHECK_INTERRUPT_STACK == 1
        extern unsigned long _vStackTop[], _pvHeapStart[];
        unsigned long ulInterruptStackSize;
#endif

        /* Init hardware */
        pm_system_init(periph_init);

#if mainCHECK_INTERRUPT_STACK == 1
        /* The size of the stack used by main and interrupts is not defined in
           the linker, but just uses whatever RAM is left.  Calculate the amount of
           RAM available for the main/interrupt/system stack, and check it against
           a reasonable number.  If this assert is hit then it is likely you don't
           have enough stack to start the kernel, or to allow interrupts to nest.
           Note - this is separate to the stacks that are used by tasks.  The stacks
           that are used by tasks are automatically checked if
           configCHECK_FOR_STACK_OVERFLOW is not 0 in FreeRTOSConfig.h - but the stack
           used by interrupts is not.  Reducing the conifgTOTAL_HEAP_SIZE setting will
           increase the stack available to main() and interrupts. */
        ulInterruptStackSize = ( ( unsigned long ) _vStackTop ) - ( ( unsigned long ) _pvHeapStart );
        OS_ASSERT( ulInterruptStackSize > 350UL );

        /* Fill the stack used by main() and interrupts to a known value, so its
           use can be manually checked. */
        memcpy( ( void * ) _pvHeapStart, ucExpectedInterruptStackValues, sizeof( ucExpectedInterruptStackValues ) );
#endif
}

/**
 * @brief Malloc fail hook
 */
void vApplicationMallocFailedHook( void )
{
        /* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to OS_MALLOC() fails.
	OS_MALLOC() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to OS_MALLOC() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
        taskDISABLE_INTERRUPTS();
        for( ;; );
}

/**
 * @brief Application idle task hook
 */
void vApplicationIdleHook( void )
{
        /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
           to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
           task. It is essential that code added to this hook function never attempts
           to block in any way (for example, call OS_QUEUE_GET() with a block time
           specified, or call OS_DELAY()).  If the application makes use of the
           OS_TASK_DELETE() API function (as this demo application does) then it is also
           important that vApplicationIdleHook() is permitted to return to its calling
           function, because it is the responsibility of the idle task to clean up
           memory allocated by the kernel to any task that has since been deleted. */

#if (dg_configTRACK_OS_HEAP == 1)
        OS_BASE_TYPE i = 0;
        OS_BASE_TYPE uxMinimumEverFreeHeapSize;

        // Generate raw status information about each task.
        UBaseType_t uxNbOfTaskEntries = uxTaskGetSystemState(pxTaskStatusArray,
                                                        mainMAX_NB_OF_TASKS, &ulTotalRunTime);

        for (i = 0; i < uxNbOfTaskEntries; i++) {
                /* Check Free Stack*/
                OS_BASE_TYPE uxStackHighWaterMark;

                uxStackHighWaterMark = uxTaskGetStackHighWaterMark(pxTaskStatusArray[i].xHandle);
                OS_ASSERT(uxStackHighWaterMark >= mainMIN_STACK_GUARD_SIZE);
        }

        /* Check Minimum Ever Free Heap against defined guard. */
        uxMinimumEverFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
        OS_ASSERT(uxMinimumEverFreeHeapSize >= mainTOTAL_HEAP_SIZE_GUARD);
#endif /* (dg_configTRACK_OS_HEAP == 1) */

#if dg_configUSE_WDOG
        sys_watchdog_notify(idle_task_wdog_id);
#endif
}

/**
 * @brief Application stack overflow hook
 */
void vApplicationStackOverflowHook( OS_TASK pxTask, char *pcTaskName )
{
        ( void ) pcTaskName;
        ( void ) pxTask;

        /* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
        taskDISABLE_INTERRUPTS();
        for( ;; );
}

/**
 * @brief Application tick hook
 */
void vApplicationTickHook( void )
{
#if mainCHECK_INTERRUPT_STACK == 1
        extern unsigned long _pvHeapStart[];

        /* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */

        /* Manually check the last few bytes of the interrupt stack to check they
	have not been overwritten.  Note - the task stacks are automatically
	checked for overflow if configCHECK_FOR_STACK_OVERFLOW is set to 1 or 2
	in FreeRTOSConifg.h, but the interrupt stack is not. */
        OS_ASSERT( memcmp( ( void * ) _pvHeapStart, ucExpectedInterruptStackValues, sizeof( ucExpectedInterruptStackValues ) ) == 0U );
#endif /* mainCHECK_INTERRUPT_STACK */
}

