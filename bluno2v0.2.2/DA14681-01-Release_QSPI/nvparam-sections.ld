SECTIONS {
        .bss : {
                __bss_start__ = .;
                __bss_end__ = .;
        }
        .nvparam : {
                FILL(0xFF);
s = 0x0000;
        . = s + 0x0000; *(section_TAG_BLE_PLATFORM_BD_ADDRESS)
        . = s + 0x0007; *(section_TAG_BLE_PLATFORM_LPCLK_DRIFT)
        . = s + 0x000A; *(section_TAG_BLE_PLATFORM_EXT_WAKEUP_TIME)
        . = s + 0x000D; *(section_TAG_BLE_PLATFORM_OSC_WAKEUP_TIME)
        . = s + 0x0010; *(section_TAG_BLE_PLATFORM_RM_WAKEUP_TIME)
        . = s + 0x0013; *(section_TAG_BLE_PLATFORM_SLEEP_ENABLE)
        . = s + 0x0015; *(section_TAG_BLE_PLATFORM_EXT_WAKEUP_ENABLE)
        . = s + 0x0017; *(section_TAG_BLE_PLATFORM_BLE_CA_TIMER_DUR)
        . = s + 0x001A; *(section_TAG_BLE_PLATFORM_BLE_CRA_TIMER_DUR)
        . = s + 0x001C; *(section_TAG_BLE_PLATFORM_BLE_CA_MIN_RSSI)
        . = s + 0x001E; *(section_TAG_BLE_PLATFORM_BLE_CA_NB_PKT)
        . = s + 0x0021; *(section_TAG_BLE_PLATFORM_BLE_CA_NB_BAD_PKT)
        . = s + 0x0024; *(section_TAG_BLE_PLATFORM_IRK)

        }
}
