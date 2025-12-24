# candleLight STM32G4 (TOPST CAN) Porting Notes

## Overview

- **Goal**: Port candleLight (gs_usb) firmware to STM32G431CBT6 MCU
- **Board Name**: TOPST CAN
- **Reference Firmware**: `topst-can-fw-canable` (SLCAN version)
- **Status**: Basic CAN working, CAN-FD requires kernel driver upgrade

---

## Hardware Specifications

| Item | Specification |
|------|---------------|
| MCU | STM32G431CBT6 |
| Core | ARM Cortex-M4 |
| Clock | 170MHz (HSI + PLL) |
| CAN | FDCAN1 |
| USB | Full-Speed (HSI48 + CRS) |
| Flash | 128KB |
| RAM | 32KB |

### Pin Map

| Function | Pin | Description |
|----------|-----|-------------|
| CAN_RX | PB8 | AF9 (FDCAN1) |
| CAN_TX | PB9 | AF9 (FDCAN1) |
| CAN_STBY | PA0 | Transceiver Standby (Active Low) |
| CAN_PWR | PB7 | CAN IO Power Control |
| LED_RX | PA15 | Blue LED (Active Low) |
| LED_TX | PB11 | Green LED (Active Low) |
| USB_DM | PA11 | USB D- |
| USB_DP | PA12 | USB D+ |

---

## Modified/Added Files

### 1. CMakeLists.txt

Added G4 target function:

```cmake
function(add_g431_target TGTNAME)
    add_target_common(${TGTNAME} STM32G431xx)
    target_compile_definitions(${TGTNAME}_fw PRIVATE BOARD_${TGTNAME} STM32G4)
    target_sources(${TGTNAME}_fw PRIVATE "src/can/fdcan.c")
    target_sources(${TGTNAME}_fw PRIVATE "src/device/device_g4.c")
endfunction()

set(TGTG431_LIST "TOPST_CAN")
foreach(TGTNAME ${TGTG431_LIST})
    add_g431_target(${TGTNAME})
endforeach()
```

### 2. include/config.h

Added TOPST_CAN board configuration (line 379~):

```c
#elif defined(BOARD_TOPST_CAN)
    #define USBD_PRODUCT_STRING_FS   "TOPST CAN gs_usb"
    #define USBD_MANUFACTURER_STRING "TOPST"
    #define DFU_INTERFACE_STRING_FS  "TOPST CAN firmware upgrade interface"

    #define TIM2_CLOCK_SPEED         170000000

    #define CAN_INTERFACE            FDCAN1
    #define CAN_CLOCK_SPEED          170000000
    #define NUM_CAN_CHANNEL          1
    #define CONFIG_CANFD             1

    #define nCANSTBY_Port            GPIOA
    #define nCANSTBY_Pin             GPIO_PIN_0
    #define nCANSTBY_Active_High     0

    #define CAN_PWR_Port             GPIOB
    #define CAN_PWR_Pin              GPIO_PIN_7

    #define LEDRX_GPIO_Port          GPIOA
    #define LEDRX_Pin                GPIO_PIN_15
    #define LEDRX_Mode               GPIO_MODE_OUTPUT_PP
    #define LEDRX_Active_High        0

    #define LEDTX_GPIO_Port          GPIOB
    #define LEDTX_Pin                GPIO_PIN_11
    #define LEDTX_Mode               GPIO_MODE_OUTPUT_PP
    #define LEDTX_Active_High        0

    #define USB_GPIO_Port            GPIOA
    #define USB_Pin_DM               GPIO_PIN_11
    #define USB_Pin_DP               GPIO_PIN_12
```

### 3. include/can.h

Added typedef for FDCAN/bxCAN type compatibility:

```c
#if defined(STM32G0) || defined(STM32G4)
typedef FDCAN_GlobalTypeDef CAN_InstanceTypeDef;
#else
typedef CAN_TypeDef CAN_InstanceTypeDef;
#endif
```

### 4. include/device.h

Added G4 FDCAN handle declaration:

```c
#if defined(STM32G4)
extern FDCAN_HandleTypeDef hfdcan1;
#endif
```

### 5. include/usbd_gs_can.h

Added G4 USB interface definitions:

```c
#elif defined(STM32G4)
# define USB_INTERFACE    USB
# define USB_INTERRUPT    USB_LP_IRQn
#endif
```

### 6. src/can/fdcan.c (New File)

FDCAN driver implementation:

```c
const struct gs_device_bt_const CAN_btconst = {
    .feature =
        GS_CAN_FEATURE_LISTEN_ONLY |
        GS_CAN_FEATURE_LOOP_BACK |
        GS_CAN_FEATURE_HW_TIMESTAMP |
        GS_CAN_FEATURE_IDENTIFY |
        GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE |
        GS_CAN_FEATURE_FD |
        GS_CAN_FEATURE_BT_CONST_EXT |
        0,
    .fclk_can = CAN_CLOCK_SPEED,
    .btc = {
        .tseg1_min = 1, .tseg1_max = 256,
        .tseg2_min = 1, .tseg2_max = 128,
        .sjw_max = 128,
        .brp_min = 1, .brp_max = 512, .brp_inc = 1,
    },
};

const struct gs_device_bt_const_extended CAN_btconst_ext = {
    .feature = /* same as CAN_btconst.feature */,
    .fclk_can = CAN_CLOCK_SPEED,
    .btc = { /* nominal bit timing */ },
    .dbtc = {
        .tseg1_min = 1, .tseg1_max = 32,
        .tseg2_min = 1, .tseg2_max = 16,
        .sjw_max = 16,
        .brp_min = 1, .brp_max = 32, .brp_inc = 1,
    },
};
```

Key functions:
- `can_init()` - Initialize FDCAN and GPIO
- `can_set_bittiming()` - Set bit timing
- `can_set_data_bittiming()` - Set FD data bit timing
- `can_enable()` / `can_disable()` - Enable/disable CAN
- `can_send()` / `can_receive()` - Send/receive frames

### 7. src/device/device_g4.c (New File)

Clock and device initialization:

```c
void device_sysclock_config(void) {
    // HSI = 16MHz
    // PLL: 16MHz / 4 * 85 / 2 = 170MHz
    // HSI48 for USB with CRS synchronization

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    // RCC configuration...
    // GPIO clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
}
```

### 8. src/interrupts.c

Added STM32G4 interrupt vector table:

```c
#elif defined(STM32G4)
__attribute__((used, section(".vectors")))
const pFunc InterruptVectorTable[102] = {
    (pFunc)(&__StackTop),
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    // ... 16 core exceptions ...
    SysTick_Handler,
    // External Interrupts
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0,     // 10-18
    0,                    /* 19: USB_HP */
    USB_Handler,          /* 20: USB_LP */
    // ... 45 more entries ...
};
#endif
```

### 9. src/usbd_conf.c

Added G4 USB clock enable:

```c
#if defined(STM32G4)
    __HAL_RCC_USB_CLK_ENABLE();
#endif
```

Added G4 PCD initialization:

```c
#elif defined(STM32G4)
    hpcd_USB_FS.Init.Sof_enable = ENABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
#endif
```

### 10. HAL Library Files (libs/STM32_HAL/)

Added files:
- `config/stm32g4xx_hal_conf.h`
- `include/cmsis/device/stm32g4xx.h`
- `include/cmsis/device/stm32g431xx.h`
- `include/cmsis/device/system_stm32g4xx.h`
- `include/stm32g4xx/` (all HAL headers)
- `src/cmsis/system_stm32g4xx.c`
- `src/stm32g4xx/` (all HAL sources)

Added to `libs/STM32_HAL/include/stm32g4xx/stm32g4xx_ll_usb.h`:

```c
#define EP_MPS_64    0U
#define EP_MPS_32    1U
#define EP_MPS_16    2U
#define EP_MPS_8     3U
```

Added to `libs/STM32_HAL/config/hal_include.h`:

```c
#elif defined(STM32G4)
#include "stm32g4xx_hal.h"
```

---

## Build Instructions

```bash
cd candleLight_fw
mkdir -p build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi-gcc.cmake
make TOPST_CAN_fw
```

Output files:
- `build/TOPST_CAN_fw` (ELF)
- `build/TOPST_CAN_fw.bin` (Binary)

---

## Flashing

```bash
st-flash write build/TOPST_CAN_fw.bin 0x08000000
```

---

## Test Results

### Basic CAN (Success)

```bash
# Set up interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Check status
ip -details link show can0
# Output: gs_usb, bitrate 500000, clock 170000000

# Send test
cansend can0 123#DEADBEEF

# Receive test
candump can0
```

### CAN-FD (Not Supported - Kernel Issue)

```bash
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on
# Error: RTNETLINK answers: Operation not supported
```

---

## CAN-FD Issue Analysis

### Problem

The gs_usb driver in Linux kernel 5.15 does not support CAN-FD.

CAN-FD support was added in **kernel 5.18** (May 2022):
- Patch by Marc Kleine-Budde: `[can-next-rfc 15/21] can: gs_usb: add CAN-FD support`
- Added `GS_CAN_FEATURE_BT_CONST_EXT` handling
- Added `GS_USB_BREQ_BT_CONST_EXT` request support

### Firmware Status (Correct)

Binary verification results:
- `CAN_btconst.feature` = 0x5b3 (includes FD 0x100, BT_CONST_EXT 0x400)
- `CAN_btconst_ext` structure fully initialized (feature, fclk_can, btc, dbtc)
- USB endpoint 64 bytes (FD capable)

### Solutions

**Option 1**: Upgrade to Linux kernel 5.18 or later

**Option 2**: Use out-of-tree driver

```bash
# linklayer/gs_usb_fd driver
git clone https://github.com/linklayer/gs_usb_fd.git
cd gs_usb_fd

sudo apt install linux-headers-$(uname -r)
make

sudo rmmod gs_usb
sudo modprobe can-dev
sudo insmod gs_usb_fd.ko
```

---

## Errors Encountered and Solutions

| Error | Cause | Solution |
|-------|-------|----------|
| `nano.specs not found` | Using host compiler | Specify `-DCMAKE_TOOLCHAIN_FILE` |
| Source files deleted | Accidental `rm -rf *` | `git restore .` |
| `stm32g4xx_hal_dma.h not found` | DMA enabled in HAL conf | Comment out `HAL_DMA_MODULE_ENABLED` |
| `uint8_t undefined` (USB lib) | G4 missing in hal_include.h | Add G4 case |
| `CAN_TypeDef undefined` | G4 uses FDCAN | Create `CAN_InstanceTypeDef` typedef |
| `USB_INTERFACE undefined` | G4 definition missing | Add to usbd_gs_can.h |
| `EP_MPS_64 undefined` | Missing in G4 USB header | Add to ll_usb.h |
| Firmware not booting (PC=0x47704718) | No vector table | Add G4 vector table to interrupts.c |
| `CAN_btconst_ext` empty | Only `.dbtc` initialized | Initialize full structure |

---

## TODO

- [ ] Test CAN-FD (on kernel 5.18+ environment)
- [ ] Fully implement `can_set_data_bittiming()`
- [ ] Implement error status handling (`can_get_error_status`, `can_parse_error_status`)
- [ ] Test DFU (firmware upgrade)
- [ ] Verify LED operation

---

## References

- [candleLight Original](https://github.com/candle-usb/candleLight_fw)
- [gs_usb CAN-FD Patch](https://lore.kernel.org/all/20220309124132.291861-16-mkl@pengutronix.de/)
- [gs_usb_fd Driver](https://github.com/linklayer/gs_usb_fd)
- [Linux gs_usb Source](https://github.com/torvalds/linux/blob/master/drivers/net/can/usb/gs_usb.c)

---

*Last Updated: 2024-12-24*
