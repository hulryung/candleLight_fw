# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

candleLight_fw is firmware for USB-to-CAN adapters implementing the Linux `gs_usb` kernel driver interface. It enables various STM32-based CAN adapters to work with Linux without custom drivers. The firmware also supports Windows via WCID USB descriptors.

**Supported MCU families:**
- STM32F042x6 (Cortex-M0, BXCAN, classic CAN only)
- STM32F072xB (Cortex-M0, BXCAN, classic CAN only)
- STM32F407xE (Cortex-M4, BXCAN, classic CAN only)
- STM32G0B1 (Cortex-M0, FDCAN, CAN-FD capable - experimental)
- STM32G431 (Cortex-M4, FDCAN, CAN-FD capable - experimental)

**Important limitation:** STM32F103 cannot be used (USB and CAN share pins).

## Build Commands

```bash
# Install toolchain (Ubuntu/Debian)
sudo apt-get install gcc-arm-none-eabi

# Configure and build all enabled targets
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi-gcc.cmake
make

# Build single target
make cantact_fw
make candleLight_fw

# Flash via DFU
make flash-cantact_fw

# List all available targets
make help
```

**Build outputs per target:** `<target>_fw` (ELF), `<target>_fw.bin`, `<target>_fw.dfu`, `<target>_fw.map`

**Enable experimental G0/G4 targets:**
```bash
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi-gcc.cmake -DBUILD_budgetcan=ON -DBUILD_HUCONN_CAN=ON
```

## Code Formatting

```bash
uncrustify -c ./uncrustify.cfg --replace --no-backup $(find include src -name "*.[ch]")
```

## Architecture

### Data Flow
```
USB Host (gs_usb driver)
       ↕
[usbd_gs_can.c] - USB class driver, gs_usb protocol, frame queuing
       ↕
[can_common.c] - Shared TX/RX/error handling
       ↕
[bxcan.c | fdcan.c] - CAN peripheral driver (selected per MCU family)
       ↕
CAN Bus
```

### Key Source Files

| File | Purpose |
|------|---------|
| `src/usbd_gs_can.c` | USB gs_usb class implementation (protocol, endpoints, frame handling) |
| `src/can/bxcan.c` | BXCAN driver for F0/F4 (classic CAN) |
| `src/can/fdcan.c` | FDCAN driver for G0/G4 (CAN-FD capable) |
| `src/can_common.c` | Shared CAN frame TX/RX/error logic |
| `src/device/device_*.c` | MCU-specific clock and GPIO initialization |
| `include/config.h` | Board configurations (pins, clocks, features per board) |
| `include/gs_usb.h` | gs_usb protocol constants and structures |

### Board Configuration System

Boards are configured via compile-time defines in `include/config.h`. Each board section defines:
- USB product string
- CAN peripheral and clock speed
- GPIO pin mappings (LEDs, transceiver control, termination)
- Feature flags (CAN-FD, hardware filtering, termination resistor)

Build system selects board via `-DBOARD_<name>` and MCU family via `-DSTM32F0`/`-DSTM32F4`/etc.

### Adding a New Board

1. Add board configuration block to `include/config.h`
2. Add target to appropriate list in `CMakeLists.txt` (e.g., `TGTF072_LIST`)
3. Use existing `add_f0xx_target()` function or create new one for different MCU family

### CAN Driver Abstraction

The `can.h` interface abstracts CAN hardware differences:
- `bxcan.c` - STM32F0/F4 basic CAN controller
- `fdcan.c` - STM32G0/G4 flexible data-rate CAN controller

Both implement the same interface; CMake selects the appropriate driver based on target MCU family.

## Hardware Constraints

- Frame echo occurs when written to peripheral, not after successful bus transmission
- CAN-FD requires Linux kernel 5.18+ (gs_usb driver CAN-FD support)
- Linux kernel < 4.5 has gs_usb bug that can crash on device removal
