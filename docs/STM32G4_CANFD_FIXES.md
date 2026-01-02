# STM32G4 CAN FD Fixes for HUCONN CAN

This document describes the bugs found and fixes applied to enable proper CAN FD support on STM32G4-based gs_usb devices.

## Summary

Two critical bugs were identified and fixed:

1. **STM32G4 HAL USB PCD Bug** - Multi-packet USB bulk transfers lost byte count
2. **gs_usb DLC Interpretation Bug** - FD frame DLC was incorrectly interpreted as byte length instead of DLC code

## Bug 1: STM32G4 HAL USB Multi-packet Transfer Bug

### Problem

When receiving USB bulk OUT transfers larger than 64 bytes (the USB full-speed max packet size), the STM32G4 HAL incorrectly reported only the last packet's byte count instead of the total accumulated count.

**Symptom**: CAN FD frames (76 bytes for a 64-byte payload) were received as only 12 bytes (the remainder after 64 bytes).

### Root Cause

In `stm32g4xx_hal_pcd.c`, the `HAL_PCD_EP_Receive()` function resets `ep->xfer_count` to 0 at line 1440. When continuing a multi-packet transfer, this reset causes the previously accumulated byte count to be lost.

```c
// Line 1440 in HAL_PCD_EP_Receive()
ep->xfer_count = 0U;  // This resets the accumulated count!
```

### Fix

Save and restore `xfer_count` when continuing multi-packet transfers in `PCD_EP_ISR_Handler()`:

```c
// libs/STM32_HAL/src/stm32g4xx/stm32g4xx_hal_pcd.c, around line 1818
else
{
    /* Save xfer_count before calling HAL_PCD_EP_Receive which resets it.
     * This is needed to correctly accumulate count for multi-packet transfers. */
    uint32_t saved_xfer_count = ep->xfer_count;
    (void)HAL_PCD_EP_Receive(hpcd, ep->num, ep->xfer_buff, ep->xfer_len);
    ep->xfer_count = saved_xfer_count;
}
```

## Bug 2: gs_usb DLC Code Interpretation Bug

### Problem

For CAN FD frames, the Linux gs_usb driver sends `can_dlc` as a **DLC code** (0-15), not the actual byte length. The firmware incorrectly interpreted this as byte length.

**Symptom**:
- Sending 12-byte FD frame: Only 9 bytes were copied (DLC code 9 = 12 bytes)
- Receiving 12-byte FD frame: Reported as 24 bytes (firmware sent byte length 12, kernel interpreted as DLC code 12 = 24 bytes)

### DLC Code to Byte Length Mapping

| DLC Code | Byte Length |
|----------|-------------|
| 0-8      | 0-8         |
| 9        | 12          |
| 10       | 16          |
| 11       | 20          |
| 12       | 24          |
| 13       | 32          |
| 14       | 48          |
| 15       | 64          |

### Fix

Updated `src/can/fdcan.c` to properly handle DLC codes for FD frames:

**In `can_send()`:**
```c
if (frame->flags & GS_CAN_FLAG_FD) {
    /* For FD frames, frame->can_dlc is DLC code (0-15), not byte length */
    tx_header.DataLength = dlc_code_to_hal(frame->can_dlc);
    uint8_t byte_len = dlc_code_to_len(frame->can_dlc);
    for (uint8_t i = 0; i < byte_len && i < 64; i++) {
        tx_data[i] = frame->canfd->data[i];
    }
}
```

**In `can_receive()`:**
```c
if (rx_header.FDFormat == FDCAN_FD_CAN) {
    /* For FD frames, gs_usb uses DLC code (0-15), not byte length */
    uint8_t dlc_code = hal_dlc_to_code(rx_header.DataLength);
    uint8_t byte_len = dlc_code_to_len(dlc_code);
    rx_frame->can_dlc = dlc_code;  // Return DLC code, not byte length
    for (uint8_t i = 0; i < byte_len && i < 64; i++) {
        rx_frame->canfd->data[i] = rx_data[i];
    }
}
```

## Files Modified

| File | Changes |
|------|---------|
| `libs/STM32_HAL/src/stm32g4xx/stm32g4xx_hal_pcd.c` | Fix multi-packet xfer_count accumulation |
| `src/can/fdcan.c` | Fix DLC code handling for FD frames |
| `src/usbd_conf.c` | Debug variables for USB callbacks |
| `src/usbd_gs_can.c` | Debug variables for frame reception |
| `src/can_common.c` | Debug variables for TX/RX tracking |
| `include/can.h` | Debug variable declarations |

## Testing

### Test Environment
- Board: HUCONN CAN (STM32G431)
- Linux kernel with gs_usb driver
- can-utils for testing

### Test Results

| Test | Result |
|------|--------|
| CAN FD Loopback (8 bytes) | PASS |
| CAN FD Loopback (12 bytes) | PASS |
| CAN FD Loopback (16 bytes) | PASS |
| CAN FD Loopback (32 bytes) | PASS |
| CAN FD Loopback (64 bytes) | PASS |
| CAN FD External (can0 ↔ can1) | PASS |
| Classic CAN Bidirectional | PASS |

## Notes

- Debug variables are included in this commit for potential future debugging
- The HAL fix is specific to STM32G4; other MCU families may have similar issues
- Classic CAN operation is unaffected by these changes
