/*

The MIT License (MIT)

Copyright (c) 2024 candleLight contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

/*
 * FDCAN driver for STM32G4/G0 series
 * Based on huconn-can-fw-canable can.c and gs_usb interface
 */

#include "can.h"
#include "config.h"
#include "device.h"
#include "gpio.h"
#include "gs_usb.h"
#include "hal_include.h"
#include "timer.h"

static FDCAN_HandleTypeDef hfdcan;

const struct gs_device_bt_const CAN_btconst = {
	.feature =
		GS_CAN_FEATURE_LISTEN_ONLY |
		GS_CAN_FEATURE_LOOP_BACK |
		GS_CAN_FEATURE_HW_TIMESTAMP |
		GS_CAN_FEATURE_IDENTIFY |
		GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE |
		GS_CAN_FEATURE_FD |
		GS_CAN_FEATURE_BT_CONST_EXT |
#ifdef TERM_Pin
		GS_CAN_FEATURE_TERMINATION |
#endif
		0,
	.fclk_can = CAN_CLOCK_SPEED,
	.btc = {
		.tseg1_min = 1,
		.tseg1_max = 256,
		.tseg2_min = 1,
		.tseg2_max = 128,
		.sjw_max = 128,
		.brp_min = 1,
		.brp_max = 512,
		.brp_inc = 1,
	},
};

const struct gs_device_bt_const_extended CAN_btconst_ext = {
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
		.tseg1_min = 1,
		.tseg1_max = 256,
		.tseg2_min = 1,
		.tseg2_max = 128,
		.sjw_max = 128,
		.brp_min = 1,
		.brp_max = 512,
		.brp_inc = 1,
	},
	.dbtc = {
		.tseg1_min = 1,
		.tseg1_max = 32,
		.tseg2_min = 1,
		.tseg2_max = 16,
		.sjw_max = 16,
		.brp_min = 1,
		.brp_max = 32,
		.brp_inc = 1,
	},
};

void can_init(can_data_t *channel, CAN_InstanceTypeDef *instance)
{
	channel->instance = instance;
	device_can_init(channel, instance);

	/* Enable FDCAN clock */
	__HAL_RCC_FDCAN_CLK_ENABLE();

	/* Initialize GPIO for CAN */
#if defined(BOARD_HUCONN_CAN)
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* CAN transceiver standby control (PA0) */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); /* Standby off */

	/* CAN IO power control (PB7) */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); /* Power on */

	/* CAN RX/TX pins (PB8/PB9) */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif

	hfdcan.Instance = instance;
}

void can_set_bittiming(can_data_t *channel, const struct gs_device_bittiming *timing)
{
	const uint16_t tseg1 = timing->prop_seg + timing->phase_seg1;

	channel->brp = timing->brp;
	channel->phase_seg1 = tseg1;
	channel->phase_seg2 = timing->phase_seg2;
	channel->sjw = timing->sjw;
}

void can_set_data_bittiming(can_data_t *channel, const struct gs_device_bittiming *timing)
{
	/* Store data bittiming - for now store in main timing fields
	 * TODO: add separate data timing fields to can_data_t */
	(void)channel;
	(void)timing;
}

void can_enable(can_data_t *channel, uint32_t mode)
{
	hfdcan.Instance = channel->instance;

	hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan.Init.AutoRetransmission = ENABLE;
	hfdcan.Init.TransmitPause = DISABLE;
	hfdcan.Init.ProtocolException = DISABLE;

	hfdcan.Init.NominalPrescaler = channel->brp;
	hfdcan.Init.NominalSyncJumpWidth = channel->sjw;
	hfdcan.Init.NominalTimeSeg1 = channel->phase_seg1;
	hfdcan.Init.NominalTimeSeg2 = channel->phase_seg2;

	/* Default data bit timing for 2Mbps with 170MHz clock */
	hfdcan.Init.DataPrescaler = 5;
	hfdcan.Init.DataSyncJumpWidth = 1;
	hfdcan.Init.DataTimeSeg1 = 14;
	hfdcan.Init.DataTimeSeg2 = 2;

	hfdcan.Init.StdFiltersNbr = 0;
	hfdcan.Init.ExtFiltersNbr = 0;
	hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

	if (mode & GS_CAN_MODE_LISTEN_ONLY) {
		hfdcan.Init.Mode = FDCAN_MODE_BUS_MONITORING;
	}

	if (mode & GS_CAN_MODE_LOOP_BACK) {
		hfdcan.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
	}

	if (mode & GS_CAN_MODE_ONE_SHOT) {
		hfdcan.Init.AutoRetransmission = DISABLE;
	}

	HAL_FDCAN_Init(&hfdcan);
	HAL_FDCAN_Start(&hfdcan);

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, !GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif
}

void can_disable(can_data_t *channel)
{
	(void)channel;

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif

	HAL_FDCAN_Stop(&hfdcan);
}

bool can_is_enabled(can_data_t *channel)
{
	(void)channel;
	return (hfdcan.State == HAL_FDCAN_STATE_BUSY);
}

bool can_is_rx_pending(can_data_t *channel)
{
	(void)channel;
	return (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0) > 0);
}

static uint8_t dlc_to_len(uint32_t dlc)
{
	static const uint8_t dlc_table[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
	uint8_t idx = (dlc >> 16) & 0xF;
	return dlc_table[idx];
}

static uint32_t len_to_dlc(uint8_t len)
{
	if (len <= 8) return len << 16;
	else if (len <= 12) return FDCAN_DLC_BYTES_12;
	else if (len <= 16) return FDCAN_DLC_BYTES_16;
	else if (len <= 20) return FDCAN_DLC_BYTES_20;
	else if (len <= 24) return FDCAN_DLC_BYTES_24;
	else if (len <= 32) return FDCAN_DLC_BYTES_32;
	else if (len <= 48) return FDCAN_DLC_BYTES_48;
	else return FDCAN_DLC_BYTES_64;
}

bool can_receive(can_data_t *channel, struct gs_host_frame *rx_frame)
{
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[64];

	if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
		return false;
	}

	rx_frame->channel = channel->nr;
	rx_frame->flags = 0;

	/* Extract CAN ID */
	if (rx_header.IdType == FDCAN_EXTENDED_ID) {
		rx_frame->can_id = rx_header.Identifier | CAN_EFF_FLAG;
	} else {
		rx_frame->can_id = rx_header.Identifier;
	}

	if (rx_header.RxFrameType == FDCAN_REMOTE_FRAME) {
		rx_frame->can_id |= CAN_RTR_FLAG;
	}

	/* Handle FD frames */
	if (rx_header.FDFormat == FDCAN_FD_CAN) {
		rx_frame->flags |= GS_CAN_FLAG_FD;
		if (rx_header.BitRateSwitch == FDCAN_BRS_ON) {
			rx_frame->flags |= GS_CAN_FLAG_BRS;
		}
		rx_frame->can_dlc = dlc_to_len(rx_header.DataLength);
		rx_frame->canfd_ts->timestamp_us = timer_get();
		for (uint8_t i = 0; i < rx_frame->can_dlc && i < 64; i++) {
			rx_frame->canfd->data[i] = rx_data[i];
		}
	} else {
		rx_frame->can_dlc = dlc_to_len(rx_header.DataLength);
		if (rx_frame->can_dlc > 8) rx_frame->can_dlc = 8;
		rx_frame->classic_can_ts->timestamp_us = timer_get();
		for (uint8_t i = 0; i < rx_frame->can_dlc; i++) {
			rx_frame->classic_can->data[i] = rx_data[i];
		}
	}

	return true;
}

bool can_send(can_data_t *channel, struct gs_host_frame *frame)
{
	(void)channel;

	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[64];

	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan) == 0) {
		return false;
	}

	/* Set up header */
	if (frame->can_id & CAN_EFF_FLAG) {
		tx_header.IdType = FDCAN_EXTENDED_ID;
		tx_header.Identifier = frame->can_id & 0x1FFFFFFF;
	} else {
		tx_header.IdType = FDCAN_STANDARD_ID;
		tx_header.Identifier = frame->can_id & 0x7FF;
	}

	if (frame->can_id & CAN_RTR_FLAG) {
		tx_header.TxFrameType = FDCAN_REMOTE_FRAME;
	} else {
		tx_header.TxFrameType = FDCAN_DATA_FRAME;
	}

	/* Handle FD vs classic frames */
	if (frame->flags & GS_CAN_FLAG_FD) {
		tx_header.FDFormat = FDCAN_FD_CAN;
		tx_header.BitRateSwitch = (frame->flags & GS_CAN_FLAG_BRS) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
		tx_header.DataLength = len_to_dlc(frame->can_dlc);
		for (uint8_t i = 0; i < frame->can_dlc && i < 64; i++) {
			tx_data[i] = frame->canfd->data[i];
		}
	} else {
		tx_header.FDFormat = FDCAN_CLASSIC_CAN;
		tx_header.BitRateSwitch = FDCAN_BRS_OFF;
		tx_header.DataLength = len_to_dlc(frame->can_dlc);
		for (uint8_t i = 0; i < frame->can_dlc && i < 8; i++) {
			tx_data[i] = frame->classic_can->data[i];
		}
	}

	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &tx_header, tx_data) != HAL_OK) {
		return false;
	}

	return true;
}

uint32_t can_get_error_status(can_data_t *channel)
{
	(void)channel;
	/* TODO: Implement error status for FDCAN */
	return 0;
}

bool can_parse_error_status(can_data_t *channel, struct gs_host_frame *frame, uint32_t err)
{
	(void)channel;
	(void)frame;
	(void)err;
	/* TODO: Implement error parsing for FDCAN */
	return false;
}
