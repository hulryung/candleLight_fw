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

/* Debug variable to store last init status */
volatile HAL_StatusTypeDef g_fdcan_init_status = HAL_OK;
volatile HAL_StatusTypeDef g_fdcan_filter_status = HAL_OK;
volatile HAL_StatusTypeDef g_fdcan_start_status = HAL_OK;
volatile uint32_t g_fdcan_init_mode = 0;

/* Debug variables for TX/RX */
volatile uint32_t g_can_send_count = 0;
volatile HAL_StatusTypeDef g_can_send_status = HAL_OK;
volatile uint32_t g_can_send_flags = 0;
volatile uint32_t g_can_rx_pending_count = 0;
volatile uint32_t g_can_receive_count = 0;

/* Debug for DLC tracking */
volatile uint32_t g_tx_frame_dlc = 0;      /* can_dlc from host frame */
volatile uint32_t g_tx_hal_dlc = 0;        /* DataLength sent to HAL */
volatile uint32_t g_rx_hal_dlc = 0;        /* DataLength from HAL */
volatile uint32_t g_rx_computed_dlc = 0;   /* computed can_dlc */
volatile uint8_t g_tx_data_first8[8] = {0};
volatile uint8_t g_rx_data_first8[8] = {0};

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

	/* Initialize GPIO for CAN RX/TX pins */
#if defined(BOARD_HUCONN_CAN)
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

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
	const uint16_t tseg1 = timing->prop_seg + timing->phase_seg1;

	channel->data_brp = timing->brp;
	channel->data_phase_seg1 = tseg1;
	channel->data_phase_seg2 = timing->phase_seg2;
	channel->data_sjw = timing->sjw;
}

void can_enable(can_data_t *channel, uint32_t mode)
{
	hfdcan.Instance = channel->instance;

	/* Ensure clean state before reconfiguring */
	HAL_FDCAN_DeInit(&hfdcan);

	hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	/* Use FD mode only when host requests it */
	if (mode & GS_CAN_MODE_FD) {
		hfdcan.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	} else {
		hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	}
	hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan.Init.AutoRetransmission = ENABLE;
	hfdcan.Init.TransmitPause = DISABLE;
	hfdcan.Init.ProtocolException = DISABLE;

	/* Fixed nominal bit timing: 500kbps with 170MHz clock */
	/* 170MHz / 20 / (1 + 14 + 2) = 500kbps */
	hfdcan.Init.NominalPrescaler = 20;
	hfdcan.Init.NominalSyncJumpWidth = 1;
	hfdcan.Init.NominalTimeSeg1 = 14;
	hfdcan.Init.NominalTimeSeg2 = 2;

	/* Fixed data bit timing: 2Mbps with 170MHz clock */
	/* 170MHz / 5 / (1 + 14 + 2) = 2Mbps */
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

	/* Store the mode for debugging */
	g_fdcan_init_mode = mode;

	/* Initialize FDCAN */
	g_fdcan_init_status = HAL_FDCAN_Init(&hfdcan);
	if (g_fdcan_init_status != HAL_OK) {
		/* Blink LED rapidly to indicate init failure */
		for (int i = 0; i < 10; i++) {
			HAL_GPIO_WritePin(LEDTX_GPIO_Port, LEDTX_Pin, GPIO_PIN_RESET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(LEDTX_GPIO_Port, LEDTX_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
		}
		return;
	}

	/* Configure global filter to accept all frames */
	g_fdcan_filter_status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan,
			FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
			FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	if (g_fdcan_filter_status != HAL_OK) {
		/* Blink LED 20 times for global filter failure */
		for (int i = 0; i < 20; i++) {
			HAL_GPIO_WritePin(LEDTX_GPIO_Port, LEDTX_Pin, GPIO_PIN_RESET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(LEDTX_GPIO_Port, LEDTX_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
		}
		return;
	}

	/* Start FDCAN */
	g_fdcan_start_status = HAL_FDCAN_Start(&hfdcan);
	if (g_fdcan_start_status != HAL_OK) {
		/* Blink LED 30 times for start failure */
		for (int i = 0; i < 30; i++) {
			HAL_GPIO_WritePin(LEDTX_GPIO_Port, LEDTX_Pin, GPIO_PIN_RESET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(LEDTX_GPIO_Port, LEDTX_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
		}
		return;
	}

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, !GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif

#ifdef CAN_PWR_Pin
	/* Enable CAN transceiver power (active low: 0 = ON) */
	HAL_GPIO_WritePin(CAN_PWR_Port, CAN_PWR_Pin, GPIO_PIN_RESET);
#endif

}

void can_disable(can_data_t *channel)
{
	(void)channel;

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif

#ifdef CAN_PWR_Pin
	/* Disable CAN transceiver power (active low: 1 = OFF) */
	HAL_GPIO_WritePin(CAN_PWR_Port, CAN_PWR_Pin, GPIO_PIN_SET);
#endif

	HAL_FDCAN_Stop(&hfdcan);
	HAL_FDCAN_DeInit(&hfdcan);
}

bool can_is_enabled(can_data_t *channel)
{
	(void)channel;
	return (hfdcan.State == HAL_FDCAN_STATE_BUSY);
}

bool can_is_rx_pending(can_data_t *channel)
{
	(void)channel;
	uint32_t fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0);
	if (fill > 0) {
		g_can_rx_pending_count++;
	}
	return (fill > 0);
}

/* DLC code (0-15) to byte length table */
static const uint8_t dlc_to_len_table[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/* Convert HAL DataLength format to byte length */
static uint8_t hal_dlc_to_len(uint32_t dlc)
{
	uint8_t idx = (dlc >> 16) & 0xF;
	return dlc_to_len_table[idx];
}

/* Convert DLC code (0-15) to byte length - for gs_usb FD frames */
static uint8_t dlc_code_to_len(uint8_t dlc_code)
{
	if (dlc_code > 15) dlc_code = 15;
	return dlc_to_len_table[dlc_code];
}

/* Convert DLC code to HAL DataLength format */
static uint32_t dlc_code_to_hal(uint8_t dlc_code)
{
	return ((uint32_t)dlc_code) << 16;
}

/* Convert HAL DataLength to DLC code (0-15) */
static uint8_t hal_dlc_to_code(uint32_t dlc)
{
	return (dlc >> 16) & 0xF;
}

/* Convert byte length to HAL DataLength format (for classic CAN) */
static uint32_t len_to_hal_dlc(uint8_t len)
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

	g_can_receive_count++;

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
		g_rx_hal_dlc = rx_header.DataLength;
		/* For FD frames, gs_usb uses DLC code (0-15), not byte length */
		uint8_t dlc_code = hal_dlc_to_code(rx_header.DataLength);
		uint8_t byte_len = dlc_code_to_len(dlc_code);
		rx_frame->can_dlc = dlc_code;
		g_rx_computed_dlc = dlc_code;
		rx_frame->canfd_ts->timestamp_us = timer_get();
		for (uint8_t i = 0; i < byte_len && i < 64; i++) {
			rx_frame->canfd->data[i] = rx_data[i];
			if (i < 8) g_rx_data_first8[i] = rx_data[i];
		}
	} else {
		/* For classic CAN, can_dlc is the actual byte length (0-8) */
		rx_frame->can_dlc = hal_dlc_to_len(rx_header.DataLength);
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

	g_can_send_count++;
	g_can_send_flags = frame->flags;

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
		/* For FD frames, frame->can_dlc is DLC code (0-15), not byte length */
		tx_header.FDFormat = FDCAN_FD_CAN;
		tx_header.BitRateSwitch = (frame->flags & GS_CAN_FLAG_BRS) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
		tx_header.DataLength = dlc_code_to_hal(frame->can_dlc);
		uint8_t byte_len = dlc_code_to_len(frame->can_dlc);
		g_tx_frame_dlc = frame->can_dlc;
		g_tx_hal_dlc = tx_header.DataLength;
		for (uint8_t i = 0; i < byte_len && i < 64; i++) {
			tx_data[i] = frame->canfd->data[i];
			if (i < 8) g_tx_data_first8[i] = tx_data[i];
		}
	} else {
		/* For classic CAN, frame->can_dlc is actual byte length (0-8) */
		tx_header.FDFormat = FDCAN_CLASSIC_CAN;
		tx_header.BitRateSwitch = FDCAN_BRS_OFF;
		tx_header.DataLength = len_to_hal_dlc(frame->can_dlc);
		for (uint8_t i = 0; i < frame->can_dlc && i < 8; i++) {
			tx_data[i] = frame->classic_can->data[i];
		}
	}

	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;

	g_can_send_status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &tx_header, tx_data);
	if (g_can_send_status != HAL_OK) {
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
