#pragma once

/* Production MS/TP tuning defaults shared by this project. */

#define USER_MSTP_ACTIVE_DEBUG_ONLY 0
#define USER_MSTP_TOKEN_PASS_ONLY_DEBUG 1
#define USER_MSTP_DEBUG_FORCE_NEXT_STATION 0

#if USER_MSTP_ACTIVE_DEBUG_ONLY
#define USER_MSTP_ZERO_CONFIG_ENABLED 0
#else
#define USER_MSTP_ZERO_CONFIG_ENABLED 1
#endif

#define USER_MSTP_PFM_REPLY_PRE_DELAY_US 0
#define USER_RS485_DE_PRE_TX_US 200 // was 0
#define USER_MSTP_TOKEN_PASS_PRE_DELAY_US 0

#define USER_RS485_CONTROL_FRAME_UART_FLUSH_BEFORE_TX 0
#define USER_RS485_CONTROL_FRAME_EXTRA_IDLE_US 0
#define USER_RS485_CONTROL_FRAME_DE_PRE_TX_US 0

/* Optional logic-analyzer marker around Reply-To-PFM DE window.
	Leave disabled unless a spare GPIO is wired to the analyzer. */
#define USER_RS485_PFM_TX_MARKER_ENABLE 0
#define USER_RS485_PFM_TX_MARKER_GPIO -1
