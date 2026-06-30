#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * Compile-time gate for verbose MS/TP RS-485 frame logging.
 * 0 = keep only essential/critical logs (default)
 * 1 = enable frame-level debug logs
 */
#ifndef MSTP_DEBUG_ENABLE
#define MSTP_DEBUG_ENABLE 0
#endif

void MSTP_RS485_Init(void);
void MSTP_RS485_Send(const uint8_t *payload, uint16_t payload_len);
bool MSTP_RS485_Read(uint8_t *buf);
bool MSTP_RS485_Transmitting(void);
uint32_t MSTP_RS485_Baud_Rate(void);
bool MSTP_RS485_Baud_Rate_Set(uint32_t baud);
uint32_t MSTP_RS485_Silence_Milliseconds(void);
void MSTP_RS485_Silence_Reset(void);
uint32_t MSTP_RS485_Rx_Bytes_Get_Reset(void);
void MSTP_RS485_Preamble_Counts_Get_Reset(uint32_t *preamble55, uint32_t *preamble55ff);
