/**
 * @file
 * @author Steve Karg <skarg@users.sourceforge.net>
 * @date February 2023
 * @brief Implementation of the Network Layer using BACnet MS/TP transport
 * @copyright SPDX-License-Identifier: MIT
 * @defgroup DLMSTP BACnet MS/TP DataLink Network Layer
 * @ingroup DataLink
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* BACnet Stack defines - first */
#include "bacnet/bacdef.h"
#include "bacnet/bacenum.h"
/* BACnet Stack API */
#include "bacnet/basic/sys/ringbuf.h"
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/datalink/crc.h"
#include "bacnet/datalink/mstp.h"
#include "bacnet/datalink/dlmstp.h"
#include "bacnet/datalink/mstpdef.h"
#include "bacnet/npdu.h"
#include "bacnet/bacaddr.h"
#include "bacnet/basic/sys/debug.h"
#include "mstp_debug_tuning.h"

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "esp_timer.h"
#endif

extern void MSTP_RS485_PFM_RX_Timestamp_Set(uint64_t rx_timestamp_us);
extern void MSTP_RS485_Token_RX_Timestamp_Set(uint64_t rx_timestamp_us);

typedef struct {
    bool valid;
    uint8_t src;
    uint8_t dst;
    uint32_t pre_delay_us;
    uint32_t de_pre_us;
    uint32_t pfm_rx_to_de_enable_us;
    uint32_t de_enable_to_uart_write_us;
    uint32_t pfm_rx_to_uart_write_us;
    uint32_t uart_write_to_tx_done_us;
    uint32_t total_pfm_rx_to_tx_done_us;
    bool tx_done_ok;
} MSTP_RS485_PFM_REPLY_TIMING;

typedef struct {
    bool valid;
    uint8_t src;
    uint8_t dst;
    uint32_t pre_delay_us;
    uint32_t de_pre_us;
    uint32_t rx_to_de_us;
    uint32_t de_to_write_us;
    uint32_t rx_to_write_us;
    uint32_t write_to_done_us;
    uint32_t total_us;
    bool tx_done_ok;
    int uart_ret;
} MSTP_RS485_TOKEN_PASS_TIMING;

extern bool MSTP_RS485_PFM_Reply_Timing_Get_Reset(MSTP_RS485_PFM_REPLY_TIMING *timing);
extern int MSTP_RS485_Last_UART_Result_Get(void);
extern bool MSTP_RS485_Token_Pass_Timing_Get_Reset(MSTP_RS485_TOKEN_PASS_TIMING *timing);

#ifndef USER_MSTP_TOKEN_PASS_ONLY_DEBUG
#define USER_MSTP_TOKEN_PASS_ONLY_DEBUG 0
#endif

#ifndef USER_MSTP_DEBUG_FORCE_NEXT_STATION
#define USER_MSTP_DEBUG_FORCE_NEXT_STATION 32
#endif

#ifndef USER_MSTP_ZERO_CONFIG_ENABLED
#define USER_MSTP_ZERO_CONFIG_ENABLED 1
#endif

#ifndef DLMSTP_PFM_NO_TOKEN_WINDOW_MS
#define DLMSTP_PFM_NO_TOKEN_WINDOW_MS 1500U
#endif

#ifndef DLMSTP_POST_PASS_OBSERVE_WINDOW_MS
#define DLMSTP_POST_PASS_OBSERVE_WINDOW_MS 2000U
#endif

#ifndef DLMSTP_POST_PFM_OBSERVE_WINDOW_MS
#define DLMSTP_POST_PFM_OBSERVE_WINDOW_MS 2000U
#endif

#define DLMSTP_RAW_LOG_MAX_BYTES 128U

/* the current MSTP port that the datalink is using */
static struct mstp_port_struct_t *MSTP_Port;
static DLMSTP_SEND_STATUS DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_OK;
static const char *DLMSTP_Last_Send_Path = "none";
#define DLMSTP_DIAG_TARGET_MAC 25U
#define DLMSTP_PRIORITY_QUEUE_CAPACITY 2U
static struct dlmstp_packet *DLMSTP_Priority_Packets = NULL;
static uint8_t DLMSTP_Priority_Head = 0;
static uint8_t DLMSTP_Priority_Tail = 0;
static uint8_t DLMSTP_Priority_Count = 0;
static bool DLMSTP_Priority_Next_Send = false;
static DLMSTP_TOKEN_DIAGNOSTICS DLMSTP_Token_Diagnostics = { 0 };
static bool DLMSTP_Token_Cycle_Active = false;
static bool DLMSTP_Token_Cycle_Passed = false;
static uint8_t DLMSTP_Token_Cycle_Source = 0xFF;
static uint8_t DLMSTP_Token_Cycle_Destination = 0xFF;
static unsigned DLMSTP_Token_Cycle_App_Frames_Sent = 0;
static uint64_t DLMSTP_Token_Pass_Deadline_Ms = 0;
static bool DLMSTP_Token_No_Pass_Logged = false;
static uint8_t DLMSTP_Observed_Successor = 0xFF;
static uint64_t DLMSTP_Post_PFM_Window_Deadline_Ms = 0;
static uint8_t DLMSTP_Post_PFM_Expected_Token_Source = 0xFF;
static uint8_t DLMSTP_Post_PFM_Expected_Token_Destination = 0xFF;
static bool DLMSTP_Post_PFM_Expected_Token_Seen = false;
static uint64_t DLMSTP_PFM_To_Us_Last_Rx_Us = 0;
static uint64_t DLMSTP_PFM_Rx_To_Tx_Sum_Us = 0;
static uint32_t DLMSTP_PFM_Rx_To_Tx_Samples = 0;
static bool DLMSTP_PFM_Reply_Experiment_Pending = false;
static uint8_t DLMSTP_PFM_Reply_Experiment_Src = 0xFF;
static uint8_t DLMSTP_PFM_Reply_Experiment_Dst = 0xFF;
static uint32_t DLMSTP_PFM_Reply_Experiment_Rx_To_Tx_Us = 0;
static bool DLMSTP_Post_Pass_Observe_Active = false;
static uint64_t DLMSTP_Post_Pass_Observe_Deadline_Ms = 0;
static uint8_t DLMSTP_Post_Pass_Observe_Dst = 0xFF;
static bool DLMSTP_Post_Pass_Activity_Seen = false;
static bool DLMSTP_Post_Pass_Next_16_Logged = false;
static bool DLMSTP_Post_PFM_Observe_Active = false;
static uint64_t DLMSTP_Post_PFM_Observe_Deadline_Ms = 0;
static bool DLMSTP_Post_PFM_Observe_Logged = false;
static bool DLMSTP_Token_Pass_Armed = false;

#ifndef USER_MSTP_PFM_REPLY_PRE_DELAY_US
#define USER_MSTP_PFM_REPLY_PRE_DELAY_US 0
#endif

#ifndef USER_RS485_DE_PRE_TX_US
#define USER_RS485_DE_PRE_TX_US 1000
#endif

/* Temporary low-noise mode for MAC25 active debugging. */
#ifndef DLMSTP_MAC25_MIN_LOG_MODE
#if defined(USER_MSTP_ACTIVE_DEBUG_ONLY)
#define DLMSTP_MAC25_MIN_LOG_MODE USER_MSTP_ACTIVE_DEBUG_ONLY
#else
#define DLMSTP_MAC25_MIN_LOG_MODE 1
#endif
#endif

#if defined(ESP_PLATFORM)
#if USER_MSTP_ACTIVE_DEBUG_ONLY
#define DLMSTP_DIAG_LOGI(...) do { } while (0)
#define DLMSTP_DIAG_LOGW(...) do { } while (0)
#define MSTP_ACTIVE_LOGW(...) ESP_LOGW("mstp_active", __VA_ARGS__)
#else
#define DLMSTP_DIAG_LOGI(...) ESP_LOGI("dlmstp_diag", __VA_ARGS__)
#define DLMSTP_DIAG_LOGW(...) ESP_LOGW("dlmstp_diag", __VA_ARGS__)
#define MSTP_ACTIVE_LOGW(...) do { } while (0)
#endif
static uint64_t dlmstp_diag_timestamp_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000);
}
#else
#define DLMSTP_DIAG_LOGI(...) debug_printf(__VA_ARGS__)
#define DLMSTP_DIAG_LOGW(...) debug_printf(__VA_ARGS__)
#define MSTP_ACTIVE_LOGW(...) debug_printf(__VA_ARGS__)
static uint64_t dlmstp_diag_timestamp_ms(void)
{
    return (uint64_t)mstimer_now();
}
#endif

static bool dlmstp_priority_queue_empty(void)
{
    return DLMSTP_Priority_Count == 0;
}

static bool dlmstp_priority_queue_full(void)
{
    return DLMSTP_Priority_Count >= DLMSTP_PRIORITY_QUEUE_CAPACITY;
}

static struct dlmstp_packet *dlmstp_priority_queue_oldest(void)
{
    if (!DLMSTP_Priority_Packets || dlmstp_priority_queue_empty()) {
        return NULL;
    }

    return &DLMSTP_Priority_Packets[DLMSTP_Priority_Head];
}

static bool dlmstp_priority_queue_push(const struct dlmstp_packet *pkt)
{
    if (!DLMSTP_Priority_Packets || !pkt || dlmstp_priority_queue_full()) {
        return false;
    }

    DLMSTP_Priority_Packets[DLMSTP_Priority_Tail] = *pkt;
    DLMSTP_Priority_Tail =
        (uint8_t)((DLMSTP_Priority_Tail + 1U) % DLMSTP_PRIORITY_QUEUE_CAPACITY);
    DLMSTP_Priority_Count++;

    return true;
}

static bool dlmstp_priority_queue_pop(struct dlmstp_packet *pkt)
{
    if (!DLMSTP_Priority_Packets || dlmstp_priority_queue_empty()) {
        return false;
    }

    if (pkt) {
        *pkt = DLMSTP_Priority_Packets[DLMSTP_Priority_Head];
    }
    DLMSTP_Priority_Head =
        (uint8_t)((DLMSTP_Priority_Head + 1U) % DLMSTP_PRIORITY_QUEUE_CAPACITY);
    DLMSTP_Priority_Count--;

    return true;
}

static void dlmstp_priority_queue_clear(void)
{
    DLMSTP_Priority_Head = 0;
    DLMSTP_Priority_Tail = 0;
    DLMSTP_Priority_Count = 0;
}

static bool dlmstp_destination_valid(const BACNET_ADDRESS *dest)
{
    if (!dest || (dest->mac_len == 0)) {
        return true;
    }
    if (dest->mac[0] <= DEFAULT_MAX_MASTER) {
        return true;
    }
    if (dest->mac[0] == MSTP_BROADCAST_ADDRESS) {
        return true;
    }

    return false;
}

static bool dlmstp_master_has_token(MSTP_MASTER_STATE master_state)
{
    return (master_state == MSTP_MASTER_STATE_USE_TOKEN) ||
        (master_state == MSTP_MASTER_STATE_DONE_WITH_TOKEN) ||
        (master_state == MSTP_MASTER_STATE_WAIT_FOR_REPLY) ||
        (master_state == MSTP_MASTER_STATE_PASS_TOKEN);
}

static bool dlmstp_master_can_transmit(MSTP_MASTER_STATE master_state)
{
    return (master_state == MSTP_MASTER_STATE_USE_TOKEN) ||
        (master_state == MSTP_MASTER_STATE_ANSWER_DATA_REQUEST);
}

static bool dlmstp_post_pfm_window_active(uint64_t now_ms)
{
    return (DLMSTP_Post_PFM_Window_Deadline_Ms != 0) &&
        (now_ms <= DLMSTP_Post_PFM_Window_Deadline_Ms);
}

static void dlmstp_log_raw_frame_bytes(
    const char *prefix,
    const uint8_t *buffer,
    uint16_t nbytes)
{
    char bytes_text[3 * 64 + 1];
    size_t i = 0;
    size_t limit = 0;
    size_t used = 0;
    int written = 0;

    if (!prefix || !buffer || (nbytes == 0U)) {
        return;
    }

    limit = (nbytes < 64U) ? (size_t)nbytes : 64U;
    bytes_text[0] = '\0';
    for (i = 0; i < limit; i++) {
        written = snprintf(
            &bytes_text[used],
            sizeof(bytes_text) - used,
            (i == 0U) ? "%02X" : " %02X",
            (unsigned)buffer[i]);
        if ((written <= 0) || ((size_t)written >= (sizeof(bytes_text) - used))) {
            break;
        }
        used += (size_t)written;
    }

    MSTP_ACTIVE_LOGW("%s bytes=%s", prefix, bytes_text);
}

static void dlmstp_post_pfm_observe_start(void)
{
    DLMSTP_Post_PFM_Observe_Active = true;
    DLMSTP_Post_PFM_Observe_Deadline_Ms =
        dlmstp_diag_timestamp_ms() + DLMSTP_POST_PFM_OBSERVE_WINDOW_MS;
    DLMSTP_Post_PFM_Observe_Logged = false;
}

static void dlmstp_post_pfm_observe_reset(void)
{
    DLMSTP_Post_PFM_Observe_Active = false;
    DLMSTP_Post_PFM_Observe_Deadline_Ms = 0;
    DLMSTP_Post_PFM_Observe_Logged = false;
}

static void dlmstp_post_pfm_observe_valid_frame(
    uint8_t source,
    uint8_t destination,
    uint8_t frame_type)
{
    if (!DLMSTP_Post_PFM_Observe_Active || DLMSTP_Post_PFM_Observe_Logged) {
        return;
    }

    DLMSTP_Post_PFM_Observe_Logged = true;
    if ((frame_type == FRAME_TYPE_TOKEN) && (source == 16U) && (destination == 17U)) {
        MSTP_ACTIVE_LOGW("POST_PFM_TOKEN_RX src=16 dst=17");
    } else {
        MSTP_ACTIVE_LOGW(
            "POST_PFM_NEXT_FRAME src=%u dst=%u frame_type=%u",
            (unsigned)source,
            (unsigned)destination,
            (unsigned)frame_type);
    }
    dlmstp_post_pfm_observe_reset();
}

static void dlmstp_post_pfm_observe_check_timeout(void)
{
    uint64_t now_ms = dlmstp_diag_timestamp_ms();

    if (!DLMSTP_Post_PFM_Observe_Active) {
        return;
    }
    if (now_ms <= DLMSTP_Post_PFM_Observe_Deadline_Ms) {
        return;
    }

    if (!DLMSTP_Post_PFM_Observe_Logged) {
        MSTP_ACTIVE_LOGW(
            "POST_PFM_NO_VALID_FRAME after_ms=%u",
            (unsigned)DLMSTP_POST_PFM_OBSERVE_WINDOW_MS);
    }
    dlmstp_post_pfm_observe_reset();
}

static void dlmstp_post_pass_observe_start(uint8_t destination_mac)
{
    DLMSTP_Post_Pass_Observe_Active = true;
    DLMSTP_Post_Pass_Observe_Deadline_Ms =
        dlmstp_diag_timestamp_ms() + DLMSTP_POST_PASS_OBSERVE_WINDOW_MS;
    DLMSTP_Post_Pass_Observe_Dst = destination_mac;
    DLMSTP_Post_Pass_Activity_Seen = false;
    DLMSTP_Post_Pass_Next_16_Logged = false;
}

static void dlmstp_post_pass_observe_reset(void)
{
    DLMSTP_Post_Pass_Observe_Active = false;
    DLMSTP_Post_Pass_Observe_Deadline_Ms = 0;
    DLMSTP_Post_Pass_Observe_Dst = 0xFF;
    DLMSTP_Post_Pass_Activity_Seen = false;
    DLMSTP_Post_Pass_Next_16_Logged = false;
}

static void dlmstp_post_pass_observe_valid_frame(
    uint8_t source,
    uint8_t destination,
    uint8_t frame_type)
{
    if (!DLMSTP_Post_Pass_Observe_Active) {
        return;
    }

    if (!DLMSTP_Post_Pass_Activity_Seen &&
        (source == DLMSTP_Post_Pass_Observe_Dst)) {
        DLMSTP_Post_Pass_Activity_Seen = true;
        MSTP_ACTIVE_LOGW(
            "POST_PASS_ACTIVITY src=%u dst=%u frame_type=%u",
            (unsigned)source,
            (unsigned)destination,
            (unsigned)frame_type);
    }

    if (!DLMSTP_Post_Pass_Next_16_Logged && (source == 16U) &&
        (destination == 17U) &&
        ((frame_type == FRAME_TYPE_POLL_FOR_MASTER) ||
            (frame_type == FRAME_TYPE_TOKEN))) {
        DLMSTP_Post_Pass_Next_16_Logged = true;
        MSTP_ACTIVE_LOGW(
            "POST_PASS_NEXT_16 src=%u dst=%u frame_type=%u",
            (unsigned)source,
            (unsigned)destination,
            (unsigned)frame_type);
    }
}

static void dlmstp_post_pass_observe_check_timeout(void)
{
    uint64_t now_ms = dlmstp_diag_timestamp_ms();

    if (!DLMSTP_Post_Pass_Observe_Active) {
        return;
    }
    if (now_ms <= DLMSTP_Post_Pass_Observe_Deadline_Ms) {
        return;
    }

    if (!DLMSTP_Post_Pass_Activity_Seen) {
        MSTP_ACTIVE_LOGW(
            "POST_PASS_NO_ACTIVITY dst=%u after_ms=%u",
            (unsigned)DLMSTP_Post_Pass_Observe_Dst,
            (unsigned)DLMSTP_POST_PASS_OBSERVE_WINDOW_MS);
    }

    dlmstp_post_pass_observe_reset();
}

static const char *dlmstp_master_state_text_short(MSTP_MASTER_STATE state);

static void dlmstp_post_pfm_log_frame_event(
    const struct mstp_port_struct_t *mstp_port,
    const char *rx_state,
    bool valid_header_crc,
    bool valid_data_crc)
{
    /* Keep this silent in fast mode; only log when token is not received in
       time via timeout handler. */
    (void)mstp_port;
    (void)rx_state;
    (void)valid_header_crc;
    (void)valid_data_crc;
    return;
}

static void dlmstp_post_pfm_window_check_timeout(void)
{
    uint64_t now_ms = dlmstp_diag_timestamp_ms();

    if (!DLMSTP_Post_PFM_Window_Deadline_Ms) {
        return;
    }
    if (now_ms <= DLMSTP_Post_PFM_Window_Deadline_Ms) {
        return;
    }

    if (!DLMSTP_Post_PFM_Expected_Token_Seen) {
        DLMSTP_Token_Diagnostics.pfm_no_token_counter++;
#if !USER_MSTP_TOKEN_PASS_ONLY_DEBUG
        MSTP_ACTIVE_LOGW(
            "PFM_NO_TOKEN src=%u dst=%u after_ms=%u",
            (unsigned)DLMSTP_Post_PFM_Expected_Token_Source,
            (unsigned)DLMSTP_Post_PFM_Expected_Token_Destination,
            (unsigned)DLMSTP_PFM_NO_TOKEN_WINDOW_MS);
#endif
    }

    DLMSTP_Post_PFM_Window_Deadline_Ms = 0;
    DLMSTP_Post_PFM_Expected_Token_Source = 0xFF;
    DLMSTP_Post_PFM_Expected_Token_Destination = 0xFF;
    DLMSTP_Post_PFM_Expected_Token_Seen = false;
    DLMSTP_PFM_Reply_Experiment_Pending = false;
}

static void dlmstp_token_pass_check_timeout(void)
{
    uint64_t now_ms = dlmstp_diag_timestamp_ms();

    if (!DLMSTP_Token_Cycle_Active || DLMSTP_Token_Cycle_Passed ||
        DLMSTP_Token_No_Pass_Logged || !DLMSTP_Token_Pass_Deadline_Ms) {
        return;
    }
    if (now_ms <= DLMSTP_Token_Pass_Deadline_Ms) {
        return;
    }

    if (MSTP_Port) {
#if !USER_MSTP_TOKEN_PASS_ONLY_DEBUG
        MSTP_ACTIVE_LOGW(
            "TOKEN_RX_NO_PASS src=%u dst=%u",
            (unsigned)DLMSTP_Token_Cycle_Source,
            (unsigned)DLMSTP_Token_Cycle_Destination);
#endif
    }
    DLMSTP_Token_No_Pass_Logged = true;
}

static const char *dlmstp_master_state_text_short(MSTP_MASTER_STATE state)
{
    switch (state) {
        case MSTP_MASTER_STATE_ANSWER_DATA_REQUEST:
            return "ANSWER_DATA_REQUEST";
        case MSTP_MASTER_STATE_USE_TOKEN:
            return "USE_TOKEN";
        case MSTP_MASTER_STATE_WAIT_FOR_REPLY:
            return "WAIT_FOR_REPLY";
        case MSTP_MASTER_STATE_DONE_WITH_TOKEN:
            return "DONE_WITH_TOKEN";
        case MSTP_MASTER_STATE_IDLE:
            return "IDLE";
        default:
            break;
    }

    return "OTHER";
}

static DLMSTP_TX_SOURCE dlmstp_classify_tx_source(
    const uint8_t *pdu,
    unsigned pdu_len)
{
    int npdu_offset = 0;
    BACNET_ADDRESS dest = { 0 };
    BACNET_ADDRESS src = { 0 };
    BACNET_NPDU_DATA npdu_data = { 0 };
    const uint8_t *apdu = NULL;
    unsigned apdu_len = 0;
    uint8_t pdu_type = 0;

    if (!pdu || (pdu_len < 2)) {
        return DLMSTP_TX_SOURCE_OTHER;
    }

    npdu_offset = bacnet_npdu_decode(pdu, (uint16_t)pdu_len, &dest, &src, &npdu_data);
    if ((npdu_offset <= 0) || ((unsigned)npdu_offset >= pdu_len)) {
        return DLMSTP_TX_SOURCE_OTHER;
    }

    apdu = &pdu[npdu_offset];
    apdu_len = pdu_len - (unsigned)npdu_offset;
    pdu_type = (uint8_t)(apdu[0] & 0xF0);

    if ((pdu_type == PDU_TYPE_SIMPLE_ACK) ||
        (pdu_type == PDU_TYPE_COMPLEX_ACK) ||
        (pdu_type == PDU_TYPE_ERROR) ||
        (pdu_type == PDU_TYPE_REJECT) ||
        (pdu_type == PDU_TYPE_ABORT)) {
        return DLMSTP_TX_SOURCE_FINAL_ACK;
    }
    if ((pdu_type == PDU_TYPE_UNCONFIRMED_SERVICE_REQUEST) && (apdu_len >= 2)) {
        if (apdu[1] == SERVICE_UNCONFIRMED_I_AM) {
            return DLMSTP_TX_SOURCE_I_AM;
        }
        if (apdu[1] == SERVICE_UNCONFIRMED_COV_NOTIFICATION) {
            return DLMSTP_TX_SOURCE_COV;
        }
    }

    return DLMSTP_TX_SOURCE_OTHER;
}

static struct dlmstp_packet *dlmstp_oldest_packet(
    struct dlmstp_user_data_t *user,
    unsigned long *age_ms)
{
    struct dlmstp_packet *pkt = NULL;
    unsigned long now_ms = mstimer_now();

    if (age_ms) {
        *age_ms = 0;
    }
    if (!user || Ringbuf_Empty(&user->PDU_Queue)) {
        return NULL;
    }

    pkt = (struct dlmstp_packet *)(void *)Ringbuf_Peek(&user->PDU_Queue);
    if (pkt && age_ms && pkt->queued_ms) {
        *age_ms = now_ms - pkt->queued_ms;
    }

    return pkt;
}

static bool dlmstp_drop_queued_source(
    struct dlmstp_user_data_t *user,
    uint8_t source_tag)
{
    bool dropped = false;
    volatile uint8_t *elem = NULL;
    volatile uint8_t *next = NULL;
    struct dlmstp_packet *pkt = NULL;

    if (!user) {
        return false;
    }

    elem = Ringbuf_Peek(&user->PDU_Queue);
    while (elem) {
        next = Ringbuf_Peek_Next(&user->PDU_Queue, (const uint8_t *)elem);
        pkt = (struct dlmstp_packet *)(void *)elem;
        if (pkt->source_tag == source_tag) {
            if (Ringbuf_Pop_Element(&user->PDU_Queue, (const uint8_t *)elem, NULL)) {
                dropped = true;
            }
        }
        elem = next;
    }

    return dropped;
}

static bool dlmstp_drop_priority_source(uint8_t source_tag)
{
    struct dlmstp_packet survivors[DLMSTP_PRIORITY_QUEUE_CAPACITY];
    uint8_t survivor_count = 0;
    uint8_t i = 0;
    bool dropped = false;

    if (!DLMSTP_Priority_Packets || dlmstp_priority_queue_empty()) {
        return false;
    }

    for (i = 0; i < DLMSTP_Priority_Count; i++) {
        uint8_t index =
            (uint8_t)((DLMSTP_Priority_Head + i) % DLMSTP_PRIORITY_QUEUE_CAPACITY);
        if (DLMSTP_Priority_Packets[index].source_tag == source_tag) {
            dropped = true;
        } else {
            survivors[survivor_count++] = DLMSTP_Priority_Packets[index];
        }
    }

    dlmstp_priority_queue_clear();
    for (i = 0; i < survivor_count; i++) {
        (void)dlmstp_priority_queue_push(&survivors[i]);
    }

    return dropped;
}

bool dlmstp_token_held(void)
{
    if (!MSTP_Port) {
        return false;
    }

    return dlmstp_master_has_token(MSTP_Port->master_state);
}

bool dlmstp_can_transmit_now(void)
{
    if (!MSTP_Port) {
        return false;
    }

    return dlmstp_master_can_transmit(MSTP_Port->master_state);
}

/**
 * @brief send an PDU via MSTP
 * @param dest - BACnet destination address
 * @param npdu_data - network layer information
 * @param pdu - PDU data to send
 * @param pdu_len - number of bytes of PDU data to send
 * @return number of bytes sent on success, zero on failure
 */
int dlmstp_send_pdu(
    BACNET_ADDRESS *dest,
    BACNET_NPDU_DATA *npdu_data,
    uint8_t *pdu,
    unsigned pdu_len)
{
    int bytes_sent = 0;
    unsigned i = 0; /* loop counter */
    struct dlmstp_user_data_t *user = NULL;
    struct dlmstp_packet *pkt = NULL;
    struct dlmstp_packet *oldest_pkt = NULL;
    unsigned queue_capacity = 0;
    unsigned long oldest_age_ms = 0;
    uint8_t frame_type = 0;
    uint8_t destination_mac = MSTP_BROADCAST_ADDRESS;
    DLMSTP_TX_SOURCE tx_source = DLMSTP_TX_SOURCE_OTHER;
    struct dlmstp_packet priority_pkt = { 0 };
    MSTP_MASTER_STATE master_state = MSTP_MASTER_STATE_IDLE;
    bool token_owned = false;
    bool can_transmit_now = false;
    bool priority_next_send = false;

    DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_OTHER;
    DLMSTP_Last_Send_Path = "failed";

    if (!MSTP_Port) {
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_NO_PORT;
        return 0;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_NO_USER;
        return 0;
    }
    if (!npdu_data || !pdu || (pdu_len == 0)) {
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_OTHER;
        return 0;
    }
    if (!dlmstp_destination_valid(dest)) {
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_INVALID_DESTINATION;
        return 0;
    }
    if (pdu_len > DLMSTP_MPDU_MAX) {
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_PDU_TOO_LARGE;
        return 0;
    }

    if (npdu_data->data_expecting_reply) {
        frame_type = FRAME_TYPE_BACNET_DATA_EXPECTING_REPLY;
    } else {
        frame_type = FRAME_TYPE_BACNET_DATA_NOT_EXPECTING_REPLY;
    }
    if (dest && dest->mac_len) {
        destination_mac = dest->mac[0];
    }
    tx_source = dlmstp_classify_tx_source(pdu, pdu_len);
    queue_capacity = Ringbuf_Size(&user->PDU_Queue);
    master_state = MSTP_Port->master_state;
    token_owned = dlmstp_master_has_token(master_state);
    can_transmit_now = dlmstp_master_can_transmit(master_state);
    priority_next_send = DLMSTP_Priority_Next_Send;
    DLMSTP_Priority_Next_Send = false;
#if USER_MSTP_ACTIVE_DEBUG_ONLY
    (void)token_owned;
#endif

    if (!can_transmit_now) {
        DLMSTP_Token_Diagnostics.app_frames_blocked_no_token_counter++;
    }

    if (tx_source == DLMSTP_TX_SOURCE_FINAL_ACK) {
        priority_next_send = false;
        (void)dlmstp_drop_priority_source((uint8_t)DLMSTP_TX_SOURCE_I_AM);
        (void)dlmstp_drop_queued_source(user, (uint8_t)DLMSTP_TX_SOURCE_I_AM);
    }

    if (!DLMSTP_MAC25_MIN_LOG_MODE) {
        DLMSTP_DIAG_LOGI(
            "app_tx_request reason=%s dst_mac=%u token_owned=%s can_transmit_now=%s mstp_state=%s",
            dlmstp_tx_source_text(tx_source),
            (unsigned)destination_mac,
            token_owned ? "yes" : "no",
            can_transmit_now ? "yes" : "no",
            dlmstp_master_state_text_short(master_state));
    }

    if (priority_next_send) {
        priority_pkt.frame_type = frame_type;
        priority_pkt.source_tag = (uint8_t)tx_source;
        priority_pkt.queued_ms = mstimer_now();
        for (i = 0; i < pdu_len; i++) {
            priority_pkt.pdu[i] = pdu[i];
        }
        priority_pkt.pdu_len = pdu_len;
        if (dest && dest->mac_len) {
            priority_pkt.address.mac_len = 1;
            priority_pkt.address.mac[0] = dest->mac[0];
            priority_pkt.address.len = 0;
        } else {
            priority_pkt.address.mac_len = 1;
            priority_pkt.address.mac[0] = MSTP_BROADCAST_ADDRESS;
            priority_pkt.address.len = 0;
        }
        if (!dlmstp_priority_queue_push(&priority_pkt)) {
            DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_QUEUE_FULL;
            DLMSTP_Last_Send_Path = "high_priority_full";
            if (!DLMSTP_MAC25_MIN_LOG_MODE) {
                DLMSTP_DIAG_LOGI(
                    "app_tx_result reason=%s dst_mac=%u token_owned=%s can_transmit_now=%s mstp_state=%s tx_ret=%d",
                    dlmstp_tx_source_text(tx_source),
                    (unsigned)destination_mac,
                    token_owned ? "yes" : "no",
                    can_transmit_now ? "yes" : "no",
                    dlmstp_master_state_text_short(master_state),
                    0);
            }

            return 0;
        }
        bytes_sent = pdu_len;
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_OK;
        DLMSTP_Last_Send_Path = "high_priority";
        if (!DLMSTP_MAC25_MIN_LOG_MODE) {
            DLMSTP_DIAG_LOGI(
                "app_tx_result reason=%s dst_mac=%u token_owned=%s can_transmit_now=%s mstp_state=%s tx_ret=%d",
                dlmstp_tx_source_text(tx_source),
                (unsigned)destination_mac,
                token_owned ? "yes" : "no",
                can_transmit_now ? "yes" : "no",
                dlmstp_master_state_text_short(master_state),
                bytes_sent);
        }

        return bytes_sent;
    }

    pkt = (struct dlmstp_packet *)(void *)Ringbuf_Data_Peek(&user->PDU_Queue);
    debug_printf(
        "dlmstp send_path=queued frame=%u dst=%u pdu_len=%u master_state=%s\n",
        (unsigned)frame_type, (unsigned)destination_mac, (unsigned)pdu_len,
        dlmstp_master_state_text_short(master_state));
    if (pkt) {
        pkt->frame_type = frame_type;
        pkt->source_tag = (uint8_t)tx_source;
        pkt->queued_ms = mstimer_now();
        for (i = 0; i < pdu_len; i++) {
            pkt->pdu[i] = pdu[i];
        }
        pkt->pdu_len = pdu_len;
        if (dest && dest->mac_len) {
            pkt->address.mac_len = 1;
            pkt->address.mac[0] = dest->mac[0];
            pkt->address.len = 0;
        } else {
            pkt->address.mac_len = 1;
            pkt->address.mac[0] = MSTP_BROADCAST_ADDRESS;
            pkt->address.len = 0;
        }
        if (Ringbuf_Data_Put(&user->PDU_Queue, (uint8_t *)pkt)) {
            bytes_sent = pdu_len;
            DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_OK;
            DLMSTP_Last_Send_Path = "queued";
        } else {
            DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_QUEUE_FULL;
            oldest_pkt = dlmstp_oldest_packet(user, &oldest_age_ms);
            (void)frame_type;
            (void)destination_mac;
            (void)pdu_len;
            (void)tx_source;
            (void)oldest_age_ms;
            (void)oldest_pkt;
        }
    } else {
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_QUEUE_FULL;
        oldest_pkt = dlmstp_oldest_packet(user, &oldest_age_ms);
        (void)queue_capacity;
        (void)frame_type;
        (void)destination_mac;
        (void)pdu_len;
        (void)tx_source;
        (void)oldest_age_ms;
        (void)oldest_pkt;
    }

    if (!DLMSTP_MAC25_MIN_LOG_MODE) {
        DLMSTP_DIAG_LOGI(
            "app_tx_result reason=%s dst_mac=%u token_owned=%s can_transmit_now=%s mstp_state=%s tx_ret=%d",
            dlmstp_tx_source_text(tx_source),
            (unsigned)destination_mac,
            token_owned ? "yes" : "no",
            can_transmit_now ? "yes" : "no",
            dlmstp_master_state_text_short(master_state),
            bytes_sent);
    }

    return bytes_sent;
}

const char *dlmstp_send_path_last_text(void)
{
    return DLMSTP_Last_Send_Path;
}

bool dlmstp_send_pdu_queue_oldest_source(
    DLMSTP_TX_SOURCE *source,
    unsigned long *age_ms)
{
    struct dlmstp_user_data_t *user = NULL;
    struct dlmstp_packet *pkt = NULL;
    struct dlmstp_packet *priority_pkt = NULL;

    if (source) {
        *source = DLMSTP_TX_SOURCE_OTHER;
    }
    if (age_ms) {
        *age_ms = 0;
    }
    if (!MSTP_Port) {
        return false;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return false;
    }

    priority_pkt = dlmstp_priority_queue_oldest();
    if (priority_pkt) {
        if (source) {
            *source = (DLMSTP_TX_SOURCE)priority_pkt->source_tag;
        }
        if (age_ms) {
            *age_ms = mstimer_now() - priority_pkt->queued_ms;
        }

        return true;
    }

    pkt = dlmstp_oldest_packet(user, age_ms);
    if (!pkt) {
        return false;
    }
    if (source) {
        *source = (DLMSTP_TX_SOURCE)pkt->source_tag;
    }

    return true;
}

const char *dlmstp_master_state_text(void)
{
    if (!MSTP_Port) {
        return "NO_PORT";
    }

    return dlmstp_master_state_text_short(MSTP_Port->master_state);
}

DLMSTP_SEND_STATUS dlmstp_send_status_last(void)
{
    return DLMSTP_Last_Send_Status;
}

const char *dlmstp_send_status_text(DLMSTP_SEND_STATUS status)
{
    switch (status) {
        case DLMSTP_SEND_STATUS_OK:
            return "ok";
        case DLMSTP_SEND_STATUS_NO_PORT:
            return "other:no-port";
        case DLMSTP_SEND_STATUS_NO_USER:
            return "other:no-user";
        case DLMSTP_SEND_STATUS_INVALID_DESTINATION:
            return "invalid destination";
        case DLMSTP_SEND_STATUS_PDU_TOO_LARGE:
            return "other:pdu-too-large";
        case DLMSTP_SEND_STATUS_QUEUE_FULL:
            return "queue full (PDU_Queue)";
        case DLMSTP_SEND_STATUS_NOT_ALLOWED_STATE:
            return "not in allowed MS/TP state";
        case DLMSTP_SEND_STATUS_OTHER:
        default:
            break;
    }

    return "other";
}

const char *dlmstp_tx_source_text(DLMSTP_TX_SOURCE source)
{
    switch (source) {
        case DLMSTP_TX_SOURCE_I_AM:
            return "I-Am";
        case DLMSTP_TX_SOURCE_FINAL_ACK:
            return "final-ACK";
        case DLMSTP_TX_SOURCE_COV:
            return "COV";
        case DLMSTP_TX_SOURCE_OTHER:
        default:
            break;
    }

    return "other";
}

bool dlmstp_send_priority_slot_ready(void)
{
    return !dlmstp_priority_queue_empty();
}

const char *dlmstp_send_priority_slot_source_text(void)
{
    struct dlmstp_packet *pkt = dlmstp_priority_queue_oldest();

    if (!pkt) {
        return "empty";
    }

    return dlmstp_tx_source_text((DLMSTP_TX_SOURCE)pkt->source_tag);
}

bool dlmstp_send_reply_postponed(uint8_t destination_mac)
{
    struct dlmstp_user_data_t *user = NULL;

    if (!MSTP_Port) {
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_NO_PORT;
        return false;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_NO_USER;
        return false;
    }
    if ((destination_mac > DEFAULT_MAX_MASTER) &&
        (destination_mac != MSTP_BROADCAST_ADDRESS)) {
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_INVALID_DESTINATION;
        return false;
    }
    if ((MSTP_Port->master_state != MSTP_MASTER_STATE_ANSWER_DATA_REQUEST) ||
        (MSTP_Port->SourceAddress != destination_mac)) {
        DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_NOT_ALLOWED_STATE;
        return false;
    }

    MSTP_Create_And_Send_Frame(
        MSTP_Port, FRAME_TYPE_REPLY_POSTPONED, destination_mac,
        MSTP_Port->This_Station, NULL, 0);
    /* Keep ANSWER_DATA_REQUEST context so the matching final ACK can
       still be transmitted by the normal reply path when ready. */
    DLMSTP_Last_Send_Status = DLMSTP_SEND_STATUS_OK;

    return true;
}

/**
 * @brief The MS/TP state machine uses this function for getting data to send
 * @param mstp_port - specific MSTP port that is used for this datalink
 * @param timeout - number of milliseconds to wait for the data
 * @return amount of PDU data
 */
uint16_t MSTP_Get_Send(struct mstp_port_struct_t *mstp_port, unsigned timeout)
{
    uint16_t pdu_len = 0;
    struct dlmstp_packet *pkt;
    struct dlmstp_packet priority_pkt = { 0 };
    struct dlmstp_user_data_t *user;

    (void)timeout;
    if (!mstp_port) {
        return 0;
    }
    user = (struct dlmstp_user_data_t *)mstp_port->UserData;
    if (!user) {
        return 0;
    }
    if (dlmstp_priority_queue_pop(&priority_pkt)) {
        pdu_len = MSTP_Create_Frame(
            &mstp_port->OutputBuffer[0], mstp_port->OutputBufferSize,
            priority_pkt.frame_type,
            priority_pkt.address.mac[0], mstp_port->This_Station,
            &priority_pkt.pdu[0], priority_pkt.pdu_len);
        user->Statistics.transmit_pdu_counter++;

        return pdu_len;
    }
    if (Ringbuf_Empty(&user->PDU_Queue)) {
        return 0;
    }
    /* look at next PDU in queue without removing it */
    pkt = (struct dlmstp_packet *)(void *)Ringbuf_Peek(&user->PDU_Queue);
    /* convert the PDU into the MSTP Frame */
    pdu_len = MSTP_Create_Frame(
        &mstp_port->OutputBuffer[0], mstp_port->OutputBufferSize,
        pkt->frame_type, pkt->address.mac[0], mstp_port->This_Station,
        &pkt->pdu[0], pkt->pdu_len);
    user->Statistics.transmit_pdu_counter++;
    (void)Ringbuf_Pop(&user->PDU_Queue, NULL);

    return pdu_len;
}

/**
 * @brief The MS/TP state machine uses this function for getting data to send
 *  as the reply to a DATA_EXPECTING_REPLY frame, or nothing
 * @param mstp_port MSTP port structure for this port
 * @param timeout number of milliseconds to wait for a packet
 * @return number of bytes, or 0 if no reply is available
 */
uint16_t MSTP_Get_Reply(struct mstp_port_struct_t *mstp_port, unsigned timeout)
{
    uint16_t pdu_len = 0;
    bool matched = false;
    struct dlmstp_user_data_t *user = NULL;
    struct dlmstp_packet *pkt;

    (void)timeout;
    if (!mstp_port) {
        return 0;
    }
    user = mstp_port->UserData;
    if (!user) {
        return 0;
    }
    if (Ringbuf_Empty(&user->PDU_Queue)) {
        return 0;
    }
    /* look at next PDU in queue without removing it */
    pkt = (struct dlmstp_packet *)(void *)Ringbuf_Peek(&user->PDU_Queue);
    /* is this the reply to the DER? */
    matched = npdu_is_data_expecting_reply(
        &mstp_port->InputBuffer[0], mstp_port->DataLength,
        mstp_port->SourceAddress, &pkt->pdu[0], pkt->pdu_len,
        pkt->address.mac[0]);
    if (!matched) {
        return 0;
    }
    if (npdu_is_segmented_complex_ack_reply(pkt->pdu, pkt->pdu_len)) {
        /* In Clause 5.4.5.3, AWAIT_RESPONSE, in the transition
            SendSegmentedComplexACK, the text "transmit a BACnet-
            ComplexACK-PDU..." shall be replaced by "direct the
            MS/TP data link to transmit a Reply Postponed frame;
            transmit a BACnet-ComplexACK-PDU...."
            It is necessary to postpone the reply because
            transmission of the segmented ComplexACK
            cannot begin until the node holds the token.*/
        return MSTP_Create_Frame(
            &mstp_port->OutputBuffer[0], mstp_port->OutputBufferSize,
            FRAME_TYPE_REPLY_POSTPONED, mstp_port->SourceAddress,
            mstp_port->This_Station, NULL, 0);
    }
    /* convert the PDU into the MSTP Frame */
    pdu_len = MSTP_Create_Frame(
        &mstp_port->OutputBuffer[0], mstp_port->OutputBufferSize,
        pkt->frame_type, pkt->address.mac[0], mstp_port->This_Station,
        &pkt->pdu[0], pkt->pdu_len);
    user->Statistics.transmit_pdu_counter++;
    (void)Ringbuf_Pop(&user->PDU_Queue, NULL);

    return pdu_len;
}

/**
 * @brief MS/TP state machine callback to use for sending a frame
 * @param mstp_port - specific MSTP port that is used for this datalink
 * @param buffer - buffer to send
 * @param nbytes - number of bytes of data to send
 */
void MSTP_Send_Frame(
    struct mstp_port_struct_t *mstp_port,
    const uint8_t *buffer,
    uint16_t nbytes)
{
    struct dlmstp_user_data_t *user;
    struct dlmstp_rs485_driver *driver;

    if (!mstp_port) {
        return;
    }
    user = mstp_port->UserData;
    if (!user) {
        return;
    }
    driver = user->RS485_Driver;
    if (!driver) {
        return;
    }

    bool fast_reply_to_pfm_frame = false;
    uint8_t fast_reply_src = 0xFF;
    uint8_t fast_reply_dst = 0xFF;
    bool token_pass_frame_pending = false;
    uint8_t token_pass_dst = 0xFF;
    uint8_t fast_reply_raw[DLMSTP_RAW_LOG_MAX_BYTES] = { 0 };
    uint16_t fast_reply_raw_len = 0;
    uint8_t token_pass_raw[DLMSTP_RAW_LOG_MAX_BYTES] = { 0 };
    uint16_t token_pass_raw_len = 0;

    if (buffer && (nbytes >= 8U) && (buffer[0] == 0x55U) && (buffer[1] == 0xFFU)) {
        uint8_t frame_type = buffer[2];
        uint8_t destination = buffer[3];
        uint8_t source = buffer[4];
        uint64_t now_ms = dlmstp_diag_timestamp_ms();
        bool window_active = dlmstp_post_pfm_window_active(now_ms);

        if ((frame_type == FRAME_TYPE_BACNET_DATA_EXPECTING_REPLY) ||
            (frame_type == FRAME_TYPE_BACNET_DATA_NOT_EXPECTING_REPLY) ||
            (frame_type == FRAME_TYPE_BACNET_EXTENDED_DATA_EXPECTING_REPLY) ||
            (frame_type == FRAME_TYPE_BACNET_EXTENDED_DATA_NOT_EXPECTING_REPLY)) {
            if (dlmstp_master_has_token(mstp_port->master_state)) {
                DLMSTP_Token_Diagnostics.app_frames_sent_with_token_counter++;
                if (DLMSTP_Token_Cycle_Active) {
                    DLMSTP_Token_Cycle_App_Frames_Sent++;
                }
            }
        }

        if ((frame_type == FRAME_TYPE_REPLY_TO_POLL_FOR_MASTER) &&
            (source == mstp_port->This_Station)) {
            fast_reply_raw_len = (nbytes < DLMSTP_RAW_LOG_MAX_BYTES) ?
                nbytes : DLMSTP_RAW_LOG_MAX_BYTES;
            if (fast_reply_raw_len > 0U) {
                memcpy(fast_reply_raw, buffer, fast_reply_raw_len);
            }
            DLMSTP_Token_Diagnostics.reply_to_poll_for_master_sent_counter++;
            DLMSTP_Token_Diagnostics.fast_reply_to_pfm_sent_counter++;
            DLMSTP_Post_PFM_Window_Deadline_Ms = now_ms + DLMSTP_PFM_NO_TOKEN_WINDOW_MS;
            DLMSTP_Post_PFM_Expected_Token_Source = destination;
            DLMSTP_Post_PFM_Expected_Token_Destination = source;
            DLMSTP_Post_PFM_Expected_Token_Seen = false;
            DLMSTP_PFM_Reply_Experiment_Pending = true;
            fast_reply_to_pfm_frame = true;
            fast_reply_src = source;
            fast_reply_dst = destination;
        }

        (void)window_active;

        if ((frame_type == FRAME_TYPE_TOKEN) && (source == mstp_port->This_Station)) {
            if (!DLMSTP_Token_Pass_Armed ||
                !dlmstp_master_has_token(mstp_port->master_state)) {
                DLMSTP_DIAG_LOGW(
                    "token_pass_blocked reason=no_local_token_ownership dst=%u state=%s",
                    (unsigned)destination,
                    dlmstp_master_state_text_short(mstp_port->master_state));
                return;
            }
            token_pass_raw_len = (nbytes < DLMSTP_RAW_LOG_MAX_BYTES) ?
                nbytes : DLMSTP_RAW_LOG_MAX_BYTES;
            if (token_pass_raw_len > 0U) {
                memcpy(token_pass_raw, buffer, token_pass_raw_len);
            }
            DLMSTP_Token_Diagnostics.token_passed_counter++;
            DLMSTP_Token_Cycle_Passed = true;
            DLMSTP_Token_Pass_Armed = false;
            DLMSTP_Token_Pass_Deadline_Ms = 0;
            token_pass_frame_pending = true;
            token_pass_dst = destination;
#if !USER_MSTP_TOKEN_PASS_ONLY_DEBUG
            MSTP_ACTIVE_LOGW(
                "TOKEN_USE done token_passed=yes app_frames_sent=%u",
                DLMSTP_Token_Cycle_App_Frames_Sent);
#endif
            if (DLMSTP_Token_Cycle_Active) {
                if (!DLMSTP_MAC25_MIN_LOG_MODE) {
                    DLMSTP_DIAG_LOGI(
                        "token_use done app_frames_sent=%u",
                        DLMSTP_Token_Cycle_App_Frames_Sent);
                }
                DLMSTP_Token_Cycle_Active = false;
            }
        }
    }

    if (buffer && (nbytes >= 8U) && (buffer[0] == 0x55U) && (buffer[1] == 0xFFU) &&
        ((buffer[2] == FRAME_TYPE_BACNET_DATA_EXPECTING_REPLY) ||
            (buffer[2] == FRAME_TYPE_BACNET_DATA_NOT_EXPECTING_REPLY) ||
            (buffer[2] == FRAME_TYPE_BACNET_EXTENDED_DATA_EXPECTING_REPLY) ||
            (buffer[2] == FRAME_TYPE_BACNET_EXTENDED_DATA_NOT_EXPECTING_REPLY)) &&
        !dlmstp_master_can_transmit(mstp_port->master_state) &&
        !DLMSTP_MAC25_MIN_LOG_MODE) {
        DLMSTP_DIAG_LOGW(
            "physical TX attempted while can_transmit_now=no frame_type=%u dst=%u src=%u mstp_state=%s",
            (unsigned)buffer[2],
            (unsigned)buffer[3],
            (unsigned)buffer[4],
            dlmstp_master_state_text_short(mstp_port->master_state));
    }

    driver->send(buffer, nbytes);

    if (token_pass_frame_pending) {
        MSTP_RS485_TOKEN_PASS_TIMING token_timing = { 0 };

        if (MSTP_RS485_Token_Pass_Timing_Get_Reset(&token_timing) && token_timing.valid) {
            MSTP_ACTIVE_LOGW(
                "TOKEN_PASS dst=%u pre_delay_us=%u de_pre_us=%u rx_to_de_us=%u de_to_write_us=%u rx_to_write_us=%u write_to_done_us=%u total_us=%u result=%s",
                (unsigned)token_pass_dst,
                (unsigned)token_timing.pre_delay_us,
                (unsigned)token_timing.de_pre_us,
                (unsigned)token_timing.rx_to_de_us,
                (unsigned)token_timing.de_to_write_us,
                (unsigned)token_timing.rx_to_write_us,
                (unsigned)token_timing.write_to_done_us,
                (unsigned)token_timing.total_us,
                token_timing.tx_done_ok ? "sent" : "fail");
        }
        if (token_pass_raw_len > 0U) {
            dlmstp_log_raw_frame_bytes("TOKEN_PASS_RAW", token_pass_raw, token_pass_raw_len);
        }

        if (token_pass_dst == USER_MSTP_DEBUG_FORCE_NEXT_STATION) {
            dlmstp_post_pass_observe_start(token_pass_dst);
        }
    }

    if (fast_reply_to_pfm_frame) {
        MSTP_RS485_PFM_REPLY_TIMING timing = { 0 };
        if (MSTP_RS485_PFM_Reply_Timing_Get_Reset(&timing) && timing.valid) {
            uint32_t rx_to_write_us = timing.pfm_rx_to_uart_write_us;
            DLMSTP_PFM_Rx_To_Tx_Sum_Us += rx_to_write_us;
            DLMSTP_PFM_Rx_To_Tx_Samples++;
            if ((DLMSTP_Token_Diagnostics.min_rx_to_tx_us == 0U) ||
                (rx_to_write_us < DLMSTP_Token_Diagnostics.min_rx_to_tx_us)) {
                DLMSTP_Token_Diagnostics.min_rx_to_tx_us = rx_to_write_us;
            }
            if (rx_to_write_us > DLMSTP_Token_Diagnostics.max_rx_to_tx_us) {
                DLMSTP_Token_Diagnostics.max_rx_to_tx_us = rx_to_write_us;
            }
            DLMSTP_PFM_Reply_Experiment_Src = timing.src;
            DLMSTP_PFM_Reply_Experiment_Dst = timing.dst;
            DLMSTP_PFM_Reply_Experiment_Rx_To_Tx_Us = rx_to_write_us;
            MSTP_ACTIVE_LOGW(
                "PFM_REPLY src=%u dst=%u pre_delay_us=%u de_pre_us=%u rx_to_de_us=%lu de_to_write_us=%lu rx_to_write_us=%lu write_to_done_us=%lu total_us=%lu tx_done=%s",
                (unsigned)timing.src,
                (unsigned)timing.dst,
                (unsigned)timing.pre_delay_us,
                (unsigned)timing.de_pre_us,
                (unsigned long)timing.pfm_rx_to_de_enable_us,
                (unsigned long)timing.de_enable_to_uart_write_us,
                (unsigned long)timing.pfm_rx_to_uart_write_us,
                (unsigned long)timing.uart_write_to_tx_done_us,
                (unsigned long)timing.total_pfm_rx_to_tx_done_us,
                timing.tx_done_ok ? "ESP_OK" : "ERR");
        } else {
            DLMSTP_PFM_Reply_Experiment_Src = fast_reply_src;
            DLMSTP_PFM_Reply_Experiment_Dst = fast_reply_dst;
            DLMSTP_PFM_Reply_Experiment_Rx_To_Tx_Us = 0U;
            MSTP_ACTIVE_LOGW(
                "PFM_REPLY src=%u dst=%u pre_delay_us=%u de_pre_us=%u rx_to_de_us=0 de_to_write_us=0 rx_to_write_us=0 write_to_done_us=0 total_us=0 tx_done=ERR",
                (unsigned)fast_reply_src,
                (unsigned)fast_reply_dst,
                (unsigned)USER_MSTP_PFM_REPLY_PRE_DELAY_US,
                (unsigned)USER_RS485_DE_PRE_TX_US);
        }
        (void)fast_reply_raw_len;

        dlmstp_post_pfm_observe_start();
    }

    user->Statistics.transmit_frame_counter++;
}

/**
 * @brief MS/TP state machine received a frame
 * @return number of bytes queued, or 0 if unable to be queued
 */
uint16_t MSTP_Put_Receive(struct mstp_port_struct_t *mstp_port)
{
    struct dlmstp_user_data_t *user = NULL;

    if (!mstp_port) {
        return 0;
    }
    user = mstp_port->UserData;
    if (!user) {
        return 0;
    }
    user->ReceivePacketPending = true;

    return mstp_port->DataLength;
}

/**
 * @brief Run the MS/TP state machines, and get packet if available
 * @param pdu - place to put PDU data for the caller
 * @param max_pdu - number of bytes of PDU data that caller can receive
 * @return number of bytes in received packet, or 0 if no packet was received
 * @note Must be called at least once every 1 milliseconds, with no more than
 *  5 milliseconds jitter.
 */
uint16_t dlmstp_receive(
    BACNET_ADDRESS *src, uint8_t *pdu, uint16_t max_pdu, unsigned timeout)
{
    uint16_t pdu_len = 0;
    uint8_t data_register = 0;
    struct dlmstp_user_data_t *user;
    struct dlmstp_rs485_driver *driver;
    uint16_t i;
    uint32_t milliseconds;
    MSTP_MASTER_STATE master_state;

    (void)timeout;
    if (!MSTP_Port) {
        return 0;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return 0;
    }
    driver = user->RS485_Driver;
    if (!driver) {
        return 0;
    }
    dlmstp_token_pass_check_timeout();
    dlmstp_post_pfm_window_check_timeout();
    dlmstp_post_pfm_observe_check_timeout();
    dlmstp_post_pass_observe_check_timeout();
    while (!MSTP_Port->InputBuffer) {
        /* FIXME: develop configure an input buffer! */
    }
    if (driver->transmitting()) {
        /* we're transmitting; do nothing else */
        return 0;
    }
    /* only do receive state machine while we don't have a frame */
    while ((MSTP_Port->ReceivedValidFrame == false) &&
           (MSTP_Port->ReceivedInvalidFrame == false) &&
           (MSTP_Port->ReceivedValidFrameNotForUs == false)) {
        MSTP_Port->DataAvailable = driver->read(&data_register);
        if (MSTP_Port->DataAvailable) {
            MSTP_Port->DataRegister = data_register;
        }
        MSTP_Receive_Frame_FSM(MSTP_Port);
        if (MSTP_Port->receive_state == MSTP_RECEIVE_STATE_PREAMBLE) {
            if (user->Preamble_Callback) {
                user->Preamble_Callback();
            }
        }
        /* process another byte, if available */
        if (!driver->read(NULL)) {
            break;
        }
    }
    if (MSTP_Port->ReceivedValidFrame || MSTP_Port->ReceivedInvalidFrame ||
        MSTP_Port->ReceivedValidFrameNotForUs) {
        /* delay after reception before transmitting - per MS/TP spec */
        milliseconds = MSTP_Port->SilenceTimer(MSTP_Port);
        if (milliseconds < MSTP_Port->Tturnaround_timeout) {
            /* we're waiting; do nothing else */
            return 0;
        }
    }
    if (MSTP_Port->ReceivedValidFrame) {
        user->Statistics.receive_valid_frame_counter++;
        dlmstp_post_pfm_observe_valid_frame(
            MSTP_Port->SourceAddress,
            MSTP_Port->DestinationAddress,
            MSTP_Port->FrameType);
        dlmstp_post_pass_observe_valid_frame(
            MSTP_Port->SourceAddress,
            MSTP_Port->DestinationAddress,
            MSTP_Port->FrameType);
        dlmstp_post_pfm_log_frame_event(MSTP_Port, "valid_for_us", true, true);
        if (MSTP_Port->FrameType == FRAME_TYPE_POLL_FOR_MASTER) {
            user->Statistics.poll_for_master_counter++;
            if (MSTP_Port->DestinationAddress == MSTP_Port->This_Station) {
                DLMSTP_Token_Diagnostics.poll_for_master_to_us_counter++;
                MSTP_ACTIVE_LOGW(
                    "PFM_RX src=%u dst=%u",
                    (unsigned)MSTP_Port->SourceAddress,
                    (unsigned)MSTP_Port->DestinationAddress);
#if defined(ESP_PLATFORM)
                DLMSTP_PFM_To_Us_Last_Rx_Us = (uint64_t)esp_timer_get_time();
#else
                DLMSTP_PFM_To_Us_Last_Rx_Us = (uint64_t)mstimer_now() * 1000ULL;
#endif
                MSTP_RS485_PFM_RX_Timestamp_Set(DLMSTP_PFM_To_Us_Last_Rx_Us);
            }
        } else if ((MSTP_Port->FrameType == FRAME_TYPE_TOKEN) &&
            (MSTP_Port->DestinationAddress == MSTP_Port->This_Station)) {
            DLMSTP_Token_Diagnostics.token_received_counter++;
            DLMSTP_Token_Pass_Armed = true;
            MSTP_ACTIVE_LOGW(
                "TOKEN_RX src=%u dst=%u",
                (unsigned)MSTP_Port->SourceAddress,
                (unsigned)MSTP_Port->DestinationAddress);
#if defined(ESP_PLATFORM)
            MSTP_RS485_Token_RX_Timestamp_Set((uint64_t)esp_timer_get_time());
#else
            MSTP_RS485_Token_RX_Timestamp_Set((uint64_t)mstimer_now() * 1000ULL);
#endif
            if ((MSTP_Port->SourceAddress == DLMSTP_Post_PFM_Expected_Token_Source) &&
                (MSTP_Port->DestinationAddress == DLMSTP_Post_PFM_Expected_Token_Destination)) {
                DLMSTP_Post_PFM_Expected_Token_Seen = true;
                DLMSTP_Token_Diagnostics.pfm_expected_token_accepted_counter++;
                DLMSTP_Post_PFM_Window_Deadline_Ms = 0;
                DLMSTP_PFM_Reply_Experiment_Pending = false;
            }
            if ((MSTP_Port->Next_Station == MSTP_Port->This_Station) &&
                (DLMSTP_Observed_Successor > MSTP_Port->This_Station) &&
                (DLMSTP_Observed_Successor <= MSTP_Port->Nmax_master)) {
                MSTP_Port->Next_Station = DLMSTP_Observed_Successor;
                if (!DLMSTP_MAC25_MIN_LOG_MODE) {
                    DLMSTP_DIAG_LOGI(
                        "token_successor_unknown_using_observed next=%u",
                        (unsigned)MSTP_Port->Next_Station);
                }
            }
            DLMSTP_Token_Cycle_Active = true;
            DLMSTP_Token_Cycle_Passed = false;
            DLMSTP_Token_Cycle_Source = MSTP_Port->SourceAddress;
            DLMSTP_Token_Cycle_Destination = MSTP_Port->DestinationAddress;
            DLMSTP_Token_Cycle_App_Frames_Sent = 0;
            DLMSTP_Token_No_Pass_Logged = false;
            DLMSTP_Token_Pass_Deadline_Ms = dlmstp_diag_timestamp_ms() +
                MSTP_Port->Tusage_timeout + 100U;
#if !USER_MSTP_TOKEN_PASS_ONLY_DEBUG
            MSTP_ACTIVE_LOGW(
                "TOKEN_USE start queue_depth=%u",
                dlmstp_send_pdu_queue_depth());
            if (!DLMSTP_MAC25_MIN_LOG_MODE) {
                DLMSTP_DIAG_LOGI(
                    "token_use start queue_depth=%u",
                    dlmstp_send_pdu_queue_depth());
            }
#endif
        }
        if (user->Valid_Frame_Rx_Callback) {
            user->Valid_Frame_Rx_Callback(
                MSTP_Port->SourceAddress, MSTP_Port->DestinationAddress,
                MSTP_Port->FrameType, MSTP_Port->InputBuffer,
                MSTP_Port->DataLength);
        }
    }
    if (MSTP_Port->ReceivedValidFrameNotForUs) {
        user->Statistics.receive_valid_frame_not_for_us_counter++;
        dlmstp_post_pfm_observe_valid_frame(
            MSTP_Port->SourceAddress,
            MSTP_Port->DestinationAddress,
            MSTP_Port->FrameType);
        dlmstp_post_pass_observe_valid_frame(
            MSTP_Port->SourceAddress,
            MSTP_Port->DestinationAddress,
            MSTP_Port->FrameType);
        if (MSTP_Port->FrameType == FRAME_TYPE_TOKEN) {
            uint8_t candidate = 0xFF;

            if ((MSTP_Port->SourceAddress > MSTP_Port->This_Station) &&
                (MSTP_Port->SourceAddress <= MSTP_Port->Nmax_master)) {
                candidate = MSTP_Port->SourceAddress;
            }
            if ((MSTP_Port->DestinationAddress > MSTP_Port->This_Station) &&
                (MSTP_Port->DestinationAddress <= MSTP_Port->Nmax_master)) {
                if ((candidate == 0xFF) ||
                    (MSTP_Port->DestinationAddress < candidate)) {
                    candidate = MSTP_Port->DestinationAddress;
                }
            }
            if (candidate != 0xFF) {
                DLMSTP_Observed_Successor = candidate;
            }
        }
        dlmstp_post_pfm_log_frame_event(MSTP_Port, "valid_not_for_us", true, true);
        if (user->Valid_Frame_Not_For_Us_Rx_Callback) {
            user->Valid_Frame_Not_For_Us_Rx_Callback(
                MSTP_Port->SourceAddress, MSTP_Port->DestinationAddress,
                MSTP_Port->FrameType, MSTP_Port->InputBuffer,
                MSTP_Port->DataLength);
        }
    }
    if (MSTP_Port->ReceivedInvalidFrame) {
        bool valid_header_crc = (MSTP_Port->HeaderCRC == 0x55);
        bool valid_data_crc = (MSTP_Port->DataLength == 0) ||
            (MSTP_Port->DataCRC == 0xF0B8);

        user->Statistics.receive_invalid_frame_counter++;
        DLMSTP_Token_Diagnostics.invalid_counter++;
        dlmstp_post_pfm_log_frame_event(
            MSTP_Port, "invalid", valid_header_crc, valid_data_crc);
        if (!DLMSTP_MAC25_MIN_LOG_MODE &&
            dlmstp_post_pfm_window_active(dlmstp_diag_timestamp_ms())) {
            DLMSTP_DIAG_LOGW(
                "post_pfm_window invalid_detail header_crc_accum=0x%02X header_crc_actual=0x%02X data_crc_accum=0x%04X data_crc_actual=0x%02X%02X",
                (unsigned)MSTP_Port->HeaderCRC,
                (unsigned)MSTP_Port->HeaderCRCActual,
                (unsigned)MSTP_Port->DataCRC,
                (unsigned)MSTP_Port->DataCRCActualMSB,
                (unsigned)MSTP_Port->DataCRCActualLSB);
        }
        if (MSTP_Port->HeaderCRC != 0x55) {
            user->Statistics.bad_crc_counter++;
            DLMSTP_Token_Diagnostics.bad_crc_counter++;
        } else if (MSTP_Port->DataCRC != 0xF0B8) {
            user->Statistics.bad_crc_counter++;
            DLMSTP_Token_Diagnostics.bad_crc_counter++;
        }
        if (user->Invalid_Frame_Rx_Callback) {
            user->Invalid_Frame_Rx_Callback(
                MSTP_Port->SourceAddress, MSTP_Port->DestinationAddress,
                MSTP_Port->FrameType, MSTP_Port->InputBuffer,
                MSTP_Port->DataLength);
        }
    }
    if (MSTP_Port->receive_state == MSTP_RECEIVE_STATE_IDLE) {
        /* only node state machines while rx is idle */
        if (MSTP_Port->SlaveNodeEnabled) {
            MSTP_Slave_Node_FSM(MSTP_Port);
        } else if (
            (MSTP_Port->This_Station <= DEFAULT_MAX_MASTER) ||
            MSTP_Port->ZeroConfigEnabled || MSTP_Port->CheckAutoBaud) {
            master_state = MSTP_Port->master_state;
            do {
                bool transition_now = MSTP_Master_Node_FSM(MSTP_Port);

                if (master_state != MSTP_Port->master_state) {
                    MSTP_MASTER_STATE new_state = MSTP_Port->master_state;

                    if (new_state == MSTP_MASTER_STATE_NO_TOKEN) {
                        user->Statistics.lost_token_counter++;
                        DLMSTP_Token_Diagnostics.lost_token_counter++;
                        if (!DLMSTP_MAC25_MIN_LOG_MODE &&
                            MSTP_Port->This_Station == DLMSTP_DIAG_TARGET_MAC) {
                            DLMSTP_DIAG_LOGW("lost_token this_station=%u", (unsigned)MSTP_Port->This_Station);
                        }
                    }

                    if (DLMSTP_Token_Cycle_Active && !DLMSTP_Token_Cycle_Passed &&
                        (master_state == MSTP_MASTER_STATE_USE_TOKEN ||
                            master_state == MSTP_MASTER_STATE_DONE_WITH_TOKEN ||
                            master_state == MSTP_MASTER_STATE_WAIT_FOR_REPLY ||
                            master_state == MSTP_MASTER_STATE_PASS_TOKEN) &&
                        (new_state == MSTP_MASTER_STATE_IDLE ||
                            new_state == MSTP_MASTER_STATE_NO_TOKEN)) {
#if !USER_MSTP_TOKEN_PASS_ONLY_DEBUG
                        MSTP_ACTIVE_LOGW(
                            "TOKEN_USE done token_passed=no app_frames_sent=%u",
                            DLMSTP_Token_Cycle_App_Frames_Sent);
#endif
                        if (!DLMSTP_MAC25_MIN_LOG_MODE) {
                            DLMSTP_DIAG_LOGW(
                                "TOKEN USE ENDED WITHOUT PASSING TOKEN src=%u dst=%u next_station=%u poll_station=%u mstp_state=%s queue_depth=%u",
                                (unsigned)DLMSTP_Token_Cycle_Source,
                                (unsigned)DLMSTP_Token_Cycle_Destination,
                                (unsigned)MSTP_Port->Next_Station,
                                (unsigned)MSTP_Port->Poll_Station,
                                dlmstp_master_state_text_short(new_state),
                                dlmstp_send_pdu_queue_depth());
                        }
                        DLMSTP_Token_Cycle_Active = false;
                            DLMSTP_Token_Pass_Deadline_Ms = 0;
                    }

                    master_state = new_state;
                }

                if (!transition_now) {
                    break;
                }
            } while (true);
        }
    }
    /* see if there is a packet available */
    if (user->ReceivePacketPending) {
        user->ReceivePacketPending = false;
        user->Statistics.receive_pdu_counter++;
        pdu_len = MSTP_Port->DataLength;
        if (pdu_len > max_pdu) {
            /* PDU is too large */
            return 0;
        }
        if (!pdu) {
            /* no place to put a PDU */
            return 0;
        }
        /* copy input buffer to PDU */
        for (i = 0; i < pdu_len; i++) {
            pdu[i] = MSTP_Port->InputBuffer[i];
        }
        if (!src) {
            /* no place to put a source address */
            return 0;
        }
        /* copy source address */
        src->len = 0;
        src->net = 0;
        src->mac_len = 1;
        src->mac[0] = MSTP_Port->SourceAddress;
    }

    return pdu_len;
}

/**
 * @brief fill a BACNET_ADDRESS with the MSTP MAC address
 * @param src - a #BACNET_ADDRESS structure
 * @param mstp_address - a BACnet MSTP address
 */
void dlmstp_fill_bacnet_address(BACNET_ADDRESS *src, uint8_t mstp_address)
{
    int i = 0;

    if (mstp_address == MSTP_BROADCAST_ADDRESS) {
        /* mac_len = 0 if broadcast address */
        src->mac_len = 0;
        src->mac[0] = 0;
    } else {
        src->mac_len = 1;
        src->mac[0] = mstp_address;
    }
    /* fill with 0's starting with index 1; index 0 filled above */
    for (i = 1; i < MAX_MAC_LEN; i++) {
        src->mac[i] = 0;
    }
    src->net = 0;
    src->len = 0;
    for (i = 0; i < MAX_MAC_LEN; i++) {
        src->adr[i] = 0;
    }
}

/**
 * @brief Set the MSTP MAC address
 * @param mac_address - MAC address to set
 */
void dlmstp_set_mac_address(uint8_t mac_address)
{
    if (MSTP_Port) {
        MSTP_Port->This_Station = mac_address;
    }

    return;
}

/**
 * @brief Get the MSTP MAC address
 * @return MSTP MAC address
 */
uint8_t dlmstp_mac_address(void)
{
    uint8_t value = 0;

    if (MSTP_Port) {
        value = MSTP_Port->This_Station;
    }

    return value;
}

/**
 * @brief Set the Max_Info_Frames parameter value
 *
 * @note This parameter represents the value of the Max_Info_Frames property
 *  of the node's Device object. The value of Max_Info_Frames specifies the
 *  maximum number of information frames the node may send before it must
 *  pass the token. Max_Info_Frames may have different values on different
 *  nodes. This may be used to allocate more or less of the available link
 *  bandwidth to particular nodes. If Max_Info_Frames is not writable in a
 *  node, its value shall be 1.
 *
 * @param max_info_frames - parameter value to set
 */
void dlmstp_set_max_info_frames(uint8_t max_info_frames)
{
    if (max_info_frames >= 1) {
        if (MSTP_Port) {
            MSTP_Port->Nmax_info_frames = max_info_frames;
        }
    }

    return;
}

/**
 * @brief Get the MSTP max-info-frames value
 * @return the MSTP max-info-frames value
 */
uint8_t dlmstp_max_info_frames(void)
{
    uint8_t value = 0;

    if (MSTP_Port) {
        value = MSTP_Port->Nmax_info_frames;
    }

    return value;
}

/**
 * @brief Set the Max_Master property value for this MSTP datalink
 *
 * @note This parameter represents the value of the Max_Master property of
 *  the node's Device object. The value of Max_Master specifies the highest
 *  allowable address for master nodes. The value of Max_Master shall be
 *  less than or equal to 127. If Max_Master is not writable in a node,
 *  its value shall be 127.
 *
 * @param max_master - value to be set
 */
void dlmstp_set_max_master(uint8_t max_master)
{
    if (max_master <= 127) {
        MSTP_Port->Nmax_master = max_master;
    }

    return;
}

/**
 * @brief Get the largest peer MAC address that we will seek
 * @return largest peer MAC address
 */
uint8_t dlmstp_max_master(void)
{
    uint8_t value = 0;

    if (MSTP_Port) {
        value = MSTP_Port->Nmax_master;
    }

    return value;
}

/**
 * @brief Initialize the data link broadcast address
 * @param my_address - address to be filled with unicast designator
 */
void dlmstp_get_my_address(BACNET_ADDRESS *my_address)
{
    int i = 0; /* counter */

    my_address->mac_len = 1;
    if (MSTP_Port) {
        my_address->mac[0] = MSTP_Port->This_Station;
    }
    my_address->net = 0; /* local only, no routing */
    my_address->len = 0;
    for (i = 0; i < MAX_MAC_LEN; i++) {
        my_address->adr[i] = 0;
    }

    return;
}

/**
 * @brief Initialize the a data link broadcast address
 * @param dest - address to be filled with broadcast designator
 */
void dlmstp_get_broadcast_address(BACNET_ADDRESS *dest)
{ /* destination address */
    int i = 0; /* counter */

    if (dest) {
        dest->mac_len = 1;
        dest->mac[0] = MSTP_BROADCAST_ADDRESS;
        dest->net = BACNET_BROADCAST_NETWORK;
        dest->len = 0; /* always zero when DNET is broadcast */
        for (i = 0; i < MAX_MAC_LEN; i++) {
            dest->adr[i] = 0;
        }
    }

    return;
}

/**
 * @brief Get the MSTP port SoleMaster status
 * @return true if the MSTP port is the SoleMaster
 */
bool dlmstp_sole_master(void)
{
    if (!MSTP_Port) {
        return false;
    }
    return MSTP_Port->SoleMaster;
}

/**
 * @brief Get the MSTP port SlaveNodeEnabled status
 * @return true if the MSTP port has SlaveNodeEnabled
 */
bool dlmstp_slave_mode_enabled(void)
{
    if (!MSTP_Port) {
        return false;
    }
    return MSTP_Port->SlaveNodeEnabled;
}

/**
 * @brief Set the MSTP port SlaveNodeEnabled flag
 * @param flag - true if the MSTP port has SlaveNodeEnabled
 * @return true if the MSTP port SlaveNodeEnabled was set
 * @note This flag is used to enable the Slave Node state machine
 * for the MSTP port.  The Slave Node state machine is used to
 * respond to requests from the Master Node.
 */
bool dlmstp_slave_mode_enabled_set(bool flag)
{
    if (!MSTP_Port) {
        return false;
    }
    MSTP_Port->SlaveNodeEnabled = flag;

    return true;
}

/**
 * @brief Get the MSTP port ZeroConfigEnabled status
 * @return true if the MSTP port has ZeroConfigEnabled
 */
bool dlmstp_zero_config_enabled(void)
{
    if (!MSTP_Port) {
        return false;
    }
    return MSTP_Port->ZeroConfigEnabled;
}

/**
 * @brief Set the MSTP port ZeroConfigEnabled flag
 * @param flag - true if the MSTP port has ZeroConfigEnabled
 * @return true if the MSTP port ZeroConfigEnabled was set
 * @note This flag is used to enable the Zero Configuration state machine
 * for the MSTP port.  The Zero Configuration state machine is used to
 * automatically assign a MAC address to the MSTP port.
 */
bool dlmstp_zero_config_enabled_set(bool flag)
{
    if (!MSTP_Port) {
        return false;
    }
#if !USER_MSTP_ZERO_CONFIG_ENABLED
    (void)flag;
    MSTP_Port->ZeroConfigEnabled = false;
#else
    MSTP_Port->ZeroConfigEnabled = flag;
#endif

    return true;
}

/**
 * @brief Get the MSTP port AutoBaudEnabled status
 * @return true if the MSTP port has AutoBaudEnabled
 */
bool dlmstp_check_auto_baud(void)
{
    if (!MSTP_Port) {
        return false;
    }
    return MSTP_Port->CheckAutoBaud;
}

/**
 * @brief Set the MSTP port AutoBaudEnabled flag
 * @param flag - true if the MSTP port has AutoBaudEnabled
 * @return true if the MSTP port AutoBaudEnabled was set
 * @note This flag is used to enable the Zero Configuration state machine
 * for the MSTP port.  The Zero Configuration state machine is used to
 * automatically assign a MAC address to the MSTP port.
 */
bool dlmstp_check_auto_baud_set(bool flag)
{
    if (!MSTP_Port) {
        return false;
    }
    MSTP_Port->CheckAutoBaud = flag;
    if (flag) {
        MSTP_Port->Auto_Baud_State = MSTP_AUTO_BAUD_STATE_INIT;
    }

    return true;
}

/**
 * @brief Get the MSTP port MAC address that this node prefers to use.
 * @return ZeroConfigStation value, or an out-of-range value if invalid
 * @note valid values are between Nmin_poll_station and Nmax_poll_station
 *  but other values such as 0 or 255 could mean 'unconfigured'
 */
uint8_t dlmstp_zero_config_preferred_station(void)
{
    if (!MSTP_Port) {
        return 0;
    }
    return MSTP_Port->Zero_Config_Preferred_Station;
}

/**
 * @brief Set the MSTP port MAC address that this node prefers to use.
 * @param station - Zero_Config_Preferred_Station value
 * @return true if the MSTP port Zero_Config_Preferred_Station was set
 * @note valid values are between Nmin_poll_station and Nmax_poll_station
 *  but other values such as 0 or 255 could mean 'unconfigured'
 */
bool dlmstp_zero_config_preferred_station_set(uint8_t station)
{
    if (!MSTP_Port) {
        return false;
    }
    MSTP_Port->Zero_Config_Preferred_Station = station;

    return true;
}

/**
 * @brief Determine if the send PDU queue is empty
 * @return true if the send PDU is empty
 */
bool dlmstp_send_pdu_queue_empty(void)
{
    bool status = false;
    struct dlmstp_user_data_t *user;

    if (MSTP_Port) {
        user = MSTP_Port->UserData;
        if (user) {
            status = Ringbuf_Empty(&user->PDU_Queue) && dlmstp_priority_queue_empty();
        }
    }

    return status;
}

/**
 * @brief Determine if the send PDU queue is full
 * @return true if the send PDU is full
 */
bool dlmstp_send_pdu_queue_full(void)
{
    bool status = false;
    struct dlmstp_user_data_t *user;

    if (MSTP_Port) {
        user = MSTP_Port->UserData;
        if (user) {
            status = Ringbuf_Full(&user->PDU_Queue);
        }
    }

    return status;
}

unsigned dlmstp_send_pdu_queue_depth(void)
{
    unsigned depth = 0;
    struct dlmstp_user_data_t *user;

    if (MSTP_Port) {
        user = MSTP_Port->UserData;
        if (user) {
            depth = Ringbuf_Count(&user->PDU_Queue);
            depth += DLMSTP_Priority_Count;
        }
    }

    return depth;
}

bool dlmstp_send_pdu_queue_drop_source(DLMSTP_TX_SOURCE source)
{
    bool dropped = false;
    struct dlmstp_user_data_t *user;

    if (!MSTP_Port) {
        return false;
    }

    if (dlmstp_drop_priority_source((uint8_t)source)) {
        dropped = true;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return false;
    }
    if (dlmstp_drop_queued_source(user, (uint8_t)source)) {
        dropped = true;
    }

    return dropped;
}

void dlmstp_send_pdu_priority_next_set(bool enable)
{
    DLMSTP_Priority_Next_Send = enable;
}

void dlmstp_send_priority_queue_clear(void)
{
    dlmstp_priority_queue_clear();
    DLMSTP_Priority_Next_Send = false;
}

/**
 * @brief Initialize the RS-485 baud rate
 * @param baudrate - RS-485 baud rate in bits per second (bps)
 * @return true if the baud rate was valid
 */
void dlmstp_set_baud_rate(uint32_t baud)
{
    struct dlmstp_user_data_t *user;
    struct dlmstp_rs485_driver *driver;

    if (!MSTP_Port) {
        return;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return;
    }
    driver = user->RS485_Driver;
    if (!driver) {
        return;
    }
    if (driver->baud_rate_set(baud)) {
        /* Tframe_abort=60 bit times, not to exceed 100 milliseconds.*/
        if (MSTP_Port->Tframe_abort <= 7) {
            /* within baud range, so auto-calculate range based on baud */
            MSTP_Port->Tframe_abort = 1 + ((60 * 1000UL) / baud);
        }
        /* Tturnaround=40 bit times */
        MSTP_Port->Tturnaround_timeout = 1 + ((Tturnaround * 1000) / baud);
    }
}

/**
 * @brief Return the RS-485 baud rate
 * @return baud - RS-485 baud rate in bits per second (bps)
 */
uint32_t dlmstp_baud_rate(void)
{
    struct dlmstp_user_data_t *user;
    struct dlmstp_rs485_driver *driver;

    if (!MSTP_Port) {
        return 0;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return 0;
    }
    driver = user->RS485_Driver;
    if (!driver) {
        return 0;
    }

    return driver->baud_rate();
}

/**
 * @brief Set the MS/TP Frame Complete callback
 * @param cb_func - callback function to be called when a frame is received
 */
void dlmstp_set_frame_rx_complete_callback(
    dlmstp_hook_frame_rx_complete_cb cb_func)
{
    struct dlmstp_user_data_t *user;

    if (!MSTP_Port) {
        return;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return;
    }
    user->Valid_Frame_Rx_Callback = cb_func;
}

/**
 * @brief Set the MS/TP Frame Complete callback
 * @param cb_func - callback function to be called when a frame is received
 */
void dlmstp_set_frame_not_for_us_rx_complete_callback(
    dlmstp_hook_frame_rx_complete_cb cb_func)
{
    struct dlmstp_user_data_t *user;

    if (!MSTP_Port) {
        return;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return;
    }
    user->Valid_Frame_Not_For_Us_Rx_Callback = cb_func;
}

/**
 * @brief Set the MS/TP Frame Complete callback
 * @param cb_func - callback function to be called when a frame is received
 */
void dlmstp_set_invalid_frame_rx_complete_callback(
    dlmstp_hook_frame_rx_complete_cb cb_func)
{
    struct dlmstp_user_data_t *user;

    if (!MSTP_Port) {
        return;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return;
    }
    user->Invalid_Frame_Rx_Callback = cb_func;
}

/**
 * @brief Set the MS/TP Preamble callback
 * @param cb_func - callback function to be called when a preamble is received
 */
void dlmstp_set_frame_rx_start_callback(dlmstp_hook_frame_rx_start_cb cb_func)
{
    struct dlmstp_user_data_t *user;

    if (!MSTP_Port) {
        return;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return;
    }
    user->Preamble_Callback = cb_func;
}

/**
 * @brief Reset the MS/TP statistics
 */
void dlmstp_reset_statistics(void)
{
    struct dlmstp_user_data_t *user;

    if (!MSTP_Port) {
        return;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return;
    }
    memset(&user->Statistics, 0, sizeof(struct dlmstp_statistics));
}

/**
 * @brief Copy the MSTP port statistics if they exist
 * @param statistics - MSTP port statistics
 */
void dlmstp_fill_statistics(struct dlmstp_statistics *statistics)
{
    struct dlmstp_user_data_t *user;

    if (!MSTP_Port) {
        return;
    }
    user = MSTP_Port->UserData;
    if (!user) {
        return;
    }
    if (statistics) {
        memmove(
            statistics, &user->Statistics, sizeof(struct dlmstp_statistics));
    }
}

void dlmstp_token_diagnostics_fill_and_reset(
    struct dlmstp_token_diagnostics *diagnostics)
{
    if (DLMSTP_PFM_Rx_To_Tx_Samples > 0U) {
        DLMSTP_Token_Diagnostics.avg_rx_to_tx_us =
            (uint32_t)(DLMSTP_PFM_Rx_To_Tx_Sum_Us / DLMSTP_PFM_Rx_To_Tx_Samples);
    } else {
        DLMSTP_Token_Diagnostics.avg_rx_to_tx_us = 0U;
    }

    if (diagnostics) {
        memmove(
            diagnostics,
            &DLMSTP_Token_Diagnostics,
            sizeof(struct dlmstp_token_diagnostics));
    }
    memset(
        &DLMSTP_Token_Diagnostics,
        0,
        sizeof(struct dlmstp_token_diagnostics));
    DLMSTP_PFM_Rx_To_Tx_Sum_Us = 0;
    DLMSTP_PFM_Rx_To_Tx_Samples = 0;
}

/**
 * @brief Get the MSTP port Max-Info-Frames limit
 * @return Max-Info-Frames limit
 */
uint8_t dlmstp_max_info_frames_limit(void)
{
    return DLMSTP_MAX_INFO_FRAMES;
}

/**
 * @brief Get the MSTP port Max-Master limit
 * @return Max-Master limit
 */
uint8_t dlmstp_max_master_limit(void)
{
    return DLMSTP_MAX_MASTER;
}

/**
 * @brief Return the RS-485 silence time in milliseconds
 * @param arg - pointer to MSTP port structure
 * @return silence time in milliseconds
 */
uint32_t dlmstp_silence_milliseconds(void *arg)
{
    uint32_t milliseconds = 0;
    struct mstp_port_struct_t *port = arg;
    struct dlmstp_user_data_t *user = NULL;
    struct dlmstp_rs485_driver *driver = NULL;

    if (port) {
        user = port->UserData;
    }
    if (user) {
        driver = user->RS485_Driver;
    }
    if (driver) {
        milliseconds = driver->silence_milliseconds();
    }

    return milliseconds;
}

/**
 * @brief Return the valid frame time in milliseconds
 * @param arg - pointer to MSTP port structure
 * @return valid frame time in milliseconds
 */
uint32_t dlmstp_valid_frame_milliseconds(void *arg)
{
    uint32_t milliseconds = 0, now = 0;
    struct mstp_port_struct_t *port = arg;
    struct dlmstp_user_data_t *user = NULL;

    if (port) {
        user = port->UserData;
    }
    if (user) {
        now = mstimer_now();
        milliseconds = now - user->Valid_Frame_Milliseconds;
    }

    return milliseconds;
}

/**
 * @brief Reset the valid frame timer
 * @param arg - pointer to MSTP port structure
 * @return valid frame time in milliseconds
 */
void dlmstp_valid_frame_milliseconds_reset(void *arg)
{
    struct mstp_port_struct_t *port = arg;
    struct dlmstp_user_data_t *user = NULL;

    if (port) {
        user = port->UserData;
    }
    if (user) {
        user->Valid_Frame_Milliseconds = mstimer_now();
    }
}

/**
 * @brief Reset the RS-485 silence time to zero
 * @param arg - pointer to MSTP port structure
 */
void dlmstp_silence_reset(void *arg)
{
    struct mstp_port_struct_t *port = arg;
    struct dlmstp_user_data_t *user = NULL;
    struct dlmstp_rs485_driver *driver = NULL;

    if (port) {
        user = port->UserData;
    }
    if (user) {
        driver = user->RS485_Driver;
    }
    if (driver) {
        driver->silence_reset();
    }
}

/**
 * @brief set the MS/TP datalink interface
 * @param ifname - interface name to set
 */
void dlmstp_set_interface(const char *ifname)
{
    MSTP_Port = (struct mstp_port_struct_t *)ifname;
}

/**
 * @brief get the MS/TP datalink intferface name
 * @return interface name
 */
const char *dlmstp_get_interface(void)
{
    return (const char *)MSTP_Port;
}

/**
 * @brief Initialize this MS/TP datalink
 * @param ifname user data structure
 * @return true if the MSTP datalink is initialized
 */
bool dlmstp_init(char *ifname)
{
    bool status = false;
    struct dlmstp_user_data_t *user;

    if (ifname) {
        MSTP_Port = (struct mstp_port_struct_t *)ifname;
    }
    if (MSTP_Port) {
        MSTP_Port->SilenceTimer = dlmstp_silence_milliseconds;
        MSTP_Port->SilenceTimerReset = dlmstp_silence_reset;
        MSTP_Port->ValidFrameTimer = dlmstp_valid_frame_milliseconds;
        MSTP_Port->ValidFrameTimerReset = dlmstp_valid_frame_milliseconds_reset;
        MSTP_Port->BaudRate = dlmstp_baud_rate;
        MSTP_Port->BaudRateSet = dlmstp_set_baud_rate;
        user = (struct dlmstp_user_data_t *)MSTP_Port->UserData;
        if (user && !user->Initialized) {
            if (!DLMSTP_Priority_Packets) {
                DLMSTP_Priority_Packets = calloc(
                    DLMSTP_PRIORITY_QUEUE_CAPACITY,
                    sizeof(struct dlmstp_packet));
                if (!DLMSTP_Priority_Packets) {
                    return false;
                }
            }
            dlmstp_priority_queue_clear();
            DLMSTP_Priority_Next_Send = false;
            Ringbuf_Initialize(
                &user->PDU_Queue, (volatile uint8_t *)user->PDU_Buffer,
                sizeof(user->PDU_Buffer), sizeof(struct dlmstp_packet),
                DLMSTP_MAX_INFO_FRAMES);
            MSTP_Init(MSTP_Port);
#if !USER_MSTP_ZERO_CONFIG_ENABLED
            MSTP_Port->ZeroConfigEnabled = false;
#endif
            user->Initialized = true;
        }
        status = true;
    }

    return status;
}