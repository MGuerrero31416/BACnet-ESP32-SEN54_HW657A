#include "mstp_rs485.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "bacnet/npdu.h"
#include "bacnet/bacdef.h"
#include "bacnet/bacenum.h"
#include "bacnet/datalink/mstp.h"
#include "bacnet/datalink/mstpdef.h"

#define MSTP_UART_PORT UART_NUM_2
#define MSTP_UART_TX_PIN GPIO_NUM_17
#define MSTP_UART_RX_PIN GPIO_NUM_16
#define MSTP_UART_DE_PIN GPIO_NUM_5
#define MSTP_UART_BAUD_DEFAULT 38400U
#define MSTP_UART_RX_BUF_SIZE 512
#define MSTP_UART_TX_BUF_SIZE 512
#define MSTP_UART_TX_WAIT_MARGIN_MS 50U
#define MSTP_UART_TX_WAIT_MIN_MS 100U
#define MSTP_UART_TX_WAIT_MAX_MS 500U
#define MSTP_UART_BITS_PER_BYTE 10U
#define MSTP_RS485_DE_PRE_TX_GUARD_MS 1
#define MSTP_RS485_DE_POST_TX_GUARD_MS 1

#ifndef USER_MSTP_PFM_REPLY_PRE_DELAY_US
#define USER_MSTP_PFM_REPLY_PRE_DELAY_US 0
#endif

#ifndef USER_RS485_DE_PRE_TX_US
#define USER_RS485_DE_PRE_TX_US 1000
#endif

#ifndef USER_MSTP_TOKEN_PASS_ONLY_DEBUG
#define USER_MSTP_TOKEN_PASS_ONLY_DEBUG 0
#endif

#ifndef USER_MSTP_TOKEN_PASS_PRE_DELAY_US
#define USER_MSTP_TOKEN_PASS_PRE_DELAY_US 0
#endif

#ifndef BACNET_MSTP_TX_HEX_DEBUG
#define BACNET_MSTP_TX_HEX_DEBUG 0
#endif

#define BACNET_MSTP_TX_HEX_DEBUG_TARGET_MAC 40
#define BACNET_MSTP_TX_HEX_DEBUG_TARGET_DEVICE_INSTANCE 55525UL

/* Board/profile gate for external RS-485 transceiver control. */
#ifndef BOARD_RS485_PHY_ENABLED
#define BOARD_RS485_PHY_ENABLED 1
#endif

/* MAX485-style modules use active-high DE. */
#ifndef MSTP_RS485_DE_ACTIVE_HIGH
#define MSTP_RS485_DE_ACTIVE_HIGH 1
#endif

/* Temporary low-noise mode for MAC25 active debugging. */
#ifndef MSTP_MAC25_MIN_LOG_MODE
#if defined(USER_MSTP_ACTIVE_DEBUG_ONLY)
#define MSTP_MAC25_MIN_LOG_MODE USER_MSTP_ACTIVE_DEBUG_ONLY
#else
#define MSTP_MAC25_MIN_LOG_MODE 1
#endif
#endif

_Static_assert(MSTP_UART_DE_PIN == GPIO_NUM_5, "Expected DE pin GPIO5");

static const char *TAG = "mstp_rs485";
static bool mstp_uart_initialized = false;
static volatile bool mstp_tx_in_progress = false;
static uint32_t mstp_baud_rate = MSTP_UART_BAUD_DEFAULT;
static int64_t mstp_last_activity_us = 0;
static volatile uint32_t mstp_rx_bytes = 0;
static volatile uint32_t mstp_preamble_55 = 0;
static volatile uint32_t mstp_preamble_55ff = 0;
static uint8_t mstp_prev_byte = 0;
static volatile uint32_t mstp_tx_frame_count = 0;
static volatile uint64_t mstp_pfm_rx_timestamp_us = 0;
static volatile uint64_t mstp_token_rx_timestamp_us = 0;
static MSTP_RS485_PFM_REPLY_TIMING mstp_last_pfm_reply_timing = { 0 };
static MSTP_RS485_TOKEN_PASS_TIMING mstp_last_token_pass_timing = { 0 };
static volatile int mstp_last_uart_result = 0;

typedef struct {
    bool valid;
    uint8_t frame_type;
    bool is_data_frame;
    uint8_t pdu_type;
    bool has_invoke_id;
    uint8_t invoke_id;
    bool has_service_choice;
    uint8_t service_choice;
    uint16_t mstp_data_len;
    uint16_t apdu_frame_offset;
    int apdu_len;
} MSTP_TX_INFO;

#define MSTP_CONFIRMED_PENDING_MAX 16

static MSTP_RS485_CONFIRMED_REQUEST_META
    mstp_confirmed_pending[MSTP_CONFIRMED_PENDING_MAX] = { 0 };

static MSTP_RS485_CONFIRMED_REQUEST_META *mstp_confirmed_track_slot(uint8_t invoke_id)
{
    MSTP_RS485_CONFIRMED_REQUEST_META *free_slot = NULL;
    size_t i = 0;

    for (i = 0; i < MSTP_CONFIRMED_PENDING_MAX; i++) {
        if (mstp_confirmed_pending[i].valid &&
            mstp_confirmed_pending[i].invoke_id == invoke_id) {
            return &mstp_confirmed_pending[i];
        }
        if (!free_slot && !mstp_confirmed_pending[i].valid) {
            free_slot = &mstp_confirmed_pending[i];
        }
    }

    if (free_slot) {
        return free_slot;
    }

    return &mstp_confirmed_pending[0];
}

static MSTP_RS485_CONFIRMED_REQUEST_META *mstp_confirmed_find_slot(uint8_t invoke_id)
{
    size_t i = 0;

    for (i = 0; i < MSTP_CONFIRMED_PENDING_MAX; i++) {
        if (mstp_confirmed_pending[i].valid &&
            mstp_confirmed_pending[i].invoke_id == invoke_id) {
            return &mstp_confirmed_pending[i];
        }
    }

    return NULL;
}

static bool mstp_decode_tx_info(
    const uint8_t *payload,
    uint16_t payload_len,
    MSTP_TX_INFO *info)
{
    uint16_t data_len = 0;
    int npdu_offset = 0;
    uint8_t frame_type = 0;
    BACNET_ADDRESS dest = { 0 };
    BACNET_ADDRESS src = { 0 };
    BACNET_NPDU_DATA npdu_data = { 0 };
    uint8_t *apdu = NULL;

    if (!info) {
        return false;
    }
    memset(info, 0, sizeof(*info));

    if (!payload || (payload_len < 10)) {
        return false;
    }
    if ((payload[0] != 0x55) || (payload[1] != 0xFF)) {
        return false;
    }

    frame_type = payload[2];
    info->frame_type = frame_type;
    if ((frame_type != FRAME_TYPE_BACNET_DATA_EXPECTING_REPLY) &&
        (frame_type != FRAME_TYPE_BACNET_DATA_NOT_EXPECTING_REPLY) &&
        (frame_type != FRAME_TYPE_BACNET_EXTENDED_DATA_EXPECTING_REPLY) &&
        (frame_type != FRAME_TYPE_BACNET_EXTENDED_DATA_NOT_EXPECTING_REPLY)) {
        return false;
    }
    info->is_data_frame = true;

    data_len = ((uint16_t)payload[5] << 8) | payload[6];
    if ((uint32_t)8 + (uint32_t)data_len > (uint32_t)payload_len) {
        return false;
    }

    npdu_offset = bacnet_npdu_decode(&payload[8], data_len, &dest, &src, &npdu_data);
    if ((npdu_offset <= 0) || (npdu_offset >= (int)data_len)) {
        return false;
    }

    info->apdu_frame_offset = (uint16_t)(8 + npdu_offset);
    apdu = (uint8_t *)&payload[info->apdu_frame_offset];
    info->apdu_len = (int)data_len - npdu_offset;
    if (info->apdu_len < 2) {
        return false;
    }

    info->valid = true;
    info->mstp_data_len = data_len;
    info->pdu_type = (uint8_t)(apdu[0] & 0xF0);
    if (info->apdu_len >= 2) {
        info->invoke_id = apdu[1];
        if ((info->pdu_type == PDU_TYPE_SIMPLE_ACK) ||
            (info->pdu_type == PDU_TYPE_COMPLEX_ACK) ||
            (info->pdu_type == PDU_TYPE_ERROR) ||
            (info->pdu_type == PDU_TYPE_REJECT) ||
            (info->pdu_type == PDU_TYPE_ABORT)) {
            info->has_invoke_id = true;
        }
    }

    if ((info->apdu_len >= 3) &&
        ((info->pdu_type == PDU_TYPE_SIMPLE_ACK) ||
            (info->pdu_type == PDU_TYPE_COMPLEX_ACK) ||
            (info->pdu_type == PDU_TYPE_ERROR))) {
        info->service_choice = apdu[2];
        info->has_service_choice = true;
    }

    return true;
}

void MSTP_RS485_Confirmed_Request_Track(const MSTP_RS485_CONFIRMED_REQUEST_META *meta)
{
    MSTP_RS485_CONFIRMED_REQUEST_META *slot = NULL;

    if (!meta) {
        return;
    }

    slot = mstp_confirmed_track_slot(meta->invoke_id);
    if (!slot) {
        return;
    }

    *slot = *meta;
    slot->valid = true;
}

bool MSTP_RS485_Send_Reply_Postponed(uint8_t requester_mac, uint8_t source_mac)
{
    uint8_t frame[16] = { 0 };
    uint16_t frame_len = 0;

    frame_len = MSTP_Create_Frame(
        frame,
        sizeof(frame),
        FRAME_TYPE_REPLY_POSTPONED,
        requester_mac,
        source_mac,
        NULL,
        0);
    if (frame_len == 0) {
        return false;
    }

    MSTP_RS485_Send(frame, frame_len);
    return true;
}

#if MSTP_DEBUG_ENABLE
/* Rate-limit timestamps for TX control-frame logging (microseconds) */
static int64_t mstp_tx_token_log_us = 0;
static int64_t mstp_tx_pfm_log_us = 0;
static int64_t mstp_tx_rpfm_log_us = 0;
static volatile uint32_t mstp_tx_token_count = 0;
static volatile uint32_t mstp_tx_pfm_count = 0;
static volatile uint32_t mstp_tx_rpfm_count = 0;

static void mstp_log_frame_debug(const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len >= 8 && payload[0] == 0x55 && payload[1] == 0xFF) {
        uint8_t ftype = payload[2];
        uint8_t fdest = payload[3];
        uint8_t fsrc = payload[4];
        uint16_t flen = ((uint16_t)payload[5] << 8) | payload[6];
        int64_t now_us = esp_timer_get_time();

        if (ftype == 0x00) { /* TOKEN */
            mstp_tx_token_count++;
            if ((now_us - mstp_tx_token_log_us) >= 5000000LL) {
                ESP_LOGD(TAG, "TX TOKEN dst=%u src=%u count=%lu", fdest, fsrc,
                    (unsigned long)mstp_tx_token_count);
                mstp_tx_token_log_us = now_us;
            }
        } else if (ftype == 0x01) { /* POLL_FOR_MASTER */
            mstp_tx_pfm_count++;
            if ((now_us - mstp_tx_pfm_log_us) >= 5000000LL) {
                ESP_LOGD(TAG, "TX POLL_FOR_MASTER dst=%u src=%u count=%lu", fdest,
                    fsrc, (unsigned long)mstp_tx_pfm_count);
                mstp_tx_pfm_log_us = now_us;
            }
        } else if (ftype == 0x02) { /* REPLY_TO_POLL_FOR_MASTER */
            mstp_tx_rpfm_count++;
            if ((now_us - mstp_tx_rpfm_log_us) >= 5000000LL) {
                ESP_LOGD(TAG, "TX REPLY_TO_PFM dst=%u src=%u count=%lu", fdest,
                    fsrc, (unsigned long)mstp_tx_rpfm_count);
                mstp_tx_rpfm_log_us = now_us;
            }
        } else {
            ESP_LOGD(TAG, "TX frm#%lu type=0x%02X dst=%u src=%u dlen=%u plen=%u",
                (unsigned long)mstp_tx_frame_count, ftype, fdest, fsrc, flen,
                payload_len);
            uint16_t dump_len = payload_len < 16u ? payload_len : 16u;
            char hex_buf[49]; /* 16*3 + 1 */
            for (uint16_t i = 0; i < dump_len; i++) {
                sprintf(&hex_buf[i * 3], "%02X ", payload[i]);
            }
            hex_buf[dump_len * 3 > 0 ? dump_len * 3 - 1 : 0] = '\0';
            ESP_LOGD(TAG, "TX hex[%u]: %s", dump_len, hex_buf);
        }
    } else {
        ESP_LOGD(TAG, "TX frm#%lu plen=%u (no preamble)",
            (unsigned long)mstp_tx_frame_count, payload_len);
        uint16_t dump_len = payload_len < 16u ? payload_len : 16u;
        char hex_buf[49];
        for (uint16_t i = 0; i < dump_len; i++) {
            sprintf(&hex_buf[i * 3], "%02X ", payload[i]);
        }
        hex_buf[dump_len * 3 > 0 ? dump_len * 3 - 1 : 0] = '\0';
        ESP_LOGD(TAG, "TX hex[%u]: %s", dump_len, hex_buf);
    }
}
#endif

static void mstp_rs485_set_tx_mode(bool enabled)
{
    int level = 0;

    if (MSTP_RS485_DE_ACTIVE_HIGH) {
        level = enabled ? 1 : 0;
    } else {
        level = enabled ? 0 : 1;
    }
    gpio_set_level(MSTP_UART_DE_PIN, level);
}

static bool mstp_is_confirmed_response_pdu(uint8_t pdu_type)
{
    return (pdu_type == PDU_TYPE_SIMPLE_ACK) ||
        (pdu_type == PDU_TYPE_COMPLEX_ACK) ||
        (pdu_type == PDU_TYPE_ERROR) ||
        (pdu_type == PDU_TYPE_REJECT) ||
        (pdu_type == PDU_TYPE_ABORT);
}

static TickType_t mstp_uart_tx_wait_ticks_for_frame(uint16_t total_len)
{
    uint32_t baud_rate = mstp_baud_rate;
    uint32_t tx_time_ms = 0;
    uint32_t timeout_ms = 0;
    TickType_t ticks = 0;

    if (baud_rate == 0U) {
        baud_rate = MSTP_UART_BAUD_DEFAULT;
    }

    tx_time_ms = ((uint32_t)total_len * MSTP_UART_BITS_PER_BYTE * 1000U) /
        baud_rate;
    timeout_ms = tx_time_ms + MSTP_UART_TX_WAIT_MARGIN_MS;
    if (timeout_ms < MSTP_UART_TX_WAIT_MIN_MS) {
        timeout_ms = MSTP_UART_TX_WAIT_MIN_MS;
    } else if (timeout_ms > MSTP_UART_TX_WAIT_MAX_MS) {
        timeout_ms = MSTP_UART_TX_WAIT_MAX_MS;
    }

    ticks = pdMS_TO_TICKS(timeout_ms);
    if (ticks == 0) {
        ticks = 1;
    }

    return ticks;
}

void MSTP_RS485_PFM_RX_Timestamp_Set(uint64_t rx_timestamp_us)
{
    mstp_pfm_rx_timestamp_us = rx_timestamp_us;
}

void MSTP_RS485_Token_RX_Timestamp_Set(uint64_t rx_timestamp_us)
{
    mstp_token_rx_timestamp_us = rx_timestamp_us;
}

bool MSTP_RS485_PFM_Reply_Timing_Get_Reset(MSTP_RS485_PFM_REPLY_TIMING *timing)
{
    if (!timing || !mstp_last_pfm_reply_timing.valid) {
        return false;
    }

    *timing = mstp_last_pfm_reply_timing;
    memset(&mstp_last_pfm_reply_timing, 0, sizeof(mstp_last_pfm_reply_timing));

    return true;
}

int MSTP_RS485_Last_UART_Result_Get(void)
{
    return mstp_last_uart_result;
}

bool MSTP_RS485_Token_Pass_Timing_Get_Reset(MSTP_RS485_TOKEN_PASS_TIMING *timing)
{
    if (!timing || !mstp_last_token_pass_timing.valid) {
        return false;
    }

    *timing = mstp_last_token_pass_timing;
    memset(&mstp_last_token_pass_timing, 0, sizeof(mstp_last_token_pass_timing));

    return true;
}

#if BACNET_MSTP_TX_HEX_DEBUG
static void mstp_log_hex_frame_full(const uint8_t *payload, uint16_t payload_len)
{
    uint16_t offset = 0;

    while (offset < payload_len) {
        uint16_t chunk_len = (uint16_t)(payload_len - offset);
        char hex_buf[(16 * 3) + 1] = { 0 };
        uint16_t i = 0;

        if (chunk_len > 16u) {
            chunk_len = 16u;
        }

        for (i = 0; i < chunk_len; i++) {
            (void)snprintf(
                &hex_buf[i * 3], sizeof(hex_buf) - (i * 3), "%02X ",
                payload[offset + i]);
        }
        if (chunk_len > 0) {
            hex_buf[(chunk_len * 3) - 1] = '\0';
        }

        ESP_LOGI(TAG, "mstp tx6 hex[%u..%u]=%s", (unsigned)offset,
            (unsigned)(offset + chunk_len - 1), hex_buf);
        offset = (uint16_t)(offset + chunk_len);
    }
}
#endif

void MSTP_RS485_Init(void)
{
    if (mstp_uart_initialized) {
        return;
    }

    uart_config_t config = {
        .baud_rate = (int)mstp_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MSTP_UART_DE_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DE pin: %d", err);
    }
    mstp_rs485_set_tx_mode(false);

    err = uart_param_config(MSTP_UART_PORT, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %d", err);
    }

    err = uart_set_pin(
        MSTP_UART_PORT, MSTP_UART_TX_PIN, MSTP_UART_RX_PIN,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %d", err);
    }

    err = uart_driver_install(
        MSTP_UART_PORT, MSTP_UART_RX_BUF_SIZE, MSTP_UART_TX_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %d", err);
    }

    mstp_last_activity_us = esp_timer_get_time();
    mstp_uart_initialized = true;

    if (!MSTP_MAC25_MIN_LOG_MODE) {
        ESP_LOGI(
            TAG,
            "MS/TP UART initialized on TX=%d RX=%d DE=%d @%lu baud rs485_phy_enabled=%d de_active_high=%d",
            (int)MSTP_UART_TX_PIN,
            (int)MSTP_UART_RX_PIN,
            (int)MSTP_UART_DE_PIN,
            (unsigned long)mstp_baud_rate,
            (int)BOARD_RS485_PHY_ENABLED,
            (int)MSTP_RS485_DE_ACTIVE_HIGH);
    }
}

void MSTP_RS485_Send(const uint8_t *payload, uint16_t payload_len)
{
    MSTP_TX_INFO tx_info = { 0 };
    MSTP_RS485_CONFIRMED_REQUEST_META *request_meta = NULL;
    int64_t now_us = 0;
    int64_t request_to_postponed_us = -1;
    int64_t request_to_final_us = -1;
    int written = 0;
    esp_err_t tx_done = ESP_FAIL;
    int de_level_after_enable = -1;
    int de_level_before_disable = -1;
    int de_level_after_disable = -1;
    TickType_t tx_wait_ticks = 0;
    bool has_mstp_header = false;
    uint8_t frame_type = 0xFF;
    bool is_control_frame = false;
    bool is_reply_to_pfm = false;
    bool is_token_frame = false;
    bool token_pass_only_timing = false;
    uint32_t de_post_guard_us = 0;
    bool de_disabled_after_tx_done = false;
    uint32_t de_pre_delay_us = 0;
    uint32_t token_pass_pre_delay_us = 0;
    uint32_t applied_pre_delay_us = 0;
    bool use_reply_like_de_pre_path = false;
    int64_t de_enable_time_us = 0;
    int64_t uart_write_time_us = 0;
    int64_t tx_done_time_us = 0;
    int64_t token_pass_start_us = 0;
    uint64_t pfm_rx_time_us = 0;
    uint64_t token_rx_time_us = 0;

    if (!payload || payload_len == 0) {
        return;
    }
    if (!mstp_uart_initialized) {
        MSTP_RS485_Init();
    }

    if (!BOARD_RS485_PHY_ENABLED) {
        ESP_LOGE(TAG, "BOARD_RS485_PHY_ENABLED=0; RS485 TX path disabled for this profile");
        return;
    }

    mstp_tx_frame_count++;
#if MSTP_DEBUG_ENABLE
    if ((payload_len >= 8U) && (payload[0] == 0x55U) && (payload[1] == 0xFFU) &&
        (payload[2] == FRAME_TYPE_REPLY_TO_POLL_FOR_MASTER)) {
        /* Fast Reply-To-PFM path: no pre-TX logs. */
    } else {
        mstp_log_frame_debug(payload, payload_len);
    }
#endif

    has_mstp_header = (payload_len >= 8) && (payload[0] == 0x55) && (payload[1] == 0xFF);
    if (has_mstp_header) {
        frame_type = payload[2];
        is_token_frame = (frame_type == FRAME_TYPE_TOKEN);
        is_control_frame = (frame_type == FRAME_TYPE_TOKEN) ||
            (frame_type == FRAME_TYPE_POLL_FOR_MASTER) ||
            (frame_type == FRAME_TYPE_REPLY_TO_POLL_FOR_MASTER);
        is_reply_to_pfm = (frame_type == FRAME_TYPE_REPLY_TO_POLL_FOR_MASTER);
        token_pass_only_timing = USER_MSTP_TOKEN_PASS_ONLY_DEBUG && is_token_frame;
        use_reply_like_de_pre_path = is_reply_to_pfm || token_pass_only_timing;
    }

    mstp_decode_tx_info(payload, payload_len, &tx_info);
    if (tx_info.valid && tx_info.has_invoke_id) {
        request_meta = mstp_confirmed_find_slot(tx_info.invoke_id);
    }

    mstp_tx_in_progress = true;

    if (token_pass_only_timing) {
        token_rx_time_us = mstp_token_rx_timestamp_us;
        token_pass_start_us = esp_timer_get_time();
        token_pass_pre_delay_us = (uint32_t)USER_MSTP_TOKEN_PASS_PRE_DELAY_US;
        applied_pre_delay_us = token_pass_pre_delay_us;
        if (token_pass_pre_delay_us > 0U) {
            esp_rom_delay_us(token_pass_pre_delay_us);
        }
    } else if (is_reply_to_pfm && (USER_MSTP_PFM_REPLY_PRE_DELAY_US > 0)) {
        applied_pre_delay_us = (uint32_t)USER_MSTP_PFM_REPLY_PRE_DELAY_US;
        esp_rom_delay_us((uint32_t)USER_MSTP_PFM_REPLY_PRE_DELAY_US);
    }

    mstp_rs485_set_tx_mode(true);
    if (use_reply_like_de_pre_path) {
        de_enable_time_us = esp_timer_get_time();
        if (is_reply_to_pfm) {
            pfm_rx_time_us = mstp_pfm_rx_timestamp_us;
        }
    }
    de_level_after_enable = gpio_get_level(MSTP_UART_DE_PIN);
    if (use_reply_like_de_pre_path) {
        de_pre_delay_us = (uint32_t)USER_RS485_DE_PRE_TX_US;
        if (de_pre_delay_us > 0U) {
            esp_rom_delay_us(de_pre_delay_us);
        }
    } else {
        vTaskDelay(pdMS_TO_TICKS(MSTP_RS485_DE_PRE_TX_GUARD_MS));
    }

    if (use_reply_like_de_pre_path) {
        uart_write_time_us = esp_timer_get_time();
    }
    if (is_reply_to_pfm && (payload_len >= 8U)) {
        written = uart_write_bytes(MSTP_UART_PORT, payload, 8);
    } else {
        written = uart_write_bytes(MSTP_UART_PORT, payload, payload_len);
    }
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
        mstp_last_uart_result = written;
    } else {
        mstp_last_uart_result = (int)tx_done;
    }

    if (is_reply_to_pfm && (payload_len >= 8U)) {
        tx_wait_ticks = mstp_uart_tx_wait_ticks_for_frame(8U);
    } else {
        tx_wait_ticks = mstp_uart_tx_wait_ticks_for_frame(payload_len);
    }

    tx_done = uart_wait_tx_done(MSTP_UART_PORT, tx_wait_ticks);
    if (written >= 0) {
        mstp_last_uart_result = (int)tx_done;
    }
    if (use_reply_like_de_pre_path) {
        tx_done_time_us = esp_timer_get_time();
    }

    if (is_reply_to_pfm) {
        de_post_guard_us = 0;
    } else if (is_control_frame && (tx_done == ESP_OK)) {
        de_post_guard_us = 0;
    } else {
        de_post_guard_us = (uint32_t)MSTP_RS485_DE_POST_TX_GUARD_MS * 1000U;
        vTaskDelay(pdMS_TO_TICKS(MSTP_RS485_DE_POST_TX_GUARD_MS));
    }
    de_level_before_disable = gpio_get_level(MSTP_UART_DE_PIN);
    de_disabled_after_tx_done = (tx_done == ESP_OK);
    mstp_rs485_set_tx_mode(false);
    de_level_after_disable = gpio_get_level(MSTP_UART_DE_PIN);

    if (is_reply_to_pfm) {
        uint64_t delta_us = 0;

        memset(&mstp_last_pfm_reply_timing, 0, sizeof(mstp_last_pfm_reply_timing));
        mstp_last_pfm_reply_timing.valid = true;
        mstp_last_pfm_reply_timing.src = payload[4];
        mstp_last_pfm_reply_timing.dst = payload[3];
        mstp_last_pfm_reply_timing.pre_delay_us = (uint32_t)USER_MSTP_PFM_REPLY_PRE_DELAY_US;
        mstp_last_pfm_reply_timing.de_pre_us = de_pre_delay_us;
        mstp_last_pfm_reply_timing.tx_done_ok = (tx_done == ESP_OK);

        if ((de_enable_time_us > 0) && (uart_write_time_us >= de_enable_time_us)) {
            delta_us = (uint64_t)(uart_write_time_us - de_enable_time_us);
            if (delta_us > 0xFFFFFFFFULL) {
                delta_us = 0xFFFFFFFFULL;
            }
            mstp_last_pfm_reply_timing.de_enable_to_uart_write_us = (uint32_t)delta_us;
        }
        if ((uart_write_time_us > 0) && (tx_done_time_us >= uart_write_time_us)) {
            delta_us = (uint64_t)(tx_done_time_us - uart_write_time_us);
            if (delta_us > 0xFFFFFFFFULL) {
                delta_us = 0xFFFFFFFFULL;
            }
            mstp_last_pfm_reply_timing.uart_write_to_tx_done_us = (uint32_t)delta_us;
        }
        if (pfm_rx_time_us > 0) {
            if ((de_enable_time_us > 0) && ((uint64_t)de_enable_time_us >= pfm_rx_time_us)) {
                delta_us = ((uint64_t)de_enable_time_us - pfm_rx_time_us);
                if (delta_us > 0xFFFFFFFFULL) {
                    delta_us = 0xFFFFFFFFULL;
                }
                mstp_last_pfm_reply_timing.pfm_rx_to_de_enable_us = (uint32_t)delta_us;
            }
            if ((uart_write_time_us > 0) && ((uint64_t)uart_write_time_us >= pfm_rx_time_us)) {
                delta_us = ((uint64_t)uart_write_time_us - pfm_rx_time_us);
                if (delta_us > 0xFFFFFFFFULL) {
                    delta_us = 0xFFFFFFFFULL;
                }
                mstp_last_pfm_reply_timing.pfm_rx_to_uart_write_us = (uint32_t)delta_us;
            }
            if ((tx_done_time_us > 0) && ((uint64_t)tx_done_time_us >= pfm_rx_time_us)) {
                delta_us = ((uint64_t)tx_done_time_us - pfm_rx_time_us);
                if (delta_us > 0xFFFFFFFFULL) {
                    delta_us = 0xFFFFFFFFULL;
                }
                mstp_last_pfm_reply_timing.total_pfm_rx_to_tx_done_us = (uint32_t)delta_us;
            }
        }
    }

    if (token_pass_only_timing && has_mstp_header && (payload_len >= 8U)) {
        uint64_t delta_us = 0;
        uint64_t total_us = 0;

        memset(&mstp_last_token_pass_timing, 0, sizeof(mstp_last_token_pass_timing));
        mstp_last_token_pass_timing.valid = true;
        mstp_last_token_pass_timing.src = payload[4];
        mstp_last_token_pass_timing.dst = payload[3];
        mstp_last_token_pass_timing.pre_delay_us = applied_pre_delay_us;
        mstp_last_token_pass_timing.de_pre_us = de_pre_delay_us;
        mstp_last_token_pass_timing.tx_done_ok = (tx_done == ESP_OK);
        mstp_last_token_pass_timing.uart_ret = written;

        if ((token_rx_time_us > 0U) &&
            (de_enable_time_us > 0) &&
            ((uint64_t)de_enable_time_us >= token_rx_time_us)) {
            delta_us = (uint64_t)de_enable_time_us - token_rx_time_us;
            if (delta_us > 0xFFFFFFFFULL) {
                delta_us = 0xFFFFFFFFULL;
            }
            mstp_last_token_pass_timing.rx_to_de_us = (uint32_t)delta_us;
        }

        if ((de_enable_time_us > 0) && (uart_write_time_us >= de_enable_time_us)) {
            delta_us = (uint64_t)(uart_write_time_us - de_enable_time_us);
            if (delta_us > 0xFFFFFFFFULL) {
                delta_us = 0xFFFFFFFFULL;
            }
            mstp_last_token_pass_timing.de_to_write_us = (uint32_t)delta_us;
        }

        if ((token_rx_time_us > 0U) &&
            (uart_write_time_us > 0) &&
            ((uint64_t)uart_write_time_us >= token_rx_time_us)) {
            delta_us = (uint64_t)uart_write_time_us - token_rx_time_us;
            if (delta_us > 0xFFFFFFFFULL) {
                delta_us = 0xFFFFFFFFULL;
            }
            mstp_last_token_pass_timing.rx_to_write_us = (uint32_t)delta_us;
        }

        if ((uart_write_time_us > 0) && (tx_done_time_us >= uart_write_time_us)) {
            delta_us = (uint64_t)(tx_done_time_us - uart_write_time_us);
            if (delta_us > 0xFFFFFFFFULL) {
                delta_us = 0xFFFFFFFFULL;
            }
            mstp_last_token_pass_timing.write_to_done_us = (uint32_t)delta_us;
        }

        if ((token_rx_time_us > 0U) &&
            (tx_done_time_us > 0) &&
            ((uint64_t)tx_done_time_us >= token_rx_time_us)) {
            total_us = (uint64_t)tx_done_time_us - token_rx_time_us;
        } else if ((token_pass_start_us > 0) && (tx_done_time_us >= token_pass_start_us)) {
            total_us = (uint64_t)(tx_done_time_us - token_pass_start_us);
        }

        if (total_us > 0U) {
            if (total_us > 0xFFFFFFFFULL) {
                total_us = 0xFFFFFFFFULL;
            }
            mstp_last_token_pass_timing.total_us = (uint32_t)total_us;
        }
    }

    if (has_mstp_header) {
        if (!MSTP_MAC25_MIN_LOG_MODE) {
            ESP_LOGI(
                TAG,
                "mstp_tx_turnaround frame_type=%u is_control_frame=%s de_pre_us=%lu de_post_guard_us=%lu de_disabled_after_tx_done=%s dst=%u src=%u uart_wait_tx_done=%s",
                (unsigned)frame_type,
                is_control_frame ? "yes" : "no",
                (unsigned long)de_pre_delay_us,
                (unsigned long)de_post_guard_us,
                de_disabled_after_tx_done ? "yes" : "no",
                (unsigned)payload[3],
                (unsigned)payload[4],
                esp_err_to_name(tx_done));
        }

        (void)is_reply_to_pfm;
    }

    if (tx_info.valid &&
        tx_info.is_data_frame &&
        (tx_info.frame_type == FRAME_TYPE_BACNET_DATA_NOT_EXPECTING_REPLY)) {
        if (!MSTP_MAC25_MIN_LOG_MODE) {
            ESP_LOGI(
                TAG,
                "mstp TX frame=%u dst=%u src=%u data_len=%u total_len=%u uart_write_ret=%d uart_wait_tx_done=%s de_en=%d de_pre_dis=%d de_dis=%d",
                (unsigned)tx_info.frame_type,
                (unsigned)payload[3],
                (unsigned)payload[4],
                (unsigned)tx_info.mstp_data_len,
                (unsigned)payload_len,
                written,
                esp_err_to_name(tx_done),
                de_level_after_enable,
                de_level_before_disable,
                de_level_after_disable);
        }
    }

#if BACNET_MSTP_TX_HEX_DEBUG
    if (tx_info.valid &&
        tx_info.is_data_frame &&
        (tx_info.frame_type == FRAME_TYPE_BACNET_DATA_NOT_EXPECTING_REPLY)) {
        uint8_t dst_mac = payload[3];
        bool target_match =
            (dst_mac == BACNET_MSTP_TX_HEX_DEBUG_TARGET_MAC) ||
            (request_meta && request_meta->valid &&
                (request_meta->object_instance ==
                    BACNET_MSTP_TX_HEX_DEBUG_TARGET_DEVICE_INSTANCE));
        int invoke_id_log = -1;
        int service_log = -1;
        int obj_type_log = -1;
        long obj_inst_log = -1;
        long prop_log = -1;

        if (target_match) {
            if (tx_info.has_invoke_id) {
                invoke_id_log = (int)tx_info.invoke_id;
            }
            if (tx_info.has_service_choice) {
                service_log = (int)tx_info.service_choice;
            }
            if (request_meta && request_meta->valid) {
                invoke_id_log = (int)request_meta->invoke_id;
                service_log = (int)request_meta->service_choice;
                obj_type_log = (int)request_meta->object_type;
                obj_inst_log = (long)request_meta->object_instance;
                prop_log = (long)request_meta->object_property;
            }

            ESP_LOGI(
                TAG,
                "mstp tx6 dbg dst=%u src=%u data_len=%u total_len=%u invoke=%d service=%d obj=%d:%ld prop=%ld apdu_len=%d",
                (unsigned)payload[3],
                (unsigned)payload[4],
                (unsigned)tx_info.mstp_data_len,
                (unsigned)payload_len,
                invoke_id_log,
                service_log,
                obj_type_log,
                obj_inst_log,
                prop_log,
                tx_info.apdu_len);
            mstp_log_hex_frame_full(payload, payload_len);
        }
    }
#endif

    if (tx_info.valid &&
        tx_info.is_data_frame &&
        tx_info.has_invoke_id &&
        mstp_is_confirmed_response_pdu(tx_info.pdu_type)) {
        if (request_meta) {
            now_us = esp_timer_get_time();
            request_to_final_us = now_us - request_meta->request_rx_us;
            if (request_meta->reply_postponed_sent &&
                (request_meta->reply_postponed_tx_us >= request_meta->request_rx_us)) {
                request_to_postponed_us =
                    request_meta->reply_postponed_tx_us - request_meta->request_rx_us;
            }

            if (!MSTP_MAC25_MIN_LOG_MODE) {
                ESP_LOGI(
                    TAG,
                    "final response sent invoke=%u requester_mac=%u service=%u object=%u:%lu property=%lu array=%lu reply_postponed=%s req_to_postponed_ms=%ld req_to_final_ms=%ld pdu=0x%02X tx_ret=%d tx_done=%d",
                    (unsigned)request_meta->invoke_id,
                    (unsigned)request_meta->requester_mac,
                    (unsigned)request_meta->service_choice,
                    (unsigned)request_meta->object_type,
                    (unsigned long)request_meta->object_instance,
                    (unsigned long)request_meta->object_property,
                    (unsigned long)request_meta->array_index,
                    request_meta->reply_postponed_sent ? "yes" : "no",
                    (long)(request_to_postponed_us >= 0 ? request_to_postponed_us / 1000 : -1),
                    (long)(request_to_final_us >= 0 ? request_to_final_us / 1000 : -1),
                    (unsigned)tx_info.pdu_type,
                    written,
                    (int)tx_done);
            }

            memset(request_meta, 0, sizeof(*request_meta));
        }
    }

    mstp_tx_in_progress = false;
    mstp_last_activity_us = esp_timer_get_time();
}

bool MSTP_RS485_Read(uint8_t *buf)
{
    if (!buf) {
        return false;
    }
    if (!mstp_uart_initialized) {
        MSTP_RS485_Init();
    }

    int len = uart_read_bytes(MSTP_UART_PORT, buf, 1, 0);
    if (len > 0) {
        mstp_last_activity_us = esp_timer_get_time();
        mstp_rx_bytes += (uint32_t)len;
        if (*buf == 0x55) {
            mstp_preamble_55++;
        }
        if (mstp_prev_byte == 0x55 && *buf == 0xFF) {
            mstp_preamble_55ff++;
        }
        mstp_prev_byte = *buf;
        return true;
    }

    return false;
}

bool MSTP_RS485_Transmitting(void)
{
    return mstp_tx_in_progress;
}

uint32_t MSTP_RS485_Baud_Rate(void)
{
    return mstp_baud_rate;
}

bool MSTP_RS485_Baud_Rate_Set(uint32_t baud)
{
    if (baud == 0) {
        return false;
    }

    mstp_baud_rate = baud;
    if (mstp_uart_initialized) {
        esp_err_t err = uart_set_baudrate(MSTP_UART_PORT, (int)baud);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UART set baud failed: %d", err);
            return false;
        }
    }

    return true;
}

uint32_t MSTP_RS485_Silence_Milliseconds(void)
{
    int64_t now_us = esp_timer_get_time();
    int64_t delta_us = now_us - mstp_last_activity_us;

    if (delta_us < 0) {
        return 0;
    }

    return (uint32_t)(delta_us / 1000);
}

void MSTP_RS485_Silence_Reset(void)
{
    mstp_last_activity_us = esp_timer_get_time();
}

uint32_t MSTP_RS485_Rx_Bytes_Get_Reset(void)
{
    uint32_t count = mstp_rx_bytes;
    mstp_rx_bytes = 0;
    return count;
}

void MSTP_RS485_Preamble_Counts_Get_Reset(uint32_t *preamble55, uint32_t *preamble55ff)
{
    if (preamble55) {
        *preamble55 = mstp_preamble_55;
    }
    if (preamble55ff) {
        *preamble55ff = mstp_preamble_55ff;
    }
    mstp_preamble_55 = 0;
    mstp_preamble_55ff = 0;
}

uint32_t MSTP_RS485_Tx_Frame_Count_Get_Reset(void)
{
    uint32_t count = mstp_tx_frame_count;
    mstp_tx_frame_count = 0;
    return count;
}
