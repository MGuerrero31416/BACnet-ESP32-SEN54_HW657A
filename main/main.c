/* Minimal example: connect to Wi‑Fi and initialize BACnet. */
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "wifi_helper.h"
#include "display.h"
#include "analog_value.h"
#include "binary_value.h"
#include "analog_input.h"
#include "binary_input.h"
#include "binary_output.h"
#include "sen54.h"
#include "mstp_rs485.h"
#include "../components/bacnet-stack/src/bacnet/datalink/mstp_debug_tuning.h"
#include "User_Settings.h"

/* bacnet-stack headers */
#include "bacnet/basic/object/device.h"
#include "bacnet/basic/object/av.h"
#include "bacnet/basic/object/bv.h"
#include "bacnet/basic/object/ai.h"
#include "bacnet/basic/object/bi.h"
#include "bacnet/basic/object/bo.h"
#include "bacnet/basic/service/s_iam.h"
#include "bacnet/basic/tsm/tsm.h"
#include "bacnet/bacaddr.h"
#include "bacnet/basic/bbmd/h_bbmd.h"
#include "bacnet/datalink/datalink.h"
#include "bacnet/datalink/bip.h"
#include "bacnet/datalink/dlmstp.h"
#include "bacnet/datalink/mstp.h"
/* service handlers from bacnet-stack library */
#include "bacnet/basic/services.h"
#include "bacnet/basic/service/h_rp.h"
#include "bacnet/basic/service/h_rpm.h"
#include "bacnet/basic/service/h_wp.h"
#include "bacnet/basic/service/h_whois.h"
#include "bacnet/basic/service/h_iam.h"
#include "bacnet/basic/service/h_cov.h"
#include "bacnet/basic/service/s_whois.h"
#include "bacnet/whois.h"
#include "bacnet/iam.h"
#include "bacnet/npdu.h"
#include "bacnet/rp.h"
#include "bacnet/rpm.h"
#include "bacnet/dcc.h"
#include "bacnet/config.h"
#include "bacnet/basic/npdu/h_npdu.h"
#include "bacnet/bacenum.h"

static const char *TAG = "bacnet";
#if USER_MSTP_ACTIVE_DEBUG_ONLY
static const char *MSTP_ACTIVE_TAG = "mstp_active";
#endif

/* Temporary discovery tracing gate.
 * Keep off in normal operation so serial logs stay focused.
 */
#ifndef BACNET_DISCOVERY_DEBUG
#define BACNET_DISCOVERY_DEBUG 0
#endif

#if USER_MSTP_ACTIVE_DEBUG_ONLY
#define MSTP_TOKEN_SUMMARY_LOGI(...) ESP_LOGW(__VA_ARGS__)
#else
#define MSTP_TOKEN_SUMMARY_LOGI(...) ESP_LOGI(__VA_ARGS__)
#endif

#if USER_MSTP_ACTIVE_DEBUG_ONLY
#define BACNET_DISCOVERY_LOGI(...) do { } while (0)
#define BACNET_DISCOVERY_LOGW(...) do { } while (0)
#define BACNET_SEN54_LOGI(...) do { } while (0)
#define BACNET_SEN54_LOGW(...) do { } while (0)
#else
#define BACNET_DISCOVERY_LOGI(...) do { } while (0)
#define BACNET_DISCOVERY_LOGW(...) do { } while (0)
#define BACNET_SEN54_LOGI(...) ESP_LOGI(__VA_ARGS__)
#define BACNET_SEN54_LOGW(...) ESP_LOGW(__VA_ARGS__)
#endif

#if USER_MSTP_ACTIVE_DEBUG_ONLY
#define BACNET_MAC25_MIN_LOG_MODE 1
#else
#define BACNET_MAC25_MIN_LOG_MODE 0
#endif

#define MSTP_TRANSPORT_BUFFER_SIZE 1024
#define MSTP_APDU_BUFFER_SIZE 1024
#define BACNET_WHOIS_TRACE_DEVICE_INSTANCE 55525UL
#define BACNET_BACROUTER_DISCOVERY_BOOT_WINDOW_US (90LL * 1000000LL)
#define BACNET_BACROUTER_DISCOVERY_RETRY_US (5LL * 1000000LL)
#define BACNET_BACROUTER_COMPAT_THROTTLE_FAST_US (2LL * 1000000LL)
#define BACNET_BACROUTER_COMPAT_THROTTLE_SLOW_US (10LL * 1000000LL)
#define BACNET_POST_DISCOVERY_IAM_MAX_QUEUE_DEPTH 1U
#define BACNET_PERIODIC_I_AM_ENABLE 0
#define BACNET_SUPPRESS_COV_UNSOLICITED_FOR_TEST 1
#define BACNET_IAM_TX_QUEUED_ONLY (-2)

int override_nvs_on_flash = 0;  /* Exported for AV/BV modules */


static void bacnet_register_with_bbmd(void);
static void bacnet_receive_task(void *pvParameters);
static void bacnet_mstp_receive_task(void *pvParameters);
static void bacnet_cov_task(void *pvParameters);
static void sen54_task(void *pvParameters);
static void bacnet_datalink_lock(char *name);
static void bacnet_datalink_unlock(void);
static void bacnet_on_bacrouter_discovery_succeeded(void);
static void handler_who_is_debug(
    uint8_t *service_request,
    uint16_t service_len,
    BACNET_ADDRESS *src);
static TaskHandle_t bacnet_cov_task_handle = NULL;
static SemaphoreHandle_t bacnet_datalink_mutex = NULL;
static volatile uint32_t mstp_pdu_count = 0;
static volatile uint32_t mstp_apdu_count = 0;
static volatile uint32_t mstp_rp_total = 0;
static volatile uint32_t mstp_wp_total = 0;
static float mstp_rp_last_value = 0.0f;

typedef struct {
    bool valid;
    const char *link;
    BACNET_ADDRESS src;
    BACNET_ADDRESS dest;
} BACNET_WHOIS_RX_CONTEXT;

static BACNET_WHOIS_RX_CONTEXT s_whois_rx_context = {
    .valid = false,
    .link = "unknown"
};
static int64_t s_bacrouter_compat_last_iam_us = 0;
static int64_t s_bacnet_boot_time_us = 0;
static bool s_bacrouter_discovery_succeeded = false;
static int64_t s_bacrouter_retry_last_iam_us = 0;
static uint32_t s_bacrouter_retry_attempt_count = 0;
static bool s_bacrouter_retry_stopped_logged = false;
static bool s_bacrouter_discovery_cleanup_done = false;
static bool s_bacrouter_startup_broadcast_pending = false;
static bool s_bacrouter_retry_broadcast_pending = false;
static uint32_t s_bacrouter_retry_pair_attempt_pending = 0;
static int s_bacrouter_retry_pair_unicast_result_pending = 0;
static uint32_t s_discovery_unicast_i_am_sent = 0;
static uint32_t s_discovery_broadcast_i_am_sent = 0;
static uint32_t s_discovery_confirmed_requests_seen = 0;
static uint32_t s_discovery_remote_iam_seen = 0;
static uint32_t s_discovery_global_whois_seen = 0;
static uint32_t s_discovery_limited_1_1_seen = 0;
static uint32_t s_discovery_compat_i_am_sent = 0;
static uint32_t s_discovery_compat_throttled = 0;

#define SEN54_AUTO_CLEANING_AV_INSTANCE 7U
#define SEN54_MEASUREMENT_ENABLE_BV_INSTANCE 2U
#define SEN54_FAN_CLEANING_BV_INSTANCE 3U
#define SEN54_CLEAR_STATUS_BV_INSTANCE 4U

#define SEN54_STATUS_FAN_ERROR_BIT  (1UL << 4)
#define SEN54_STATUS_LASER_ERROR_BIT (1UL << 5)
#define SEN54_STATUS_RHT_ERROR_BIT  (1UL << 6)
#define SEN54_STATUS_VOC_ERROR_BIT  (1UL << 7)

static bool sen54_auto_cleaning_interval_valid = false;
static uint32_t sen54_auto_cleaning_interval_seconds = 0;

static void sen54_set_command_bv_inactive(uint32_t instance)
{
    Binary_Value_Present_Value_Set(instance, BINARY_INACTIVE);
}

static bool wifi_connected_now(void)
{
    if (!USER_ENABLE_BACNET_IP) {
        return false;
    }

    wifi_ap_record_t ap_info = {0};
    return esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
}

static void wifi_ip_string_now(char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return;
    }

    out[0] = '\0';
    if (!USER_ENABLE_BACNET_IP) {
        snprintf(out, out_len, "No IP");
        return;
    }

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        snprintf(out, out_len, "No IP");
        return;
    }

    esp_netif_ip_info_t ip_info = {0};
    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
        snprintf(out, out_len, IPSTR, IP2STR(&ip_info.ip));
    } else {
        snprintf(out, out_len, "No IP");
    }
}

static void bacnet_log_whois_iam(const uint8_t *apdu, int apdu_len, const char *link)
{
    if (!apdu || apdu_len < 2) {
        return;
    }

    uint8_t pdu_type = apdu[0] & 0xF0;
    if (pdu_type != PDU_TYPE_UNCONFIRMED_SERVICE_REQUEST) {
        return;
    }

    uint8_t service_choice = apdu[1];
    if (service_choice == SERVICE_UNCONFIRMED_WHO_IS) {
#if BACNET_DISCOVERY_DEBUG
        int32_t low_limit = -1;
        int32_t high_limit = -1;
        int len = whois_decode_service_request(
            &apdu[2], (unsigned)(apdu_len - 2), &low_limit, &high_limit);
        if (len >= 0) {
            bool in_range = true;
            if (low_limit >= 0 && high_limit >= 0) {
                uint32_t instance = USER_BACNET_DEVICE_INSTANCE;
                in_range = (instance >= (uint32_t)low_limit &&
                    instance <= (uint32_t)high_limit);
            }
            BACNET_DISCOVERY_LOGI(
                TAG,
                "%s Who-Is low=%ld high=%ld local_instance=%lu match=%s",
                link,
                (long)low_limit,
                (long)high_limit,
                (unsigned long)USER_BACNET_DEVICE_INSTANCE,
                in_range ? "yes" : "no");
            (void)in_range;
            (void)low_limit;
            (void)high_limit;
        } else {
            BACNET_DISCOVERY_LOGW(TAG, "%s Who-Is decode failed len=%d", link, apdu_len);
        }
#endif
    } else if (service_choice == SERVICE_UNCONFIRMED_I_AM) {
        uint32_t device_id = BACNET_MAX_INSTANCE;
        unsigned max_apdu = 0;
        int segmentation = SEGMENTATION_NONE;
        uint16_t vendor_id = 0;
        int len = iam_decode_service_request(
            &apdu[2], &device_id, &max_apdu, &segmentation, &vendor_id);
        if (len >= 0) {
            s_discovery_remote_iam_seen++;
            if (USER_BACNET_LOG_REMOTE_I_AM || USER_BACNET_DISCOVERY_LOG_VERBOSE) {
                BACNET_DISCOVERY_LOGI(
                    TAG,
                    "%s I-Am device=%lu vendor=%u max_apdu=%u",
                    link,
                    (unsigned long)device_id,
                    (unsigned)vendor_id,
                    max_apdu);
            }
        } else {
            BACNET_DISCOVERY_LOGW(TAG, "%s I-Am decode failed len=%d", link, apdu_len);
        }
    }
}

#if BACNET_DISCOVERY_DEBUG
static const char *bacnet_whois_target_type(const BACNET_ADDRESS *dest)
{
    if (dest && dest->net == BACNET_BROADCAST_NETWORK) {
        return "global_broadcast";
    }
    if (dest && ((dest->net > 0U) || (dest->mac_len > 0U))) {
        return "directed";
    }

    return "local_broadcast";
}
#endif

static void bacnet_whois_context_set(
    const char *link,
    const BACNET_ADDRESS *src,
    const BACNET_ADDRESS *dest)
{
    s_whois_rx_context.valid = true;
    s_whois_rx_context.link = link ? link : "unknown";

    if (src) {
        s_whois_rx_context.src = *src;
    } else {
        memset(&s_whois_rx_context.src, 0, sizeof(s_whois_rx_context.src));
    }

    if (dest) {
        s_whois_rx_context.dest = *dest;
    } else {
        memset(&s_whois_rx_context.dest, 0, sizeof(s_whois_rx_context.dest));
    }
}

static void bacnet_whois_context_clear(void)
{
    s_whois_rx_context.valid = false;
    s_whois_rx_context.link = "unknown";
    memset(&s_whois_rx_context.src, 0, sizeof(s_whois_rx_context.src));
    memset(&s_whois_rx_context.dest, 0, sizeof(s_whois_rx_context.dest));
}

static int bacnet_src_mac_or_negative(const BACNET_ADDRESS *src)
{
    if (src && src->mac_len > 0U) {
        return (int)src->mac[0];
    }

    return -1;
}

static void bacnet_log_i_am_mstp_failure(const char *reason, int bytes_sent)
{
    DLMSTP_TX_SOURCE oldest_source = DLMSTP_TX_SOURCE_OTHER;
    unsigned long oldest_age_ms = 0;
    bool has_oldest = false;

    has_oldest = dlmstp_send_pdu_queue_oldest_source(&oldest_source, &oldest_age_ms);
    (void)oldest_source;
    (void)oldest_age_ms;
    (void)has_oldest;
    BACNET_DISCOVERY_LOGW(
        TAG,
        "I-Am TX fail reason=%s datalink_send=%d queue_depth=%u oldest_src=%s oldest_age_ms=%lu master_state=%s token=%s can_tx=%s status=%s",
        reason ? reason : "unspecified",
        bytes_sent,
        dlmstp_send_pdu_queue_depth(),
        has_oldest ? dlmstp_tx_source_text(oldest_source) : "n/a",
        has_oldest ? oldest_age_ms : 0UL,
        dlmstp_master_state_text(),
        dlmstp_token_held() ? "yes" : "no",
        dlmstp_can_transmit_now() ? "yes" : "no",
        dlmstp_send_status_text(dlmstp_send_status_last()));
}

static bool bacnet_is_deterministic_discovery_pair_reason(const char *reason)
{
    return reason &&
        ((strcmp(reason, "bacrouter_discovery_retry_unicast") == 0) ||
            (strcmp(reason, "bacrouter_discovery_retry_broadcast") == 0) ||
            (strcmp(reason, "bacrouter_discovery_startup_unicast") == 0) ||
            (strcmp(reason, "bacrouter_discovery_startup_broadcast") == 0));
}

static int64_t bacnet_bacrouter_compat_throttle_us_now(void)
{
    int64_t now_us = esp_timer_get_time();

    if (!s_bacrouter_discovery_succeeded &&
        (s_bacnet_boot_time_us > 0) &&
        ((now_us - s_bacnet_boot_time_us) < BACNET_BACROUTER_DISCOVERY_BOOT_WINDOW_US)) {
        return BACNET_BACROUTER_COMPAT_THROTTLE_FAST_US;
    }

    return BACNET_BACROUTER_COMPAT_THROTTLE_SLOW_US;
}

static int bacnet_send_i_am_with_reason(
    const char *reason,
    const char *link,
    bool dcc_gate,
    int whois_src_mac)
{
    BACNET_ADDRESS dest = { 0 };
    BACNET_NPDU_DATA npdu_data = { 0 };
    uint32_t device_instance = Device_Object_Instance_Number();
    unsigned max_apdu = Device_Max_APDU_Accepted();
    int segmentation = SEGMENTATION_NONE;
    uint16_t vendor_id = Device_Vendor_Identifier();
    int pdu_len = 0;
    int bytes_sent = 0;
    int dst_mac = -1;
    const char *dst_scope = "local_broadcast";
    const char *send_path = "unknown";
    bool dropped_i_am = false;
    bool deterministic_discovery_pair_i_am = false;
    bool discovery_i_am = false;
    bool directed_bacrouter_unicast = false;
    bool broadcast_discovery_i_am = false;
    bool high_priority_i_am = false;
    bool token_owned = false;
    bool can_transmit_now = false;
    bool queued_only_no_physical_tx = false;
    const char *mstp_state = "n/a";
    unsigned queue_depth = 0;

    if (dcc_gate && dcc_communication_initiation_disabled()) {
        BACNET_DISCOVERY_LOGI(
            TAG,
            "I-Am TX reason=%s via=%s skipped: communication initiation disabled",
            reason ? reason : "unspecified",
            link ? link : "unknown");
        return 0;
    }

    deterministic_discovery_pair_i_am = (link && strcmp(link, "MSTP") == 0) &&
        bacnet_is_deterministic_discovery_pair_reason(reason);
    discovery_i_am = deterministic_discovery_pair_i_am ||
        (reason && (strstr(reason, "discovery") != NULL)) ||
        (reason && (strcmp(reason, "response_to_who_is_compat") == 0));
    broadcast_discovery_i_am =
        reason && (strstr(reason, "_broadcast") != NULL);
    if (deterministic_discovery_pair_i_am && !broadcast_discovery_i_am) {
        dropped_i_am = dlmstp_send_pdu_queue_drop_source(DLMSTP_TX_SOURCE_I_AM);
        if (dropped_i_am) {
            BACNET_DISCOVERY_LOGI(
                TAG,
                "I-Am pre-send cleanup reason=%s dropped_i_am=%s queue_depth_now=%u",
                reason,
                dropped_i_am ? "yes" : "no",
                dlmstp_send_pdu_queue_depth());
        }
    }

    pdu_len = iam_encode_pdu(Handler_Transmit_Buffer, &dest, &npdu_data);
    if (deterministic_discovery_pair_i_am && broadcast_discovery_i_am) {
        dest.net = BACNET_BROADCAST_NETWORK;
        dest.len = 0;
        dest.mac_len = 1;
        dest.mac[0] = 0xFF;
        high_priority_i_am = true;
        dlmstp_send_pdu_priority_next_set(true);
    } else if (deterministic_discovery_pair_i_am && (whois_src_mac == 0)) {
        dest.net = 0;
        dest.len = 0;
        dest.mac_len = 1;
        dest.mac[0] = 0;
        directed_bacrouter_unicast = true;
        high_priority_i_am = true;
        dlmstp_send_pdu_priority_next_set(true);
    }
    if (dest.mac_len > 0U) {
        dst_mac = (int)dest.mac[0];
    }
    if (directed_bacrouter_unicast) {
        dst_scope = "bacrouter_unicast";
    } else if (dest.net == BACNET_BROADCAST_NETWORK) {
        dst_scope = "global_broadcast";
    } else if ((dest.net > 0U) || (dest.mac_len > 0U)) {
        dst_scope = "directed";
    }

    if (link && strcmp(link, "MSTP") == 0) {
        token_owned = dlmstp_token_held();
        can_transmit_now = dlmstp_can_transmit_now();
        mstp_state = dlmstp_master_state_text();
        queue_depth = dlmstp_send_pdu_queue_depth();
        if (high_priority_i_am && !token_owned) {
            BACNET_DISCOVERY_LOGW(
                TAG,
                "I-Am TX attempted without MS/TP token ownership reason=%s dst_mac=%d mstp_state=%s queue_depth=%u",
                reason ? reason : "unspecified",
                dst_mac,
                mstp_state,
                queue_depth);
        }
    }

    bytes_sent =
        datalink_send_pdu(&dest, &npdu_data, Handler_Transmit_Buffer, pdu_len);
    if (link && strcmp(link, "MSTP") == 0) {
        send_path = dlmstp_send_path_last_text();
        queue_depth = dlmstp_send_pdu_queue_depth();

        /* dlmstp_send_pdu() returns pdu_len when enqueue succeeds, even before
           physical UART TX occurs. In IDLE/no-token, report a distinct queued
           status so logs do not imply an immediate wire transmission. */
        if (!token_owned && !can_transmit_now && (bytes_sent > 0) &&
            (dlmstp_send_status_last() == DLMSTP_SEND_STATUS_OK) &&
            ((strcmp(send_path, "high_priority") == 0) ||
                (strcmp(send_path, "queued") == 0))) {
            queued_only_no_physical_tx = true;
            bytes_sent = BACNET_IAM_TX_QUEUED_ONLY;
            BACNET_DISCOVERY_LOGW(
                TAG,
                "I-Am queued only; no physical TX reason=%s dst_mac=%d mstp_state=%s queue_depth=%u path=%s",
                reason ? reason : "unspecified",
                dst_mac,
                mstp_state,
                queue_depth,
                send_path);
        }

        if (discovery_i_am && !token_owned && (bytes_sent > 0 || queued_only_no_physical_tx)) {
            BACNET_DISCOVERY_LOGI(
                TAG,
                "discovery I-Am queued; waiting for token reason=%s dst_mac=%d mstp_state=%s queue_depth=%u",
                reason ? reason : "unspecified",
                dst_mac,
                mstp_state,
                queue_depth);
        }

        if (discovery_i_am && (bytes_sent <= 0) &&
            (dlmstp_send_status_last() == DLMSTP_SEND_STATUS_QUEUE_FULL)) {
            dropped_i_am = dlmstp_send_pdu_queue_drop_source(DLMSTP_TX_SOURCE_I_AM);
            if (dropped_i_am) {
                bytes_sent = datalink_send_pdu(
                    &dest, &npdu_data, Handler_Transmit_Buffer, pdu_len);
                send_path = dlmstp_send_path_last_text();
                queue_depth = dlmstp_send_pdu_queue_depth();
                if (!BACNET_MAC25_MIN_LOG_MODE) {
                    BACNET_DISCOVERY_LOGW(
                        TAG,
                        "discovery I-Am queue full; dropped stale I-Am and retried reason=%s dst_mac=%d retry_ret=%d queue_depth=%u",
                        reason ? reason : "unspecified",
                        dst_mac,
                        bytes_sent,
                        queue_depth);
                }
            }
        }
    }

    if (deterministic_discovery_pair_i_am && (bytes_sent > 0 || queued_only_no_physical_tx)) {
        if (broadcast_discovery_i_am) {
            s_discovery_broadcast_i_am_sent++;
        } else {
            s_discovery_unicast_i_am_sent++;
        }
    }

    BACNET_DISCOVERY_LOGI(
        TAG,
        "I-Am TX reason=%s via=%s dst_mac=%d dst_scope=%s path=%s token_owned=%s can_transmit_now=%s mstp_state=%s queue_depth=%u high_priority=%s tx_ret=%d device_instance=%lu max_apdu=%u segmentation=%d vendor_id=%u",
        reason ? reason : "unspecified",
        link ? link : "unknown",
        dst_mac,
        dst_scope,
        send_path,
        token_owned ? "yes" : "no",
        can_transmit_now ? "yes" : "no",
        mstp_state,
        queue_depth,
        high_priority_i_am ? "yes" : "no",
        bytes_sent,
        (unsigned long)device_instance,
        max_apdu,
        segmentation,
        (unsigned)vendor_id);

    if (link && strcmp(link, "MSTP") == 0) {
        BACNET_DISCOVERY_LOGI(TAG, "I-Am MS/TP send result=%d", bytes_sent);
        if (bytes_sent == BACNET_IAM_TX_QUEUED_ONLY) {
            /* Queue accepted; TX will occur later only when state machine can transmit. */
        } else if (bytes_sent <= 0) {
            bacnet_log_i_am_mstp_failure(reason, bytes_sent);
        }
    }

    (void)device_instance;
    (void)max_apdu;
    (void)segmentation;
    (void)vendor_id;
    (void)dst_mac;
    (void)dst_scope;
    (void)mstp_state;
    (void)queue_depth;

    return bytes_sent;
}

#if !USER_MSTP_ACTIVE_DEBUG_ONLY
static void bacnet_discovery_summary_log_and_reset(void)
{
    s_discovery_global_whois_seen = 0;
    s_discovery_limited_1_1_seen = 0;
    s_discovery_compat_throttled = 0;
    s_discovery_remote_iam_seen = 0;
}
#endif

static void bacnet_mstp_discovery_retry_tick(void)
{
    int64_t now_us = esp_timer_get_time();
    bool in_boot_window = false;
    int unicast_result = 0;
    int broadcast_result = 0;

    if (!USER_ENABLE_BACNET_MSTP) {
        return;
    }

    if (s_bacnet_boot_time_us > 0) {
        in_boot_window =
            ((now_us - s_bacnet_boot_time_us) < BACNET_BACROUTER_DISCOVERY_BOOT_WINDOW_US);
    }

    if (s_bacrouter_discovery_succeeded) {
        if (!s_bacrouter_retry_stopped_logged) {
            BACNET_DISCOVERY_LOGI(
                TAG,
                "BACRouter discovery retry stopped succeeded=yes attempts=%lu",
                (unsigned long)s_bacrouter_retry_attempt_count);
            s_bacrouter_retry_stopped_logged = true;
        }
        return;
    }

    if (!in_boot_window) {
        return;
    }

    if ((s_bacrouter_retry_last_iam_us != 0) &&
        ((now_us - s_bacrouter_retry_last_iam_us) < BACNET_BACROUTER_DISCOVERY_RETRY_US)) {
        return;
    }

    s_bacrouter_retry_last_iam_us = now_us;
    s_bacrouter_retry_attempt_count++;

    bacnet_datalink_lock("mstp");
    unicast_result = bacnet_send_i_am_with_reason(
        "bacrouter_discovery_retry_unicast", "MSTP", false, 0);
    broadcast_result = bacnet_send_i_am_with_reason(
        "bacrouter_discovery_retry_broadcast", "MSTP", false, -1);
    bacnet_datalink_unlock();

    BACNET_DISCOVERY_LOGI(
        TAG,
        "BACRouter discovery retry pair attempt=%lu unicast_result=%d broadcast_result=%d discovered=%s",
        (unsigned long)s_bacrouter_retry_attempt_count,
        unicast_result,
        broadcast_result,
        s_bacrouter_discovery_succeeded ? "yes" : "no");

    (void)unicast_result;
    (void)broadcast_result;
}

static void bacnet_on_bacrouter_discovery_succeeded(void)
{
    bool dropped_i_am = false;
    unsigned queue_depth_now = 0;

    if (s_bacrouter_discovery_cleanup_done || !USER_ENABLE_BACNET_MSTP) {
        return;
    }

    bacnet_datalink_lock("mstp");
    dropped_i_am = dlmstp_send_pdu_queue_drop_source(DLMSTP_TX_SOURCE_I_AM);
    dlmstp_send_priority_queue_clear();
    s_bacrouter_startup_broadcast_pending = false;
    s_bacrouter_retry_broadcast_pending = false;
    s_bacrouter_retry_pair_attempt_pending = 0;
    s_bacrouter_retry_pair_unicast_result_pending = 0;
    queue_depth_now = dlmstp_send_pdu_queue_depth();
    bacnet_datalink_unlock();

    s_bacrouter_discovery_cleanup_done = true;
    BACNET_DISCOVERY_LOGI(
        TAG,
        "discovery cleanup: purged stale I-Am, queue_depth_now=%u dropped_i_am=%s",
        queue_depth_now,
        dropped_i_am ? "yes" : "no");

    (void)dropped_i_am;
    (void)queue_depth_now;
}

static void bacnet_mstp_discovery_pending_broadcast_tick(void)
{
    int broadcast_result = 0;

    if (!USER_ENABLE_BACNET_MSTP) {
        return;
    }

    if (s_bacrouter_discovery_succeeded) {
        s_bacrouter_startup_broadcast_pending = false;
        s_bacrouter_retry_broadcast_pending = false;
        s_bacrouter_retry_pair_attempt_pending = 0;
        s_bacrouter_retry_pair_unicast_result_pending = 0;
        return;
    }

    if (!s_bacrouter_startup_broadcast_pending &&
        !s_bacrouter_retry_broadcast_pending) {
        return;
    }

    if (!dlmstp_send_pdu_queue_empty() ||
        !dlmstp_token_held() ||
        !dlmstp_can_transmit_now()) {
        return;
    }

    bacnet_datalink_lock("mstp");
    if (s_bacrouter_startup_broadcast_pending) {
        broadcast_result = bacnet_send_i_am_with_reason(
            "bacrouter_discovery_startup_broadcast", "MSTP", false, -1);
    } else {
        broadcast_result = bacnet_send_i_am_with_reason(
            "bacrouter_discovery_retry_broadcast", "MSTP", false, -1);
    }
    bacnet_datalink_unlock();

    if (s_bacrouter_startup_broadcast_pending) {
        s_bacrouter_startup_broadcast_pending = false;
    } else {
        BACNET_DISCOVERY_LOGI(
            TAG,
            "discovery retry pair attempt=%lu unicast_result=%d broadcast_result=%d",
            (unsigned long)s_bacrouter_retry_pair_attempt_pending,
            s_bacrouter_retry_pair_unicast_result_pending,
            broadcast_result);
        s_bacrouter_retry_broadcast_pending = false;
        s_bacrouter_retry_pair_attempt_pending = 0;
        s_bacrouter_retry_pair_unicast_result_pending = 0;
    }

    (void)broadcast_result;
}

static void handler_who_is_debug(
    uint8_t *service_request,
    uint16_t service_len,
    BACNET_ADDRESS *src)
{
    int decode_len = BACNET_STATUS_ERROR;
    int32_t low_limit = -1;
    int32_t high_limit = -1;
    uint32_t local_instance = Device_Object_Instance_Number();
    bool has_limits = false;
    bool in_range = true;
    bool trace_device_in_range = false;
    bool send_i_am = false;
    const char *i_am_reason = "response_to_who_is";
    const BACNET_ADDRESS *ctx_src = src;
    int src_mac = -1;
#if BACNET_DISCOVERY_DEBUG
    BACNET_ADDRESS empty_dest = { 0 };
    const BACNET_ADDRESS *ctx_dest = &empty_dest;
#endif
    const char *link = "unknown";

    if (s_whois_rx_context.valid) {
        ctx_src = &s_whois_rx_context.src;
#if BACNET_DISCOVERY_DEBUG
        ctx_dest = &s_whois_rx_context.dest;
#endif
        link = s_whois_rx_context.link;
    }

    src_mac = bacnet_src_mac_or_negative(ctx_src);

#if BACNET_DISCOVERY_DEBUG
    ESP_LOGI(
        TAG,
        "%s Who-Is RX src_mstp_mac=%d dest_mac=%d class=%s len=%u (pre-filter)",
        link,
        (ctx_src && ctx_src->mac_len > 0U) ? (int)ctx_src->mac[0] : -1,
        (ctx_dest && ctx_dest->mac_len > 0U) ? (int)ctx_dest->mac[0] : -1,
        bacnet_whois_target_type(ctx_dest),
        (unsigned)service_len);

    if (service_len > 0U) {
        BACNET_DISCOVERY_LOGI(TAG, "%s Who-Is raw APDU payload (%u bytes):", link, (unsigned)service_len);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, service_request, service_len, ESP_LOG_INFO);
    } else {
        BACNET_DISCOVERY_LOGI(TAG, "%s Who-Is raw APDU payload: <empty>", link);
    }
#endif

    if (service_len == 0U) {
        has_limits = false;
        in_range = true;
        send_i_am = true;
    } else {
        decode_len = whois_decode_service_request(
            service_request, service_len, &low_limit, &high_limit);
        has_limits =
            (decode_len > 0) &&
            ((uint16_t)decode_len == service_len) &&
            (low_limit >= 0) &&
            (high_limit >= 0);

        if (has_limits) {
            in_range =
                (local_instance >= (uint32_t)low_limit) &&
                (local_instance <= (uint32_t)high_limit);
            send_i_am = in_range;
        } else {
            in_range = false;
            send_i_am = false;
        }
    }

#if BACNET_DISCOVERY_DEBUG
    if (!has_limits && service_len > 0U) {
        ESP_LOGW(
            TAG,
            "%s Who-Is limit decode failed/partial decode_len=%d service_len=%u",
            link,
            decode_len,
            (unsigned)service_len);
    }
#endif

    if (has_limits) {
        in_range =
            (local_instance >= (uint32_t)low_limit) &&
            (local_instance <= (uint32_t)high_limit);
        send_i_am = in_range;
    }

    if (service_len == 0U) {
        s_discovery_global_whois_seen++;
        trace_device_in_range = true;
    } else if (has_limits) {
        trace_device_in_range =
            (BACNET_WHOIS_TRACE_DEVICE_INSTANCE >= (uint32_t)low_limit) &&
            (BACNET_WHOIS_TRACE_DEVICE_INSTANCE <= (uint32_t)high_limit);
        if ((low_limit == 1) && (high_limit == 1)) {
            s_discovery_limited_1_1_seen++;
        }
    }

#if USER_BACNET_BACROUTER_DISCOVERY_COMPAT
    if (!send_i_am &&
        !s_bacrouter_discovery_succeeded &&
        has_limits &&
        (src_mac == 0) &&
        (low_limit == 1) &&
        (high_limit == 1) &&
        (local_instance == BACNET_WHOIS_TRACE_DEVICE_INSTANCE)) {
        int64_t now_us = esp_timer_get_time();
        int64_t throttle_us = bacnet_bacrouter_compat_throttle_us_now();
        if ((s_bacrouter_compat_last_iam_us == 0) ||
            ((now_us - s_bacrouter_compat_last_iam_us) >= throttle_us)) {
            s_bacrouter_compat_last_iam_us = now_us;
            send_i_am = true;
            s_discovery_compat_i_am_sent++;
            i_am_reason = "response_to_who_is_compat";
            BACNET_DISCOVERY_LOGI(
                TAG,
                "BACRouter discovery compat: responding to Who-Is 1..1 with I-Am for device 55525");
        } else {
            s_discovery_compat_throttled++;
            if (USER_BACNET_DISCOVERY_LOG_VERBOSE) {
                BACNET_DISCOVERY_LOGI(
                    TAG,
                    "BACRouter discovery compat: throttled window=%lldms",
                    (long long)(throttle_us / 1000));
            }
        }
    }
#endif

    if (s_bacrouter_discovery_succeeded) {
        bool allowed_after_discovery =
            (service_len == 0U) || (has_limits && in_range);
        if (!allowed_after_discovery) {
            send_i_am = false;
        }
        if (send_i_am &&
            (dlmstp_send_pdu_queue_depth() > BACNET_POST_DISCOVERY_IAM_MAX_QUEUE_DEPTH)) {
            send_i_am = false;
            if (USER_BACNET_DISCOVERY_LOG_VERBOSE) {
                BACNET_DISCOVERY_LOGI(
                    TAG,
                    "post-discovery Who-Is response skipped due to TX queue depth=%u",
                    dlmstp_send_pdu_queue_depth());
            }
        }
    }

    if (USER_BACNET_DISCOVERY_LOG_VERBOSE || send_i_am || trace_device_in_range) {
        BACNET_DISCOVERY_LOGI(
            TAG,
            "%s Who-Is RX src_mac=%d low=%ld high=%ld device_55525_in_range=%s will_send_i_am=%s",
            link,
            src_mac,
            (long)low_limit,
            (long)high_limit,
            trace_device_in_range ? "yes" : "no",
            send_i_am ? "yes" : "no");
    }

#if BACNET_DISCOVERY_DEBUG
    ESP_LOGI(
        TAG,
        "%s Who-Is eval has_limits=%s low_limit=%ld high_limit=%ld local_instance=%lu in_range=%s send_i_am=%s",
        link,
        has_limits ? "yes" : "no",
        (long)low_limit,
        (long)high_limit,
        (unsigned long)local_instance,
        in_range ? "yes" : "no",
        send_i_am ? "yes" : "no");
#endif

    if (send_i_am) {
        (void)bacnet_send_i_am_with_reason(i_am_reason, link, false, src_mac);
    }
}

static char datalink_bip[] = "bip";
static char datalink_mstp[] = "mstp";
static char *datalink_default = NULL;
static bool s_bacnet_ip_active = false;

static uint8_t mstp_rx_buffer[MSTP_TRANSPORT_BUFFER_SIZE];
static uint8_t mstp_tx_buffer[MSTP_TRANSPORT_BUFFER_SIZE];
static struct mstp_port_struct_t mstp_port;
static struct dlmstp_user_data_t mstp_user;
static struct dlmstp_rs485_driver mstp_rs485_driver = {
    .init = MSTP_RS485_Init,
    .send = MSTP_RS485_Send,
    .read = MSTP_RS485_Read,
    .transmitting = MSTP_RS485_Transmitting,
    .baud_rate = MSTP_RS485_Baud_Rate,
    .baud_rate_set = MSTP_RS485_Baud_Rate_Set,
    .silence_milliseconds = MSTP_RS485_Silence_Milliseconds,
    .silence_reset = MSTP_RS485_Silence_Reset
};

static void bacnet_track_confirmed_request_mstp(
    const uint8_t *apdu,
    int apdu_len,
    const BACNET_ADDRESS *src,
    const BACNET_ADDRESS *dest,
    const char *link)
{
    MSTP_RS485_CONFIRMED_REQUEST_META track = { 0 };
    BACNET_READ_PROPERTY_DATA rp_data = { 0 };
    BACNET_RPM_DATA rpm_data = { 0 };
    uint16_t object_type = 0xFFFF;
    uint32_t object_instance = BACNET_MAX_INSTANCE;
    uint32_t object_property = 0xFFFFFFFFUL;
    uint32_t array_index = BACNET_ARRAY_ALL;
    bool routed_request = false;
    bool bacrouter_discovery_path = false;
    bool should_send_reply_postponed = false;
    bool reply_postponed_sent = false;
    int64_t request_rx_us = 0;
    int64_t reply_postponed_tx_us = 0;
    int len = 0;
    uint8_t pdu_type = 0;
    uint8_t service_choice = 0;
    uint8_t invoke_id = 0;
    uint8_t requester_mac = 0xFF;

    if (!apdu || (apdu_len < 5)) {
        return;
    }

    pdu_type = apdu[0] & 0xF0;
    if (pdu_type != PDU_TYPE_CONFIRMED_SERVICE_REQUEST) {
        return;
    }
    if ((apdu[0] & BIT(3)) != 0) {
        /* Segmented request: leave behavior unchanged. */
        return;
    }

    invoke_id = apdu[2];
    service_choice = apdu[3];
    if (src && src->mac_len > 0) {
        requester_mac = src->mac[0];
    }

    if (src &&
        (src->net != 0) &&
        (src->net != BACNET_BROADCAST_NETWORK)) {
        routed_request = true;
    }

    if (service_choice == SERVICE_CONFIRMED_READ_PROPERTY) {
        len = rp_decode_service_request(&apdu[4], (uint16_t)(apdu_len - 4), &rp_data);
        if (len > 0) {
            object_type = (uint16_t)rp_data.object_type;
            object_instance = rp_data.object_instance;
            object_property = (uint32_t)rp_data.object_property;
            array_index = rp_data.array_index;
            if (rp_data.object_property == PROP_OBJECT_LIST) {
                should_send_reply_postponed = true;
            }
        }
    } else if (service_choice == SERVICE_CONFIRMED_READ_PROP_MULTIPLE) {
        len = rpm_decode_object_id(&apdu[4], (unsigned)(apdu_len - 4), &rpm_data);
        if (len > 0) {
            object_type = (uint16_t)rpm_data.object_type;
            object_instance = rpm_data.object_instance;
            len += rpm_decode_object_property(
                &apdu[4 + len],
                (unsigned)(apdu_len - 4 - len),
                &rpm_data);
            if (len > 0) {
                object_property = (uint32_t)rpm_data.object_property;
                array_index = rpm_data.array_index;
            }
        }
        should_send_reply_postponed = true;
    }

    if (routed_request) {
        should_send_reply_postponed = true;
    }

    if (!s_bacrouter_discovery_succeeded &&
        (service_choice == SERVICE_CONFIRMED_READ_PROPERTY) &&
        routed_request &&
        (requester_mac == 0) &&
        dest &&
        (dest->net == 0)) {
        bacrouter_discovery_path = true;
    }

    request_rx_us = esp_timer_get_time();
    BACNET_DISCOVERY_LOGI(
        TAG,
        "%s request received invoke=%u requester_mac=%u service=%u object=%u:%lu property=%lu array=%lu",
        link,
        (unsigned)invoke_id,
        (unsigned)requester_mac,
        (unsigned)service_choice,
        (unsigned)object_type,
        (unsigned long)object_instance,
        (unsigned long)object_property,
        (unsigned long)array_index);

    if (bacrouter_discovery_path) {
        s_bacrouter_discovery_succeeded = true;
        BACNET_DISCOVERY_LOGI(TAG, "BACRouter discovery succeeded: confirmed request received");
        bacnet_on_bacrouter_discovery_succeeded();
    }

    if (USER_MSTP_ENABLE_REPLY_POSTPONED &&
        should_send_reply_postponed &&
        requester_mac != 0xFF) {
        reply_postponed_sent =
            dlmstp_send_reply_postponed(requester_mac);
        if (reply_postponed_sent) {
            reply_postponed_tx_us = esp_timer_get_time();
        }
    }

    BACNET_DISCOVERY_LOGI(
        TAG,
        "%s reply postponed sent invoke=%u requester_mac=%u yes=%s req_to_postponed_ms=%ld",
        link,
        (unsigned)invoke_id,
        (unsigned)requester_mac,
        reply_postponed_sent ? "yes" : "no",
        (long)(reply_postponed_sent ? ((reply_postponed_tx_us - request_rx_us) / 1000) : -1));

    track.invoke_id = invoke_id;
    track.requester_mac = requester_mac;
    track.service_choice = service_choice;
    track.object_type = object_type;
    track.object_instance = object_instance;
    track.object_property = object_property;
    track.array_index = array_index;
    track.reply_postponed_sent = reply_postponed_sent;
    track.request_rx_us = request_rx_us;
    track.reply_postponed_tx_us = reply_postponed_tx_us;
    (void)dest;
    MSTP_RS485_Confirmed_Request_Track(&track);
    s_discovery_confirmed_requests_seen++;
}

static void bacnet_datalink_lock(char *name)
{
    if (bacnet_datalink_mutex) {
        xSemaphoreTake(bacnet_datalink_mutex, portMAX_DELAY);
    }
    datalink_set(name);
}

static void bacnet_datalink_unlock(void)
{
    if (datalink_default) {
        datalink_set(datalink_default);
    }
    if (bacnet_datalink_mutex) {
        xSemaphoreGive(bacnet_datalink_mutex);
    }
}

static bool bacnet_mstp_init(void)
{
    uint8_t configured_max_master = USER_MSTP_MAX_MASTER;

    MSTP_RS485_Init();

    memset(&mstp_port, 0, sizeof(mstp_port));
    memset(&mstp_user, 0, sizeof(mstp_user));

    mstp_user.RS485_Driver = &mstp_rs485_driver;
    mstp_port.UserData = &mstp_user;
    mstp_port.InputBuffer = mstp_rx_buffer;
    mstp_port.InputBufferSize = sizeof(mstp_rx_buffer);
    mstp_port.OutputBuffer = mstp_tx_buffer;
    mstp_port.OutputBufferSize = sizeof(mstp_tx_buffer);

    dlmstp_set_interface((const char *)&mstp_port);
    dlmstp_set_mac_address(USER_MSTP_MAC_ADDRESS);
    dlmstp_set_max_info_frames(USER_MSTP_MAX_INFO_FRAMES);
    if (configured_max_master > 127U) {
        configured_max_master = 127U;
    }
    dlmstp_set_max_master(configured_max_master);
    dlmstp_set_baud_rate(USER_MSTP_BAUD_RATE);
    dlmstp_check_auto_baud_set(USER_MSTP_AUTO_BAUD);
    dlmstp_slave_mode_enabled_set(false);

    ESP_LOGI(
        TAG,
        "MS/TP startup profile: mac=%u baud=%lu max_master=%u max_info=%u tx_buf=%u rx_buf=%u apdu_buf=%u max_apdu=%u",
        (unsigned)USER_MSTP_MAC_ADDRESS,
        (unsigned long)USER_MSTP_BAUD_RATE,
        (unsigned)configured_max_master,
        (unsigned)dlmstp_max_info_frames(),
        (unsigned)sizeof(mstp_tx_buffer),
        (unsigned)sizeof(mstp_rx_buffer),
        (unsigned)MSTP_APDU_BUFFER_SIZE,
        (unsigned)Device_Max_APDU_Accepted());

    return dlmstp_init((char *)&mstp_port);
}

/* BACnet receive task - processes incoming BACnet messages */
static void bacnet_receive_task(void *pvParameters)
{
    (void)pvParameters;
    BACNET_ADDRESS src = {0};
    static uint8_t rx_buffer[MSTP_APDU_BUFFER_SIZE];
    uint16_t pdu_len = 0;

    ESP_LOGI(TAG, "BACnet receive task started");

    while (1) {
        /* Poll for incoming BACnet messages */
        memset(&src, 0, sizeof(src));
        pdu_len = bip_receive(&src, rx_buffer, sizeof(rx_buffer), 100);
        if (pdu_len > 0) {
            /* Save original source from UDP socket before NPDU decode modifies it */
            BACNET_ADDRESS orig_src = src;
            BACNET_ADDRESS dest = {0};
            BACNET_NPDU_DATA npdu_data = {0};
            int apdu_offset = bacnet_npdu_decode(
                rx_buffer, pdu_len, &dest, &src, &npdu_data);
            /* If NPDU didn't have source routing info, restore from UDP socket */
            if (src.len == 0) {
                src = orig_src;
            }
            if (apdu_offset > 0 && apdu_offset < (int)pdu_len) {
                bacnet_log_whois_iam(&rx_buffer[apdu_offset], pdu_len - apdu_offset, "bip");
                bacnet_datalink_lock(datalink_bip);
                if (((rx_buffer[apdu_offset] & 0xF0U) ==
                        PDU_TYPE_UNCONFIRMED_SERVICE_REQUEST) &&
                    (apdu_offset + 1 < (int)pdu_len) &&
                    (rx_buffer[apdu_offset + 1] == SERVICE_UNCONFIRMED_WHO_IS)) {
                    bacnet_whois_context_set("BIP", &src, &dest);
                }
                apdu_handler(&src, &rx_buffer[apdu_offset], pdu_len - apdu_offset);
                bacnet_whois_context_clear();
                bacnet_datalink_unlock();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* BACnet MS/TP receive task - processes incoming MS/TP frames */
static void bacnet_mstp_receive_task(void *pvParameters)
{
    (void)pvParameters;
    BACNET_ADDRESS src = {0};
    static uint8_t rx_buffer[MSTP_APDU_BUFFER_SIZE];
    uint16_t pdu_len = 0;

    ESP_LOGI(TAG, "BACnet MS/TP receive task started");

    while (1) {
        memset(&src, 0, sizeof(src));
        pdu_len = dlmstp_receive(&src, rx_buffer, sizeof(rx_buffer), 0);
        if (pdu_len > 0) {
            mstp_pdu_count++;
            BACNET_ADDRESS dest = {0};
            BACNET_NPDU_DATA npdu_data = {0};
            int apdu_offset = bacnet_npdu_decode(
                rx_buffer, pdu_len, &dest, &src, &npdu_data);
            if (apdu_offset > 0 && apdu_offset < (int)pdu_len) {
                mstp_apdu_count++;
                bacnet_track_confirmed_request_mstp(
                    &rx_buffer[apdu_offset],
                    pdu_len - apdu_offset,
                    &src,
                    &dest,
                    "mstp");
                if ((apdu_offset + 4) <= (int)pdu_len) {
                    uint8_t pdu_type = rx_buffer[apdu_offset] & 0xF0;
                    uint8_t service_choice = rx_buffer[apdu_offset + 3];
                    if (pdu_type == PDU_TYPE_CONFIRMED_SERVICE_REQUEST &&
                        service_choice == SERVICE_CONFIRMED_READ_PROPERTY) {
                        mstp_rp_total++;
                        mstp_rp_last_value = Analog_Value_Present_Value(1);
                    } else if (pdu_type == PDU_TYPE_CONFIRMED_SERVICE_REQUEST &&
                        service_choice == SERVICE_CONFIRMED_WRITE_PROPERTY) {
                        mstp_wp_total++;
                    }
                }
                bacnet_log_whois_iam(&rx_buffer[apdu_offset], pdu_len - apdu_offset, "mstp");
                bacnet_datalink_lock(datalink_mstp);
                if (((rx_buffer[apdu_offset] & 0xF0U) ==
                        PDU_TYPE_UNCONFIRMED_SERVICE_REQUEST) &&
                    (apdu_offset + 1 < (int)pdu_len) &&
                    (rx_buffer[apdu_offset + 1] == SERVICE_UNCONFIRMED_WHO_IS)) {
                    bacnet_whois_context_set("MSTP", &src, &dest);
                }
                apdu_handler(&src, &rx_buffer[apdu_offset], pdu_len - apdu_offset);
                bacnet_whois_context_clear();
                bacnet_datalink_unlock();
            } else {
                ESP_LOGW(TAG, "MS/TP RX frame decode failed: len=%u apdu_offset=%d src.len=%u src.mac=%u",
                    (unsigned)pdu_len, apdu_offset, (unsigned)src.len,
                    (unsigned)(src.len ? src.mac[0] : 0));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* Application entry: init Wi‑Fi and BACnet */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    const char *wifi_status_summary = "disabled";
    s_bacnet_ip_active = false;
    s_bacnet_boot_time_us = esp_timer_get_time();
    s_bacrouter_discovery_succeeded = false;
    s_bacrouter_retry_last_iam_us = 0;
    s_bacrouter_retry_attempt_count = 0;
    s_bacrouter_retry_stopped_logged = false;
    s_bacrouter_discovery_cleanup_done = false;
    s_discovery_unicast_i_am_sent = 0;
    s_discovery_broadcast_i_am_sent = 0;
    s_discovery_confirmed_requests_seen = 0;
    s_bacrouter_startup_broadcast_pending = false;
    s_bacrouter_retry_broadcast_pending = false;
    s_bacrouter_retry_pair_attempt_pending = 0;
    s_bacrouter_retry_pair_unicast_result_pending = 0;

#if USER_MSTP_ACTIVE_DEBUG_ONLY
    /* Temporary minimal mode: keep only MS/TP active-debug lines and all ERROR logs. */
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set(MSTP_ACTIVE_TAG, ESP_LOG_WARN);
    esp_log_level_set("dlmstp_diag", ESP_LOG_ERROR);
    esp_log_level_set("mstp_rs485", ESP_LOG_ERROR);
    esp_log_level_set("SEN54", ESP_LOG_ERROR);
    esp_log_level_set("display", ESP_LOG_ERROR);
    esp_log_level_set("rp_tx", ESP_LOG_ERROR);
    esp_log_level_set("rpm_tx", ESP_LOG_ERROR);
    esp_log_level_set("adx_diag", ESP_LOG_ERROR);
    esp_log_level_set("obj_list", ESP_LOG_ERROR);
    esp_log_level_set("obj_list_dbg", ESP_LOG_ERROR);
#endif

    if (USER_ENABLE_BACNET_MSTP) {
        Device_Set_Max_APDU_Accepted(USER_BACNET_MSTP_MAX_APDU);
    }

    bacnet_datalink_mutex = xSemaphoreCreateMutex();
    if (!bacnet_datalink_mutex) {
        ESP_LOGE(TAG, "Failed to create BACnet datalink mutex");
    }
    
    /* If OVERRIDE_NVS_ON_FLASH is set, erase NVS to reset to code defaults.
     * Guard against wiping provisioned Wi-Fi when no compile-time defaults exist.
     */
    override_nvs_on_flash = USER_OVERRIDE_NVS_ON_FLASH;
    if (override_nvs_on_flash && USER_WIFI_SSID[0] == '\0') {
        ESP_LOGW(TAG, "OVERRIDE_NVS_ON_FLASH ignored: USER_WIFI_SSID is empty and erase would remove provisioned Wi-Fi credentials");
        override_nvs_on_flash = 0;
    }

    if (override_nvs_on_flash) {
        ESP_LOGI(TAG, "Override flag set - erasing NVS to reset to defaults");
        nvs_flash_erase();
        ret = nvs_flash_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reinitialize NVS after erase: %d", ret);
        } else {
            ESP_LOGI(TAG, "NVS reinitialized successfully");
        }
    } else if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "NVS needs initialization");
        nvs_flash_erase();
        nvs_flash_init();
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NVS initialized from existing data");
    }

#if !USER_MSTP_ACTIVE_DEBUG_ONLY
    User_Settings_Print();
#endif

    if (USER_ENABLE_BACNET_IP) {
        wifi_startup_status_t wifi_status;

        /* Initialize network stack (must be done before WiFi init) */
        esp_netif_init();
        esp_event_loop_create_default();

        wifi_status = wifi_init_sta();

        if (wifi_status == WIFI_STARTUP_CONNECTED) {
            wifi_status_summary = "connected";
            s_bacnet_ip_active = true;
        } else {
            wifi_status_summary = "unavailable";
            ESP_LOGW(TAG, "WiFi unavailable; continuing with BACnet MS/TP only");
        }

        ESP_LOGI(TAG, "WiFi status: %s", wifi_status_summary);

        if (s_bacnet_ip_active) {
            ESP_LOGI(TAG, "Initializing BACnet stack (B/IP)");
            datalink_set(datalink_bip);
            if (!datalink_init(NULL)) {
                ESP_LOGE(TAG, "Failed to initialize BACnet datalink");
                s_bacnet_ip_active = false;
            } else {
                bacnet_register_with_bbmd();
            }
        }
    } else {
        ESP_LOGI(TAG, "WiFi status: %s", wifi_status_summary);
    }

    if (USER_ENABLE_BACNET_MSTP) {
        ESP_LOGI(TAG, "Initializing BACnet MS/TP");
        if (!bacnet_mstp_init()) {
            ESP_LOGE(TAG, "Failed to initialize BACnet MS/TP datalink");
        } else {
            datalink_set(datalink_mstp);
            if (!datalink_init((char *)&mstp_port)) {
                ESP_LOGE(TAG, "Failed to initialize BACnet MS/TP datalink interface");
            }
        }
    }

    if (s_bacnet_ip_active) {
        datalink_default = datalink_bip;
    } else if (USER_ENABLE_BACNET_MSTP) {
        datalink_default = datalink_mstp;
    }
    if (datalink_default) {
        datalink_set(datalink_default);
    }

    Device_Init(NULL);
    User_Settings_InitDeviceIdentity();

    ESP_LOGI(
        TAG,
        "BACnet startup: device_instance=%lu device_name=%s mstp_mac=%u baud=%lu max_master=%u max_info_frames=%u max_apdu=%u routed_compat_mode=%s reply_postponed=%s",
        (unsigned long)Device_Object_Instance_Number(),
        Device_Object_Name_ANSI() ? Device_Object_Name_ANSI() : "(null)",
        (unsigned)USER_MSTP_MAC_ADDRESS,
        (unsigned long)USER_MSTP_BAUD_RATE,
        (unsigned)USER_MSTP_MAX_MASTER,
        (unsigned)USER_MSTP_MAX_INFO_FRAMES,
        (unsigned)Device_Max_APDU_Accepted(),
        USER_BACNET_ROUTED_COMPAT_MODE ? "enabled" : "disabled",
        USER_MSTP_ENABLE_REPLY_POSTPONED ? "enabled" : "disabled");

    /* Register service handlers - using bacnet-stack library handlers */
    ESP_LOGI(TAG, "Registering BACnet service handlers");
    apdu_set_unconfirmed_handler(SERVICE_UNCONFIRMED_I_AM, handler_i_am_add);
    apdu_set_unconfirmed_handler(SERVICE_UNCONFIRMED_WHO_IS, handler_who_is_debug);
    apdu_set_unrecognized_service_handler_handler(handler_unrecognized_service);
    /* Read Property - REQUIRED for BACnet devices */
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_READ_PROPERTY, handler_read_property);
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_READ_PROP_MULTIPLE, handler_read_property_multiple);
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_WRITE_PROPERTY, handler_write_property);
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_SUBSCRIBE_COV, handler_cov_subscribe);
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_SUBSCRIBE_COV_PROPERTY, handler_cov_subscribe_property);

    /* Initialize COV subscription list */
    handler_cov_init();

    /* Create BACnet objects (AV, BV, AI, BI, BO) */
    bacnet_create_analog_values();
    bacnet_create_binary_values();
    bacnet_create_analog_inputs();
    bacnet_create_binary_inputs();
    bacnet_create_binary_outputs_with_gpio_sync();  /* Create BO with GPIO sync task */

    BACNET_DISCOVERY_LOGI(TAG, "Sending startup I-Am");
    if (s_bacnet_ip_active) {
        bacnet_datalink_lock(datalink_bip);
        (void)bacnet_send_i_am_with_reason("startup", "BIP", true, -1);
        bacnet_datalink_unlock();
    }
    if (USER_ENABLE_BACNET_MSTP) {
        bacnet_datalink_lock(datalink_mstp);
        (void)bacnet_send_i_am_with_reason(
            "bacrouter_discovery_startup_unicast", "MSTP", false, 0);
        (void)bacnet_send_i_am_with_reason(
            "bacrouter_discovery_startup_broadcast", "MSTP", false, -1);
        bacnet_datalink_unlock();
    }

    /* Initialize display */
    ESP_LOGI(TAG, "Initializing display");
    display_init();

    /* Start BACnet receive task to handle incoming messages */
    if (s_bacnet_ip_active) {
        if (xTaskCreate(bacnet_receive_task, "bacnet_rx", 16384, NULL, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create bacnet_rx task");
        }
    }
  if (USER_ENABLE_BACNET_MSTP) {
    if (xTaskCreatePinnedToCore(bacnet_mstp_receive_task, "bacnet_mstp_rx", 12288, NULL, 10, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create bacnet_mstp_rx task");
    }
}
    if (xTaskCreate(bacnet_cov_task, "bacnet_cov", 24576, NULL, 4, &bacnet_cov_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create bacnet_cov task");
    }
    if (xTaskCreate(sen54_task, "sen54", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sen54 task");
    }

    if (USER_ENABLE_BACNET_MSTP) {
        (void)dlmstp_send_pdu_queue_drop_source(DLMSTP_TX_SOURCE_I_AM);
    }

    if (USER_ENABLE_BACNET_MSTP) {
        ESP_LOGI(TAG, "BACnet MS/TP ready");
        dlmstp_reset_statistics();
    #if !BACNET_PERIODIC_I_AM_ENABLE
        BACNET_DISCOVERY_LOGI(TAG, "Periodic I-Am disabled for discovery stability test");
    #endif
    #if BACNET_SUPPRESS_COV_UNSOLICITED_FOR_TEST
        BACNET_DISCOVERY_LOGI(TAG, "COV unsolicited traffic suppressed for discovery stability test");
    #endif
    #if USER_MSTP_ACTIVE_DEBUG_ONLY
        ESP_LOGW(
            MSTP_ACTIVE_TAG,
            "MSTP_ACTIVE_DEBUG mac=%u device=%lu baud=%lu de_pre_us=%u pfm_pre_delay_us=%u",
            (unsigned)USER_MSTP_MAC_ADDRESS,
            (unsigned long)USER_BACNET_DEVICE_INSTANCE,
            (unsigned long)USER_MSTP_BAUD_RATE,
            (unsigned)USER_RS485_DE_PRE_TX_US,
            (unsigned)USER_MSTP_PFM_REPLY_PRE_DELAY_US);
    #endif
    }

    /* Keep the task alive - maintenance + display updates */
    uint32_t display_tick = 0;
    uint32_t mstp_rx_tick = 0;
    uint32_t mstp_last_seen_pdu = 0;
    uint8_t mstp_alive_ticks = 0;
    bool mstp_ring_join_logged = false;
    while (1) {
        if (s_bacnet_ip_active) {
            bacnet_datalink_lock(datalink_bip);
            datalink_maintenance_timer(1);
            bacnet_datalink_unlock();
        }

        bacnet_mstp_discovery_pending_broadcast_tick();
        bacnet_mstp_discovery_retry_tick();

#if BACNET_PERIODIC_I_AM_ENABLE
        if (USER_ENABLE_BACNET_MSTP) {
            if (dlmstp_send_pdu_queue_empty() &&
                dlmstp_token_held() &&
                dlmstp_can_transmit_now()) {
                bacnet_datalink_lock(datalink_mstp);
                (void)bacnet_send_i_am_with_reason("periodic", "MSTP", true, -1);
                bacnet_datalink_unlock();
            }
        }
#endif

        if (USER_ENABLE_BACNET_MSTP && ++mstp_rx_tick % 30 == 0) {
            uint32_t rx_bytes = MSTP_RS485_Rx_Bytes_Get_Reset();
            uint32_t preamble_55 = 0;
            uint32_t preamble_55ff = 0;
            struct dlmstp_statistics mstp_stats = {0};
            struct dlmstp_token_diagnostics mstp_diag = {0};
            MSTP_RS485_Preamble_Counts_Get_Reset(&preamble_55, &preamble_55ff);
            dlmstp_fill_statistics(&mstp_stats);
            dlmstp_token_diagnostics_fill_and_reset(&mstp_diag);
#if !MSTP_DEBUG_ENABLE
            (void)rx_bytes;
            (void)preamble_55;
            (void)preamble_55ff;
#endif
            if (!USER_MSTP_ACTIVE_DEBUG_ONLY &&
                (mstp_stats.bad_crc_counter > 0 ||
                mstp_stats.receive_invalid_frame_counter > 0 ||
                mstp_stats.lost_token_counter > 0)) {
                ESP_LOGW(
                    TAG,
                    "MS/TP errors(30s): bad_crc=%lu invalid=%lu lost_token=%lu",
                    (unsigned long)mstp_stats.bad_crc_counter,
                    (unsigned long)mstp_stats.receive_invalid_frame_counter,
                    (unsigned long)mstp_stats.lost_token_counter);
            }
#if USER_MSTP_ACTIVE_DEBUG_ONLY
            MSTP_TOKEN_SUMMARY_LOGI(
                MSTP_ACTIVE_TAG,
                "MSTP_SUMMARY mac=%u pfm_to_us=%lu replies=%lu accepted=%lu no_token=%lu tokens_rx=%lu tokens_passed=%lu bad_crc=%lu invalid=%lu lost_token=%lu",
                (unsigned)USER_MSTP_MAC_ADDRESS,
                (unsigned long)mstp_diag.poll_for_master_to_us_counter,
                (unsigned long)mstp_diag.reply_to_poll_for_master_sent_counter,
                (unsigned long)mstp_diag.pfm_expected_token_accepted_counter,
                (unsigned long)mstp_diag.pfm_no_token_counter,
                (unsigned long)mstp_diag.token_received_counter,
                (unsigned long)mstp_diag.token_passed_counter,
                (unsigned long)mstp_diag.bad_crc_counter,
                (unsigned long)mstp_diag.invalid_counter,
                (unsigned long)mstp_diag.lost_token_counter);
#else
            MSTP_TOKEN_SUMMARY_LOGI(
                TAG,
                "MS/TP token summary(30s): pfm_to_us=%lu fast_reply_to_pfm_sent=%lu min_rx_to_tx_us=%lu max_rx_to_tx_us=%lu avg_rx_to_tx_us=%lu tokens_received=%lu tokens_passed=%lu app_frames_sent_with_token=%lu app_frames_blocked_no_token=%lu bad_crc=%lu invalid=%lu lost_token=%lu",
                (unsigned long)mstp_diag.poll_for_master_to_us_counter,
                (unsigned long)mstp_diag.fast_reply_to_pfm_sent_counter,
                (unsigned long)mstp_diag.min_rx_to_tx_us,
                (unsigned long)mstp_diag.max_rx_to_tx_us,
                (unsigned long)mstp_diag.avg_rx_to_tx_us,
                (unsigned long)mstp_diag.token_received_counter,
                (unsigned long)mstp_diag.token_passed_counter,
                (unsigned long)mstp_diag.app_frames_sent_with_token_counter,
                (unsigned long)mstp_diag.app_frames_blocked_no_token_counter,
                (unsigned long)mstp_diag.bad_crc_counter,
                (unsigned long)mstp_diag.invalid_counter,
                (unsigned long)mstp_diag.lost_token_counter);
            if (!mstp_ring_join_logged &&
                (mstp_diag.token_received_counter > 0U) &&
                (mstp_diag.token_passed_counter > 0U)) {
                ESP_LOGI(
                    TAG,
                    "MS/TP ring join complete: token_rx=%lu token_passed=%lu max_master=%u",
                    (unsigned long)mstp_diag.token_received_counter,
                    (unsigned long)mstp_diag.token_passed_counter,
                    (unsigned)dlmstp_max_master());
                mstp_ring_join_logged = true;
            }
#endif
            mstp_pdu_count = 0;
            mstp_apdu_count = 0;
            mstp_rp_total = 0;
            mstp_wp_total = 0;
            dlmstp_reset_statistics();

#if !USER_MSTP_ACTIVE_DEBUG_ONLY
            bacnet_discovery_summary_log_and_reset();
#endif
        }

        if (USER_ENABLE_BACNET_MSTP) {
            if (mstp_pdu_count != mstp_last_seen_pdu) {
                mstp_last_seen_pdu = mstp_pdu_count;
                mstp_alive_ticks = 6;
            } else if (mstp_alive_ticks > 0) {
                mstp_alive_ticks--;
            }
        } else {
            mstp_alive_ticks = 0;
        }

        display_set_link_status(
            wifi_connected_now(),
            USER_ENABLE_BACNET_MSTP && (mstp_alive_ticks > 0));
        
        /* Update display every 2 seconds */
        if (++display_tick % 2 == 0) {
            float av1 = Analog_Value_Present_Value(1);
            float av2 = Analog_Value_Present_Value(2);
            float av3 = Analog_Value_Present_Value(3);
            float av4 = Analog_Value_Present_Value(4);
            char ip_text[20];
            wifi_ip_string_now(ip_text, sizeof(ip_text));
            display_update_values(av1, av2, av3, av4);
            display_update_footer(
                USER_BACNET_DEVICE_INSTANCE,
                USER_MSTP_MAC_ADDRESS,
                ip_text);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* COV task - handles COV timer and notifications */
static void bacnet_cov_task(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
#if BACNET_SUPPRESS_COV_UNSOLICITED_FOR_TEST
        static bool cov_suppressed_logged = false;
        if (!cov_suppressed_logged) {
            ESP_LOGI(TAG, "COV task active but notifications are suppressed");
            cov_suppressed_logged = true;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
#endif
        char *active_datalink = datalink_default;
        if (!active_datalink) {
            if (s_bacnet_ip_active) {
                active_datalink = datalink_bip;
            } else if (USER_ENABLE_BACNET_MSTP) {
                active_datalink = datalink_mstp;
            }
        }

        if (active_datalink) {
            bacnet_datalink_lock(active_datalink);
            handler_cov_timer_seconds(1);
            handler_cov_task();
            bacnet_datalink_unlock();
        } else {
            handler_cov_timer_seconds(1);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void sen54_set_status_bi_if_changed(uint32_t instance,
                                           bool active,
                                           const char *label)
{
    BACNET_BINARY_PV current;
    BACNET_BINARY_PV next;

    current = Binary_Input_Present_Value(instance);
    next = active ? BINARY_ACTIVE : BINARY_INACTIVE;

    if (current != next) {
        Binary_Input_Present_Value_Set(instance, next);
        BACNET_SEN54_LOGI(TAG, "[SEN54] %s: %s", label, active ? "ON" : "OFF");
    }
}

static void sen54_update_status_objects(uint32_t status)
{
    sen54_set_status_bi_if_changed(1, (status & SEN54_STATUS_FAN_ERROR_BIT) != 0,
                                   "Fan Failure");
    sen54_set_status_bi_if_changed(2, (status & SEN54_STATUS_LASER_ERROR_BIT) != 0,
                                   "Laser Error");
    sen54_set_status_bi_if_changed(3, (status & SEN54_STATUS_VOC_ERROR_BIT) != 0,
                                   "VOC Sensor Error");
    sen54_set_status_bi_if_changed(4, (status & SEN54_STATUS_RHT_ERROR_BIT) != 0,
                                   "RHT Sensor Error");
}

static bool sen54_poll_and_publish_status(void)
{
    static bool status_valid = false;
    static uint32_t last_status = 0;
    uint32_t status = 0;

    if (!sen54_read_device_status(&status)) {
        BACNET_SEN54_LOGW(TAG, "[SEN54] Device status read failed");
        return false;
    }

    if (!status_valid || (status != last_status)) {
        BACNET_SEN54_LOGI(TAG, "[SEN54] Device Status = 0x%08lX", (unsigned long)status);
        sen54_update_status_objects(status);
        last_status = status;
        status_valid = true;
    }

    return true;
}

static bool sen54_publish_auto_cleaning_interval(uint32_t seconds)
{
    if (!Analog_Value_Present_Value_Set(
            SEN54_AUTO_CLEANING_AV_INSTANCE, (float)seconds, 16)) {
        return false;
    }

    sen54_auto_cleaning_interval_seconds = seconds;
    sen54_auto_cleaning_interval_valid = true;

    return true;
}

static void sen54_init_auto_cleaning_interval(void)
{
    uint32_t seconds = 0;

    if (sen54_read_auto_cleaning_interval(&seconds)) {
        if (sen54_publish_auto_cleaning_interval(seconds)) {
            BACNET_SEN54_LOGI(TAG, "[SEN54] Auto Cleaning Interval = %lu s",
                (unsigned long)seconds);
        } else {
            ESP_LOGE(TAG,
                "[SEN54] Auto Cleaning Interval read succeeded but AV%u update failed",
                (unsigned)SEN54_AUTO_CLEANING_AV_INSTANCE);
        }
    } else {
        ESP_LOGE(TAG, "[SEN54] Auto Cleaning Interval read failed");
    }
}

bool bacnet_av7_auto_cleaning_interval_write(
    float requested_value,
    float *applied_value,
    BACNET_ERROR_CLASS *error_class,
    BACNET_ERROR_CODE *error_code)
{
    uint32_t requested_seconds = 0;
    uint32_t old_seconds = 0;

    if (error_class) {
        *error_class = ERROR_CLASS_PROPERTY;
    }
    if (error_code) {
        *error_code = ERROR_CODE_VALUE_OUT_OF_RANGE;
    }

    if (!isfinite(requested_value) || requested_value <= 0.0f ||
        requested_value > 4294967295.0f) {
        return false;
    }

    requested_seconds = (uint32_t)requested_value;
    if ((float)requested_seconds != requested_value) {
        return false;
    }

    if (!sen54_write_auto_cleaning_interval(requested_seconds)) {
        if (error_class) {
            *error_class = ERROR_CLASS_DEVICE;
        }
        if (error_code) {
            *error_code = ERROR_CODE_OPERATIONAL_PROBLEM;
        }
        return false;
    }

    old_seconds = sen54_auto_cleaning_interval_valid
        ? sen54_auto_cleaning_interval_seconds
        : (uint32_t)Analog_Value_Present_Value(SEN54_AUTO_CLEANING_AV_INSTANCE);

    if (old_seconds != requested_seconds) {
        BACNET_SEN54_LOGI(TAG, "[SEN54] Auto Cleaning Interval changed:");
        BACNET_SEN54_LOGI(TAG, "Old: %lu s", (unsigned long)old_seconds);
        BACNET_SEN54_LOGI(TAG, "New: %lu s", (unsigned long)requested_seconds);
    }

    sen54_auto_cleaning_interval_seconds = requested_seconds;
    sen54_auto_cleaning_interval_valid = true;

    if (applied_value) {
        *applied_value = (float)requested_seconds;
    }

    return true;
}

/* SEN54 task - reads sensor data and writes to BACnet Analog Value objects
 *
 * PERIPHERAL-TO-BACNET MAPPING:
 * - AV1 (instance 1): Temperature (°C)
 * - AV2 (instance 2): Relative Humidity (%RH)
 * - AV3 (instance 3): PM2.5 concentration (μg/m³)
 * - AV4 (instance 4): VOC Index (dimensionless)
 */
static void sen54_task(void *pvParameters)
{
    (void)pvParameters;
    sen54_data_t sensor_data;
    uint8_t consecutive_failures = 0;
    TickType_t last_status_poll = 0;
    const TickType_t status_poll_interval_ticks = pdMS_TO_TICKS(5000);

    /* Give SEN54 time to boot after power-up */
    vTaskDelay(pdMS_TO_TICKS(3000));

    sen54_init();
    /* Publish BV2 = ACTIVE after every successful boot initialization.
     * BV2 (SEN54 Measurement Enable) is a runtime operating command only and
     * is intentionally not persisted across reboots.  This device is designed
     * for continuous environmental monitoring; measurement-enabled is always
     * the default state after any firmware restart, watchdog reset, brownout,
     * or power cycle, regardless of the value BV2 had before the reset. */
    if (sen54_is_measurement_enabled()) {
        Binary_Value_Present_Value_Set(SEN54_MEASUREMENT_ENABLE_BV_INSTANCE, BINARY_ACTIVE);
    }
    /* BV3 is a momentary command-only object and must always boot INACTIVE. */
    sen54_set_command_bv_inactive(SEN54_FAN_CLEANING_BV_INSTANCE);
    /* BV4 is a momentary command-only object and must always boot INACTIVE. */
    sen54_set_command_bv_inactive(SEN54_CLEAR_STATUS_BV_INSTANCE);
    sen54_init_auto_cleaning_interval();

    /* Initialize BI1..BI4 (BI0..BI3 logical mapping) from current SEN54 status. */
    sen54_poll_and_publish_status();
    last_status_poll = xTaskGetTickCount();

    /* Wait for the sensor fan and particle chamber to stabilize */
    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1) {
        if ((xTaskGetTickCount() - last_status_poll) >= status_poll_interval_ticks) {
            sen54_poll_and_publish_status();
            last_status_poll = xTaskGetTickCount();
        }

        /* BV1 written ACTIVE triggers a full SEN54 reset (I2C 0xD304) */
        if (Binary_Value_Present_Value(1) == BINARY_ACTIVE) {
            BACNET_SEN54_LOGI(TAG, "[SEN54] Full reset command requested (BV1 ACTIVE)");
            esp_err_t err = sen54_full_reset();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "[SEN54] Full reset command failed (%s)", esp_err_to_name(err));
            }
            sen54_set_command_bv_inactive(1);
            /* Sync BV2 — full reset restarts measurement */
            Binary_Value_Present_Value_Set(SEN54_MEASUREMENT_ENABLE_BV_INSTANCE,
                sen54_is_measurement_enabled() ? BINARY_ACTIVE : BINARY_INACTIVE);

            if (sen54_poll_and_publish_status()) {
                last_status_poll = xTaskGetTickCount();
            }
            continue;
        }

        /* BV2 controls SEN54 measurement enable/disable */
        {
            BACNET_BINARY_PV bv2 = Binary_Value_Present_Value(SEN54_MEASUREMENT_ENABLE_BV_INSTANCE);
            bool currently_enabled = sen54_is_measurement_enabled();

            if (bv2 == BINARY_ACTIVE && !currently_enabled) {
                if (!sen54_start_measurement()) {
                    /* Command failed — revert BV2 to reflect actual state */
                    Binary_Value_Present_Value_Set(SEN54_MEASUREMENT_ENABLE_BV_INSTANCE, BINARY_INACTIVE);
                }
            } else if (bv2 == BINARY_INACTIVE && currently_enabled) {
                if (!sen54_stop_measurement()) {
                    /* Command failed — revert BV2 to reflect actual state */
                    Binary_Value_Present_Value_Set(SEN54_MEASUREMENT_ENABLE_BV_INSTANCE, BINARY_ACTIVE);
                }
            }
        }

        /* BV3 behaves as a momentary pushbutton for manual fan cleaning. */
        if (Binary_Value_Present_Value(SEN54_FAN_CLEANING_BV_INSTANCE) == BINARY_ACTIVE) {
            sen54_start_fan_cleaning();

            /* Always auto-return to INACTIVE (momentary command object). */
            sen54_set_command_bv_inactive(SEN54_FAN_CLEANING_BV_INSTANCE);
        }

        /* BV4 behaves as a momentary command object for clear device status. */
        if (Binary_Value_Present_Value(SEN54_CLEAR_STATUS_BV_INSTANCE) == BINARY_ACTIVE) {
            if (sen54_clear_device_status()) {
                if (sen54_poll_and_publish_status()) {
                    last_status_poll = xTaskGetTickCount();
                } else {
                    ESP_LOGE(TAG, "[SEN54] Device status refresh after clear failed");
                }
            }

            /* Always auto-return to INACTIVE (momentary command object). */
            sen54_set_command_bv_inactive(SEN54_CLEAR_STATUS_BV_INSTANCE);
        }

        if (sen54_is_measurement_enabled()) {
            if (sen54_read(&sensor_data)) {
                consecutive_failures = 0;
                Analog_Value_Present_Value_Set(1, sensor_data.temperature, 16);
                Analog_Value_Present_Value_Set(2, sensor_data.humidity,    16);
                Analog_Value_Present_Value_Set(3, sensor_data.pm2_5,       16);
                Analog_Value_Present_Value_Set(4, sensor_data.voc_index,   16);
                Analog_Value_Present_Value_Set(5, sensor_data.pm1_0,       16);
                Analog_Value_Present_Value_Set(6, sensor_data.pm4_0,       16);
            } else {
                consecutive_failures++;

                /* -1 signals no valid data to BACnet clients */
                Analog_Value_Present_Value_Set(1, -1.0f, 16);
                Analog_Value_Present_Value_Set(2, -1.0f, 16);
                Analog_Value_Present_Value_Set(3, -1.0f, 16);
                Analog_Value_Present_Value_Set(4, -1.0f, 16);
                Analog_Value_Present_Value_Set(5, -1.0f, 16);
                Analog_Value_Present_Value_Set(6, -1.0f, 16);

                if (consecutive_failures >= 5) {
                    ESP_LOGW(TAG, "SEN54 read failed %u times, reinitializing sensor",
                             (unsigned)consecutive_failures);
                    sen54_init();
                    if (sen54_poll_and_publish_status()) {
                        last_status_poll = xTaskGetTickCount();
                    }
                    consecutive_failures = 0;
                    vTaskDelay(pdMS_TO_TICKS(1500));
                }
            }
        } /* if (sen54_is_measurement_enabled()) */

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void bacnet_register_with_bbmd(void)
{
    BACNET_IP_ADDRESS bbmd_addr = { { USER_BBMD_IP_OCTET_1, USER_BBMD_IP_OCTET_2,
                                     USER_BBMD_IP_OCTET_3, USER_BBMD_IP_OCTET_4 },
                                    USER_BBMD_PORT };
    int result = bvlc_register_with_bbmd(&bbmd_addr, USER_BBMD_TTL_SECONDS);
    ESP_LOGI(TAG, "BBMD register result: %d", result);
}