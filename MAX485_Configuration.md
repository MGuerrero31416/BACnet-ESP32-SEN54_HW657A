# ESP32 MAX485 MS/TP Production Configuration

This document summarizes the settings and implementation details that produced stable BACnet MS/TP operation with a MAX485 transceiver on a production trunk connected through BACRouter-S.

GPIO assignments are intentionally omitted because they vary by ESP32 board.

## Validated Result

The working configuration achieved:

* Stable BACRouter-S MS/TP activity
* Continuous token participation
* Successful YABE discovery
* Successful Object List reads
* Successful ReadProperty responses
* No sustained CRC or invalid-frame errors
* No truncated token frames

The validated production token sequence was:

```text
Existing master -> ESP32 -> BACRouter-S
MAC32           -> MAC33 -> MAC0
```

MAC addresses and Device Instances must remain unique and project-specific.

---

## 1. UART Configuration

Recommended UART settings:

```c
#define MSTP_UART_BAUD_DEFAULT 38400U
#define MSTP_UART_RX_BUF_SIZE  512
#define MSTP_UART_TX_BUF_SIZE  0
```

UART framing:

```c
.data_bits = UART_DATA_8_BITS,
.parity    = UART_PARITY_DISABLE,
.stop_bits = UART_STOP_BITS_1,
.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
```

### Critical TX-buffer requirement

Use:

```c
#define MSTP_UART_TX_BUF_SIZE 0
```

Do not use a nonzero ESP-IDF UART TX software buffer when MAX485 direction is controlled manually.

With a nonzero TX buffer, `uart_write_bytes()` may return after copying the frame into the software buffer but before all bytes reach the UART wire. DE may then be released too early, causing truncated MS/TP frames.

Setting the TX buffer to zero makes the UART transmission path blocking and compatible with manual DE control.

---

## 2. MAX485 Direction Control

Use normal UART mode:

```c
uart_set_mode(MSTP_UART_PORT, UART_MODE_UART);
```

Do not combine manual DE control with:

```c
UART_MODE_RS485_HALF_DUPLEX
```

Connect MAX485 `DE` and `/RE` together and control them with one ESP32 GPIO.

Direction logic:

```text
DE high -> transmit
DE low  -> receive
```

Required transmission sequence:

```c
set_de_high();

optional_pre_tx_delay();

uart_write_bytes(...);
uart_wait_tx_done(...);

optional_post_tx_guard();

set_de_low();
```

Only one direction-control method must be active.

Do not mix:

* Manual GPIO DE control
* UART RTS-controlled DE
* ESP-IDF RS485 half-duplex mode
* External auto-direction logic

---

## 3. Wait for Complete Transmission

Always wait for the UART to finish transmitting before returning the MAX485 to receive mode:

```c
esp_err_t tx_done =
    uart_wait_tx_done(MSTP_UART_PORT, tx_wait_ticks);
```

Release DE only after `uart_wait_tx_done()` returns `ESP_OK`.

The timeout should account for:

```text
frame bytes x 10 bits per byte / baud rate
```

plus a reasonable safety margin.

The validated configuration also retains a short post-transmission guard:

```c
#define MSTP_RS485_DE_POST_TX_GUARD_MS 1
```

Example:

```c
if (tx_done == ESP_OK) {
    esp_rom_delay_us(
        (uint32_t)MSTP_RS485_DE_POST_TX_GUARD_MS * 1000U);
}

set_de_low();
```

This ensures that the final stop bit has cleared the transceiver before DE is released.

---

## 4. Receive Callback Behavior

The BACnet MS/TP stack calls the UART receive callback with `NULL` to ask whether more bytes remain buffered.

The callback must not return `false` automatically when `buf == NULL`.

Correct behavior:

```c
bool MSTP_RS485_Read(uint8_t *buf)
{
    if (!mstp_uart_initialized) {
        MSTP_RS485_Init();
    }

    /*
     * The BACnet stack calls read(NULL) to determine whether
     * additional UART bytes are waiting.
     */
    if (!buf) {
        size_t buffered_len = 0;

        if (uart_get_buffered_data_len(
                MSTP_UART_PORT,
                &buffered_len) != ESP_OK) {
            return false;
        }

        return buffered_len > 0;
    }

    int len = uart_read_bytes(
        MSTP_UART_PORT,
        buf,
        1,
        0);

    if (len > 0) {
        mstp_last_activity_us = esp_timer_get_time();
        mstp_rx_bytes += (uint32_t)len;

        if (*buf == 0x55) {
            mstp_preamble_55++;
        }

        if ((mstp_prev_byte == 0x55) &&
            (*buf == 0xFF)) {
            mstp_preamble_55ff++;
        }

        mstp_prev_byte = *buf;
        return true;
    }

    return false;
}
```

This allows the BACnet stack to drain all waiting UART bytes during the same processing cycle.

The incorrect implementation:

```c
if (!buf) {
    return false;
}
```

caused approximately one byte to be processed per MS/TP task iteration. On a busy production bus, this produced:

* UART receive backlog
* Delayed Reply-to-Poll-for-Master frames
* CRC errors
* Invalid-frame errors
* Missed Tokens
* Random brief BACRouter-S activity

---

## 5. MS/TP Task Configuration

The validated task configuration is:

```c
xTaskCreatePinnedToCore(
    bacnet_mstp_receive_task,
    "bacnet_mstp_rx",
    12288,
    NULL,
    10,
    NULL,
    1);
```

Validated values:

```text
Stack size: 12288 bytes
Priority:   10
Core:       1
```

The selected core may vary on another ESP32 target.

Important requirements:

* Give the MS/TP task a high priority.
* Use predictable task scheduling.
* Avoid heavy logging inside the receive and token-processing path.
* Avoid long sensor, display, Wi-Fi, or storage operations in the MS/TP task.
* Keep any task delay short and consistent.

---

## 6. MS/TP Network Parameters

Validated parameters:

```c
USER_MSTP_BAUD_RATE       = 38400U;
USER_MSTP_MAX_INFO_FRAMES = 1;
USER_MSTP_MAX_MASTER      = 34;
USER_MSTP_AUTO_BAUD       = false;
USER_BACNET_MSTP_MAX_APDU = 480;
```

### Baud rate

Use the actual production trunk baud rate.

The validated trunk used:

```text
38400 baud
```

### Max Info Frames

Recommended:

```text
Max_Info_Frames = 1
```

This prevents the ESP32 from holding the token for excessive application traffic.

### Max Master

`Max_Master` is site-specific.

Set it deliberately to the highest master MAC address that must participate on the trunk.

Do not copy `34` to another project unless it matches that trunk.

An unnecessarily high value causes Poll-for-Master traffic to scan unused addresses.

---

## 7. Fixed Successor for BACRouter-S

The validated topology used:

```text
MAC32 -> MAC33 -> MAC0
```

The ESP32 was configured to pass the token directly to BACRouter-S MAC0:

```c
#define USER_MSTP_DEBUG_FORCE_NEXT_STATION 0
```

Although the current macro name contains `DEBUG`, it functions as a fixed production successor setting in this implementation.

For another project, use:

```c
#define USER_MSTP_DEBUG_FORCE_NEXT_STATION \
    <BACROUTER_MSTP_MAC>
```

Use a fixed successor only when:

* The production topology is known
* The BACRouter-S address is fixed
* The ESP32 should always pass the token to that router
* Standard successor discovery is unreliable on the specific trunk

Do not force MAC0 if the BACRouter-S uses another address or another master should follow the ESP32.

A clearer future macro name would be:

```c
#define USER_MSTP_FIXED_NEXT_STATION 0
```

---

## 8. Validated Timing and Debug Settings

Validated production settings:

```c
#define USER_MSTP_ACTIVE_DEBUG_ONLY                   0
#define USER_MSTP_TOKEN_PASS_ONLY_DEBUG               0
#define USER_MSTP_DEBUG_FORCE_NEXT_STATION             0

#define USER_MSTP_PFM_REPLY_PRE_DELAY_US               0
#define USER_RS485_DE_PRE_TX_US                      200
#define USER_MSTP_TOKEN_PASS_PRE_DELAY_US               0

#define USER_RS485_CONTROL_FRAME_UART_FLUSH_BEFORE_TX   0
#define USER_RS485_CONTROL_FRAME_EXTRA_IDLE_US          0
#define USER_RS485_CONTROL_FRAME_DE_PRE_TX_US            0

#define USER_RS485_PFM_TX_MARKER_ENABLE                 0
```

The 200 microsecond DE pre-transmission delay was retained in the validated configuration:

```c
#define USER_RS485_DE_PRE_TX_US 200
```

However, the essential fixes were:

1. UART TX buffer set to zero
2. Correct `read(NULL)` handling
3. Manual DE control
4. Waiting for UART TX completion
5. High-priority MS/TP task

Additional delays should not be increased unless supported by an actual packet capture.

---

## 9. Production Logging

Recommended logs to retain:

```text
MS/TP startup summary
First successful ring join
30-second token/error summary
UART or driver failures
Real BACnet protocol errors
Task crashes, watchdogs, or panics
```

Logs to remove or downgrade to debug level:

```text
Every received Token
Every transmitted Token
Every Poll-for-Master
Every Reply-to-Poll-for-Master
Every UART byte
Every TX turnaround
Every queued I-Am
Every token-use iteration
Detailed CRC accumulator values
Discovery retry messages
Successful ReadProperty diagnostics
```

Heavy per-frame logging can interfere with MS/TP timing and make useful events difficult to identify.

---

## 10. Validation Procedure

Validate the configuration in this order.

### Step 1: Token-only test

Temporarily use:

```c
#define USER_MSTP_TOKEN_PASS_ONLY_DEBUG 1
```

Confirm with BACRouter-S capture:

```text
TOKEN previous-master -> ESP32
TOKEN ESP32 -> BACRouter-S
```

The received and passed Token counters should remain approximately equal.

### Step 2: Enable BACnet application traffic

Restore:

```c
#define USER_MSTP_TOKEN_PASS_ONLY_DEBUG 0
```

Confirm:

* Device remains Active on BACRouter-S
* YABE discovers the device
* YABE reads Object List
* YABE reads device and object properties
* BACnet responses are visible in the capture

### Step 3: Monitor errors

The 30-second summary should normally show:

```text
tokens_received > 0
tokens_passed > 0
app_frames_sent_with_token > 0
bad_crc = 0 or very low
invalid = 0 or very low
lost_token = 0
```

### Step 4: Inspect physical capture

Serial counters alone are insufficient.

Use BACRouter-S packet capture to confirm complete on-wire frames.

Look for:

```text
Poll-for-Master -> ESP32
Reply-to-Poll-for-Master -> previous master
Token -> ESP32
Token -> next master
BACnet data request -> ESP32
BACnet response -> requesting master
```

Check that frames are complete and are not truncated after the preamble.

---

## 11. Transfer Checklist

When adapting this configuration to another ESP32/MAX485 project:

* [ ] Assign board-specific UART TX, RX, and DE GPIOs.
* [ ] Use unique BACnet MAC and Device Instance values.
* [ ] Match the production trunk baud rate.
* [ ] Use UART 8-N-1.
* [ ] Use normal UART mode.
* [ ] Use manual DE control.
* [ ] Do not mix manual and UART-managed direction control.
* [ ] Set the UART TX software buffer to zero.
* [ ] Retain an adequate UART RX buffer.
* [ ] Wait for `uart_wait_tx_done()` before releasing DE.
* [ ] Retain a short validated post-TX DE guard.
* [ ] Make `MSTP_RS485_Read(NULL)` report buffered RX data.
* [ ] Drain all waiting RX bytes per processing cycle.
* [ ] Run MS/TP at high task priority.
* [ ] Avoid heavy per-frame logging.
* [ ] Set `Max_Info_Frames = 1`.
* [ ] Set `Max_Master` for the actual trunk.
* [ ] Configure the correct next station if using a fixed topology.
* [ ] Disable token-only debug mode for production.
* [ ] Validate with BACRouter-S packet capture.
* [ ] Validate complete discovery and property reads with YABE.

---

## 12. Most Important Requirements

The two most important code-level settings are:

```c
#define MSTP_UART_TX_BUF_SIZE 0
```

and correct handling of:

```c
MSTP_RS485_Read(NULL)
```

using:

```c
uart_get_buffered_data_len(...)
```

Without these corrections, the ESP32 may appear Active briefly while still:

* Processing received frames too slowly
* Missing Tokens
* Replying too late to Poll-for-Master
* Releasing DE before transmission is complete
* Producing truncated Token frames
* Disappearing from BACRouter-S
