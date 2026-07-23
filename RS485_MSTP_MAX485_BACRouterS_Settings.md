# RS485 / BACnet MS/TP Settings for MAX485 and BACRouter-S

This file summarizes the settings that produced stable BACnet MS/TP operation on the production bus using an ESP32, MAX485 transceiver, and BACRouter-S.

GPIO assignments are not included because they vary by board.

## 1. UART configuration

```c
#define MSTP_UART_BAUD_DEFAULT 38400U
#define MSTP_UART_RX_BUF_SIZE  512
#define MSTP_UART_TX_BUF_SIZE  0
```

Use:

```c
.data_bits = UART_DATA_8_BITS,
.parity    = UART_PARITY_DISABLE,
.stop_bits = UART_STOP_BITS_1,
.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
```

The critical setting is:

```c
#define MSTP_UART_TX_BUF_SIZE 0
```

A zero TX buffer prevents `uart_write_bytes()` from returning while frame bytes are still waiting in the ESP-IDF software TX buffer. This is required when MAX485 direction is controlled manually.

## 2. MAX485 direction control

Tie MAX485 `DE` and `/RE` together and control them from one ESP32 GPIO.

```text
DE high = transmit
DE low  = receive
```

Use normal UART mode:

```c
uart_set_mode(MSTP_UART_PORT, UART_MODE_UART);
```

Do not combine manual DE control with:

```c
UART_MODE_RS485_HALF_DUPLEX
```

Use only one direction-control method.

## 3. Transmission sequence

Use this sequence for every MS/TP frame:

```c
set_de_high();

esp_rom_delay_us(200);

uart_write_bytes(...);

uart_wait_tx_done(...);

esp_rom_delay_us(1000);

set_de_low();
```

Validated timing:

```c
#define USER_RS485_DE_PRE_TX_US          200
#define MSTP_RS485_DE_POST_TX_GUARD_MS     1
```

Always wait for `uart_wait_tx_done()` before releasing DE.

## 4. UART receive callback

The BACnet stack calls the receive callback with `NULL` to check whether more UART bytes are waiting.

Do not use:

```c
if (!buf) {
    return false;
}
```

Use:

```c
if (!buf) {
    size_t buffered_len = 0;

    if (uart_get_buffered_data_len(
            MSTP_UART_PORT,
            &buffered_len) != ESP_OK) {
        return false;
    }

    return buffered_len > 0;
}
```

This allows the BACnet stack to drain all pending UART bytes during the same processing cycle.

Without this correction, only one byte was processed per task iteration, causing delayed frames, UART backlog, CRC errors, missed tokens, and intermittent BACRouter-S activity.

## 5. MS/TP task

Validated task configuration:

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
Stack size: 12288
Priority:   10
Core:       1
```

The selected core may vary on another ESP32 target. The important requirements are high priority, predictable scheduling, and no heavy work or logging inside the MS/TP task.

## 6. BACnet MS/TP parameters

Validated settings:

```c
USER_MSTP_BAUD_RATE        = 38400U;
USER_MSTP_MAX_INFO_FRAMES  = 1;
USER_MSTP_MAX_MASTER       = 34;
USER_MSTP_AUTO_BAUD        = false;
USER_BACNET_MSTP_MAX_APDU  = 480;
```

`Max_Info_Frames = 1` is recommended for similar production trunks.

`Max_Master = 34` is site-specific. Set it to the highest master MAC required on the actual trunk.

## 7. Fixed next station

The validated production token sequence was:

```text
MAC32 -> ESP32 MAC33 -> BACRouter-S MAC0
```

For this topology:

```c
#define USER_MSTP_DEBUG_FORCE_NEXT_STATION 0
```

This value is site-specific. Use the actual BACRouter-S MS/TP MAC address when the topology is fixed and the ESP32 should pass the token directly to the router.

Do not copy `0` if the BACRouter-S uses another MAC or another master should follow the ESP32.

## 8. Production debug settings

Use:

```c
#define USER_MSTP_ACTIVE_DEBUG_ONLY       0
#define USER_MSTP_TOKEN_PASS_ONLY_DEBUG   0
```

Token-only debug mode must be disabled for YABE discovery and normal BACnet application traffic.

## 9. Logging

Keep only:

- MS/TP startup summary
- First successful ring-join message
- 30-second token/error summary
- UART failures
- Genuine BACnet protocol warnings and errors
- Task crashes, watchdogs, and panics

Remove or downgrade:

- Per-token logs
- Per-frame TX/RX logs
- TX turnaround logs
- I-Am queue diagnostics
- Poll-for-Master detail logs
- CRC accumulator details
- Repetitive discovery retry logs

Heavy logging can interfere with MS/TP timing.

## 10. Validation

A successful 30-second summary should normally show:

```text
tokens_received > 0
tokens_passed > 0
app_frames_sent_with_token > 0
bad_crc = 0 or very low
invalid = 0 or very low
lost_token = 0
```

Confirm operation with both:

1. BACRouter-S packet capture
2. YABE device and object discovery

Expected token sequence:

```text
TOKEN previous master -> ESP32
TOKEN ESP32 -> BACRouter-S
```

Expected BACnet operation:

```text
Who-Is / confirmed request -> ESP32
I-Am / ACK response        -> requesting device
```

## Essential transferable settings

The most important requirements are:

```c
#define MSTP_UART_TX_BUF_SIZE 0
```

Correct `MSTP_RS485_Read(NULL)` handling using:

```c
uart_get_buffered_data_len(...)
```

Manual MAX485 DE control with normal UART mode.

Waiting for:

```c
uart_wait_tx_done(...)
```

before releasing DE.

High-priority, predictably scheduled MS/TP task.

```c
Max_Info_Frames = 1
```

Deliberately configured `Max_Master` and next-station values for the actual production trunk.
