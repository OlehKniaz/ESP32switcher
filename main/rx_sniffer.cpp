// rx_sniffer.cpp
//
// ESP32 multi-receiver CRSF bridge:
//   - Flight Controller <-> ESP32 (UART1)
//   - Up to 5 CRSF receivers on ESP32 (all via UART2 with dynamic pin remap)
//
// Changes vs your version:
//   1) Uses pdMS_TO_TICKS() everywhere (no tick-math surprises)
//   2) Non-blocking probe/read helper using uart_get_buffered_data_len()
//   3) Background "sniff" of non-active receivers while a receiver is active
//      -> fast failover when multiple receivers are already sending
//   4) Faster scan when no active receiver (no per-receiver blocking delays)
//   5) Lower LINK_TIMEOUT_MS (tune as needed)
//
// Notes:
// - Background sniff reads bytes from non-active RX but does NOT forward them to FC.
// - The FC<->ActiveRX forwarding remains transparent.

#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"   // ets_delay_us

// Flight Controller UART on ESP32 (UART1)
#define UART_FC_RX   16   // ESP32 RX1 (connect to FC TX)
#define UART_FC_TX   17   // ESP32 TX1 (connect to FC RX)

// Receivers 1-5 on ESP32 (all share UART2 via pin remap)
#define RECIVER1RX   18
#define RECIVER1TX   19

#define RECIVER2RX   21
#define RECIVER2TX   22

#define RECIVER3RX   23
#define RECIVER3TX   25

#define RECIVER4RX   26
#define RECIVER4TX   27


#define CRSF_BAUD    420000

static const char *TAG = "CRSF_BRIDGE";

enum Link {
    LINK_1 = 0,
    LINK_2 = 1,
    LINK_3 = 2,
    LINK_4 = 3,
    LINK_5 = 4
};

static const int NUM_RECEIVERS = 5;

static const gpio_num_t rxPins[NUM_RECEIVERS] = {
    (gpio_num_t)RECIVER1RX,
    (gpio_num_t)RECIVER2RX,
    (gpio_num_t)RECIVER3RX,
    (gpio_num_t)RECIVER4RX,
};

static const gpio_num_t txPins[NUM_RECEIVERS] = {
    (gpio_num_t)RECIVER1TX,
    (gpio_num_t)RECIVER2TX,
    (gpio_num_t)RECIVER3TX,
    (gpio_num_t)RECIVER4TX,
};

// Active receiver index: -1 means "no active"
static int  activeRxIndex = -1;

// Which receiver is currently mapped on UART2
static int  mappedRxIndex = -1;

static Link activeLink = LINK_1;

static int64_t lastPktMs[NUM_RECEIVERS] = {0};
static bool    havePkt[NUM_RECEIVERS]   = {false};

// Tune these
static const int64_t LINK_TIMEOUT_MS = 50;     // was 100; CRSF frames are often ~20ms
static const int64_t STATUS_LOG_INTERVAL_MS = 2000;

// Background sniff tuning
static const int64_t SNIFF_INTERVAL_MS = 25;  // how often to peek other receivers
static const int     SNIFF_SETTLE_US   = 200; // allow GPIO matrix/UART to settle

// Read timeouts (actual UART read timeouts)
static const TickType_t READ_FC_TICKS     = pdMS_TO_TICKS(2);
static const TickType_t READ_ACTIVE_TICKS = pdMS_TO_TICKS(2);

// ------------------------------------------------------------
// Small CRSF custom telemetry sender (your existing logic)
// ------------------------------------------------------------
static void send_crsf_5link_telemetry(bool a1, bool a2, bool a3, bool a4, bool a5)
{
    uint8_t frame[5] = {
        (uint8_t)(a1 ? 1 : 0),
        (uint8_t)(a2 ? 1 : 0),
        (uint8_t)(a3 ? 1 : 0),
        (uint8_t)(a4 ? 1 : 0),
        (uint8_t)(a5 ? 1 : 0)
    };

    uint8_t packet[9];
    packet[0] = 0xEE;
    packet[1] = 5 + 2;   // type + payload length? (your original)
    packet[2] = 0x90;    // custom type
    packet[3] = frame[0];
    packet[4] = frame[1];
    packet[5] = frame[2];
    packet[6] = frame[3];
    packet[7] = frame[4];

    // NOTE: This is XOR, not CRSF CRC8. Kept as-is from your code.
    uint8_t crc = 0;
    for (int i = 2; i <= 7; i++) {
        crc ^= packet[i];
    }
    packet[8] = crc;

    uart_write_bytes(UART_NUM_1, (const char *)packet, sizeof(packet));
}

// ------------------------------------------------------------
// UART2 pin remap helper
// ------------------------------------------------------------
static void uart2_select_receiver(int idx)
{
    if (idx < 0 || idx >= NUM_RECEIVERS) return;
    if (mappedRxIndex == idx) return;

    esp_err_t err = uart_set_pin(UART_NUM_2,
                                 txPins[idx],
                                 rxPins[idx],
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin(UART2) failed for receiver %d, err=%d", idx + 1, (int)err);
        return;
    }

    mappedRxIndex = idx;

    // REMOVE THIS:
    // ESP_LOGI(TAG, "UART2 mapped to receiver %d (TX=%d, RX=%d)", ...);
}


// ------------------------------------------------------------
// Non-blocking read helper (reads whatever is already buffered)
// ------------------------------------------------------------
static inline int uart2_try_read(uint8_t *buf, int maxlen)
{
    size_t buffered = 0;
    uart_get_buffered_data_len(UART_NUM_2, &buffered);
    if (buffered == 0) return 0;

    if (buffered > (size_t)maxlen) buffered = (size_t)maxlen;
    return uart_read_bytes(UART_NUM_2, buf, (uint32_t)buffered, 0);
}

// ------------------------------------------------------------
// Pick best receiver quickly from "alive" state (no scanning delays)
// Returns index or -1.
// ------------------------------------------------------------
static int pick_best_alive(int64_t nowMs)
{
    for (int i = 0; i < NUM_RECEIVERS; i++) {
        if (havePkt[i] && (nowMs - lastPktMs[i]) < LINK_TIMEOUT_MS) {
            return i;
        }
    }
    return -1;
}

// ------------------------------------------------------------
// Fast scan: quickly hop and non-blocking check for bytes.
// Returns index or -1.
// ------------------------------------------------------------
static int fast_scan_for_any(uint8_t *buf, int bufSize, int64_t nowMs, int *outLen)
{
    for (int i = 0; i < NUM_RECEIVERS; i++) {
        uart2_select_receiver(i);
        ets_delay_us(SNIFF_SETTLE_US);

        int n = uart2_try_read(buf, bufSize);
        if (n > 0) {
            havePkt[i]   = true;
            lastPktMs[i] = nowMs;
            if (outLen) *outLen = n;
            return i;
        }
    }
    if (outLen) *outLen = 0;
    return -1;
}

// ------------------------------------------------------------
// Background sniff scheduler state
// ------------------------------------------------------------
static int64_t g_lastSniffMs = 0;
static int     g_sniffIdx    = 0;

static void background_sniff_non_active(uint8_t *buf, int bufSize, int64_t nowMs)
{
    if (activeRxIndex < 0) return;
    if ((nowMs - g_lastSniffMs) < SNIFF_INTERVAL_MS) return;

    g_lastSniffMs = nowMs;

    // advance sniff index to a non-active receiver
    g_sniffIdx = (g_sniffIdx + 1) % NUM_RECEIVERS;
    if (g_sniffIdx == activeRxIndex) {
        g_sniffIdx = (g_sniffIdx + 1) % NUM_RECEIVERS;
    }

    // hop to sniff receiver
    uart2_select_receiver(g_sniffIdx);
    ets_delay_us(SNIFF_SETTLE_US);

    int n = uart2_try_read(buf, bufSize);
    if (n > 0) {
        havePkt[g_sniffIdx]   = true;
        lastPktMs[g_sniffIdx] = nowMs;
        // Important: do NOT forward these bytes (they are from non-active RX)
    }

    // hop back immediately
    uart2_select_receiver(activeRxIndex);
}

// ------------------------------------------------------------
// Main task
// ------------------------------------------------------------
static void uart_task(void *p)
{
    ESP_LOGI(TAG, "uart_task started. CRSF baud=%d", CRSF_BAUD);

    uint8_t buf[256];
    int64_t lastStatusLogMs = 0;

    while (true) {
        int64_t nowMs = esp_timer_get_time() / 1000; // ms

        // 1) Read from ACTIVE receiver -> FC
        if (activeRxIndex >= 0) {
            uart2_select_receiver(activeRxIndex);

            int lenRx = uart_read_bytes(UART_NUM_2, buf, sizeof(buf), READ_ACTIVE_TICKS);
            if (lenRx > 0) {
                havePkt[activeRxIndex]   = true;
                lastPktMs[activeRxIndex] = nowMs;

                uart_write_bytes(UART_NUM_1, (const char *)buf, lenRx);
            }
        }

        // 2) Read from FC -> ACTIVE receiver
        int lenFc = uart_read_bytes(UART_NUM_1, buf, sizeof(buf), READ_FC_TICKS);
        if (lenFc > 0 && activeRxIndex >= 0) {
            uart2_select_receiver(activeRxIndex);
            uart_write_bytes(UART_NUM_2, (const char *)buf, lenFc);
        }

        // 3) Background sniff other receivers so we already know who's alive
        background_sniff_non_active(buf, sizeof(buf), nowMs);

        // 4) Compute alive[] and detect timeout of active
        bool alive[NUM_RECEIVERS] = {false};

        for (int i = 0; i < NUM_RECEIVERS; i++) {
            bool a = havePkt[i] && ((nowMs - lastPktMs[i]) < LINK_TIMEOUT_MS);
            alive[i] = a;

            if (i == activeRxIndex && !a) {
                ESP_LOGW(TAG, "Receiver %d timed out", i + 1);
                activeRxIndex = -1;
            }
        }

        // 5) Send telemetry to FC
        send_crsf_5link_telemetry(alive[0], alive[1], alive[2], alive[3], alive[4]);

        // 6) Failover logic
        if (activeRxIndex < 0) {
            // 6a) Instant switch if we already sniffed someone alive
            int best = pick_best_alive(nowMs);
            if (best >= 0) {
                activeRxIndex = best;
                activeLink = (Link)best;

                uart2_select_receiver(best);
                // Optionally flush just once when we lock:
                uart_flush_input(UART_NUM_2);

                ESP_LOGI(TAG, "Fast failover to receiver %d", best + 1);
            } else {
                // 6b) Otherwise do a fast scan (non-blocking)
                int gotLen = 0;
                int idx = fast_scan_for_any(buf, sizeof(buf), nowMs, &gotLen);
                if (idx >= 0) {
                    activeRxIndex = idx;
                    activeLink = (Link)idx;

                    uart2_select_receiver(idx);
                    uart_flush_input(UART_NUM_2);

                    ESP_LOGI(TAG, "Locking onto receiver %d", idx + 1);

                    // Forward whatever we captured to FC (only when locking)
                    if (gotLen > 0) {
                        uart_write_bytes(UART_NUM_1, (const char *)buf, gotLen);
                    }
                }
            }
        }

        // 7) Status log
        if ((nowMs - lastStatusLogMs) > STATUS_LOG_INTERVAL_MS) {
            lastStatusLogMs = nowMs;
            ESP_LOGI(TAG,
                     "STATUS: R1=%d R2=%d R3=%d R4=%d R5=%d active=%d",
                     alive[0], alive[1], alive[2], alive[3], alive[4],
                     (activeRxIndex >= 0) ? (activeRxIndex + 1) : 0);
        }

        vTaskDelay(1);
    }
}

extern "C" void rx_sniffer_start()
{
    ESP_LOGI(TAG, "Starting multi-receiver CRSF bridge...");

    for (int i = 0; i < NUM_RECEIVERS; i++) {
        gpio_reset_pin(rxPins[i]);
        gpio_set_direction(rxPins[i], GPIO_MODE_INPUT);
    }

    uart_config_t cfg_crsf = {
        .baud_rate = CRSF_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = 0
    };

    // FC UART
    uart_driver_install(UART_NUM_1, 4096, 4096, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &cfg_crsf);
    uart_set_pin(UART_NUM_1,
                 (gpio_num_t)UART_FC_TX,
                 (gpio_num_t)UART_FC_RX,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);

    // Shared receiver UART
    uart_driver_install(UART_NUM_2, 4096, 4096, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &cfg_crsf);

    // Initial mapping
    uart2_select_receiver(0);
    uart_flush_input(UART_NUM_2);

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
}
