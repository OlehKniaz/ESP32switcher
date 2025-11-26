#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// UART pins
#define UART_FC_RX   16
#define UART_FC_TX   17

#define UART_24_RX   26
#define UART_24_TX   27

#define UART_900_RX  15
#define UART_900_TX  4

#define RF_SELECT    GPIO_NUM_18

#define CRSF_BAUD    420000
static const char *TAG = "CRSF_BRIDGE";

enum Link { LINK_24 = 0, LINK_900 = 1 };
static Link activeLink = LINK_24;

unsigned long lastPkt24  = 0;
unsigned long lastPkt900 = 0;
const unsigned long LINK_TIMEOUT = 300;


// ---------------- UART TASK ----------------
void uart_task(void *p)
{
    printf("UART task started.\n");
    uint8_t data[256];
    ESP_LOGI(TAG, "UART task started.");

    while (true)
    {
        int64_t now = esp_timer_get_time() / 1000;

    // ---------- READ 2.4 GHZ ----------
    int len24 = uart_read_bytes(UART_NUM_1, data, sizeof(data), 10 / portTICK_PERIOD_MS);
    printf("2.4 GHz read: %d bytes\n", len24);

    if (len24 > 0) {
        lastPkt24 = now;

        if (activeLink == LINK_24)
            uart_write_bytes(UART_NUM_0, (const char *)data, len24);
    }

// ---------- READ 900 MHZ ----------
int len900 = uart_read_bytes(UART_NUM_2, data, sizeof(data), 10 / portTICK_PERIOD_MS);
printf("900 MHz read: %d bytes\n", len900);

if (len900 > 0) {
    lastPkt900 = now;

    if (activeLink == LINK_900)
        uart_write_bytes(UART_NUM_0, (const char *)data, len900);
}


        // ---------- DECISION LOGIC ----------
        bool alive24  = (now - lastPkt24)  < LINK_TIMEOUT;
        bool alive900 = (now - lastPkt900) < LINK_TIMEOUT;

        Link newLink = activeLink;

        if (alive24 && !alive900){
            newLink = LINK_24;
            printf("Switching to 2.4 GHz\n");
        }
        else if (!alive24 && alive900){
            newLink = LINK_900;
            printf("Switching to 900 MHz\n");
        }
        else if (alive24 && alive900){
            newLink = LINK_24; 
            printf("Both links alive, prefer 2.4 GHz\n");
        } 

        if (newLink != activeLink) {
            activeLink = newLink;

            gpio_set_level(RF_SELECT, (activeLink == LINK_900) ? 1 : 0);

            ESP_LOGW(TAG,
                     "LINK SWITCH → %s",
                     (activeLink == LINK_24) ? "2.4 GHz" : "900 MHz");
            printf("LINK SWITCH → %s\n",
                     (activeLink == LINK_24) ? "2.4 GHz" : "900 MHz");
        }

        // Optional slow heartbeat
        static uint32_t lastPrint = 0;
        if (now - lastPrint > 2000) {
            lastPrint = now;
            ESP_LOGI(TAG,
                     "Alive: 2.4=%d | 900=%d | Active=%s",
                     alive24,
                     alive900,
                     (activeLink == LINK_24) ? "2.4" : "900");
            printf("Alive: 2.4=%d | 900=%d | Active=%s\n",
                     alive24,
                     alive900,
                        (activeLink == LINK_24) ? "2.4" : "900");
        }
    }
}


// ---------------- START FUNCTION ----------------
extern "C" void rx_sniffer_start()
{
    ESP_LOGI(TAG, "Starting CRSF bridge...");

    // Configure GPIO for RF switch
    gpio_reset_pin(RF_SELECT);
    gpio_set_direction(RF_SELECT, GPIO_MODE_OUTPUT);
    gpio_set_level(RF_SELECT, 0);

    // ---- UART CONFIG ----
    uart_config_t uart_config = {
        .baud_rate = CRSF_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
    };

    // FC UART0
    uart_driver_install(UART_NUM_0, 4096, 4096, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_FC_TX, UART_FC_RX, -1, -1);

    // 2.4 UART1
    uart_driver_install(UART_NUM_1, 4096, 4096, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_24_TX, UART_24_RX, -1, -1);

    // 900 UART2
    uart_driver_install(UART_NUM_2, 4096, 4096, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, UART_900_TX, UART_900_RX, -1, -1);

    ESP_LOGI(TAG, "UARTs initialized.");

    // Start RTOS task
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
}
