#include "rx_sniffer.h"
#include "esp_log.h"

extern "C" void app_main(void)
{
    ESP_LOGI("MAIN", "Starting CRSF multiplexer...");
    rx_sniffer_start();
}
