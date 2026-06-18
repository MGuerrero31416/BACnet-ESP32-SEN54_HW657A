#ifndef WIFI_HELPER_H
#define WIFI_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/**
 * Initialize WiFi in station mode and connect to the configured network.
 * Blocks until connected or timeout occurs.
 */
void wifi_init_sta(void);

/**
 * Store Wi-Fi credentials in application NVS namespace.
 * These values take priority over compile-time defaults on next boot.
 */
esp_err_t wifi_save_credentials_nvs(const char *ssid, const char *password);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_HELPER_H */
