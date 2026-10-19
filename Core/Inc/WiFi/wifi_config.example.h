/* Copy to wifi_config.local.h and configure your own network and TCP receiver.
 * wifi_config.local.h is intentionally ignored by Git.
 */
#ifndef STEP_WIFI_CONFIG_H
#define STEP_WIFI_CONFIG_H

#define STEP_WIFI_SSID "YOUR_WIFI_SSID"
#define STEP_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
/* Documentation-only address. Replace all four octets with your receiver IP. */
#define STEP_WIFI_REMOTE_IP {192, 0, 2, 1}
#define STEP_WIFI_REMOTE_PORT 8080

#endif /* STEP_WIFI_CONFIG_H */
