#ifndef WIFI_CONFIG_H_
#define WIFI_CONFIG_H_

/* Default WIFI name or AP SSID */
#define DEFAULT_WIFI_SSID_NAME                             "Enter WIFI name here"
/* Default security type, typically either:
 * SL_WLAN_SEC_TYPE_OPEN or SL_WLAN_SEC_TYPE_WPA_WPA2 */
#define DEFAULT_WIFI_SECURITY_TYPE                         SL_WLAN_SEC_TYPE_WPA_WPA2
/* Default password for WIFI or the secured AP */
#define DEFAULT_WIFI_SECURITY_KEY                          "Enter WIFI password here"

/* Maximum length of WIFI name or password */
#ifndef MAX_SSID_LENGTH
#define MAX_SSID_LENGTH                                     (32)
#endif /* MAX_SSID_LENGTH */

/* Default IP Address of desktop computer running server (Python script or Iperf) */
#define SERVER_DEST_IP_ADDR                                 SL_IPV4_VAL(192,168,39,200)
/* Default port for server (5001 for Iperf server) */
#define SERVER_PORT                                         (5001)
/* Maximum transmission buffer size used by both TCP Client and server */
#define SERVER_FRAME_LENGTH                                 (80)

#endif /* WIFI_CONFIG_H_ */
