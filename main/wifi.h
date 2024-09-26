#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"
#include <string.h>
#include "esp_smartconfig.h"
#include <errno.h>
#include "protocol_examples_common.h"
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "mqtt_client.h"
#include "freertos/semphr.h"

#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

//#define HOST_IP_ADDR "192.168.3.115"  // 服务器的IP地址
#define PORT 7230                      // 服务器监听的端口
#define TAG "TCP_CLIENT"

#define wifi_ssid "daydayup_Wi-Fi5"
#define wifi_pass "88888888"
#define MAX_RETRY  7

static const int CONNECTED_BIT = BIT0;

//smartconfig完成事件
static const int ESPTOUCH_DONE_BIT = BIT1;



#define NVS_WIFI_NAMESPACE_NAME         "DEV_WIFI"
#define NVS_SSID_KEY                    "ssid"
#define NVS_PASSWORD_KEY                "password"

//缓存一份ssid
static char s_ssid_value[33] = {0};

//缓存一份password
static char s_password_value[65] = {0};

//用一个标志来表示是否处于smartconfig中
static bool s_is_smartconfig = false;
//MQTT
static esp_mqtt_client_handle_t mqtt_handle = NULL;
#define MQTT_addr "mqtt://broker-cn.emqx.io"
#define MQTT_port 1883
#define MQTT_client_id "sync_esp"
#define MQTT_USERNAME "esp32"
#define MQTT_PASSWORD "esp32pass"
#define MQTT_TOPIC1 "/topic/esp32_114"//esp32 send
#define MQTT_TOPIC2 "/topic/esp32_514"//esp32 receieve

static SemaphoreHandle_t s_wifi_connect_sem = NULL;


void wifi_init();