#include "wifi.h"


// NVS SAVE

static size_t read_nvs_ssid(char* ssid,int maxlen)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret_val = ESP_FAIL;
    size_t required_size = 0;
    ESP_ERROR_CHECK(nvs_open(NVS_WIFI_NAMESPACE_NAME, NVS_READWRITE, &nvs_handle));
    ret_val = nvs_get_str(nvs_handle, NVS_SSID_KEY, NULL, &required_size);
    if(ret_val == ESP_OK && required_size <= maxlen)
    {
        nvs_get_str(nvs_handle,NVS_SSID_KEY,ssid,&required_size);
    }
    else
        required_size = 0;
    nvs_close(nvs_handle);
    return required_size;
}

/** 写入SSID到NVS中
 * @param ssid 需写入的ssid
 * @return ESP_OK or ESP_FAIL
*/
static esp_err_t write_nvs_ssid(char* ssid)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    ESP_ERROR_CHECK(nvs_open(NVS_WIFI_NAMESPACE_NAME, NVS_READWRITE, &nvs_handle));
    
    ret = nvs_set_str(nvs_handle, NVS_SSID_KEY, ssid);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return ret;
}

/** 从NVS中读取PASSWORD
 * @param ssid 读到的password
 * @param maxlen 外部存储password数组的最大值
 * @return 读取到的字节数
*/
static size_t read_nvs_password(char* pwd,int maxlen)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret_val = ESP_FAIL;
    size_t required_size = 0;
    ESP_ERROR_CHECK(nvs_open(NVS_WIFI_NAMESPACE_NAME, NVS_READWRITE, &nvs_handle));
    ret_val = nvs_get_str(nvs_handle, NVS_PASSWORD_KEY, NULL, &required_size);
    if(ret_val == ESP_OK && required_size <= maxlen)
    {
        nvs_get_str(nvs_handle,NVS_SSID_KEY,pwd,&required_size);
    }
    else 
        required_size = 0;
    nvs_close(nvs_handle);
    return required_size;
}

/** 写入PASSWORD到NVS中
 * @param pwd 需写入的password
 * @return ESP_OK or ESP_FAIL
*/
static esp_err_t write_nvs_password(char* pwd)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    ESP_ERROR_CHECK(nvs_open(NVS_WIFI_NAMESPACE_NAME, NVS_READWRITE, &nvs_handle));
    ret = nvs_set_str(nvs_handle, NVS_PASSWORD_KEY, pwd);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return ret;
}
// data transmit

void MQTT_event_cb(void* event_handler_arg,esp_event_base_t event_base,int32_t event_id,void* event_data)
{
    esp_mqtt_event_handle_t data = (esp_mqtt_event_handle_t)event_data;
    switch (event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT","CONNECT");
        esp_mqtt_client_subscribe_single(mqtt_handle,MQTT_TOPIC2,1);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI("MQTT","DISCONNECT");

        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT","PUBLISHEED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI("MQTT","SUBSCRIBED");
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI("MQTT","TOPIC->%s",data->topic);
        ESP_LOGI("MQTT","payload->%s",data->data);
        break;
    default:
        break;
    }
}





void MQTT_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {0};
    mqtt_cfg.broker.address.uri=MQTT_addr;
    mqtt_cfg.broker.address.port=MQTT_port;
    mqtt_cfg.credentials.client_id=MQTT_client_id;
    mqtt_cfg.credentials.username=MQTT_USERNAME;
    mqtt_cfg.credentials.authentication.password=MQTT_PASSWORD;

    mqtt_handle=esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(mqtt_handle,MQTT_EVENT_ANY,MQTT_event_cb,NULL);

    esp_mqtt_client_start(mqtt_handle);

}


static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    // Failed to retransmit, giving up
                    return;
                }
                to_write -= written;
            }
        }
    } while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

#ifdef CONFIG_EXAMPLE_IPV4
    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
#ifdef CONFIG_EXAMPLE_IPV4
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
#ifdef CONFIG_EXAMPLE_IPV6
        if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}


void wifi_event_handle(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if(event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START://sta设置成功
            esp_wifi_connect();//连接wifi
            break;
        case WIFI_EVENT_STA_CONNECTED://连接wifi成功
            ESP_LOGI("WIFI","SUCESSFULLY CONNECTED");
            break;
        case WIFI_EVENT_STA_DISCONNECTED://wifi断开连接
        {
           static int retry_count = 0;
            if (retry_count < MAX_RETRY) {
        esp_wifi_connect();
        ESP_LOGI("WIFI", "Retrying connection... (%d)", ++retry_count);
         } 
         else {
        ESP_LOGI("WIFI", "Exceeded max retry attempts");
         }
            break;
        }
            break;
        default:
            break;
        }
    }
    else if(event_base == IP_EVENT)
    {
        switch (event_id)
        {
        case IP_EVENT_STA_GOT_IP://获取IP成功
            ESP_LOGI("WIFI","SUCESSFULLY GOT IP");
            MQTT_start();
            //xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL,1);
            break;

        default:
            break;
        }
    }
    else if(event_base == SC_EVENT)
    {
        switch (event_id)
        {
        case SC_EVENT_SCAN_DONE://SC扫描成功
            ESP_LOGI("WIFI","SC SCAN DONE");
            break;
        case SC_EVENT_GOT_SSID_PSWD://SC获取到ssid以及密码
        {
            smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t*)event_data;
            wifi_config_t wifi_config ={0};
            memset(&wifi_config,0,sizeof(wifi_config));
            snprintf((char*)wifi_config.sta.ssid,sizeof(wifi_config.sta.ssid),"%s",(char*)evt->ssid);
            snprintf((char*)wifi_config.sta.password,sizeof(wifi_config.sta.password),"%s",(char*)evt->password);
            wifi_config.sta.bssid_set=evt->bssid_set;
            if(wifi_config.sta.bssid_set)
            {
                memcpy(wifi_config.sta.bssid,evt->bssid,6);
            }
            write_nvs_ssid((char*)wifi_config.sta.ssid);
            write_nvs_password((char*)wifi_config.sta.password);

            esp_wifi_disconnect();
            esp_wifi_set_config(WIFI_IF_STA,&wifi_config);
            esp_wifi_connect();
            break;
        } 
        case SC_EVENT_SEND_ACK_DONE:
            ESP_LOGI("SC","SC_DONE");
            esp_smartconfig_stop();
            break;
        case SC_EVENT_FOUND_CHANNEL:
            ESP_LOGI("SC","FOUND CHANNEL");
            break;
        default:
            break;
        }
    }
}

void wifi_direct_connect()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());//TCP协议栈初始化
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_config=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));

    wifi_config_t wifi_config1 = {
        .sta = {
            .ssid = wifi_ssid,
            .password = wifi_pass,
            .threshold={
                .authmode=WIFI_AUTH_WPA2_PSK
            }
        },
    };
    esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event_handle,NULL);
    esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,wifi_event_handle,NULL);
    esp_event_handler_register(SC_EVENT,ESP_EVENT_ANY_ID,wifi_event_handle,NULL);//事件注册
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config1) );
    ESP_ERROR_CHECK(esp_wifi_start() );

}





void wifi_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());//TCP协议栈初始化
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_config=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));

    esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event_handle,NULL);
    esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,wifi_event_handle,NULL);
    esp_event_handler_register(SC_EVENT,ESP_EVENT_ANY_ID,wifi_event_handle,NULL);//事件注册
  
    esp_wifi_set_mode(WIFI_MODE_STA);
    
    esp_wifi_start();
    
    esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS);//设置SC版本
    smartconfig_start_config_t sc_cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&sc_cfg));
    ESP_LOGI("SC","FINISH");
}