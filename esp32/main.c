#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include <arpa/inet.h>

#define WIFI_SSID "**********"
#define WIFI_PASS "**********"
#define BROKER_URI "mqtt://192.168.0.42:1883"

static bool wifi_connected = false;
static esp_mqtt_client_handle_t client;
static adc_oneshot_unit_handle_t adc_handle;
static TaskHandle_t adc_task_handle = NULL;
static TimerHandle_t adc_timer_handle = NULL;

/**
 * @brief Reads ADC value from the given channel and publishes it over MQTT.
 * 
 * @param ch The ADC channel to read from (e.g., 0, 3, 4, 7).
 */
void read_and_send(int ch)
{
    int val;
    int pr;
    esp_err_t ret;

    // read ADC
    switch (ch)
    {
    case 0:
        ret = adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &val);
        pr = 1;
        break;
    case 3:
        ret = adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &val);
        pr = 2;
        break;
    case 4:
        ret = adc_oneshot_read(adc_handle, ADC_CHANNEL_4, &val);
        pr = 3;
        break;
    case 7:
        ret = adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &val);
        pr = 4;
        break;
    default:
        ESP_LOGE("ADC", "Unknown ADC Channel %d", ch);
        return;
    }
    // publish ADC values
    if (ret == ESP_OK)
    {
        ESP_LOGI("ADC", "ADC channel %d: %d", ch, val);
        if (client)
        {
            char str[16];
            sprintf(str, "%d", val);
            char topic[17];
            sprintf(topic, "/sensors/photo_%d", pr);
            esp_mqtt_client_publish(client, topic, str, 0, 1, 0);
        }
    }
    else
    {
        ESP_LOGE("ADC", "ADC channel %d read failed", ch);
    }
}

/**
 * @brief FreeRTOS task that continuously polls multiple ADC channels and publishes the values.
 * 
 * @param pvParam Pointer to task parameters (unused).
 */
void adc_task(void *pvParam)
{
    // continually poll all 4 ADCs
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        read_and_send(0);
        read_and_send(3);
        read_and_send(4);
        read_and_send(7);
    }
}

/**
 * @brief Timer callback that notifies the ADC task to perform a reading cycle.
 * 
 * @param xTimer Handle to the timer that triggered the callback.
 */
void adc_timer_callback(TimerHandle_t xTimer)
{
    if (adc_task_handle != NULL)
    {
        xTaskNotifyGive(adc_task_handle);
    }
}

/**
 * @brief Initializes the ADC oneshot driver and configures the ADC channels.
 */
static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t ch_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &ch_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &ch_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_4, &ch_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &ch_config));
}

/**
 * @brief Callback function that handles MQTT client events.
 * 
 * @param handler_args User-defined argument (unused).
 * @param base Event base ID.
 * @param event_id Specific MQTT event ID.
 * @param event_data Pointer to event-specific data.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT", "MQTT Connected");
        esp_mqtt_client_publish(client, "/sensors/test", "this is a test", 0, 1, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGE("MQTT", "MQTT Disconnected");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE("MQTT", "MQTT ERROR type: %d", event->error_handle->error_type);
        break;
    default:
        ESP_LOGI("MQTT", "Other event: %d", event->event_id);
        break;
    }
}

/**
 * @brief Callback invoked when the device receives an IP address.
 * 
 * @param arg User-defined argument (unused).
 * @param event_base The base of the received event.
 * @param event_id The specific event ID.
 * @param event_data Event-specific data (expected to be of type ip_event_got_ip_t).
 */
static void on_got_ip(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if (event && event->ip_info.ip.addr != 0)
    {
        if ((ntohl(event->ip_info.ip.addr) & 0xFFFFFF00) != 0xC0A80000)
        {
            ESP_LOGW("WIFI", "Got IP event, but IP is " IPSTR, IP2STR(&event->ip_info.ip));
        }
        else
        {
            ESP_LOGI("WIFI", "Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
            wifi_connected = true;
        }
    }
    else
    {
        ESP_LOGW("WIFI", "⚠️  Got IP event, but IP is 0.0.0.0");
    }
}

/**
 * @brief Initializes Wi-Fi in station mode and connects to the configured network.
 */
static void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &on_got_ip,
                                        NULL,
                                        &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &on_got_ip,
                                        NULL,
                                        &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI("WIFI", "📡 Wi-Fi initialization complete, connecting...");
    esp_wifi_connect();
}

/**
 * @brief Main application entry point.
 * 
 * Initializes NVS, Wi-Fi, MQTT, and ADC, then starts the ADC polling task and timer.
 */
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT", ESP_LOG_VERBOSE);
    esp_log_level_set("WIFI", ESP_LOG_VERBOSE);
    esp_log_level_set("ADC", ESP_LOG_VERBOSE);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init_sta();

    int retries = 30;
    while (!wifi_connected && retries--)
    {
        ESP_LOGI("WIFI", "Waiting for IP... (%d)", retries);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!wifi_connected)
    {
        ESP_LOGE("WIFI", "Failed to connect to Wi-Fi. Check SSID/password and signal.");
        return;
    }

    ESP_LOGI("MQTT", "Starting MQTT Client...");

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URI};

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    init_adc();

    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, &adc_task_handle);

    adc_timer_handle = xTimerCreate("ADC", pdMS_TO_TICKS(1000), pdTRUE, NULL, adc_timer_callback);
    if (adc_timer_handle)
    {
        xTimerStart(adc_timer_handle, 0);
    }
}
