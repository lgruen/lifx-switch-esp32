#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "driver/touch_pad.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"

#include "lifx.h"

// Which touch pad connection to use.
// Touch0 corresponds to pin IO4 on the ESP32 DevKitC V4.
#define TOUCH_PAD_INDEX 0

static const char* TAG = "Light switch";

// Set CONFIG_ESP_WIFI_SSID and CONFIG_ESP_WIFI_PASSWORD using menuconfig
// to configure the WiFi network.

// FreeRTOS event group to signal when we are connected.
static EventGroupHandle_t s_wifi_event_group;

// The event group allows multiple bits for each event, but we only care
// about one event -- are we connected to the AP with an IP?
const int WIFI_CONNECTED_BIT = BIT0;
static bool wifi_connected = false;

static bool lifx_initialized = false;
static bulb_service_t** bulbs;
static int bulb_index = -1;

static bool touched = false;
static int iters_since_last_touched = 0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    ESP_LOGI(TAG, "retry to connect to the AP");
    wifi_connected = false;
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->ip_info.ip));
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    wifi_connected = true;
  }
}

void wifi_init_sta() {
  s_wifi_event_group = xEventGroupCreate();

  tcpip_adapter_init();

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &wifi_event_handler, NULL));

  wifi_config_t wifi_config = {
      .sta = {.ssid = CONFIG_ESP_WIFI_SSID,
              .password = CONFIG_ESP_WIFI_PASSWORD},
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");
  ESP_LOGI(TAG, "Connect to AP SSID: %s", CONFIG_ESP_WIFI_SSID);
}

static void print_bulb(bulb_service_t* bulb) {
  ESP_LOGI(TAG,
           "Bulb\n"
           "    target: 0x%llx\n"
           "    service: %d\n"
           "    port: %d\n",
           bulb->target, bulb->service, bulb->port);
}

// Converts a MAC address string like "D0:73:D5:50:EF:20"
// to the corresponding integer (e.g. 0x20ef50d573d0).
static uint64_t mac_string_to_uint64(const char* mac_str) {
  unsigned char mac[6];
  if (sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[5], &mac[4],
             &mac[3], &mac[2], &mac[1], &mac[0]) != 6) {
    ESP_LOGE(TAG, "Couldn't parse MAC address");
    abort();
  }
  uint64_t result = 0;
  for (int i = 0; i < 6; ++i) {
    result <<= 8;
    result |= (uint64_t)mac[i];
  }
  return result;
}

static bool init_lifx() {
  ESP_LOGI(TAG, "Initializing LIFX library");
  int err = init_lifx_lib();
  if (err != 0) {
    ESP_LOGE(TAG, "init_lifx_lib error: %d", err);
    return false;
  }
  err = discoverBulbs(&bulbs);
  if (err != 0) {
    ESP_LOGE(TAG, "discoverBulbs error: %d", err);
    return false;
  }

  const uint64_t bulb_mac = mac_string_to_uint64(CONFIG_ESP_LIFX_MAC);
  ESP_LOGI(TAG, "Looking for bulb with MAC address %s (0x%llx)",
           CONFIG_ESP_LIFX_MAC, bulb_mac);
  for (int i = 0; bulbs[i] != NULL; ++i) {
    print_bulb(bulbs[i]);
    if (bulbs[i]->target == bulb_mac) {
      bulb_index = i;
      ESP_LOGI(TAG, "Bulb found at index %d", i);
      break;
    }
  }

  if (bulb_index == -1) {
    ESP_LOGE(TAG, "Couldn't find bulb");
    return false;
  }

  return true;
}

static void toggle_light() {
  bool on;
  int err = getPower(bulbs[bulb_index], &on);
  if (err != 0) {
    ESP_LOGE(TAG, "getPower error: %d", err);
    return;
  }
  ESP_LOGI(TAG, "Current state: %s", on ? "on" : "off");

  err = setPower(bulbs[bulb_index], !on, 500);
  if (err != 0) {
    ESP_LOGE(TAG, "setPower error: %d", err);
    return;
  }
}

static void tp_read_task(void* pvParameter) {
  while (true) {
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Lazily initialize LIFX once we have an IP address.
    if (!lifx_initialized) {
      if (!wifi_connected) {  // Wait until connected.
        continue;
      }
      if (init_lifx()) {
        lifx_initialized = true;
      } else {
        abort();
      }
    }

    // Ignore any subsequent touches to debounce.
    if (iters_since_last_touched < 20) {
      ++iters_since_last_touched;
      touched = false;
      continue;
    }

    if (touched) {
      ESP_LOGI(TAG, "Touch registered");
      iters_since_last_touched = 0;
      touched = false;

      toggle_light();
    }
  }
}

static void tp_rtc_intr(void* arg) {
  uint32_t pad_intr = touch_pad_get_status();
  touch_pad_clear_status();                       // Clear interrupt.
  touched = ((pad_intr >> TOUCH_PAD_INDEX) & 1);  // Check touch pad.
}

void app_main() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "Connecting to WiFi");
  wifi_init_sta();

  ESP_LOGI(TAG, "Initializing touch pad");
  touch_pad_init();
  touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
  // Set reference voltage for charging/discharging.
  // For most usage scenarios, we recommend using the following combination:
  // the high reference valtage will be 2.7V - 1V = 1.7V, The low reference
  // voltage will be 0.5V.
  touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
  // Init touch pad IO.
  touch_pad_config(TOUCH_PAD_INDEX, 0);
  // Set up interrupt threshold as fraction of initial reading.
  touch_pad_filter_start(10);
  uint16_t touch_value;
  touch_pad_read_filtered(TOUCH_PAD_INDEX, &touch_value);
  ESP_LOGI(TAG, "initial touch pad reading is %d", touch_value);
  ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_PAD_INDEX, touch_value * 2 / 3));
  // Register touch interrupt ISR.
  touch_pad_isr_register(tp_rtc_intr, NULL);
  touch_pad_intr_enable();
  // Start a task to show what pads have been touched.
  xTaskCreate(&tp_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
}
