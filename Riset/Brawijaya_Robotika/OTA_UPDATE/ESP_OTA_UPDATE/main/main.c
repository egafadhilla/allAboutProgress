#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_https_ota.h"
#include "mdns.h"
#include "nvs_flash.h"

#define TAG "OTA_ESP"
#define SERVICE_NAME "sumo_bot"
#define OTA_USERNAME "admin"
#define OTA_PASSWORD "password123"

static httpd_handle_t server = NULL;

/* Handler untuk upload firmware */
static esp_err_t ota_update_handler(httpd_req_t *req) {
    char buf[1024];
    int received;
    esp_ota_handle_t ota_handle;
    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
    
    // Autentikasi Basic
    char auth_header[100];
    if (httpd_req_get_hdr_value_str(req, "Authorization", auth_header, sizeof(auth_header)) == ESP_OK) {
        if (strstr(auth_header, "Basic ") != auth_header || 
            strcmp(auth_header+6, "YWRtaW46cGFzc3dvcmQxMjM=") != 0) { // "admin:password123" base64
            httpd_resp_set_status(req, "401 Unauthorized");
            httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"OTA Update\"");
            return httpd_resp_send(req, NULL, 0);
        }
    } else {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"OTA Update\"");
        return httpd_resp_send(req, NULL, 0);
    }

    ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));
    
    while ((received = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        esp_ota_write(ota_handle, buf, received);
    }
    
    if (esp_ota_end(ota_handle) == ESP_OK) {
        ESP_LOGI(TAG, "OTA Success!");
        esp_ota_set_boot_partition(ota_partition);
        httpd_resp_set_status(req, "200 OK");
        httpd_resp_send(req, "Update Success! Rebooting...", HTTPD_RESP_USE_STRLEN);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA Failed!");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA Failed");
    }
    return ESP_OK;
}

/* Web Interface Sederhana */
static const char* html_form = 
"<html><body>"
"<h1>Sumo Bot OTA Update</h1>"
"<form method='POST' action='/update' enctype='multipart/form-data'>"
"<input type='file' name='firmware'>"
"<input type='submit' value='Upload'>"
"</form></body></html>";

static esp_err_t root_handler(httpd_req_t *req) {
    return httpd_resp_send(req, html_form, HTTPD_RESP_USE_STRLEN);
}

/* Start Web Server */
void start_web_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler
    };
    
    httpd_uri_t ota_uri = {
        .uri = "/update",
        .method = HTTP_POST,
        .handler = ota_update_handler
    };
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &ota_uri);
    }
}

/* mDNS Initialization */
void init_mdns() {
    mdns_init();
    mdns_hostname_set(SERVICE_NAME);
    mdns_instance_name_set("Sumo Bot Controller");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
}

void app_main(void) {
    // Inisialisasi NVS dan WiFi
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    init_mdns();
    start_web_server();
    
    ESP_LOGI(TAG, "OTA Service Ready! Access via http://" SERVICE_NAME ".local");
}