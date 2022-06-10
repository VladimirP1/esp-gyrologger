extern "C" {
#include "http.h"

#include <esp_http_server.h>
#include <esp_event.h>
#include <esp_log.h>

#include <stdio.h>

#include "global_context.h"
}

static const char* TAG = "http-server";

#include "http_strings.hpp"

static esp_err_t download_get_handler(httpd_req_t* req) {
    FILE* f = fopen("/spiflash/gyro.bin", "rb");
    char* buf3 = (char*)malloc(1024);

    for (int i = 0; i < 1000; ++i) {
        int read = fread(buf3, 1, 1024, f);
        if (!read) {
            httpd_resp_send_chunk(req, NULL, 0);
            break;
        }
        httpd_resp_send_chunk(req, buf3, read);
    }

    free(buf3);
    fclose(f);

    return ESP_OK;
}

static const httpd_uri_t download_get = {
    .uri = "/download", .method = HTTP_GET, .handler = download_get_handler, NULL};

static esp_err_t root_get_handler(httpd_req_t* req) {
    httpd_resp_send(req, testhtml, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static const httpd_uri_t root_get = {
    .uri = "/", .method = HTTP_GET, .handler = root_get_handler, NULL};

static esp_err_t root_post_handler(httpd_req_t* req) {
    char content[100];
    size_t recv_size = MIN(req->content_len, sizeof(content));

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    if (strncmp(content, "command=record", 14) == 0) {
        ESP_LOGI(TAG, "Start logging!");
        if (!xSemaphoreTake(ctx.logger_control.mutex, 10)) {
            return ESP_FAIL;
        }

        ctx.logger_control.active = true;

        xSemaphoreGive(ctx.logger_control.mutex);
    }

    if (strncmp(content, "command=stop", 12) == 0) {
        ESP_LOGI(TAG, "Stop logging!");
        if (!xSemaphoreTake(ctx.logger_control.mutex, 10)) {
            return ESP_FAIL;
        }

        ctx.logger_control.active = false;

        xSemaphoreGive(ctx.logger_control.mutex);
    }

    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    
    return httpd_resp_send(req, "", HTTPD_RESP_USE_STRLEN);
}

static const httpd_uri_t root_post = {
    .uri = "/", .method = HTTP_POST, .handler = root_post_handler, NULL};

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &root_get);
        httpd_register_uri_handler(server, &download_get);
        httpd_register_uri_handler(server, &root_post);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server) { return httpd_stop(server); }

extern "C" {
void http_init() {
    static httpd_handle_t server = NULL;

    server = start_webserver();
}
}