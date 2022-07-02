// SPDX-License-Identifier: LGPL-2.1-or-later

#include "http.hpp"

extern "C" {
#include <esp_http_server.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_partition.h>
#include <ff.h>

#include <sys/types.h>
#include <dirent.h>

#include "storage/storage_fat.h"
#include "spi_flash_chip_driver.h"
}
#include <cstdio>

#include "global_context.hpp"

#include "compression/lib/compression.hpp"

#include <string>

static const char* TAG = "http-server";

#include "http_strings.hpp"

#define HANDLE(x)        \
    if ((x) != ESP_OK) { \
        return ESP_FAIL; \
    }

#define HANDLE_FINALLY(x, y) \
    if ((x) != ESP_OK) {     \
        y;                   \
        return ESP_FAIL;     \
    }

static esp_err_t respond_with_file_raw(httpd_req_t* req, const char* filename) {
    char buf2[256];
    int bytes_total = 0;

    gctx.pause_polling = true;

    ESP_LOGI(TAG, "Reading file");
    FILE* f = fopen(filename, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }

    fseek(f, 0L, SEEK_END);
    bytes_total = ftell(f);
    ESP_LOGI(TAG, "file size is %d bytes", bytes_total);
    fseek(f, 0L, SEEK_SET);

    while (true) {
        int read_bytes = fread(buf2, 1, MIN(bytes_total, 256), f);

        if (read_bytes < 0) {
            ESP_LOGE(TAG, "read error");
            break;
        }

        HANDLE_FINALLY(httpd_resp_send_chunk(req, buf2, read_bytes), fclose(f);
                       gctx.continue_polling = true);
    }

    fclose(f);

    HANDLE_FINALLY(httpd_resp_send_chunk(req, NULL, 0), gctx.continue_polling = true);

    gctx.continue_polling = true;

    return ESP_OK;
}

static esp_err_t respond_with_file(httpd_req_t* req, const char* filename) {
    static uint8_t buf2[2000];
    static char buf_text[4096];

    gctx.pause_polling = true;

    const double sample_rate = 1.0 / 0.00180;
    const double gscale = 1 / 0.00053263221;

    HANDLE(httpd_resp_send_chunk(req, R"--(GYROFLOW IMU LOG
version,1.1
id,esplog
orientation,YxZ
tscale,0.00180
gscale,0.00053263221
ascale,0.0001
t,gx,gy,gz,ax,ay,ax
)--",
                                 HTTPD_RESP_USE_STRLEN));

    ESP_LOGI(TAG, "Reading file");
    FILE* f = fopen(filename, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }

    fseek(f, 0L, SEEK_END);
    ESP_LOGI(TAG, "file size is %ld bytes", ftell(f));
    fseek(f, 0L, SEEK_SET);

    Coder decoder(kBlockSize);

    int have_bytes = 0;
    int time = 0;
    quat::quat prev_quat{};
    char* wptr = buf_text;
    while (true) {
        int read_bytes = fread(buf2 + have_bytes, 1, 2000 - have_bytes, f);
        have_bytes += read_bytes;

        if (have_bytes < 2000) break;

        auto [decoded_bytes, dquats, scale] = decoder.decode_block(buf2);
        for (auto& q : dquats) {
            quat::vec rv = (q.conj() * prev_quat).axis_angle();
            quat::vec gravity = q.conj().rotate_point(
                {quat::base_type{}, quat::base_type{}, quat::base_type{-1.0}});

            prev_quat = q;
            if (time != 0) {
                double scale = sample_rate * gscale;
                double ascale = 10000;
                int size =
                    snprintf(wptr, sizeof(buf_text) - (wptr - buf_text), "%d,%d,%d,%d,%d,%d,%d\n",
                             time, (int)(double(rv.x) * scale), (int)(double(rv.y) * scale),
                             (int)(double(rv.z) * scale), (int)(double(gravity.x) * ascale),
                             (int)(double(gravity.y) * ascale), (int)(double(gravity.z) * ascale));
                wptr += size;
            }
            time++;

            if (wptr - buf_text > 3200) {
                HANDLE_FINALLY(httpd_resp_send_chunk(req, buf_text, HTTPD_RESP_USE_STRLEN),
                               fclose(f);
                               gctx.continue_polling = true);
                wptr = buf_text;
            }
        }

        have_bytes -= decoded_bytes;
        ESP_LOGI(TAG, "%d block uncompressed, scale = %d", decoded_bytes, scale);

        taskYIELD();
        memmove(buf2, buf2 + decoded_bytes, have_bytes);
    }
    fclose(f);

    if (wptr != buf_text) {
        HANDLE_FINALLY(httpd_resp_send_chunk(req, buf_text, HTTPD_RESP_USE_STRLEN),
                       gctx.continue_polling = true);
    }

    HANDLE_FINALLY(httpd_resp_send_chunk(req, NULL, 0), gctx.continue_polling = true);

    gctx.continue_polling = true;

    return ESP_OK;
}

static esp_err_t download_get_handler(httpd_req_t* req) {
    char buf[128];
    int buf_len = httpd_req_get_url_query_len(req) + 1;
    if (gctx.logger_control.busy || buf_len > sizeof(buf)) {
        return ESP_FAIL;
    }
    if (buf_len > 1 && httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
        ESP_LOGI(TAG, "Found URL query => %s", buf);
        char param[32];
        bool raw_mode = false;
        if (httpd_query_key_value(buf, "raw", param, sizeof(param)) == ESP_OK) {
            ESP_LOGI(TAG, "raw_mode=%s", param);
            raw_mode = atoi(param);
        }
        if (httpd_query_key_value(buf, "name", param, sizeof(param)) == ESP_OK) {
            ESP_LOGI(TAG, "want to download %s", param);
            snprintf(buf, sizeof(buf), "/spiflash/%s", param);
            if (raw_mode) {
                return respond_with_file_raw(req, buf);
            } else {
                httpd_resp_set_hdr(req, "content-disposition", "attachment;filename=gyro.gcsv");
                return respond_with_file(req, buf);
            }
        }
    }

    return ESP_FAIL;
}

static const httpd_uri_t download_get = {
    .uri = "/download", .method = HTTP_GET, .handler = download_get_handler, .user_ctx = NULL};

static esp_err_t format_get_handler(httpd_req_t* req) {
    const esp_partition_t* storage_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");
    ESP_ERROR_CHECK(esp_partition_erase_range(storage_partition, 0, 4096));
    esp_restart();
    return ESP_OK;
}

static const httpd_uri_t format_get = {
    .uri = "/format", .method = HTTP_GET, .handler = format_get_handler, .user_ctx = NULL};

static std::pair<int, int> get_free_space_kb() {
    constexpr int kSectorBytes = 4096;
    FATFS* fs;
    DWORD fre_clust, fre_sect, tot_sect;
    /* Get volume information and free clusters of drive 0 */
    FRESULT res = f_getfree("0:", &fre_clust, &fs);
    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;
    return {kSectorBytes * fre_sect / 1024, kSectorBytes * tot_sect / 1024};
}

static esp_err_t status_get_handler(httpd_req_t* req) {
    HANDLE(httpd_resp_send_chunk(req, R"--(
    <table class="status_table">
        <tr>
            <td class="status_table_name_cell">Active</td>
            <td class="status_value_table_cell">)--",
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, std::to_string(gctx.logger_control.busy).c_str(),
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, R"--(</td>
        </tr>
        <tr>
            <td class="status_table_name_cell">Avg gyro sample int. (ns)</td>
            <td class="status_value_table_cell">)--",
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req,
                                 std::to_string(gctx.logger_control.avg_sample_interval_ns).c_str(),
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, R"--(</td>
        </tr>
        <tr>
            <td class="status_table_name_cell">Free space (kBytes)</td>
            <td class="status_value_table_cell">)--",
                                 HTTPD_RESP_USE_STRLEN));

    auto free_space = get_free_space_kb();
    HANDLE(httpd_resp_send_chunk(req, std::to_string(free_space.first).c_str(),
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, R"--(</td>
        </tr>
        <tr>
            <td class="status_table_name_cell">Free heap size (kBytes)</td>
            <td class="status_value_table_cell">)--",
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, std::to_string(esp_get_free_heap_size() / 1024).c_str(),
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, R"--(</td>
        </tr>
    <tr>
        <td class="status_table_name_cell">Last log length (samples)</td>
        <td class="status_value_table_cell">)--",
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req,
                                 std::to_string(gctx.logger_control.total_samples_written).c_str(),
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, R"--(</td>
        </tr>
    <tr>
        <td class="status_table_name_cell">Last log length (Bytes)</td>
        <td class="status_value_table_cell">)--",
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req,
                                 std::to_string(gctx.logger_control.total_bytes_written).c_str(),
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, R"--(</td>
        </tr>
        <tr>
            <td class="status_table_name_cell">Last log avg rate (Bytes/min)</td>
            <td class="status_value_table_cell">)--",
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(
        req, std::to_string(gctx.logger_control.avg_logging_rate_bytes_min).c_str(),
        HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, R"--(</td>
        </tr>
    </table>)--",
                                 HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, NULL, 0));
    return ESP_OK;
}

static const httpd_uri_t status_get = {
    .uri = "/status", .method = HTTP_GET, .handler = status_get_handler, .user_ctx = NULL};

static esp_err_t files_get_handler(httpd_req_t* req) {
    HANDLE(httpd_resp_send_chunk(req, "<table class=\"download_table\">", HTTPD_RESP_USE_STRLEN));
    bool busy = true;
    if (xSemaphoreTake(gctx.logger_control.mutex, 2)) {
        busy = gctx.logger_control.busy;
        xSemaphoreGive(gctx.logger_control.mutex);
    }
    if (!busy) {
        DIR* dp;
        struct dirent* ep;

        dp = opendir("/spiflash");
        if (dp != NULL) {
            while ((ep = readdir(dp))) {
                static constexpr char templ[] = "/spiflash/%s";
                char buf[30], buf2[300];
                snprintf(buf, sizeof(buf), templ, ep->d_name);
                FILE* f = fopen(buf, "rb");
                if (f) {
                    fseek(f, 0, SEEK_END);
                    int size_kb = ftell(f) / 1024;
                    fclose(f);

                    if (0) {
                        (void)snprintf(buf2, sizeof(buf2), R"--(<tr class="download_table_name_cell">
            <td><a href="/download?name=%s">%s</a></td>
            <td class="download_table_mid_cell">%dKB</td>)--",
                                 ep->d_name, ep->d_name, size_kb);

                        HANDLE(httpd_resp_send_chunk(req, buf2, HTTPD_RESP_USE_STRLEN));

                        (void)snprintf(buf2, sizeof(buf2), R"--(<td>
                <button style="color:black;" class="delete_btn"  onclick="post_command('unlink=%s')">&#x274c;</button>
            </td>
        </tr>)--",
                                 ep->d_name);
                    } else {
                        (void)snprintf(buf2, sizeof(buf2), R"--(<tr class="download_table_name_cell">
            <td><a href="#" onclick='download_and_decode_log("/download?name=%s&raw=1", "%s.gcsv");return false;'>%s</a></td>
            <td class="download_table_mid_cell">%dKB</td>)--",
                                 ep->d_name, ep->d_name, ep->d_name, size_kb);

                        HANDLE(httpd_resp_send_chunk(req, buf2, HTTPD_RESP_USE_STRLEN));

                        (void)snprintf(buf2, sizeof(buf2), R"--(<td>
                <button style="color:black;" class="delete_btn"  onclick="post_command('unlink=%s')">&#x274c;</button>
            </td>
        </tr>)--",
                                 ep->d_name);
                    }

                    HANDLE(httpd_resp_send_chunk(req, buf2, HTTPD_RESP_USE_STRLEN));
                }
            }
            (void)closedir(dp);
        } else {
            ESP_LOGE(TAG, "Couldn't open the directory");
        }
    } else {
        HANDLE(
            httpd_resp_send_chunk(req, "<tr><td>Logger is BUSY!</td></tr>", HTTPD_RESP_USE_STRLEN));
    }
    HANDLE(httpd_resp_send_chunk(req, "</table>", HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, NULL, 0));
    return ESP_OK;
}

static const httpd_uri_t files_get = {
    .uri = "/files", .method = HTTP_GET, .handler = files_get_handler, .user_ctx = NULL};

static esp_err_t root_get_handler(httpd_req_t* req) {
    HANDLE(httpd_resp_send_chunk(req, html_prefix, HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, R"--(<body><h1>EspLog <div id="network_activity"></div></h1>
    <h2>Control</h2>
    <button style="color:green;" class="command_btn" onclick="post_command('command=calibrate')">0</button>
    <button style="color:red;" class="command_btn" onclick="post_command('command=record')">&#x23fa;</button>
    <button style="color:black;" class="command_btn" onclick="post_command('command=stop')">&#x23F9;</button>

    <h2>Status</h2>
     <div id="status_table">
    Loading...
    </div>
    <h2>Log download</h2>
     <div id="files_table">
    Loading...
    </div>
    <form id="form_simple" method="post" action=""></form>
</body>)--",
                                 HTTPD_RESP_USE_STRLEN));

    HANDLE(httpd_resp_send_chunk(req, html_stylesheet, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, js_xhr_status_updater, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, js_wasm_decoder, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, html_suffix, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, NULL, 0));
    return ESP_OK;
}

static const httpd_uri_t root_get = {
    .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL};

extern "C" {
extern void spi_flash_disable_interrupts_caches_and_other_cpu(void);
extern void spi_flash_enable_interrupts_caches_and_other_cpu(void);
}

typedef struct {
    char* buf;
    size_t buf_size;
} copy_flash_args_t;

void IRAM_ATTR copy_flash_task(void* vargs) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    copy_flash_args_t& args = *static_cast<copy_flash_args_t*>(vargs);
    char* buf = args.buf;
    size_t buf_size = args.buf_size;

    const esp_partition_t* storage_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");

    const esp_partition_t* app_partition =
        esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, "factory");

    uint32_t sector_size = app_partition->flash_chip->chip_drv->sector_size;

    ESP_LOGI(TAG, "sector size = %u", sector_size);
    ESP_LOGI(TAG, "storage partition: %p", storage_partition);
    ESP_LOGI(TAG, "app partition: %p", app_partition);

    ESP_LOGI(TAG, "Starting update!");

    wdt_off();

    spi_flash_disable_interrupts_caches_and_other_cpu();

    for (size_t ofs = 0; ofs + buf_size < app_partition->size; ofs += buf_size) {
        storage_partition->flash_chip->chip_drv->read(storage_partition->flash_chip, buf,
                                                      storage_partition->address + ofs, buf_size);
        if ((app_partition->address + ofs) % sector_size == 0) {
            app_partition->flash_chip->chip_drv->erase_sector(app_partition->flash_chip,
                                                              app_partition->address + ofs);
        }
        storage_partition->flash_chip->chip_drv->write(app_partition->flash_chip, buf,
                                                       app_partition->address + ofs, buf_size);
    }

    while (1)
        ;
}

static esp_err_t update_post_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "Receiving file ...");

    gctx.terminate_for_update = true;
    storage_fat_deinit();

    static constexpr int SCRATCH_BUFSIZE = 256;
    char buf[SCRATCH_BUFSIZE];
    int received;

    const esp_partition_t* storage_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");

    ESP_ERROR_CHECK(esp_partition_erase_range(storage_partition, 0, storage_partition->size));
    ESP_LOGI(TAG, "Data partition erased");

    size_t offset{};
    int remaining = req->content_len;
    while (remaining > 0) {
        if ((received = httpd_req_recv(req, buf, MIN(remaining, SCRATCH_BUFSIZE))) <= 0) {
            if (received <= 0) {
                if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                    httpd_resp_send_408(req);
                }
                return ESP_FAIL;
            }

            ESP_LOGE(TAG, "File reception failed!");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
            return ESP_FAIL;
        }

        ESP_ERROR_CHECK(esp_partition_write_raw(storage_partition, offset, buf, received));

        offset += received;

        remaining -= received;
    }
    ESP_LOGI(TAG, "File reception complete");

    copy_flash_args_t* args = new copy_flash_args_t;
    args->buf = buf;
    args->buf_size = SCRATCH_BUFSIZE;

    xTaskCreate(copy_flash_task, "update", 4096, args, configMAX_PRIORITIES - 1, nullptr);

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_sendstr(req, "Update task started. Please wait.\n");
    return ESP_OK;
}

static const httpd_uri_t update_post = {
    .uri = "/update", .method = HTTP_POST, .handler = update_post_handler, .user_ctx = NULL};

static esp_err_t root_post_handler(httpd_req_t* req) {
    char content[100];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    if (strncmp(content, "command=record", 14) == 0) {
        ESP_LOGI(TAG, "Start logging!");
        if (!xSemaphoreTake(gctx.logger_control.mutex, 10)) {
            return ESP_FAIL;
        }

        gctx.logger_control.active = true;

        xSemaphoreGive(gctx.logger_control.mutex);
    }

    if (strncmp(content, "command=calibrate", 17) == 0) {
        ESP_LOGI(TAG, "Request calibration!");
        if (!xSemaphoreTake(gctx.logger_control.mutex, 10)) {
            return ESP_FAIL;
        }

        gctx.logger_control.calibration_pending = true;

        xSemaphoreGive(gctx.logger_control.mutex);
    }

    if (strncmp(content, "command=stop", 12) == 0) {
        ESP_LOGI(TAG, "Stop logging!");
        if (!xSemaphoreTake(gctx.logger_control.mutex, 10)) {
            return ESP_FAIL;
        }

        gctx.logger_control.active = false;

        xSemaphoreGive(gctx.logger_control.mutex);
    }

    if (strncmp(content, "unlink=", 7) == 0) {
        content[recv_size] = 0;
        std::string path = content + 7;
        if (unlink(("/spiflash/" + path).c_str())) {
            ESP_LOGE(TAG, "Could not remove '%s'", ("/spiflash/" + path).c_str());
            perror("???");
        }
    }

    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");

    return httpd_resp_send(req, "", HTTPD_RESP_USE_STRLEN);
}

static const httpd_uri_t root_post = {
    .uri = "/", .method = HTTP_POST, .handler = root_post_handler, .user_ctx = NULL};

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &root_get);
        httpd_register_uri_handler(server, &download_get);
        httpd_register_uri_handler(server, &format_get);
        httpd_register_uri_handler(server, &update_post);
        httpd_register_uri_handler(server, &status_get);
        httpd_register_uri_handler(server, &files_get);
        httpd_register_uri_handler(server, &root_post);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server) { return httpd_stop(server); }

void http_init() {
    static httpd_handle_t server = NULL;

    server = start_webserver();
}
