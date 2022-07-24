// SPDX-License-Identifier: LGPL-2.1-or-later

#include "http.hpp"

extern "C" {
#include <esp_http_server.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_partition.h>
#include <ff.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#include "storage/storage_fat.h"
#include "spi_flash_chip_driver.h"
}
#include <cstdio>

#include "global_context.hpp"

#include "compression/lib/compression.hpp"
#include "storage/settings.hpp"

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

        HANDLE_FINALLY(httpd_resp_send_chunk(req, buf2, read_bytes), fclose(f));
    }

    fclose(f);

    HANDLE_FINALLY(httpd_resp_send_chunk(req, NULL, 0), ;);

    return ESP_OK;
}

static esp_err_t respond_with_file(httpd_req_t* req, const char* filename) {
    static uint8_t buf2[2000];
    static char buf_text[4096];

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
                               fclose(f));
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
        HANDLE_FINALLY(httpd_resp_send_chunk(req, buf_text, HTTPD_RESP_USE_STRLEN), ;);
    }

    HANDLE_FINALLY(httpd_resp_send_chunk(req, NULL, 0), ;);

    return ESP_OK;
}

static esp_err_t download_get_handler(httpd_req_t* req) {
    char buf[128];
    int buf_len = httpd_req_get_url_query_len(req) + 1;
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
    std::string resp;

    auto entry = [&resp](std::string name, std::string value) {
        resp.append(R"--(<tr><td class="status_table_name_cell">)--");
        resp.append(name);
        resp.append(R"--(</td><td class="status_value_table_cell">)--");
        resp.append(value);
        resp.append(R"--(</td></tr>)--");
    };

    resp.append(R"--(<table class="status_table">)--");

    if (esp_timer_get_time() - gctx.logger_control.last_block_time_us > 2000000ULL) {
        entry("I2C FAILURE", "1");
    }

    entry("Active", std::to_string(gctx.logger_control.busy));

    entry("Avg gyro sample int. (ns)", std::to_string(gctx.logger_control.avg_sample_interval_ns));

    auto free_space = get_free_space_kb();
    entry("Free space (kBytes)", std::to_string(free_space.first));

    entry("Free heap size (kBytes)", std::to_string(esp_get_free_heap_size() / 1024));

    entry("Last log length (samples)", std::to_string(gctx.logger_control.total_samples_written));

    entry("Last log length (Bytes)", std::to_string(gctx.logger_control.total_bytes_written));

    entry("Last log avg rate (Bytes/min)",
          std::to_string(gctx.logger_control.avg_logging_rate_bytes_min));

    resp.append("</table>");

    HANDLE(httpd_resp_send(req, resp.c_str(), HTTPD_RESP_USE_STRLEN));
    return ESP_OK;
}

static const httpd_uri_t status_get = {
    .uri = "/status", .method = HTTP_GET, .handler = status_get_handler, .user_ctx = NULL};

static SemaphoreHandle_t file_list_mtx;
static std::vector<std::pair<std::string, int>> file_list;

static esp_err_t files_get_handler(httpd_req_t* req) {
    xSemaphoreTake(file_list_mtx, portMAX_DELAY);
    std::string resp;
    resp.append("<table class=\"download_table\">");
    for (auto& f : file_list) {
        resp.append(
            R"--(<tr class="download_table_name_cell"><td><a href="#" onclick='download_and_decode_log("/download?name=)--");
        resp.append(f.first);
        resp.append("&raw=1\", \"");
        resp.append(f.first);
        resp.append(".gcsv\");return false;'>");
        resp.append(f.first);
        resp.append(R"--(</a></td> <td class="download_table_mid_cell">)--");
        resp.append(std::to_string(f.second));
        resp.append("KB</td>");
        resp.append(
            R"--(<td><div style="color:black;display:inline-block;" class="delete_btn"  onclick="post_command('unlink=)--");
        resp.append(f.first);
        resp.append(R"--(')">&#x274c;</div></td></tr>)--");
    }
    resp.append("</table>");
    xSemaphoreGive(file_list_mtx);
    HANDLE(httpd_resp_send(req, resp.c_str(), HTTPD_RESP_USE_STRLEN));
    return ESP_OK;
}

static const httpd_uri_t files_get = {
    .uri = "/files", .method = HTTP_GET, .handler = files_get_handler, .user_ctx = NULL};

static esp_err_t root_get_handler(httpd_req_t* req) {
    HANDLE(httpd_resp_send_chunk(req, html_prefix, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, html_body, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, html_stylesheet, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, js_xhr_status_updater, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, js_wasm_decoder_0, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, gctx.settings_manager->GetString("imu_orientation").c_str(),
                                 HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, js_wasm_decoder_1, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, html_suffix, HTTPD_RESP_USE_STRLEN));
    HANDLE(httpd_resp_send_chunk(req, NULL, 0));
    return ESP_OK;
}

static const httpd_uri_t root_get = {
    .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL};

static esp_err_t settings_get_handler(httpd_req_t* req) {
    std::string resp;
    resp +=
        R"--(<!doctype html><html lang=en><head><meta charset=utf-8><title>EspLog Setings</title></head><body>)--";
    resp += "<h1>EspLog Settings (<a href=\"/\">go back</a>)</h1>";
    resp += R"--(<table class="settings_table">)--";
    for (auto& s : kSettingDescriptor) {
        resp += R"--(<tr><td class="setting_name cell">)--";
        resp += s.desc;
        resp += "</td>\n<td class=\"cell\">";
        resp += R"--(<input id=")--";
        resp += s.name;
        resp += R"--(" type="number" autocomplete="off" step="0.001" value=")--";
        resp += std::to_string(gctx.settings_manager->Get(s.name));
        resp += "\" min=\"";
        resp += std::to_string(s.min_value);
        resp += "\" max=\"";
        resp += std::to_string(s.max_value);
        resp += "\">";
        resp += "</td>\n";
        resp += R"--(<td class="apply_btn cell" id=")--";
        resp += s.name;
        resp += "__btn";
        resp += R"--(" onclick=")--";
        resp += "apply_setting('";
        resp += s.name;
        resp += "')";
        resp += "\">Apply</td>\n";
        resp += "</tr>";
    }
    for (auto& s : kStringSettingDescriptor) {
        resp += R"--(<tr><td class="setting_name cell">)--";
        resp += s.desc;
        resp += "</td>\n<td class=\"cell\">";
        resp += R"--(<input id=")--";
        resp += s.name;
        resp += R"--(" type="text" autocomplete="off" value=")--";
        resp += gctx.settings_manager->GetString(s.name);
        resp += "\">";
        resp += "</td>\n";
        resp += R"--(<td class="apply_btn cell" id=")--";
        resp += s.name;
        resp += "__btn";
        resp += R"--(" onclick=")--";
        resp += "apply_setting('";
        resp += s.name;
        resp += "')";
        resp += "\">Apply</td>\n";
        resp += "</tr>";
    }
    resp += "</table>";
    resp += html_update_uploader;
    resp += "</body>";
    resp += settings_style;
    resp += js_settings;
    resp += js_update_uploader;
    resp += html_suffix;
    HANDLE(httpd_resp_send(req, resp.c_str(), HTTPD_RESP_USE_STRLEN));

    return ESP_OK;
}

static const httpd_uri_t settings_get = {
    .uri = "/settings", .method = HTTP_GET, .handler = settings_get_handler, .user_ctx = NULL};

static esp_err_t settings_post_handler(httpd_req_t* req) {
    std::string buf;
    buf.resize(300);
    int received = httpd_req_recv(req, buf.data(), buf.size());
    if (received <= 0) {
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    buf.resize(received);

    auto pos = buf.find("=");
    if (pos == std::string::npos) {
        return ESP_FAIL;
    }
    auto name = buf.substr(0, pos);
    if (gctx.settings_manager->IsDoubleValue(name.c_str())) {
        auto value = stod(buf.substr(pos + 1));
        ESP_LOGI(TAG, "%s=%f", name.c_str(), value);
        HANDLE(gctx.settings_manager->Set(name.c_str(), value));
    } else if (gctx.settings_manager->IsStringValue(name.c_str())) {
        auto value = buf.substr(pos + 1);
        ESP_LOGI(TAG, "%s=%s", name.c_str(), value.c_str());
        HANDLE(gctx.settings_manager->SetString(name.c_str(), value.c_str()));
    }

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_sendstr(req, "OK\n");
    return ESP_OK;
}

static const httpd_uri_t settings_post = {
    .uri = "/settings", .method = HTTP_POST, .handler = settings_post_handler, .user_ctx = NULL};

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

static esp_err_t calibration_get_handler(httpd_req_t* req) {
    HANDLE(httpd_resp_send(req, html_calibration, HTTPD_RESP_USE_STRLEN));
    return ESP_OK;
}

static const httpd_uri_t calibration_get = {.uri = "/calibration",
                                            .method = HTTP_GET,
                                            .handler = calibration_get_handler,
                                            .user_ctx = NULL};

static esp_err_t calibration_post_handler(httpd_req_t* req) {
    std::string content;
    content.resize(req->content_len);
    int ret = httpd_req_recv(req, content.data(), req->content_len);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    if (content == "get_acceleration") {
        httpd_resp_set_status(req, "200 OK");
        httpd_resp_set_type(req, "application/json");

        double accel[3] = {};
        if (xSemaphoreTake(gctx.logger_control.accel_raw_mtx, portMAX_DELAY)) {
            accel[0] = gctx.logger_control.accel_raw[0];
            accel[1] = gctx.logger_control.accel_raw[1];
            accel[2] = gctx.logger_control.accel_raw[2];
            xSemaphoreGive(gctx.logger_control.accel_raw_mtx);
        }
        std::string json_str = "{\"acceleration\" : [" + std::to_string(accel[0]) + "," +
                               std::to_string(accel[1]) + "," + std::to_string(accel[2]) + "]}";
        HANDLE(httpd_resp_sendstr(req, json_str.c_str()));
        return ESP_OK;
    } else if (strncmp(content.c_str(), "set_offsets_ug", 14) == 0) {
        int pos;
        long long xyz[3];

        pos = content.find("/");
        for (int i = 0; i < 3; ++i) {
            content = content.substr(pos + 1);
            pos = content.find("/");
            ESP_LOGI(TAG, "%s", content.substr(0, pos - 1).c_str());
            xyz[i] = std::stoll(content.substr(0, pos));
        }

        ESP_LOGI(TAG, "%lld %lld %lld", xyz[0], xyz[1], xyz[2]);

        HANDLE(gctx.settings_manager->Set("accel_ofs_x", xyz[0] / 1e6));
        HANDLE(gctx.settings_manager->Set("accel_ofs_y", xyz[1] / 1e6));
        HANDLE(gctx.settings_manager->Set("accel_ofs_z", xyz[2] / 1e6));

        HANDLE(httpd_resp_sendstr(req, "OK\n"));
        return ESP_OK;
    }

    httpd_resp_send_500(req);
    return ESP_OK;
}

static const httpd_uri_t calibration_post = {.uri = "/calibration",
                                             .method = HTTP_POST,
                                             .handler = calibration_post_handler,
                                             .user_ctx = NULL};

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 12;

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
        httpd_register_uri_handler(server, &settings_get);
        httpd_register_uri_handler(server, &settings_post);
        httpd_register_uri_handler(server, &calibration_post);
        httpd_register_uri_handler(server, &calibration_get);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server) { return httpd_stop(server); }

static void background_scanner_task(void* param) {
    while (true) {
        std::vector<std::pair<std::string, int>> new_file_list;
        DIR* dp;
        struct dirent* ep;
        dp = opendir("/spiflash");
        if (dp != NULL) {
            while ((ep = readdir(dp))) {
                static constexpr char templ[] = "/spiflash/%s";
                char buf[30];
                snprintf(buf, sizeof(buf), templ, ep->d_name);
                struct stat st;
                if (stat(buf, &st) == 0) {
                    int size_kb = st.st_size / 1024;
                    new_file_list.push_back({ep->d_name, size_kb});
                }
            }
            (void)closedir(dp);
        } else {
            ESP_LOGE(TAG, "Couldn't open the directory");
        }
        std::sort(new_file_list.begin(), new_file_list.end());
        xSemaphoreTake(file_list_mtx, portMAX_DELAY);
        file_list = std::move(new_file_list);
        xSemaphoreGive(file_list_mtx);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void http_init() {
    file_list_mtx = xSemaphoreCreateMutex();

    xTaskCreate(background_scanner_task, "background-scanner", 3084, nullptr,
                configMAX_PRIORITIES - 10, nullptr);

    static httpd_handle_t server = NULL;

    server = start_webserver();
}
