#include "server.hpp"


#include "mongoose.h"

#include "esp_system.h"

#include "storage/filenames.hpp"

#include <sys/unistd.h>

static int s_debug_level = MG_LL_INFO;
static const char *s_listening_address = "http://0.0.0.0:80";

#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wformat"

static void proc_upload(struct mg_connection *c, struct mg_http_message *hm) {
    int fd{};
    if (c->data[0] == 'U') {
        MG_INFO(("New chunk"));
        memcpy(&fd, c->data + 1, 4);
    } else if (c->data[0] == 0) {
        fd = open("/flash/upload.bin", O_CREAT | O_TRUNC | O_WRONLY);
        if (fd < 0) {
            MG_INFO(("Failed to open file"));
            mg_http_reply(c, 403, "Content-Type: text/plain\r\n", "Error");
            return;
        }
        MG_INFO(("File opened"));
        c->data[0] = 'U';
        memcpy(c->data + 1, &fd, 4);
    }
    uint64_t time = esp_timer_get_time();
    memcpy(c->data + 9, &time, 8);
    if (hm->chunk.len != 0) {
        write(fd, hm->chunk.ptr, hm->chunk.len);
    }
    if (hm->chunk.len == 0) {
        c->data[0] = 0;
        MG_INFO(("OK!"));
        mg_http_reply(c, 200, "Content-Type: text/plain\r\n", "OK");
        close(fd);
    }
}

static void cb(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    struct mg_mgr *mgr = (struct mg_mgr *)fn_data;
    if (ev == MG_EV_OPEN) {
        c->data[0] = 0;
    } else if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message *hm = (mg_http_message *)ev_data;
        if (mg_http_match_uri(hm, "/download")) {
            char buf[64], buf2[64];
            mg_http_get_var(&hm->query, "name", buf, sizeof(buf));
            snprintf(buf2, sizeof(buf2), "/flash/%s", buf);
            int fd = open(buf2, O_RDONLY);
            if (fd < 0) {
                mg_http_reply(c, 404, "Content-Type: text/plain\r\n", "Not found");
                return;
            }
            int bytes = lseek(fd, (size_t)0, SEEK_END);
            lseek(fd, 0, SEEK_SET);
            mg_printf(c, "HTTP/1.1 200 OK\r\nContent-Length: %d\r\n\r\n", bytes);
            c->data[0] = 'F';
            uint64_t time = esp_timer_get_time();
            memcpy(c->data + 1, &fd, 4);
            memcpy(c->data + 5, &bytes, 4);
            memcpy(c->data + 9, &time, 8);
        } else if (mg_http_match_uri(hm, "/list")) {
            mg_printf(c, "HTTP/1.1 200 OK \r\nTransfer-Encoding: chunked\r\n\r\n");
            DIR *dp = opendir("/flash");
            if (dp != NULL) {
                struct dirent *ep;
                while ((ep = readdir(dp))) {
                    if (validate_file_name(ep->d_name)) {
                        mg_http_write_chunk(c, ep->d_name, strlen(ep->d_name));
                        mg_http_write_chunk(c, "\n", 1);
                    }
                }
                mg_http_write_chunk(c, "", 0);
                closedir(dp);
            }
        } else if (mg_http_match_uri(hm, "/status")) {
            mg_http_reply(c, 200, "Content-Type: text/plain\r\n", "free heap: %d\n",
                          esp_get_free_heap_size());
        }
    } else if (ev == MG_EV_HTTP_CHUNK) {
        struct mg_http_message *hm = (mg_http_message *)ev_data;
        if (mg_http_match_uri(hm, "/upload")) {
            proc_upload(c, hm);
            mg_http_delete_chunk(c, hm);
        }
    } else if (ev == MG_EV_WRITE) {
        if (c->data[0] == 'F') {
            char buf[4096];
            int fd{}, bytes{};
            memcpy(&fd, c->data + 1, 4);
            memcpy(&bytes, c->data + 5, 4);
            int nread{};
            if (c->send.len >= 2048) {
                return;
            }
            if (bytes > 0) {
                nread = read(fd, buf, std::min((size_t)sizeof(buf), (size_t)bytes));
            }
            if (nread > 0) {
                if (mg_send(c, buf, nread)) {
                    bytes -= nread;
                }
                uint64_t time = esp_timer_get_time();
                memcpy(c->data + 5, &bytes, 4);
                memcpy(c->data + 9, &time, 8);
            } else {
                c->is_closing = 1;
                c->data[0] = 0;
                close(fd);
            }
        }
    } else if (ev == MG_EV_CLOSE) {
        if (c->data[0] == 'F') {
            MG_INFO(("connection closed - closing file"));
            int fd{}, bytes{};
            memcpy(&fd, c->data + 1, 4);
            memcpy(&bytes, c->data + 5, 4);
            c->data[0] = 0;
            close(fd);
        }
    }
    (void)fn_data;
}

void terminate_stalled_transfers(void *arg) {
    struct mg_mgr *mgr = (struct mg_mgr *)arg;

    uint64_t cur_time = esp_timer_get_time();
    for (mg_connection *c = mgr->conns; c != NULL; c = c->next) {
        if (c->data[0] == 'F' || c->data[0] == 'U') {
            uint64_t time{};
            int fd{};
            memcpy(&fd, c->data + 1, 4);
            memcpy(&time, c->data + 9, 8);
            if (cur_time - time > 5000000ULL) {
                MG_INFO(("no data transferred for more than 5 seconds - terminating connection"));
                c->is_closing = 1;
                c->data[0] = 0;
                close(fd);
            }
        }
    }
}

void server_task(void *) {
    struct mg_mgr mgr;
    struct mg_connection *c;

    mg_log_set(s_debug_level);
    mg_mgr_init(&mgr);
    mg_timer_add(&mgr, 500, MG_TIMER_REPEAT, terminate_stalled_transfers, &mgr);
    if ((c = mg_http_listen(&mgr, s_listening_address, cb, &mgr)) == NULL) {
        MG_ERROR(("Cannot listen on %s. Use http://ADDR:PORT or :PORT", s_listening_address));
        return;
    }

    MG_INFO(("Mongoose version : v%s", MG_VERSION));
    MG_INFO(("Listening on     : %s", s_listening_address));
    while (1) {
        mg_mgr_poll(&mgr, 100);
    }
    mg_mgr_free(&mgr);
    return;
}

#pragma GCC diagnostic pop
