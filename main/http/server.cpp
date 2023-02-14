#include "server.hpp"

#include "mongoose.h"

#include "esp_system.h"

#include "storage/filenames.hpp"

#include <stdlib.h>
#include <sys/unistd.h>
#include <cstddef>

static int s_debug_level = MG_LL_INFO;
static const char *s_listening_address = "http://0.0.0.0:80";

#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wformat"

struct data_t {
    char type{};
    char is_timed{};
    uint64_t timeout{};
};

void reset_timeout(void *data, bool on) {
    data_t *d = (data_t *)data;
    d->is_timed = on;
    d->timeout = mg_millis() + 5'000ULL;
}

bool is_expired(void *data) {
    data_t *d = (data_t *)data;
    MG_INFO(("check"));
    if (d->is_timed) {
        uint64_t cur = mg_millis();
        MG_INFO(("timed %lld %lld", d->timeout, cur));
        if (d->timeout < cur) {
            MG_INFO(("!!!"));

            return true;
        } else if (d->timeout - cur > 10'000ULL) {
            // wrap happened. we do not know the period. reset timeout
            reset_timeout(data, d->is_timed);
        }
    }
    return false;
}

struct upload_state {
    int fd{};
};

static void handle_upload(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    upload_state *state = (upload_state *)fn_data;
    if (ev == MG_EV_HTTP_CHUNK) {
        reset_timeout(c->data, true);
        struct mg_http_message *hm = (mg_http_message *)ev_data;
        if (hm->chunk.len == 0) {
            MG_INFO(("OK!"));
            mg_http_reply(c, 200, "Content-Type: text/plain\r\n", "OK");
            c->is_draining = 1;
        }
        if (state == NULL) {
            int fd = open("/flash/upload.bin", O_CREAT | O_TRUNC | O_WRONLY);
            if (fd < 0) {
                MG_INFO(("Failed to open file"));
                mg_http_reply(c, 403, "Content-Type: text/plain\r\n", "Error");
                return;
            }
            MG_INFO(("File opened"));
            state = (upload_state *)calloc(1, sizeof(upload_state));
            state->fd = fd;
            c->fn = handle_upload;
            c->fn_data = state;
        } else {
            MG_INFO(("New chunk"));
        }

        write(state->fd, hm->chunk.ptr, hm->chunk.len);
        mg_http_delete_chunk(c, hm);
    } else if (ev == MG_EV_CLOSE && state != NULL) {
        MG_INFO(("connection closed - closing file"));
        close(state->fd);
        free(state);
        c->is_closing = 1;
        c->fn_data = NULL;
    }
}

struct download_state {
    int fd{};
    size_t bytes_left{};
};

static void handle_download(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    download_state *state = (download_state *)fn_data;
    if (ev == MG_EV_HTTP_MSG && state == NULL) {
        reset_timeout(c->data, true);
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
            download_state *new_state = (download_state *)calloc(1, sizeof(download_state));
            new_state->fd = fd;
            new_state->bytes_left = bytes;
            c->fn = handle_download;
            c->fn_data = new_state;
        }
    } else if (ev == MG_EV_WRITE && state != NULL) {
        reset_timeout(c->data, true);
        char buf[4096];
        int nread{};
        if (c->send.len >= 2048) {
            return;
        }
        if (state->bytes_left > 0) {
            nread = read(state->fd, buf, std::min((size_t)sizeof(buf), (size_t)state->bytes_left));
        }
        if (nread > 0) {
            if (mg_send(c, buf, nread)) {
                state->bytes_left -= nread;
            }
        } else {
            c->is_draining = 1;
        }

    } else if (ev == MG_EV_CLOSE && state != NULL) {
        MG_INFO(("connection closed - closing file"));
        close(state->fd);
        free(state);
        c->is_closing = 1;
        c->fn_data = NULL;
    }
}

static void cb(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    struct mg_mgr *mgr = (struct mg_mgr *)fn_data;
    if (ev == MG_EV_OPEN) {
        c->data[0] = 0;
    } else if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message *hm = (mg_http_message *)ev_data;

        if (mg_http_match_uri(hm, "/download")) {
            c->fn = handle_download;
            c->fn_data = NULL;
            handle_download(c, ev, ev_data, c->fn_data);
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
            c->fn = handle_upload;
            c->fn_data = NULL;
            handle_upload(c, ev, ev_data, c->fn_data);
        }
    }
    (void)fn_data;
}

void terminate_stalled_transfers(void *arg) {
    struct mg_mgr *mgr = (struct mg_mgr *)arg;
    for (mg_connection *c = mgr->conns; c != NULL; c = c->next) {
        if (is_expired(c->data)) {
            c->is_closing = 1;
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
