#include "server.hpp"

#include "mongoose.h"

#include "storage/filenames.hpp"

static int s_debug_level = MG_LL_INFO;
static const char *s_root_dir = "/flash/";
static const char *s_listening_address = "http://0.0.0.0:8000";

// Event handler for the listening connection.
// Simply serve static files from `s_root_dir`
static void cb(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    struct mg_mgr *mgr = (struct mg_mgr *)fn_data;
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message *hm = (mg_http_message *)ev_data, tmp = {0};
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
            MG_INFO(("closing connection"));
            int fd{}, bytes{};
            memcpy(&fd, c->data + 1, 4);
            memcpy(&bytes, c->data + 5, 4);
            close(fd);
        }
    }
    (void)fn_data;
}

void terminate_stalled_downloads(void *arg) {
    struct mg_mgr *mgr = (struct mg_mgr *)arg;

    int n_conns{};
    uint64_t cur_time = esp_timer_get_time();
    for (mg_connection *c = mgr->conns; c != NULL; c = c->next) {
        ++n_conns;
        if (c->data[0] == 'F') {
            uint64_t time{};
            int fd{}, bytes{};
            memcpy(&fd, c->data + 1, 4);
            memcpy(&bytes, c->data + 5, 4);
            memcpy(&time, c->data + 9, 8);
            if (cur_time - time > 5000000ULL) {
                MG_INFO(("download stall - terminating connection"));
                c->is_closing = 1;
                c->data[0] = 0;
                close(fd);
            }
        }
    }
    MG_INFO(("check timeouts %d", n_conns));
}

void server_task(void *) {
    struct mg_mgr mgr;
    struct mg_connection *c;

    // Initialise stuff
    mg_log_set(s_debug_level);
    mg_mgr_init(&mgr);
    mg_timer_add(&mgr, 500, MG_TIMER_REPEAT, terminate_stalled_downloads, &mgr);
    if ((c = mg_http_listen(&mgr, s_listening_address, cb, &mgr)) == NULL) {
        MG_ERROR(("Cannot listen on %s. Use http://ADDR:PORT or :PORT", s_listening_address));
        return;
    }

    // Start infinite event loop
    MG_INFO(("Mongoose version : v%s", MG_VERSION));
    MG_INFO(("Listening on     : %s", s_listening_address));
    MG_INFO(("Web root         : [%s]", s_root_dir));
    while (1) {
        mg_mgr_poll(&mgr, 100);
    }
    mg_mgr_free(&mgr);
    return;
}
