// SPDX-License-Identifier: LGPL-2.1-or-later

namespace {
const char html_prefix[] = R"--(
<!doctype html>
<html lang=en>

<head>
    <meta charset=utf-8>
    <title>EspLog</title>
</head>
)--";

constexpr char html_suffix[] = "</html>";

const char html_stylesheet[] = R"--(
<style>
    .command_btn {
        width: 4rem;
        height: 4rem;
        font-size: 2rem;
        border-radius: .5rem;
        border: .2rem solid gray;
    }

    .status_table {
        font-size: 1.5rem;
        font-style: bold;
    }

    .status_table_name_cell {
        width: 18rem;
    }

    .status_value_table_cell {
        padding-left: 2rem;
    }

    .download_table {
        font-size: 1.5rem;
    }

    .download_table_name_cell {
        width: 8rem;
    }

    .delete_btn {
        border: none;
        padding: 0 0 0 0;
    }

    .download_table_mid_cell {
        padding-left: 1rem;
        padding-right: 1rem;
    }

    #network_activity {
        display: inline-block;
        width: 1rem;
        height: 1rem;
        border-radius: 1rem;
    }
</style>
)--";

const char js_xhr_status_updater[] = R"--(
<script type="text/javascript">
    var ind_state = false;
    var indicate = function() {
        var elem = document.getElementById("network_activity");
        ind_state = !ind_state;
        if (ind_state) {
            elem.style.backgroundColor ="orange";
        } else {
            elem.style.backgroundColor ="black";
        }
    };
    function make_updater(path, element, interval, timeout, indicate) {
        var xhr = new XMLHttpRequest();
        var do_request = function() {
            xhr.open('GET', path);
            xhr.timeout = timeout;
            xhr.send();
        }
        xhr.onload = function() {
            setTimeout(do_request, interval);
            indicate();
            document.getElementById(element).innerHTML = xhr.responseText;
        };
        xhr.onerror = function() {
            setTimeout(do_request, 1000);
        };
        xhr.ontimeout = xhr.onerror;
        do_request();
    }
    make_updater( '/status', 'status_table', 500, 200, indicate);
    make_updater( '/files', 'files_table', 2000, 1000, indicate);

    function post_command(command) {
        let xhr = new XMLHttpRequest();
        xhr.open('POST', '/');
        xhr.send(command);
        xhr.timeout = 5000;
    }
</script>)--";

}  // namespace