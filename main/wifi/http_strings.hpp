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

    .status_value_table_cell {
        padding-left: 2rem;
    }

    .download_table {
        font-size: 1.5rem;
    }

    .delete_btn {
        border: none;
        padding: 0 0 0 0;
    }

    .download_table_mid_cell {
        padding-left: 1rem;
        padding-right: 1rem;
    }
</style>
)--";
}  // namespace