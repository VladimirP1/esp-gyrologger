namespace {
const char stylesheet[] = R"--(
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


const char testhtml[] = R"--(
<!doctype html>
<html lang=en>

<head>
    <meta charset=utf-8>
    <title>EspLog</title>
</head>

<body>
    <h1>EspLog (IDLE)</h1>
    <h2>Control</h2>
    <button style="color:red;" class="command_btn" name="command" form="form_simple" value="record">&#x23fa;</button>
    <button style="color:black;" class="command_btn" name="command" form="form_simple" value="stop">&#x23F9;</button>

    <h2>Status</h2>
    <table class="status_table">
        <tr>
            <td>Active</td>
            <td class="status_value_table_cell">0</td>
        </tr>
        <tr>
            <td>Free space (kBytes)</td>
            <td class="status_value_table_cell">0</td>
        </tr>
    </table>
    <h2>Log download</h2>
    <table class="download_table">
        <tr>
            <td><a href="/download?name=log001.bin">log001.bin</a></td>
            <td class="download_table_mid_cell">12KB</td>
            <td>
                <button style="color:black;" class="delete_btn" name="unlink" form="form_simple"
                    value="log001.bin">&#x274c;</button>
            </td>
        </tr>
    </table>


    <form id="form_simple" method="post" action=""></form>
</body>
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

</html>
)--";
}