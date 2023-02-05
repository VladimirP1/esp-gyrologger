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

const char html_body[] = R"--(
    <body>
    <h1>EspLog</h1>
    <div class="container">
    <div class="column ">
        <div class="center button_cloumn" style="max-width: 33rem;">
            <svg class="rec_btn command_btn" viewBox="0 0 24 24" onclick="post_command('command=record')" x="0px"
                y="0px" width="24" height="24">
                <circle cx="12" cy="12" r="4" fill="#ff0000" />
            </svg>
            <svg class="stop_btn command_btn" viewBox="0 0 24 24" onclick="post_command('command=stop')" x="0px" y="0px"
                width="24" height="24">
                <path d="M 8 8 L 16 8 L 16 16 L 8 16 L 8 8" fill="#000000" />
            </svg>
            <svg class="calibrate_btn command_btn" viewBox="0 0 24 24" onclick="post_command('command=calibrate')"
                x="0px" y="0px" width="24" height="24">
                <circle cx="12" cy="12" r="4" stroke="#42ab05" stroke-width="1" fill="#00000000" />
            </svg>
        </div>
    </div>
    <div class="column">
        <div class="center status_column" style="max-width: 33rem;">
            <div id="network_activity"></div>
            <div id="status_table">
                Loading...
            </div>
        </div>
    </div>
    <div class="column">
        <div class="center download_column" style="max-width: 33rem;min-height:1rem;">
             <div id="files_table">
                Loading...
            </div>
        </div>
    </div>
    </div>
</body>)--";

const char html_stylesheet[] = R"--(
<style>
    .rec_btn {
        background-color: #ff99c4;
    }

    .stop_btn {
        background-color: #ffe499;
    }

    .calibrate_btn {
        background-color: #a7d6fc;
    }

    .command_btn {
        height: auto;
        float: left;
        width: 33.33%;
        padding: 0;
        margin: 0;
    }

    .command_btn:active {
        background-color: #505050;
    }

    .cmd_btns:after {
        content: "";
        display: table;
        clear: both;
    }
    .container {
        display: flex;
        flex-wrap: wrap;
    }

    .button_cloumn {
        background-color: aliceblue;
    }

    .center {
        margin-left: auto;
    margin-right: auto;
    }

    .status_column {
        background-color:bisque;
    }

    .download_column {
        background-color: cornsilk;
    }
    .column {
        flex-grow: 1;
        flex-basis: 33%;
        padding-left: .1rem;
        padding-right: .1rem;
    }

    @media screen and (max-width: 67rem) {
        .column {
            flex-basis: 100%;
        }
    }

    .status_table {
        font-size: 1.2rem;
        font-style: bold;
    }

    .status_table_name_cell {
        width: 18rem;
    }

    .status_value_table_cell {
        padding-left: 2rem;
    }

    .download_table {
        font-size: 2rem;
    }

    .download_table_name_cell {
        width: 8rem;
    }

    .delete_btn {
        border: none;
        padding: 0 0 0 0;
    }

    .delete_btn:active {
        background-color: #505050;
    }

    .download_table_mid_cell {
        padding-left: 1rem;
        padding-right: 1rem;
    }

    #network_activity {
        width: 100%;
        height: 0.5rem;
    }
</style>
)--";

const char settings_style[] = R"--(
    <style>
    .setting_name {
        font-weight: bold;
    }

    .apply_btn {
        background-color: lightgreen;
        font-weight: bold;
    }

    .apply_btn:active {
        background-color: gray;
    }

    .cell {
        padding-left: 0.5em;
        padding-right: 0.5em;
    }
    </style>
)--";

const char js_settings[] = R"--(
    <script type="text/javascript">
    function apply_setting(s) {
        let elem = document.getElementById(s);
        let btn_elem = document.getElementById(s + "__btn");
        btn_elem.style.color="red";
        let xhr = new XMLHttpRequest();
        xhr.open('POST', '/settings');
        xhr.onload = function() {
            btn_elem.style.color="black";
        };
        xhr.send(s + "=" + elem.value);
        xhr.timeout = 10000;
    }
    </script>
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
    var update = function(path, element, interval, timeout, indicate) {
        let controller = new AbortController();
        let restarted = false;
        function restart(t) {
            if (!restarted) {
                restarted = true;
                setTimeout(()=>{update(path, element, interval, timeout, indicate);}, t);
            }
        }
        let to = setTimeout(()=>{ controller.abort(); }, timeout);
        fetch(path, {signal: controller.signal}).then((resp)=>{
            resp.text().then((text)=>{ 
                clearTimeout(to); 
                document.getElementById(element).innerHTML = text; 
                indicate(); 
                restart(interval); 
            }, (error) => { clearTimeout(to); restart(1000); });
        }, (error) => { clearTimeout(to); restart(1000); });
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
    var status_last_update = (new Date()).getTime();
    var files_last_update = (new Date()).getTime();
    update( '/status', 'status_table', 500, 1000, ()=>{
        indicate();
        status_last_update = (new Date()).getTime();});
    update( '/files', 'files_table', 1000, 1000,  ()=>{
        indicate();
        files_last_update = (new Date()).getTime();});

    setInterval(()=>{ 
        let activity_div = document.getElementById("network_activity");
        if ((new Date()).getTime() - status_last_update > 2000) {
            activity_div.style.backgroundColor = "red";
        }
        if ((new Date()).getTime() - files_last_update > 2000) {
            activity_div.style.backgroundColor = "red";
        }
    }, 1000);

    function post_command(command) {
        let xhr = new XMLHttpRequest();
        xhr.open('POST', '/');
        xhr.send(command);
        xhr.timeout = 10000;
    }
</script>)--";

const char html_update_uploader[] = R"--(
    <div>
        <p style="font-weight:bold; color:red; font-size:1.2em;">WARNING: make sure you select the correct firmware,
            there is NO verification and NO integrity checking!</p>
        <p style="font-weight:bold">Update firmware</p>
        <input id="file_input" type="file">
        <button id="upload_button" type="button" onclick="upload()">Upload</button>
        <div id="upload_log"></div>
    </div>
)--";

const char js_update_uploader[] = R"--(
    <script>
    function upload() {
        let fileInput = document.getElementById("file_input").files;
        let file = fileInput[0];
        let xhttp = new XMLHttpRequest();
        let log_elem = document.getElementById("upload_log");
        log_elem.innerHTML = "";
        let line = document.createElement("p");
        line.innerText = "Starting upload!";
        line.style.color = "green";
        log_elem.appendChild(line);
        
        xhttp.timeout = 40000;
        xhttp.onreadystatechange = function () {
            if (xhttp.readyState == 4) {
                if (xhttp.status == 200) {
                    let line = document.createElement("p");
                    line.innerText = "Upload done, update should start automatically. This will take some time. Please, wait for more than 30 seconds and then power-cycle the device";
                    line.style.color = "green";
                    log_elem.appendChild(line);
                } else {
                    let line = document.createElement("p");
                    line.innerText = "Update failed. Wait around 30 seconds, reboot the device and try again.";
                    line.style.color = "red";
                    log_elem.appendChild(line);
                }
            }
        };
        xhttp.open("POST", "/update", true);
        xhttp.send(file);
    }
</script>
)--";

const char html_calibration[] = R"--(
<!doctype html>
<html lang=en>
<head>
    <meta charset=utf-8>
    <title>EspLog Calibration</title>
</head>
<body>
    <h1>Accelerometer calibration</h1>
    <div class="scene_container">
        <div class="scene">
            <div id="point_container">
            </div>
            <div class="big_circle object3d"></div>
            <div class="hor_bar object3d"></div>
            <div class="ver_bar object3d"></div>
        </div>
    </div>
    <div style="padding:0.5rem;" id="calc_result">---</div>
    <br>
    <div class="button" onclick="add_point();">Add point</div>
    <div class="button" onclick="remove_last_point();">Remove last</div>
    <div class="button" onclick="clear_points();">Clear points</div>
    <br>
    <div class="button" onclick="sphere_fit(points);">Calculate offsets</div>
    <div class="button" id="save_button" onclick="save_offsets(offsets);">Save to flash</div>
</body>
<style>
    .button {
        display: inline-block;
        border: 0.1rem solid black;
        cursor: default;
        margin: 0.2rem;
        padding: 0.3rem;
        user-select: none;
    }

    .button:hover {
        background-color: lightgray;
    }

    .button:active {
        background-color: gray;
    }

    .scene_container {
        width: 22rem;
        height: 22rem;
        position: relative;
    }

    .scene {
        width: 100%;
        height: 100%;
    }

    .object3d {
        position: absolute;
        top: 0rem;
        left: 0rem;
    }

    .big_circle {
        border-radius: 11rem;
        border: 0.1rem solid blue;

        width: 20rem;
        height: 20rem;
        top: 1rem;
        left: 1rem;
    }

    .hor_bar {
        background-color: red;

        width: 22.2rem;
        height: 0.1rem;
        transform: translate(0, 11rem) translate(0, -0.05rem);
    }

    .ver_bar {
        background-color: green;

        height: 22.2rem;
        width: 0.1rem;
        transform: translate(11rem, 0) translate(-0.05rem, 0);
    }

    .calib_point {
        background-color: red;
        border-radius: 1rem;

        width: 0.7rem;
        height: 0.7rem;
    }
</style>
<script type="text/javascript">

    function draw_points(points) {
        let scene_mult = 10;
        let scene_ofs = 1;
        let container = document.getElementById("point_container");

        container.innerHTML = '';

        for (let i = 0; i < points.length; ++i) {
            let x = scene_mult * (points[i][0] + 1) + scene_ofs;
            let y = scene_mult * (points[i][1] + 1) + scene_ofs;
            let z = scene_mult * (points[i][2]);
            let elem = document.createElement("div");
            elem.classList = "calib_point object3d";
            elem.style.transform = `translate3d(${x}rem,${y}rem, ${z}rem) translate3d(-0.35rem, -0.35rem, 0)`;

            container.appendChild(elem);
        }
    }

    let points = [];
    let offsets = [0,0,0];

    function add_point() {
        let xhr = new XMLHttpRequest();
        xhr.responseType = 'json';
        xhr.open('POST', '/calibration');
        xhr.send("get_acceleration");
        xhr.timeout = 2000;
        xhr.onload = function () {
            points.push(xhr.response.acceleration);
            draw_points(points);
            console.log(points);
        };
    }

    function save_offsets(xyz) {
        let btn = document.getElementById("save_button");
        btn.style.color = "red";
        let xhr = new XMLHttpRequest();
        xhr.responseType = 'json';
        xhr.open('POST', '/calibration');
        xhr.send(`set_offsets_ug/${-Math.round(xyz[0] * 1e6)}/${-Math.round(xyz[1] * 1e6)}/${-Math.round(xyz[2] * 1e6)}`);
        xhr.timeout = 2000;
        xhr.onload = function () {
            btn.style.color = "black";
        };
    }

    function clear_points() {
        points = [];
        draw_points(points);
    }

    function remove_last_point() {
        points.pop();
        draw_points(points);
    }

    function sphere_fit(points) {
        const lr = 1e-3;
        const x = new Array(4).fill(0);
        const f = new Array(points.length).fill(0);
        for (let i = 0; i < points.length; ++i) {
            for (let j = 0; j < 3; ++j) {
                f[i] += points[i][j] * points[i][j];
            }
        }

        console.log(f);

        for (let t = 0; t < 1000000; ++t) {
            for (let i = 0; i < points.length; ++i) {
                let aa = -f[i];
                for (let k = 0; k < 3; ++k) {
                    aa += points[i][k] * x[k];
                }
                aa += x[3];
                for (let k = 0; k < 3; ++k) {
                    x[k] -= lr * 2 * aa * points[i][k];
                }
                x[3] -= lr * 2 * aa;
            }
        }

        rad = Math.sqrt(x[0] * x[0] / 4 + x[1] * x[1] / 4 + x[2] * x[2] / 4 + x[3]);
        cx = x[0] / 2;
        cy = x[1] / 2;
        cz = x[2] / 2;

        let result_text = `X = ${cx.toFixed(4)}g, Y = ${cy.toFixed(4)}g, Z = ${cz.toFixed(4)}g, radius = ${rad.toFixed(4)}`;
        document.getElementById("calc_result").innerText = result_text;

        offsets = [cx, cy, cz];

        return offsets;
    }
</script>
</html>
)--";

const char html_display_0[] = R"--(<!doctype html>
<html lang=en>

<head>
    <meta charset=utf-8>
    <title>EspLog</title>
</head>

<body>)--";
const char html_display_1[] = R"--(
</body>
<style>
    #canvas {
        width: 6.4rem;
        height: 3.2rem;
        image-rendering: pixelated;
    }
</style>
<script type="text/javascript">
    function setRGB(data, x, y, r, g, b) {
        data.data[((y * (data.width * 4)) + (x * 4)) + 0] = r;
        data.data[((y * (data.width * 4)) + (x * 4)) + 1] = g;
        data.data[((y * (data.width * 4)) + (x * 4)) + 2] = b;
        data.data[((y * (data.width * 4)) + (x * 4)) + 3] = 255;
    }
    
    function renderBuffer(buffer) {
        let view = new Uint8Array(buffer);
        const canvas = document.getElementById('canvas');
        if (canvas.getContext) {
            const ctx = canvas.getContext('2d');
            ctx.fillStyle = "black";
            ctx.fillRect(0, 0, 64, 32);
            var data = ctx.createImageData(64, 32);
            for (let x = 0; x < ctx.canvas.width; ++x) {
                for (let y = 0; y < ctx.canvas.height; ++y) {
                    let idx = x + ctx.canvas.width * y;
                    let val = 0;
                    if (view[(idx / 8) | 0] & (1 << (idx % 8))) {
                        val = 255;
                    }
                    setRGB(data, x, y, val, val, val);
                }
            }
            ctx.putImageData(data, 0, 0);
        } else {

        }
    }

    var update = function(interval, timeout) {
        let controller = new AbortController();
        let restarted = false;
        function restart(t) {
            if (!restarted) {
                restarted = true;
                setTimeout(()=>{update(interval, timeout);}, t);
            }
        }
        let to = setTimeout(()=>{ controller.abort(); }, timeout);
        fetch('./display_data', {signal: controller.signal}).then((resp)=>{
            resp.arrayBuffer().then((buf)=>{ 
                clearTimeout(to); 
                renderBuffer(buf);
                restart(interval); 
            }, (error) => { clearTimeout(to); restart(1000); });
        }, (error) => { clearTimeout(to); restart(1000); });
    };
    update(500, 2000);
    // fetch('./display_data')
    //     .then(response => response.arrayBuffer()).then(buf => renderBuffer(buf));
</script>
</html>)--";

}  // namespace