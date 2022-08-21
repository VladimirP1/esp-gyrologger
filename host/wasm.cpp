#include "lib/compression.hpp"
#include <emscripten.h>
#include <sys/types.h>

std::vector<uint8_t> input;
std::vector<int> output;
bool fail = false;
static constexpr int kMaxOutputSize = 1024 * 1024;
static constexpr int kBlockSize = 256;
static constexpr double sample_rate = 1.0 / 0.00180;
static constexpr double gscale = 1 / 0.00053263221;

static int ztime = 0;
static int pos = 0;
static quat::quat prev_quat(quat::base_type{1}, {}, {}, {});
static Coder decoder(kBlockSize);

extern "C" {
EMSCRIPTEN_KEEPALIVE
uint8_t* allocate_input(int size) {
    input.resize(size);
    ztime = 0;
    pos = 0;
    prev_quat = {quat::base_type{1}, {}, {}, {}};
    decoder = Coder{kBlockSize};
    return input.data();
}

EMSCRIPTEN_KEEPALIVE
int get_output_size() { return output.size(); }

EMSCRIPTEN_KEEPALIVE
int* get_output_ptr() { return output.data(); }

static quat::vec decode_accel(int16_t* ptr) {
    const double scale = 256 * 32768 / 16;
    return quat::vec{quat::base_type{ptr[0] / scale}, quat::base_type{ptr[1] / scale},
                     quat::base_type{ptr[2] / scale}};
}

EMSCRIPTEN_KEEPALIVE
int decode() {
    output.clear();
    while (pos < input.size() && output.size() <= 1024) {
        auto [decoded_bytes, dquats, scale] =
            decoder.decode_block(input.data() + pos, input.size() - pos);
        if (fail || !decoded_bytes) break;
        pos += decoded_bytes;

        int accel_count = input[pos++];
        int16_t* accel_data = (int16_t*)input.data() + pos;

        int i = 0;
        for (auto& q : dquats) {
            quat::vec rv = (q.conj() * prev_quat).axis_angle();
            quat::vec accel = decode_accel(accel_data + std::min(i / 55, accel_count - 1) * 3);
            // accel = rv.conj().rotate_point(accel);
            prev_quat = q;
            if (ztime != 0) {
                double scale = sample_rate * gscale;
                double ascale = 10000;
                output.push_back(ztime);
                output.push_back((int)(double(rv.x) * scale));
                output.push_back((int)(double(rv.y) * scale));
                output.push_back((int)(double(rv.z) * scale));
                output.push_back((int)(double(accel.x) * ascale * 256.0));
                output.push_back((int)(double(accel.y) * ascale * 256.0));
                output.push_back((int)(double(accel.z) * ascale * 256.0));
            }
            ztime++;
        }
    }
    return 0;
}

void _Exit(int) { fail = true; }
int __stdio_close(int) { return {}; }
int __stdio_write(int, int, int) { return {}; }
off_t __lseek(int, off_t, int) { return {}; }
}
