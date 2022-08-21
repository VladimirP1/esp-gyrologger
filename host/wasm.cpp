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

EMSCRIPTEN_KEEPALIVE
int decode() {
    output.clear();
    while (pos < input.size() && output.size() <= 1024) {
        auto [decoded_bytes, dquats, scale] =
            decoder.decode_block(input.data() + pos, input.size() - pos);
        if (fail || !decoded_bytes) break;
        pos += decoded_bytes;
        for (auto& q : dquats) {
            quat::vec rv = (q.conj() * prev_quat).axis_angle();
            quat::vec gravity = q.conj().rotate_point(
                {quat::base_type{}, quat::base_type{}, quat::base_type{-1.0}});

            prev_quat = q;
            if (ztime != 0) {
                double scale = sample_rate * gscale;
                double ascale = 10000;
                output.push_back(ztime);
                output.push_back((int)(double(rv.x) * scale));
                output.push_back((int)(double(rv.y) * scale));
                output.push_back((int)(double(rv.z) * scale));
                output.push_back((int)(double(gravity.x) * ascale));
                output.push_back((int)(double(gravity.y) * ascale));
                output.push_back((int)(double(gravity.z) * ascale));
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
