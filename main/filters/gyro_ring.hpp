#pragma once

#include "compression/lib/fixquat.hpp"
#include "global_context.hpp"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
}

#include <variant>
#include <vector>

static constexpr float kGyroToRads = 1.0 / 32.8 * 3.141592 / 180.0;
static constexpr float kAccelToG = 16.0 / 32767;
struct raw_sample {
    int gx, gy, gz;
    int ax, ay, az;
    int flags;
};

struct sample {
    uint32_t duration_ns;
    std::variant<raw_sample, quat::quat> sample;
};

static constexpr int kFlagHaveAccel = 1;
static constexpr char kLogTag[] = "gyro_ring";

class DurationSmoother {
   public:
    DurationSmoother(int thresh) : thresh(thresh) {}
    uint32_t Smooth(uint32_t d) {
        sum += d;
        count += 1;
        if (count > thresh) {
            dur_avg = sum / count;
            sum = 0;
            count = 0;
            gctx.logger_control.avg_sample_interval_ns = dur_avg;
        }
        return dur_avg;
    }

   private:
    uint64_t sum{};
    int count{};
    int thresh{};
    uint32_t dur_avg{};
};

class Calibrator {
   public:
    void ProcessSample(sample &s) {
        auto &rs = std::get<raw_sample>(s.sample);
        rs.gx += g_ofs_x;
        rs.gy += g_ofs_y;
        rs.gz += g_ofs_z;

        RunGyroCalibration(s);
    }

   private:
    int g_ofs_x{}, g_ofs_y{}, g_ofs_z{};

    int gyr_samples{};
    int64_t sum_gx{}, sum_gy{}, sum_gz{};
    static constexpr int kGyroCalibrationSamples = 1000;
    void RunGyroCalibration(const sample &s) {
        if (gctx.logger_control.calibration_pending) {
            gctx.logger_control.calibration_pending = false;
            sum_gx = sum_gy = sum_gz = 0;
            gyr_samples = kGyroCalibrationSamples + 1;
            ESP_LOGI(kLogTag, "Gyro calibration started");
        }
        if (gyr_samples == 1) {
            g_ofs_x = -sum_gx / kGyroCalibrationSamples;
            g_ofs_y = -sum_gy / kGyroCalibrationSamples;
            g_ofs_z = -sum_gz / kGyroCalibrationSamples;
            gyr_samples = 0;
            ESP_LOGI(kLogTag, "Gyro calibration complete %d %d %d", g_ofs_x, g_ofs_y, g_ofs_z);
        } else if (gyr_samples) {
            auto &rs = std::get<raw_sample>(s.sample);
            sum_gx += rs.gx;
            sum_gy += rs.gy;
            sum_gz += rs.gz;
            gyr_samples -= 1;
        }
    }
};

class GyroRing {
   public:
    GyroRing() {}

    void Init(int capacity, int chunk_size, uint32_t desired_interval) {
        this->chunk_size_ = chunk_size;
        this->desired_interval_ = desired_interval;
        ring_.resize(capacity);
        quats_chunk_.reserve(chunk_size);
        inv_desired_interval_ = quat::base_type{1.0 / desired_interval_};
        ESP_LOGI(kLogTag, "inv_interval %f", (double)inv_desired_interval_);
    }

    void Push(uint32_t dur_ns, int gx, int gy, int gz, int ax, int ay, int az, int flags) {
        static int shadow_wptr{};

        auto &s = ring_[shadow_wptr];
        s.duration_ns = dur_smoother.Smooth(dur_ns);
        raw_sample rs = {
            .gx = gx, .gy = gy, .gz = gz, .ax = ax, .ay = ay, .az = az, .flags = flags};
        s.sample = rs;

        shadow_wptr = (shadow_wptr + 1) % ring_.size();

        taskENTER_CRITICAL_ISR(&ptr_mux_);
        wptr_ = shadow_wptr;
        taskEXIT_CRITICAL_ISR(&ptr_mux_);
    }

    quat::quat *Work() {
        taskENTER_CRITICAL(&ptr_mux_);
        int cached_wptr = wptr_;
        taskEXIT_CRITICAL(&ptr_mux_);

        // ESP_LOGI(kLogTag, "rptr: %d, wptr: %d, count: %d", rptr_, cached_wptr,
        //  ((cached_wptr + ring_.size()) - rptr_) % ring_.size());

        if (quats_chunk_.size() >= chunk_size_) {
            quats_chunk_.clear();
        }

        // Integrate gyro
        while (rptr_ != cached_wptr) {
            auto &s = ring_[rptr_];
            auto &rs = std::get<raw_sample>(s.sample);

            calib_.ProcessSample(s);

            float gscale = kGyroToRads * s.duration_ns / 1e9;
            auto gyro = quat::vec{quat::base_type{gscale * rs.gx}, quat::base_type{gscale * rs.gy},
                                  quat::base_type{gscale * rs.gz}};
            quat_rptr_ = (accel_correction_ * quat_rptr_ * quat::quat{gyro});

            if (rs.flags & kFlagHaveAccel) {
                quat::base_type ascale{kAccelToG / 16.0};
                quat::vec accel = quat::vec{ascale * rs.ax, ascale * rs.ay, ascale * rs.az};
                if (accel.norm() > quat::base_type{0.9 / 16.0} &&
                    accel.norm() < quat::base_type{1.1 / 16.0}) {
                    accel = accel.normalized();
                    quat::vec gp = quat_rptr_.rotate_point(accel);
                    auto dq0 = (gp.z + quat::base_type{1.0}) * quat::base_type{0.5};
                    auto corr = quat::quat(quat::base_type{dq0}, -gp.y / quat::base_type{2.0},
                                           gp.x / quat::base_type{2.0}, quat::base_type{0.0})
                                    .normalized();

                    static uint8_t cnt;
                    if (cnt++ == 0)
                        printf("corr: %f\n",
                               ((double)(corr.axis_angle() / quat::base_type{4}).norm()) * 4 *
                                   180.0 / M_PI);
                    accel_correction_ =
                        quat::quat{(corr.axis_angle() * quat::base_type{-0.0001 * .01}) +
                                   (accel_correction_.axis_angle() * quat::base_type{.99})};
                } else {
                    accel_correction_ = {};
                }
            }

            MaybeNormalize(quat_rptr_);

            s.sample = quat_rptr_;

            rptr_ = (rptr_ + 1) % ring_.size();
        }

        // See how many samples we have buffered
        int samples_buffered = ((cached_wptr + ring_.size()) - sptr_) % ring_.size();

        if (samples_buffered < kResamlpingLag) {
            return nullptr;
        }

        // ESP_LOGI(kLogTag, "%d %u", samples_buffered, ring_[sptr_].duration_ns);

        // // Resample the quaternions
        samples_buffered -= kResamlpingLag;

        if (quats_chunk_.size() >= chunk_size_) {
            quats_chunk_.clear();
        }

        while (samples_buffered) {
            {  // advance sptr while we can
                while (samples_buffered) {
                    int next_sptr = (sptr_ + 1) % ring_.size();
                    uint32_t next_sptr_ts = sptr_ts_ + ring_[next_sptr].duration_ns;
                    if (next_sptr_ts < interp_ts_) {
                        sptr_ = next_sptr;
                        sptr_ts_ = next_sptr_ts;
                        samples_buffered -= 1;
                    } else {
                        break;
                    }
                }
                if (!samples_buffered) break;
            }
            {  // wrap the timestamps if needed
                static constexpr uint32_t kTsWrapInterval = 200000000;
                if (interp_ts_ >= 2 * kTsWrapInterval) {
                    interp_ts_ -= kTsWrapInterval;
                    sptr_ts_ -= kTsWrapInterval;
                    ESP_LOGI(kLogTag, "wrap");
                }
            }
            {  // Interpolate
                quat::quat q{{}, {}, {}, {}};
                quat::base_type k{};
                quat::base_type k_sum{};

                int dbg_cnt{};
                int cached_sptr = (sptr_ + 1) % ring_.size();
                int cached_sptr_ts = sptr_ts_ + ring_[cached_sptr].duration_ns;
                // to the right
                while ((k = Kernel((cached_sptr_ts - interp_ts_) / 1000)) != quat::base_type{}) {
                    q += std::get<quat::quat>(ring_[cached_sptr].sample) * k;
                    k_sum += k;
                    cached_sptr = (cached_sptr + 1) % ring_.size();
                    cached_sptr_ts += ring_[cached_sptr].duration_ns;
                    if (!ring_[cached_sptr].duration_ns) break;
                    ++dbg_cnt;
                }
                // to the left
                cached_sptr = sptr_;
                cached_sptr_ts = sptr_ts_;
                while ((k = Kernel((interp_ts_ - cached_sptr_ts) / 1000)) != quat::base_type{}) {
                    q += std::get<quat::quat>(ring_[cached_sptr].sample) * k;
                    k_sum += k;
                    if (!ring_[cached_sptr].duration_ns) break;
                    cached_sptr_ts -= ring_[cached_sptr].duration_ns;
                    cached_sptr = (cached_sptr + ring_.size() - 1) % ring_.size();
                    ++dbg_cnt;
                }
                interp_ts_ += desired_interval_ * 1000;

                static uint8_t xx{};
                if (!++xx) {
                    ESP_LOGI(kLogTag, "rg %d %u", dbg_cnt, ring_[cached_sptr].duration_ns);
                }
                if (k_sum != quat::base_type{}) {
                    q.w /= k_sum;
                    q.x /= k_sum;
                    q.y /= k_sum;
                    q.z /= k_sum;
                }
                quats_chunk_.push_back(q);
                if (quats_chunk_.size() >= chunk_size_) {
                    ESP_LOGI(kLogTag, "produced chunk size %d", chunk_size_);
                    return quats_chunk_.data();
                }
            }
        }
        return nullptr;
    }

   private:
    void MaybeNormalize(quat::quat &q) {
        static uint8_t x = 0;
        if ((++x % 16) == 0) q = q.normalized();
    }

    static quat::base_type sinc(quat::base_type x) {
        x = x * quat::base_type::pi();
        if (fpm::abs(x) < quat::base_type{1e-04})
            return quat::base_type{1} +
                   x * x * (quat::base_type{-1.0 / 6.0} + x * x * quat::base_type{1.0 / 120.0});
        return fpm::sin(x) / x;
    }

    quat::base_type Kernel(uint32_t t) {
        if (t < 3 * desired_interval_) {
            return sinc(inv_desired_interval_ * t) * sinc(inv_desired_interval_ * (t / 3));
        }
        return {};
    }

   private:
    static constexpr int kResamlpingLag = 64;

    int chunk_size_;
    std::vector<sample> ring_;
    DurationSmoother dur_smoother{1000};
    Calibrator calib_{};

    uint32_t desired_interval_;
    quat::base_type inv_desired_interval_;
    uint32_t interp_ts_{};
    uint32_t sptr_ts_{};

    portMUX_TYPE ptr_mux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile int wptr_{};
    int rptr_{}, sptr_{};
    quat::quat quat_rptr_{};
    quat::quat accel_correction_{};

    std::vector<quat::quat> quats_chunk_;
};