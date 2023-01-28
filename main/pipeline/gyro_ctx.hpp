#pragma once
#include <cstdint>
#include <cstdlib>

#include "gyro/gyro.hpp"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
}

struct Descriptor {
    void* ptr{};
    uint32_t size1{};
    uint32_t size2{};
    uint32_t dt{};
};

struct BufQueue {
    BufQueue(size_t block_count, size_t block_size);
    ~BufQueue();

    BufQueue(const BufQueue&) = delete;
    BufQueue(BufQueue&&) = delete;
    BufQueue& operator=(const BufQueue&) = delete;
    BufQueue& operator=(BufQueue&&) = delete;

    bool alloc(Descriptor* desc);
    bool free(Descriptor* desc);
    bool pop(Descriptor* desc);
    bool push(Descriptor* desc);

    size_t block_count{};
    size_t block_size{};

   private:
    uint8_t* data{};
    QueueHandle_t alloc_queue{};
    QueueHandle_t desc_queue{};
};

struct GyroCtx {
    GyroHal* hal{};
    BufQueue* queue{};
    Descriptor desc{};
    size_t gyr_block{};
    size_t acc_block{};
    size_t gyr_cnt{};
    size_t acc_cnt{};
    size_t drops{};
};

bool gyro_ctx_init(GyroCtx* ctx, GyroHal* hal);