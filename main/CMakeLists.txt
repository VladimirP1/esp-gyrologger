idf_component_register(
    SRCS 
        "esp-gyrologger.cpp" 
        "global_context.cpp"
        "bus/mini_i2c.c" 
        "bus/aux_i2c.cpp" 
        "gyro/gyro_bmi160.cpp" 
        "gyro/gyro_mpu6050.cpp" 
        "gyro/gyro_icm42688.cpp" 
        "gyro/gyro_probe.cpp"
        "hal/fs.cpp"
        "pipeline/pt_filter.cpp" 
        "pipeline/gyro_ctx.cpp"
        "ebin-encoder-cpp/lib/quant.cpp"
        "ebin-encoder-cpp/lib/compress.cpp"
        "ebin-encoder-cpp/lib/laplace_model.cpp"
        "ebin-encoder-cpp/lib/writer.cpp"
    INCLUDE_DIRS "."
    LDFRAGMENTS "linker.lf")
