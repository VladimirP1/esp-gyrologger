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
        "gyro/gyro.cpp"
        "pipeline/pt_filter.cpp" 
        "pipeline/gyro_ctx.cpp"
    INCLUDE_DIRS "."
    LDFRAGMENTS "linker.lf")
