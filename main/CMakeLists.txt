idf_component_register(
    SRCS 
        "esp-gyrologger.cpp" 
        "global_context.cpp"
        "gyro/gyro_bmi160.cpp" 
        # "gyro/gyro_lsm6.cpp" 
        "gyro/gyro_mpu6050.cpp" 
        "gyro/gyro_probe.cpp" 
        "logger/logger.cpp" 
        "wifi/http.cpp" 
        "bus/bus_i2c.c" 
        # "misc/misc.c" 
        "storage/storage_fat.c" 
        "wifi/wifi.cpp" 
        "wifi/random_names.cpp" 
        "wifi/cam_control.cpp" 
    INCLUDE_DIRS "."
    LDFRAGMENTS "linker.lf")
