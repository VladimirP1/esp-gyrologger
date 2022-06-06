idf_component_register(
    SRCS "esp-gyrologger.c" "gyro/gyro_mpu6050.c" "bus/bus_i2c.c" "storage/storage_fat.c" "logger/logger.cpp" "compression/quat.cpp"
    INCLUDE_DIRS "."
    LDFRAGMENTS "linker.lf")
