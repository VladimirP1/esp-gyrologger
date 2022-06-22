### What is this?
This is a GyroFlow-compatible data logger firmware for the esp32 and esp32-c3 microcontrollers (and probably others supported by esp-idf).

### Supported IMUs
The following IMUs can be used over the i2c bus:
* MPU-6050 / MPU-6000
* LSM6DSR

### How to build
#### Option 1 - from modules
This is all you need (except wires), this should coust about $5:
* [ESP32-C3-01M (4M)](https://aliexpress.ru/item/1005003092302540.html)
* [MPU6050](https://aliexpress.ru/item/32340949017.html)
* [AMS1117 3.3v LDO](https://aliexpress.ru/item/32659371619.html)
* [USB-C breakout bord](https://aliexpress.ru/item/1005002795893679.html)

#### Option 2 - JLCPCB PCBA order (in development)
TODO

### Hardware prototypes
#### EspLog
![esp32 + mpu6050 logger](img/small_logger_esplog_lsm6dsr.jpg)

Note: the final version won't need an external 3.3v LDO

#### Simple esp32c3 logger
![esp32 + mpu6050 logger](img/small_logger_esp32c3_mpu6050.jpg)

#### Esp32 logger
![esp32 + mpu6050 logger](img/small_logger_esp32_mpu6050.jpg)