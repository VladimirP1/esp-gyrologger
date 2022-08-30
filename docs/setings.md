### SDA / SCL pin
The i2c pins where the IMU is connected. These can be any pins, but make sure they do not have some side effects (not used by the SPI flash, are not strapping pins).
### Led pin / Led type
The pin where the LED is connected it can be either an ordinary LED or ws2812. In case of an ordinary LED no resistor is neccesary.
### Button pin
The pin where the button is connected. Just a button which shorts the pin to ground when pressed.
### Max wifi tx power (dbm)
Reduces TX power so that PCB designs with a bad antenna can successfuly transmit at least something.
### Disable wifi when started with button
When logging is started by button, disable wifi to save power / reduce heat until logging is stopped.
### Accel offset X/Y/Z
Accelerometer calibration offsets. To calculate use [http://192.168.4.1/calibration](http://192.168.4.1/calibration)
### Quantization parameter
Higher values reduce logging accuracy and bitrate. Reasonable values are in range somwhere in [8; 22], but usually you should not change this if you do not know what you are doing
### \[Accel\] PT? / PT? cutoff
Disable LPF / PT1 LPF / PT2 LPF / PT3 LPF and its cutoff frequency.
### Dyn filter
This is an attempt to mimic the behaviour of betaflight's dynamic filter, but the algorithm is completely different, based on gradient descent opposed to DTFT in betaflight. This is experimental, do not enable if you do not know what you are doing.
### Cam control type
Different methods either to start logging with camera or to start camera with logging.

0. Disable camera control.
1. Momentary ground. Short the pin specified in "Cam control pin" to ground for 300ms when logging is started and when it is stopped.
2. Firefly X Lite over wifi. This will connect to the camera through wifi network specified in "WiFi station ssid/password" and listen for recording started/stopped events to start/stop logging.
3. Runcam protocol. This will setup an UART RX on "Cam control pin" and wait for recording to be started from FC to start/stop logging.
### Start logging on boot
This will automatically start logging on boot.
### Wifi auto off (seconds)
This will disable wifi automatically if nobody is connected for this amount of seconds.
### Epoch
This number is used to generate the log filename (eg 0 results in prefix LAA, 1 in LAB, 2 in LAC).
You can increment this number by pressing the button for > 2s.
This can be useful if you changed the memory card 
### IMU Orientation
IMU orientation which will be embedded into the gcsv files. Make sure you reload the download page after changing this.
### WiFi password
Wifi access point password.

## Setings reset
It is possible to reset settings to defaults if the button pin is configured correctly. For this, press the button pin for > 40s.