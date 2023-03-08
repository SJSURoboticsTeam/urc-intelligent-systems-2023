# Clock Issue and Troubleshooting
If you are having issues with the BN08X IMU check the following:
1. Make sure the IMU is powered on and connected to the correct i2c bus.
2. Change the clock speed to 800kHz. This can be done by changing the /boot/config.txt file. [Helpful Link](https://learn.adafruit.com/raspberry-pi-i2c-clock-stretching-fixes/change-the-clock-speed)
These should help to get the device working.