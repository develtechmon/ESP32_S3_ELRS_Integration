# Getting Stared

For this project, i'm using `Happy Model 2.4G EP2 TCX0` and `ESp32 S3` MCU as shown below.

<img width="765" height="383" alt="image" src="https://github.com/user-attachments/assets/b5dae979-2c9e-4c1a-89ae-6797c31edeb8" />

Below is schematic pin configuration

| ESP32-S3 Super Mini | ELRS Happy Model EP2 TCX0 |
|---------------------|---------------------------|
| VDD (5V)            | 5V (Red)                  |
| GND                 | GND (Black)               |
| RX - 6              | TX (Blue)                 |
| TX - 5              | RX (Yellow)               |

# Instructions

In this repository, i've include the following code:


* `ESP32_ELRS_Library_Read`  - This code contain the `ELRS_CRSF.h` library and `main code` to print all the channels and how to use this library
* `ESP32_ELRS_Library_Static_Display_Read` - This code contain the `ELRS_CRSF.h` library and `main code` which is customize based on `flight controller code`. The code will print a `static display` that show all the information


