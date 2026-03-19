# RP2040-ICM42688-3D-Attitude-Visualization
Real-time 3D attitude visualization using RP2040 and ICM42688 IMU, with Madgwick filter and OLED display. PolyU Ideation Funding Scheme project.
# 6-DOF IMU Attitude Visualization with 3D Cube on OLED

## 📖 Description
A real-time attitude estimation and 3D visualization system based on Arduino RP2040 Connect and ICM42688-P sensor. Implements Madgwick filter for sensor fusion and renders a rotating cube with coordinate axes on OLED display.

## 🎯 Features
- **6-axis data acquisition**: Real-time accelerometer and gyroscope readings from ICM42688-P
- **Sensor fusion**: Madgwick filter for quaternion-based attitude estimation
- **3D real-time rendering**: Rotating cube with X/Y/Z axes on 0.96" OLED screen
- **Euler angles output**: Real-time display of Roll/Pitch/Yaw angles
- **Serial monitoring**: Attitude data output via Serial for debugging and recording

## 🛠️ Hardware
- MCU: Arduino RP2040 Connect
- IMU: ICM42688-P (6-axis accelerometer + gyroscope)
- Display: 0.96" OLED (128x64, SSD1306, I2C address 0x3C)
- Connection: I2C bus (IMU address 0x68)

## 📦 Required Libraries
- `ICM42688.h` - ICM42688 sensor driver
- `Adafruit_SSD1306.h` - OLED driver
- `Adafruit_GFX.h` - Graphics library

## 🔧 Key Parameters
- Attitude update rate: 100Hz
- Display refresh rate: 15Hz
- Filter: Madgwick (β=0.05) + Low-pass filter (α=0.3)

## 🚀 Quick Start
1. Install Arduino IDE and add RP2040 support
2. Install required libraries via Library Manager
3. Connect hardware according to the pin table below
4. Upload the code to Arduino RP2040 Connect

### Hardware Connection
| RP2040 Connect | ICM42688-P | OLED SSD1306 |
|----------------|------------|--------------|
| 3.3V           | VCC        | VCC          |
| GND            | GND        | GND          |
| SDA (GP2)      | SDA        | SDA          |
| SCL (GP3)      | SCL        | SCL          |

## 📷 Demo

Watch the demo video on Bilibili:  
[**ICM42688P + RP2040 Connect + OLED 3D Visualization**](https://www.bilibili.com/video/BV1mw6UB2ERL/)

*Demo showing real-time attitude estimation and 3D cube rendering with coordinate axes.*

## 📜 License
MIT

## 🙏 Acknowledgements
- Thanks to B站UP主 **LShang001** for the tutorial video "icm42688P+ESP Attitude Estimation, TFT 3D Visualization" which provided reference for the Madgwick filter implementation
