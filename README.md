# ESP32 PFD + GPS + Phone Mirror

An ESP32-based Primary Flight Display (PFD) demo that renders an attitude indicator (MPU6050), GPS-derived speed/altitude (NEO-6M / “GPS6MV2”), and mirrors the instrument to a phone over Wi-Fi via a lightweight web UI + WebSocket stream.

> **Safety / Disclaimer**
>
> This project is for educational and hobby use only. It is **not** certified avionics, and it is **not intended** for real-world comercial flight navigation. Do not rely on it for any operational decisions.

---

## Features

- **PFD rendering on ILI9341** (TFT_eSPI)
  - Attitude indicator (pitch/roll)
  - Pitch ladder
  - Roll scale with **yellow** roll pointer
  - Slip/skid ball + turn rate (bottom of screen)
  - Airspeed tape + altitude tape
- **MPU6050 IMU**
  - Startup **zero calibration**: current sensor orientation becomes 0° pitch/roll
  - Optional smoothing to reduce vibration/jitter
  - **Fault / degraded** handling with blinking annunciations
- **GPS (NEO-6M / GPS6MV2)**
  - Boot screen **waits for GPS fix**
  - Shows fix type, satellites, HDOP, lat/lon, UTC time
  - Uses GPS speed + altitude instead of simulation
  - Minimal smoothing (tunable) with “zero speed” clamp logic
- **Phone mirror**
  - ESP32 runs a Wi-Fi AP (`PFD-ESP32`) and serves a web page
  - Web page renders a simplified PFD and updates via WebSocket

---

## Hardware

### Required
- ESP32 development board (ESP32-WROOM recommended)
- ILI9341-based TFT (320x240)
- MPU6050 module (I2C)
- GPS module (NEO-6M / GPS6MV2) UART

### Recommended
- Stable 5V / 3.3V power (GPS modules can be noisy)
- Proper grounding and short wiring for IMU

---

## Wiring / Pinout

### TFT (ILI9341 via TFT_eSPI)
TFT wiring depends on your TFT_eSPI `User_Setup.h` / `User_Setup_Select.h`.
Configure TFT_eSPI for your specific display and SPI pins.

### MPU6050 (I2C)
- ESP32 `GPIO21` → MPU6050 SDA  
- ESP32 `GPIO22` → MPU6050 SCL  
- 3.3V + GND

### GPS (UART2)
Default pins used by the sketch:
- ESP32 `GPIO16` (RX2) ← GPS TX
- ESP32 `GPIO17` (TX2) → GPS RX *(optional; many modules work fine without this)*

> If you suspect wiring issues: **GPS TX must go to ESP32 RX**, and **GPS RX to ESP32 TX** (crossed).

---

## Software / Dependencies

- Arduino IDE or PlatformIO
- Libraries:
  - **TFT_eSPI**
  - **MPU6050** (I2Cdevlib or compatible)
  - **TinyGPSPlus**
  - **ESPAsyncWebServer**
  - **AsyncTCP**
  - ESP32 core for Arduino

---

## Repo Layout 

```text
.
├── PFD.ino                  # Main avionics logic (no HTML here)
├── PhoneMirror.h            # Wi-Fi + WebServer + WebSocket + page serving

