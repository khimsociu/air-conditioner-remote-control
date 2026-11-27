# ❄️ Smart Air Conditioner Controller (ESP32-S3)

An IoT universal air conditioner (AC) remote control project built on **PlatformIO** for the **ESP32-S3 Super Mini** microcontroller. The system supports receiving and transmitting Infrared (IR) signals for various popular AC brands, featuring an intuitive OLED display interface and remote control via an embedded Web Server and MQTT.

## ✨ Key Features

* **Wide AC Brand Compatibility:** Powered by the `IRremoteESP8266` library to decode and transmit IR signals for most major AC manufacturers (Daikin, Panasonic, LG, Mitsubishi, Casper, Gree, Toshiba, Samsung, Carrier, etc.).
* **Web-Based Control (Web Server):** Built-in local Web Server allowing users to toggle power, adjust temperature, switch modes (Cool, Dry, Fan, Auto), and control fan speed directly from a smartphone or PC browser.
* **IoT Integration via MQTT:** Supports the MQTT protocol (`PubSubClient`) for seamless integration with smart home platforms (Home Assistant, Node-RED, or custom mobile apps).
* **Visual Status Display:** Utilizes the `U8g2` graphics library to display operating states, target temperature, and Wi-Fi/MQTT connection status on an OLED/LCD screen.
* **Persistent Storage:** Uses ESP32 non-volatile Flash memory (`Preferences`) to save Wi-Fi credentials, MQTT configurations, and the last known AC state across power cycles.
* **GPS & Real-Time Expansion:** Includes `TinyGPSPlus` library support for time synchronization or location-based automation (Geofencing).

## 🛠️ Hardware Requirements

* **Microcontroller:** ESP32-S3 Super Mini
* **IR Transmitter Module:** 940nm Infrared LED (paired with a Transistor driver circuit such as 2N2222/S8050 for extended transmission range).
* **IR Receiver Module:** 38kHz Infrared Receiver (VS1838B / TSOP4838) *(used for learning commands from original remotes)*.
* **Display:** I2C OLED Display (SSD1306 / SH1106).

## 🔌 Pinout Configuration

> *Note: Please verify and update these GPIO pin assignments to match your actual wiring and `src/main.cpp` definitions.*

| Component | Component Pin | ESP32-S3 Super Mini Pin | Notes |
| :--- | :--- | :--- | :--- |
| **IR Transmitter LED** | Signal (Data) | `GPIO 4` *(Update per code)* | Recommended to drive via Transistor |
| **IR Receiver** | OUT | `GPIO 5` *(Update per code)* | 3.3V Power Supply |
| **OLED Display** | SDA / SCL | `GPIO 8` / `GPIO 9` *(I2C)* | Default I2C interface |

## 📚 Libraries Used (PlatformIO)

This project is built on the Arduino Framework using PlatformIO with the following core dependencies:

* [`IRremoteESP8266`](https://github.com/crankyoldgit/IRremoteESP8266) - Infrared signal decoding and transmission for AC units.
* [`U8g2`](https://github.com/olikraus/u8g2) - Monochrome graphics library for OLED/LCD displays.
* [`PubSubClient`](https://github.com/knolleary/pubsubclient) - MQTT client messaging.
* `WebServer`, `Preferences`, `FS` - Built-in ESP32 Arduino Core libraries.

## 🚀 Installation & Flashing Guide

### 1. Prerequisites
* Install **Visual Studio Code**.
* Install the **PlatformIO IDE** extension within VS Code.

### 2. Setup Steps
1. **Clone the repository:**
   ```bash
   git clone https://github.com/username/DK_dieu_hoa.git
   cd DK_dieu_hoa
   ```

2. **Open the project in PlatformIO:**
   * Launch VS Code -> Open **PlatformIO Home** -> Click **Open Project** -> Select the `DK_dieu_hoa` folder.

3. **Configure parameters:**
   * Open the main source file inside the `src/` directory to update your Wi-Fi credentials, MQTT broker settings, and verify GPIO pin definitions.

4. **Build and Upload:**
   * Connect your ESP32-S3 Super Mini to your computer via a USB-C cable.
   * Click **Build (✓)** on the PlatformIO toolbar to compile the firmware.
   * Click **Upload (→)** to flash the firmware onto the board.

## 📂 Project Structure

```text
DK_dieu_hoa/
├── include/          # Header files (.h) for variable declarations and pin definitions
├── lib/              # Custom/private libraries (if any)
├── src/              # Main application source code (main.cpp)
├── platformio.ini    # PlatformIO board configuration (ESP32-S3) and dependencies
├── .gitignore        # Excludes build artifacts (.pio/) from Git tracking
└── README.md         # Project documentation
```

## 📝 Note on Pushing to GitHub
* The local `.pio/` directory contains compiled build artifacts (~180MB). 
* The `.gitignore` file is already configured to exclude `.pio/`. When running `git add .` and `git push`, GitHub will cleanly store only your lightweight source code without uploading temporary build files.

## 🤝 Contributing & Contact
Contributions, bug reports, and feature requests are welcome! Feel free to open an **Issue** or submit a **Pull Request** directly on this repository.
Gmail: trankhiem723@gmail.com
