# Astra Firmware v1.0.0 for the Rev 5-5 Board

Flightcontroller firmware for a custom ESP32S3-N8R8 embedded board, ICM-20948, LPS25HBTR, And Controller. PCB Gerber files are also within this repo and are easily reproducable.

Info Document [here](https://docs.google.com/document/d/1mX9GRuzyko-Ee7_DHJvfnAwM2Fita9zQ-YIju0DpfQU/edit?usp=sharing).

<p align="center">
  <img src="photos/pcbimage.jpeg" width="650">
</p>

# Flight Controller Hardware

* ESP32-S3 Module
* LDO
* GPIO18 LED
* ICM-20948 Gyro, Magnetometer, and Accelerometer
* LPS25HBTR Barometer
* Fully Custom PCB
* Couple Resistors, Capacitors, Buttons and 2.54mm Pin Headers
* Usb-C 2.0 Port

# Controller Hardware
* ESP32-C3-Supermini
* Joystick
* 3 Pushbuttons
* Potentiometer
* Custom PCB

### PCB Layout

<p align="center">
  <img src="photos/pcblayout.png" width="700">
</p>

Rev 5.5 PCB layout designed in KiCad for the ESP32-S3, IMU, barometer, power regulation, USB-C, and debug/status circuitry.

---

# Software

* C++
* FreeRTOS 
* PlatformIO + Visual Studio Code

### Firmware Architecture

<p align="center">
  <img src="photos/Arch.jpg" width="700">
</p>


# Project Structure

```bash
.
├── src/            # Main code
├── include/        # Empty
├── lib/            # Empty
├── test/           # Empty
├── platformio.ini  # Build configuration
├── README.md
├── Gerber.zip      # Gerber files for PCB production
└── controller/     # Firmware for the controller itself
```

---

# Controls
Not really full designed as of yet but currently
The buttons are for Holding, Traversing the Controller Screen, and Arming/Disarming.
Potentiometer for height when held.
joystick for x,y movement.

---

# Getting Started

### 0. Building the Hardware
Order the gerber files from JLCPCB or an equivilant. Solder the hardware together (Guide not yet made).

### 1. Clone the repository

```bash
git clone https://github.com/yourusername/astra-1.git
cd astra-1
```

### 2. Build the project

Use PlatformIO vscode extention to build the project

### 3. Upload to ESP32-S3-N8R8
Use PlatformIO vscode extention to upload directly to the ESP32.

### 4. Upload to the ESP32-C3-Supermini
Use the Arduino IDE to to compile and upload directly to the esp32.

---



