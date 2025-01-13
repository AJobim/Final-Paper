# Anxiety Monitor with Vibrotactile Feedback

This project is a wearable device designed to monitor heart rate and assist in managing anxiety episodes through vibrotactile feedback. Built with an ESP32 microcontroller, it uses a pulse sensor to detect heart rate and activates a vibration motor when elevated BPM levels are detected, simulating a calming heartbeat rhythm.

## Features

- **Real-Time Heart Rate Monitoring:** Continuously tracks BPM using Pulse Sensor.
- **Dynamic Threshold Detection:** Automatically adjusts to the user's average BPM to detect abnormal increases.
- **Vibrotactile Feedback:** Activates a motor to simulate a calming heartbeat during anxiety episodes.
- **Monitoring app:** Provides real-time feedback on BPM and system status.
- **Bluetooth Control:** Allows remote control of the device via Bluetooth commands.

## Bluetooth Commands

- `AMOSTRA`: Captures the user's resting BPM for threshold calibration.
- `LIGA`: Starts heart rate monitoring.
- `DESLIGA`: Stops heart rate monitoring.
- `ATIVA`: Manually activates the vibration motor.
- `DESATIVA`: Manually deactivates the vibration motor.

## Hardware Requirements

- ESP32 Microcontroller
- Pulse Sensor (connected to GPIO 15)
- Vibration Motor (connected to GPIO 4)
- LCD I2C Display (16x2)
- Button for manual sampling (connected to GPIO 25)
- Necessary resistors and wiring
