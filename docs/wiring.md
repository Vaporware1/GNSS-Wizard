# Wiring Guide

## Overview

The GNSS Wizard uses an ESP32 connected to two peripherals:
- **ARK DAN L1/L5** GPS module (via UART)
- **IIS2MDC** magnetometer (via I2C)

Both are powered from the ESP32's VIN (5V) pin.

---

## Pin Connections

```
ARK DAN L1/L5 (6-pin JST-GH)          ESP32 Dev Module
────────────────────────────           ────────────────
5V   ──────────────────────────────►  VIN
GND  ──────────────────────────────►  GND
TX   ──────────────────────────────►  GPIO 16  (RX2)
RX   ──────────────────────────────►  GPIO 17  (TX2)
SCL  ──────────────────────────────►  GPIO 22  (I2C SCL)
SDA  ──────────────────────────────►  GPIO 21  (I2C SDA)
```

---

## Notes

- GPS baud rate: **38400**
- Magnetometer I2C address: **0x1E**
- If heading reads **-1 or is frozen**, reseat the SDA/SCL solder joints — this was the known dropout cause in v2
- The magnetometer and GPS share the same JST-GH connector on the ARK DAN module

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| Heading always -1 | Magnetometer not found on I2C | Reseat SDA/SCL joints |
| No GPS fix | Wrong baud rate or TX/RX swapped | Swap GPIO 16 and 17 |
| Won't upload | ESP32 not in bootloader mode | Hold BOOT button during upload |
| Binary too large | Default partition too small | Switch to "No OTA (2MB APP / 2MB SPIFFS)" |
