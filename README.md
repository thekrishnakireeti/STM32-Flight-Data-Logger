# ✈️ Flight Data Logger using STM32F446RE

> Real-time multi-sensor flight data logger built on STM32F446RE + FreeRTOS  
> B.Tech Final Year Project | RGUKT Nuzvid Campus | ECE Department

---

## 📌 Overview
A low-cost, open-source flight data logger that records GPS position,
barometric altitude, and IMU data simultaneously at 10 Hz using FreeRTOS
multi-tasking on the STM32F446RE Nucleo board.

## 🔧 Hardware Used
| Component     | Function              | Interface |
|---------------|-----------------------|-----------|
| STM32F446RE   | Main MCU              | —         |
| MPU-6050      | Accelerometer + Gyro  | I2C       |
| BMP280        | Pressure + Altitude   | SPI       |
| NEO-6M GPS    | Position + Speed      | UART      |

## 💻 Software Stack
- STM32CubeIDE + HAL Drivers
- FreeRTOS (CMSIS-V2)
- Embedded C
- Python (pyserial + csv)
- PuTTY Terminal

## 📊 Results
- ✅ 10 Hz logging rate
- ✅ GPS accuracy: ±2 m
- ✅ Pressure accuracy: ±1 hPa
- ✅ 100% UART reliability

## 🚀 How to Run
1. Flash `Core/Src/main.c` using STM32CubeIDE
2. Connect Nucleo board via USB
3. Run `python/receiver_logger.py`
4. Open `sensor_log.csv` in Excel

## 👥 Authors
- Krishna Kireeti Kasinadhuni (N200026)
- Yarrabhathula Ashok (N200273)

**Guide:** Mr. Vinod Babu, Asst. Professor, ECE Dept, RGUKT Nuzvid
