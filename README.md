# Intelligent-Air-Quality-Monitoring-System

This project implements a real-time **Intelligent Air Quality Monitoring System** using the **ATmega32A** microcontroller. The system measures indoor environmental conditions, evaluates air quality levels, and automatically adjusts ventilation based on sensor data and room occupancy.

### 🔧 System Features

- **Air Quality Measurement**
  - MQ135 gas sensor for CO₂ and harmful gas detection  
  - DHT11 sensor for temperature and humidity monitoring  

- **Occupancy Detection**
  - IR sensors detect human presence  
  - Fan speed increases when the room is occupied and air quality drops  

- **Smart Fan Control**
  - PWM-based fan control for dynamic ventilation  
  - Automatically responds to air quality index (AQI) thresholds  

- **Real-Time Display**
  - 16x2 I²C LCD shows AQI, temperature, humidity, and system status  

- **Data Logging**
  - SD card module records sensor values with timestamps  
  - Enables trend analysis and performance evaluation  

- **Remote Monitoring**
  - Wi-Fi module provides live monitoring capability  

### 🛠️ Key Contributions

- Sensor interfacing and calibration (MQ135, DHT11, IR sensors)  
- PWM-based ventilation fan control logic  
- LCD display integration for real-time feedback  
- SD card data logging system with timestamps  
- Embedded C programming using AVR-GCC  
- Modular and scalable system architecture  

### 📌 Project Outcome

This system delivers a **low-cost, efficient**, and **scalable** solution for indoor air quality management.  
It combines smart ventilation, sensor fusion, and real-time monitoring, providing a strong foundation for future **IoT-enabled environmental control systems**.

