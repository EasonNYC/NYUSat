# NYU CubeSAT Mission Science Payload
## Real-time Embedded Systems for Space Applications (2016-2017)

<div align="center">
  <img src="https://img.shields.io/badge/Platform-STM32F407VTG-blue" alt="Platform">
  <img src="https://img.shields.io/badge/RTOS-FreeRTOS-green" alt="RTOS">
  <img src="https://img.shields.io/badge/Language-C%2FC%2B%2B-orange" alt="Language">
  <img src="https://img.shields.io/badge/Size-1U%20CubeSat-purple" alt="Size">
  <img src="https://img.shields.io/badge/Mission-90%20days-red" alt="Mission">
</div>

---

<div align="center">
  <img src="https://easonnyc.github.io/portfolio/assets/images/cubesat.jpg" alt="NYU CubeSAT" width="600">
</div>

## 📡 Project Overview

As the **embedded software engineer** on NYU's CubeSAT team, I developed the mission science payload system for a 1U CubeSAT designed to collect weather-related scientific data as part of **NASA's CubeSat Launch Initiative (CSLI)**.

This project involved designing and implementing a sophisticated multi-sensor platform using FreeRTOS on STM32 microcontrollers, with custom PCB design, real-time data acquisition, and space-grade reliability requirements.

### 🚀 Mission Stats
- **Duration**: 90-day orbital mission
- **Sensors**: 5 integrated sensors (GPS, Geiger Counter, Humidity/Temperature, Pressure, IMU)
- **Hardware**: 2 custom PCB modules designed in Altium
- **Performance**: Real-time performance with sub-millisecond response requirements
- **Team Size**: 5 interdisciplinary engineers

---

## ⚙️ Technical Implementation

### Core Architecture
- **Microcontroller**: STM32F407VTG ARM Cortex-M4 @ 100MHz
- **RTOS**: FreeRTOS with multi-threaded sensor management
- **Communication**: UART, I2C, SPI, CAN protocols
- **Development**: Eclipse IDE with ARM-GCC toolchain
- **Debug**: Segger J-Link EDU with SWD interface

### 🛰️ Sensor Integration & Algorithms

#### GPS Module (Venus638FLPx)
- Binary protocol implementation with state machine parsing
- 1PPS interrupt-driven timing synchronization
- Upper/lower half threading for real-time constraints

#### Radiation Detection (Geiger Counter)
- Custom 450V boost circuit design (simulated in LTSpice)
- Sliding window algorithm for clicks-per-minute calculation
- External interrupt handling with timer-based averaging

#### Environmental Sensors
- **SI7021** Humidity/Temperature sensor via I2C
- **BMP085** Pressure sensor with EOC pin interrupt optimization
- Mutex-based I2C bus arbitration for multi-device access

### ⚡ Power Management
- **TPS62162** voltage regulation (16.8V → 3.3V)
- **TPS22994** I2C load sequencing to prevent in-rush current
- **INA219** current monitoring and battery voltage sensing

---

## 🧠 Key Technical Achievements

### Real-Time Multi-Threading Architecture
Implemented sophisticated RTOS synchronization using FreeRTOS:

```c
// GPS processing with interrupt-driven UART handling
void GPS_ProcessThread(void *pvParameters) {
    while(1) {
        // Block until UART interrupt unblocks this thread
        xSemaphoreTake(uartSemaphore, portMAX_DELAY);
        ProcessGPSData();
        // Yield to highest priority ready thread
        taskYIELD();
    }
}
```

### Custom PCB Design
- **Module A**: OBC, GPS, CAN transceiver, power regulation
- **Module B**: Geiger counter, IMU, environmental sensors
- Interlocking PCB stack design for power/communication routing
- 3D modeling and DFM analysis in Altium Designer

### Space-Grade Reliability
- Watchdog timer implementation for autonomous recovery
- Fault-tolerant communication with checksum validation
- Power sequencing to prevent component damage
- EMI/EMC considerations for space environment

---

## 🔧 Development Process

- **Hardware-in-the-loop** testing using STM32F4 Discovery board
- **Mixed-signal simulation** in LTSpice for high-voltage circuits
- **Multi-threaded debugging** with Segger SystemView
- **Version control** with Git and collaborative development

---

## 📊 Engineering Impact

This project demonstrated end-to-end embedded systems development from requirements through flight-ready implementation:

- **Systems Integration**: Successfully coordinated hardware/software co-design with 4 other engineering disciplines
- **Real-World Constraints**: Met NASA's strict reliability, power, and size requirements for space applications
- **Technical Leadership**: Managed complex multi-sensor integration with real-time performance guarantees
- **Innovation**: Developed custom algorithms for radiation detection and GPS synchronization in resource-constrained environment

The mission science payload was designed to autonomously collect and process scientific data during the satellite's 90-day orbital mission.

---

## 🛠️ Technical Skills Demonstrated

### Embedded Programming
- ARM Cortex-M4 firmware development in C/C++
- FreeRTOS real-time operating system
- STM32CubeMX hardware abstraction layer
- Multi-threaded synchronization and IPC

### Hardware Design
- Schematic capture and PCB layout (Altium Designer)
- Mixed-signal circuit simulation (LTSpice)
- Power management and load sequencing
- Multi-layer PCB stackup design

### Communication Protocols
- UART, I2C, SPI, CAN bus implementation
- Binary protocol parsing and state machines
- Interrupt-driven data acquisition
- Fault-tolerant communication design

### Development Tools
- Eclipse IDE with ARM-GCC toolchain
- Segger J-Link debugging with SWD
- Oscilloscope and logic analyzer debugging
- Git version control and collaborative development

---

## 🔗 Links & Resources

- **NASA CubeSAT Info**: [NASA CubeSat Launch Initiative](https://www.nasa.gov/directorates/heo/home/CubeSats_initiative)

---

## 👥 Team Members

| Name | Role | Graduation |
|------|------|------------|
| **Eason Smith** | Embedded SW Engineer (Mission Science Payload Lead) | BS E.E. 2017 |
| Dymytro Moyseyev | Radio/Antenna modules | BS E.E. 2017 |
| Danny Chiang | Power/Battery/Solar Array module | BS E.E. 2017 |
| Matt Cocca | Mechanical Design/Radio/Antenna Release | BS Comp.E. 2017 |
| Abhimanyu Ghosh | Primary On Board Computer | MS E.E. 2017 |

---

## 🏗️ Hardware Requirements

- **Primary MCU**: STM32F407VGT6 ARM Cortex-M4
- **Development Board**: STM32F4 Discovery (for prototyping)
- **Debug Interface**: Segger J-Link EDU with SWD

## 📦 Software Requirements

- **HAL**: STM32CubeMX / HAL / CMSIS library
- **RTOS**: FreeRTOS 9 (non-STM32CubeMX edition)
- **IDE**: Eclipse with ARM-GCC toolchain
- **Simulation**: LTSpice for circuit analysis

---

## 🌟 Currently Supported Sensors

- ✅ **Venus638FLPx** (GPS with 1PPS)
- ✅ **Si7021** (Humidity and Temperature)
- ✅ **Geiger Counter** (Radiation Detection)

---

## 📄 License

This project was developed as part of academic research at NYU Tandon School of Engineering. Please contact the team for usage permissions.

## 📧 Contact

**Eason Smith** - Lead Embedded Software Engineer  
📧 [Eason@EasonRobotics.com](mailto:Eason@EasonRobotics.com)  
🌐 [EasonRobotics.com](http://www.easonrobotics.com)  
💼 [LinkedIn](https://linkedin.com/in/easonsmith)