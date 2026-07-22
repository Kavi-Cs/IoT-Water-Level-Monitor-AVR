## 🗺️ Project Roadmap

**Phase 1: Hardware & PCB Design**
- [x] Initial schematic design for ATmega328P and external modules.
- [x] Integrate JSN-SR04T, SIM800L, ESP-01, SD Card, RTC, and LCD.
- [x] PCB Layout routing with optimized power lines (1mm+ tracks) and Ground planes.
- [x] Clear DRC errors and generate Gerber files.
- [x] PCB manufacturing and hardware assembly (Soldering).

**Phase 2: Firmware Development**
- [x] Sensor interfacing (Rain sensor, JSN-SR04T water level).
- [x] I2C LCD configuration and DS3231 RTC setup for real-time tracking.
- [x] SD Card data logging implementation.
- [x] SIM800L integration for offline SMS alerts.
- [x] ESP-01 Wi-Fi configuration for online data transmission.

**Phase 3: System Integration & Testing**
- [x ] Power stability and voltage drop testing on the custom PCB.
- [x ] Real-world field testing (Simulating flood scenarios and heavy rain).
- [x ] Code optimization and bug fixing.

**Phase 4: Future Enhancements (Upcoming)**
- [ ] Build a Web Dashboard or Mobile App (Flutter/React Native) for real-time monitoring.
- [ ] Implement OTA (Over-The-Air) firmware updates via the ESP-01.
- [ ] Add a Solar charge controller + Battery Management System (BMS) for off-grid operation.
- [ ] Integrate API weather data to predict floods before rain starts.
