# FARC Robot Codebase - Team F3DN43a

[![Arduino](https://img.shields.io/badge/Arduino-ESP32-blue.svg)](https://www.arduino.cc/)
[![Competition](https://img.shields.io/badge/Competition-FARC%202025-red.svg)](https://www.facebook.com/FPTUAiroboticschallenge/)

*[English](#english) | [Tiếng Việt](#tiếng-việt)*

---

## English

### Project Overview

Open-source robotics codebase for **FPTU AI & Robotics Challenge (FARC) 2025** competition. This repository contains a professional-grade robot control system featuring advanced motor control, 3D printer-style homing sequences, and comprehensive safety systems.

**Team Information:**
- **Team ID:** FU433
- **Team Name:** F3DN43a
- **Competition:** FPTU AI & Robotics Challenge 2025
- **School:** FPT High School Da Nang

### Unique Features

#### 3D Printer-Style Homing Sequence
- **4-phase precision homing** (Fast Approach → Retraction → Slow Approach → Final Position)
- **Marlin firmware inspired** implementation with bump distance and precision positioning
- **Safety timeouts** and emergency abort functionality
- **Visual feedback** through LED status indicators

#### Advanced Motor Control System
- **Electromagnetic braking** with mixed-decay control (fast/slow decay phases)
- **Infinite brake time** support for outtake mechanism
- **Speed ramping** and acceleration control
- **Direction change protection** with automatic braking

#### PS2 Controller Integration
- **Comprehensive safety monitoring** with automatic shutdown
- **Input validation** and debouncing
- **Emergency override** capabilities
- **Real-time connection status** tracking

#### Modular Architecture
- **State machine design** for reliable operation
- **Centralized configuration** system (config.h)
- **Hardware abstraction layer** (HAL) for easy porting
- **Comprehensive error handling** and recovery

#### LED Status Feedback System
- **Real-time visual feedback** for all robot states
- **Priority-based status** indication
- **Homing progress** visualization
- **Error and warning** notifications

#### VIA "Banh Mi" v2023 Board Support
- **Native ESP32-WROOM-32E** support
- **PCA9685 PWM driver** integration
- **Optimized pin configuration** for competition robotics
- **Hardware-specific optimizations**

### Hardware Requirements

#### Core Components
- **Microcontroller:** ESP32-WROOM-32E (VIA "banh mi" v2023 board)
- **PWM Driver:** PCA9685 16-channel PWM controller
- **Motors:** TA6586 H-bridge compatible DC motors
- **Servos:** MG996R or compatible servos
- **Controller:** PS2 DualShock controller with wireless adapter

#### Sensors & Components
- **Limit Switch:** Mechanical endstop for homing
- **Ultrasonic Sensor:** HC-SR04 (optional)
- **IMU:** MPU6050/MPU6000 (optional)
- **IR Obstacle Detector** (optional)
- **WS2812B LED Strip** for status indication

#### Pin Configuration
```
ESP32 GPIO Pins:
├── PS2 Controller (SPI)
│   ├── GPIO 12: PS2_CLK
│   ├── GPIO 13: PS2_CMD  
│   ├── GPIO 14: PS2_ATT
│   └── GPIO 15: PS2_DAT
├── I2C (PCA9685)
│   ├── GPIO 21: SDA
│   └── GPIO 22: SCL
├── Limit Switch
│   └── GPIO 32: Limit switch input
└── LED Strip
    └── GPIO 25: WS2812B data
```

### Software Dependencies

#### Required Libraries
```cpp
// Core libraries (install via Arduino Library Manager)
#include <Wire.h>              // I2C communication
#include <Adafruit_PWMServoDriver.h>  // PCA9685 control
#include <PsxNewLib.h>         // PS2 controller
#include <FastLED.h>           // WS2812B LED control
```

#### Development Environment
- **Arduino IDE** 2.0+ or **PlatformIO**
- **ESP32 Arduino Core** 2.0.0+
- **Board Selection:** "ESP32 Dev Module"

### Quick Start Guide

#### 1. Hardware Setup
1. Connect ESP32 to PCA9685 via I2C (GPIO 21/22)
2. Wire PS2 controller to SPI pins (GPIO 12-15)
3. Connect limit switch to GPIO 32
4. Wire motors through TA6586 H-bridges to PCA9685
5. Connect servos to PCA9685 channels
6. Optional: Connect LED strip to GPIO 25

#### 2. Software Installation
```bash
# Clone repository
git clone https://github.com/quan5839/FARC_FU0433_F3DN43a.git
cd FARC_FU0433_F3DN43a

# Install required libraries in Arduino IDE:
# - Adafruit PWM Servo Driver Library
# - PsxNewLib
# - FastLED

# Open robot/robot.ino in Arduino IDE
# Select Board: "ESP32 Dev Module"
# Select Port: Your ESP32 port
# Upload code
```

#### 3. Configuration
Edit `robot/src/config.h` to match your hardware setup:
```cpp
// Motor configuration
constexpr bool INVERT_DRIVE_LEFT = false;
constexpr bool INVERT_DRIVE_RIGHT = true;

// Homing configuration  
constexpr bool ENABLE_STARTUP_HOMING = true;
constexpr int HOMING_FAST_SPEED_PERCENT = 50;

// Safety settings
constexpr bool LIMIT_SWITCH_DISABLED = false;
```

### Controller Mapping

#### Drive Controls
- **Left Joystick:** Tank drive (left motor)
- **Right Joystick:** Tank drive (right motor)

#### Outtake Controls
- **L1:** Outtake forward (intake)
- **L2:** Outtake reverse (outtake)
- **SELECT:** Toggle control lock / Auto reverse mode

#### Servo Controls
- **R2:** Toggle ball servo (open/close)
- **Circle:** Toggle fruit servo (open/close)
- **Square/Cross:** Continuous fruit intake servos

#### Special Commands
- **START:** Emergency abort (during homing)
- **Right D-pad:** Toggle limit switch override
- **SELECT + START:** System diagnostics

### Homing Sequence Operation

The robot automatically performs a 3D printer-style homing sequence on startup:

1. **Phase 1 - Fast Approach (50% speed)**
   - Moves outtake downward until limit switch triggers
   - LED: Fast white pulse

2. **Phase 2 - Retraction (200ms)**
   - Moves upward to clear limit switch
   - LED: Slow white pulse

3. **Phase 3 - Slow Approach (15% speed)**
   - Precise downward movement until limit switch triggers
   - LED: Very slow white pulse

4. **Phase 4 - Final Position (500ms)**
   - Moves to safe operating position above home
   - LED: White breathing

**Emergency Abort:** Press START button during homing to abort sequence.

### Safety Features

#### Automatic Safety Systems
- **Controller timeout** monitoring (1 second)
- **Limit switch protection** (prevents damage)
- **Motor stall detection** (current-based)
- **Emergency stop** functionality

#### Manual Safety Controls
- **Limit switch override** (Right D-pad)
- **Emergency abort** (START button)
- **Control lock mode** (SELECT button)

### Competition vs Development Mode

Toggle between modes in `robot/src/config.h`:
```cpp
constexpr bool COMPETITION_MODE = true;  // Disable debug output for performance
```

**Competition Mode:**
- ✅ Maximum performance
- ✅ No debug output
- ✅ Optimized timing
- ✅ Reduced memory usage

**Development Mode:**
- ✅ Detailed debug output
- ✅ Error diagnostics
- ✅ Performance monitoring
- ✅ Safety warnings

### Code Structure

```
robot/
├── robot.ino              # Main Arduino sketch
├── src/
│   ├── config.h           # Central configuration
│   ├── robot/
│   │   ├── robot.h        # Main robot class
│   │   └── robot.cpp      # Robot implementation
│   ├── hal/               # Hardware Abstraction Layer
│   │   ├── motor.h        # Motor control
│   │   ├── motor.cpp      # Motor implementation
│   │   └── pca9685_driver.h # PWM driver
│   ├── controller/        # PS2 controller handling
│   ├── safety/           # Safety monitoring
│   ├── led/              # LED status system
│   └── utils/            # Utility functions
└── Component Document/    # Hardware documentation
    ├── ESP32 datasheet
    ├── PCA9685 manual
    ├── Motor driver specs
    └── Sensor documentation
```

### Adding New Features

#### 1. Adding a New Sensor
```cpp
// 1. Add pin definition in config.h
constexpr uint8_t NEW_SENSOR_PIN = 26;

// 2. Add initialization in robot.cpp init()
pinMode(NEW_SENSOR_PIN, INPUT);

// 3. Add reading function
int readNewSensor() {
    return digitalRead(NEW_SENSOR_PIN);
}

// 4. Use in main loop
if (readNewSensor()) {
    // Handle sensor input
}
```

#### 2. Modifying Motor Behavior
```cpp
// Edit robot/src/config.h
constexpr int DRIVE_MAX_SPEED_PERCENT = 75;  // Reduce max speed
constexpr int ACCELERATION_TIME_MS = 200;    // Slower acceleration
```

#### 3. Adding New Controller Commands
```cpp
// In handleSpecialCommands() function
if (controllerState.triangle_pressed) {
    // Add your custom command here
    DEBUG_PRINTLN("Custom command executed");
}
```

### Troubleshooting

#### Common Issues

**Robot doesn't respond to controller:**
```cpp
// Check PS2 connection in Serial Monitor
DEBUG_PRINTLN("PS2 Controller Status: " + String(ps2x.readType()));
```

**Motors not moving:**
```cpp
// Verify PCA9685 I2C connection
Wire.beginTransmission(0x40);  // PCA9685 default address
if (Wire.endTransmission() == 0) {
    DEBUG_PRINTLN("PCA9685 connected");
} else {
    ERROR_PRINTLN("PCA9685 not found");
}
```

**Homing sequence fails:**
```cpp
// Check limit switch wiring
DEBUG_PRINTLN("Limit switch state: " + String(digitalRead(LIMIT_SWITCH_PIN)));
```

**Compilation errors:**
- Ensure all required libraries are installed
- Check ESP32 board package version (2.0.0+)
- Verify correct board selection: "ESP32 Dev Module"

#### Debug Output
Enable detailed debugging in `robot/src/config.h`:
```cpp
constexpr bool COMPETITION_MODE = false;  // Enable debug output
constexpr bool ENABLE_DEBUG_OUTPUT = true;
```

### Component Documentation

The `Component Document/` folder contains comprehensive documentation for all supported hardware:

#### Available Components
- **ESP32-WROOM-32E** - Main microcontroller datasheet
- **PCA9685** - 16-channel PWM driver manual
- **TA6586** - H-bridge motor driver specifications
- **MG996R** - Servo motor documentation
- **HC-SR04** - Ultrasonic sensor guide
- **MPU6050/6000** - IMU sensor documentation
- **PS2 Controller** - Communication protocol reference
- **Mechanical Endstop** - Limit switch specifications
- **IR Obstacle Detector** - Infrared sensor guide
- **VIA Board Schematics** - Complete board documentation

#### Useful Notes
Check the `Note/` folder for:
- **Useful links.md** - Collection of helpful resources
- **ESP32 Technical Reference** - Detailed ESP32 documentation
- **Competition tips** and best practices

### 🤝 Contributing

We welcome contributions from the FARC robotics community!

#### How to Contribute
1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to the branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

#### Code Style Guidelines
- Use **descriptive variable names**
- Add **comments** for complex logic
- Follow **Arduino coding standards**
- Update **documentation** for new features
- Test on **actual hardware** before submitting

#### Reporting Issues
- Use the **GitHub Issues** tab
- Provide **detailed description** of the problem
- Include **hardware configuration**
- Add **error messages** or **unexpected behavior**
- Specify **Arduino IDE version** and **board package version**

### 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

### 📞 Contact

- **Team Lead:** [Your Name]
- **Email:** [your.email@fpt.edu.vn]
- **University:** FPT University
- **Competition:** FPTU AI & Robotics Challenge 2025

### 🙏 Acknowledgments

- **FPTU AI & Robotics Challenge** organizers
- **FPT University** for hardware support
- **Arduino** and **ESP32** communities
- **Open-source robotics** community
- **VIA Electronics** for the "banh mi" v2023 board

---

## Tiếng Việt

### 🏆 Tổng Quan Dự Án

Mã nguồn mở cho robot tham gia **FPTU AI & Robotics Challenge (FARC) 2025**. Repository này chứa hệ thống điều khiển robot chuyên nghiệp với tính năng điều khiển động cơ tiên tiến, chuỗi homing kiểu máy in 3D, và hệ thống an toàn toàn diện.

**Thông Tin Đội:**
- **Mã Đội:** FU433 F3DN43a
- **Cuộc Thi:** FPTU AI & Robotics Challenge 2025
- **Trường:** Đại học FPT
- **Giấy Phép:** MIT License

### ✨ Tính Năng Độc Đáo

#### 🎯 Chuỗi Homing Kiểu Máy In 3D
- **Homing 4 pha chính xác** (Tiếp cận nhanh → Rút lui → Tiếp cận chậm → Vị trí cuối)
- **Triển khai lấy cảm hứng từ firmware Marlin** với khoảng cách bump và định vị chính xác
- **Timeout an toàn** và chức năng hủy khẩn cấp
- **Phản hồi trực quan** qua đèn LED báo trạng thái

#### ⚡ Hệ Thống Điều Khiển Động Cơ Tiên Tiến
- **Phanh điện từ** với điều khiển mixed-decay (pha fast/slow decay)
- **Hỗ trợ thời gian phanh vô hạn** cho cơ cấu outtake
- **Tăng tốc từ từ** và điều khiển gia tốc
- **Bảo vệ thay đổi hướng** với phanh tự động

#### 🎮 Tích Hợp Tay Cầm PS2
- **Giám sát an toàn toàn diện** với tắt máy tự động
- **Xác thực đầu vào** và chống nhiễu
- **Khả năng ghi đè khẩn cấp**
- **Theo dõi trạng thái kết nối** thời gian thực

#### 🏗️ Kiến Trúc Modular
- **Thiết kế state machine** cho hoạt động đáng tin cậy
- **Hệ thống cấu hình tập trung** (config.h)
- **Lớp trừu tượng phần cứng** (HAL) để dễ dàng chuyển đổi
- **Xử lý lỗi toàn diện** và phục hồi

#### 💡 Hệ Thống Phản Hồi LED
- **Phản hồi trực quan thời gian thực** cho tất cả trạng thái robot
- **Báo hiệu dựa trên độ ưu tiên**
- **Hiển thị tiến trình homing**
- **Thông báo lỗi và cảnh báo**

#### 🔧 Hỗ Trợ Board VIA "Bánh Mì" v2023
- **Hỗ trợ ESP32-WROOM-32E** nguyên bản
- **Tích hợp driver PWM PCA9685**
- **Cấu hình pin tối ưu** cho robot thi đấu
- **Tối ưu hóa đặc thù phần cứng**

### 🔧 Yêu Cầu Phần Cứng

#### Linh Kiện Cốt Lõi
- **Vi điều khiển:** ESP32-WROOM-32E (board VIA "bánh mì" v2023)
- **Driver PWM:** PCA9685 điều khiển PWM 16 kênh
- **Động cơ:** Động cơ DC tương thích H-bridge TA6586
- **Servo:** MG996R hoặc servo tương thích
- **Tay cầm:** Tay cầm PS2 DualShock với adapter không dây

#### Cảm Biến & Linh Kiện
- **Limit Switch:** Endstop cơ học cho homing
- **Cảm biến siêu âm:** HC-SR04 (tùy chọn)
- **IMU:** MPU6050/MPU6000 (tùy chọn)
- **Cảm biến hồng ngoại** phát hiện vật cản (tùy chọn)
- **Dải LED WS2812B** để báo trạng thái

### 🚀 Hướng Dẫn Bắt Đầu Nhanh

#### 1. Thiết Lập Phần Cứng
1. Kết nối ESP32 với PCA9685 qua I2C (GPIO 21/22)
2. Nối tay cầm PS2 với các pin SPI (GPIO 12-15)
3. Kết nối limit switch với GPIO 32
4. Nối động cơ qua H-bridge TA6586 với PCA9685
5. Kết nối servo với các kênh PCA9685
6. Tùy chọn: Kết nối dải LED với GPIO 25

#### 2. Cài Đặt Phần Mềm
```bash
# Clone repository
git clone https://github.com/quan5839/FARC_FU0433_F3DN43a.git
cd FARC_FU0433_F3DN43a

# Cài đặt thư viện cần thiết trong Arduino IDE:
# - Adafruit PWM Servo Driver Library
# - PsxNewLib
# - FastLED

# Mở robot/robot.ino trong Arduino IDE
# Chọn Board: "ESP32 Dev Module"
# Chọn Port: Port ESP32 của bạn
# Upload code
```

#### 3. Cấu Hình
Chỉnh sửa `robot/src/config.h` để phù hợp với thiết lập phần cứng:
```cpp
// Cấu hình động cơ
constexpr bool INVERT_DRIVE_LEFT = false;
constexpr bool INVERT_DRIVE_RIGHT = true;

// Cấu hình homing
constexpr bool ENABLE_STARTUP_HOMING = true;
constexpr int HOMING_FAST_SPEED_PERCENT = 50;

// Cài đặt an toàn
constexpr bool LIMIT_SWITCH_DISABLED = false;
```

### 🎮 Sơ Đồ Tay Cầm

#### Điều Khiển Di Chuyển
- **Joystick trái:** Tank drive (động cơ trái)
- **Joystick phải:** Tank drive (động cơ phải)

#### Điều Khiển Outtake
- **L1:** Outtake tiến (intake)
- **L2:** Outtake lùi (outtake)
- **SELECT:** Chuyển đổi khóa điều khiển / Chế độ tự động lùi

#### Điều Khiển Servo
- **R2:** Chuyển đổi servo bóng (mở/đóng)
- **Circle:** Chuyển đổi servo trái cây (mở/đóng)
- **Square/Cross:** Servo intake trái cây liên tục

#### Lệnh Đặc Biệt
- **START:** Hủy khẩn cấp (trong quá trình homing)
- **Right D-pad:** Chuyển đổi ghi đè limit switch
- **SELECT + START:** Chẩn đoán hệ thống

### 🏠 Hoạt Động Chuỗi Homing

Robot tự động thực hiện chuỗi homing kiểu máy in 3D khi khởi động:

1. **Pha 1 - Tiếp Cận Nhanh (50% tốc độ)**
   - Di chuyển outtake xuống cho đến khi limit switch kích hoạt
   - LED: Nhấp nháy trắng nhanh

2. **Pha 2 - Rút Lui (200ms)**
   - Di chuyển lên để thoát khỏi limit switch
   - LED: Nhấp nháy trắng chậm

3. **Pha 3 - Tiếp Cận Chậm (15% tốc độ)**
   - Di chuyển xuống chính xác cho đến khi limit switch kích hoạt
   - LED: Nhấp nháy trắng rất chậm

4. **Pha 4 - Vị Trí Cuối (500ms)**
   - Di chuyển đến vị trí hoạt động an toàn phía trên home
   - LED: Thở trắng

**Hủy Khẩn Cấp:** Nhấn nút START trong quá trình homing để hủy chuỗi.

### 🛡️ Tính Năng An Toàn

#### Hệ Thống An Toàn Tự Động
- **Giám sát timeout tay cầm** (1 giây)
- **Bảo vệ limit switch** (ngăn hư hỏng)
- **Phát hiện động cơ bị kẹt** (dựa trên dòng điện)
- **Chức năng dừng khẩn cấp**

#### Điều Khiển An Toàn Thủ Công
- **Ghi đè limit switch** (Right D-pad)
- **Hủy khẩn cấp** (nút START)
- **Chế độ khóa điều khiển** (nút SELECT)

### 🏁 Chế Độ Thi Đấu vs Phát Triển

Chuyển đổi giữa các chế độ trong `robot/src/config.h`:
```cpp
constexpr bool COMPETITION_MODE = true;  // Tắt debug output để tối ưu hiệu suất
```

**Chế độ Thi Đấu:**
- ✅ Hiệu suất tối đa
- ✅ Không có debug output
- ✅ Timing tối ưu
- ✅ Giảm sử dụng bộ nhớ

**Chế độ Phát Triển:**
- ✅ Debug output chi tiết
- ✅ Chẩn đoán lỗi
- ✅ Giám sát hiệu suất
- ✅ Cảnh báo an toàn

### 📋 Tài Liệu Linh Kiện

Thư mục `Component Document/` chứa tài liệu toàn diện cho tất cả phần cứng được hỗ trợ:

#### Linh Kiện Có Sẵn
- **ESP32-WROOM-32E** - Datasheet vi điều khiển chính
- **PCA9685** - Hướng dẫn driver PWM 16 kênh
- **TA6586** - Thông số kỹ thuật driver động cơ H-bridge
- **MG996R** - Tài liệu servo motor
- **HC-SR04** - Hướng dẫn cảm biến siêu âm
- **MPU6050/6000** - Tài liệu cảm biến IMU
- **Tay cầm PS2** - Tham chiếu giao thức truyền thông
- **Mechanical Endstop** - Thông số kỹ thuật limit switch
- **IR Obstacle Detector** - Hướng dẫn cảm biến hồng ngoại
- **Sơ đồ Board VIA** - Tài liệu board hoàn chỉnh

#### Ghi Chú Hữu Ích
Kiểm tra thư mục `Note/` để có:
- **Useful links.md** - Bộ sưu tập tài nguyên hữu ích
- **ESP32 Technical Reference** - Tài liệu ESP32 chi tiết
- **Mẹo thi đấu** và thực hành tốt nhất

### 🤝 Đóng Góp

Chúng tôi hoan nghênh sự đóng góp từ cộng đồng robot FARC!

#### Cách Đóng Góp
1. **Fork** repository
2. **Tạo** nhánh tính năng (`git checkout -b feature/tinh-nang-tuyet-voi`)
3. **Commit** thay đổi (`git commit -m 'Thêm tính năng tuyệt vời'`)
4. **Push** lên nhánh (`git push origin feature/tinh-nang-tuyet-voi`)
5. **Mở** Pull Request

#### Hướng Dẫn Code Style
- Sử dụng **tên biến mô tả**
- Thêm **comment** cho logic phức tạp
- Tuân theo **chuẩn coding Arduino**
- Cập nhật **tài liệu** cho tính năng mới
- Test trên **phần cứng thực** trước khi submit

### 📄 Giấy Phép

Dự án này được cấp phép theo **MIT License** - xem file [LICENSE](LICENSE) để biết chi tiết.

### 📞 Liên Hệ

- **Trưởng Nhóm:** [Tên của bạn]
- **Email:** [your.email@fpt.edu.vn]
- **Trường:** Đại học FPT
- **Cuộc Thi:** FPTU AI & Robotics Challenge 2025

### 🙏 Lời Cảm Ơn

- **Ban tổ chức FPTU AI & Robotics Challenge**
- **Đại học FPT** vì sự hỗ trợ phần cứng
- **Cộng đồng Arduino** và **ESP32**
- **Cộng đồng robot mã nguồn mở**
- **VIA Electronics** cho board "bánh mì" v2023

---

*Made with ❤️ by Team FU433 F3DN43a for FARC 2025*
