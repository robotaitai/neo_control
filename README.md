# Omni-Wheel Robot Control (ESP32-WROOM32)

## Overview
This project implements low-level control, odometry, and velocity closed-loop control for a 4-wheel omni-directional robot using an ESP32-WROOM32 (30-pin) microcontroller. A Jetson NX provides high-level commands over serial.

Key features:
- Motor control via L298N dual-H-bridge drivers
- Quadrature encoder feedback (Hall sensors, 12 CPR, 34:1 gearbox, 4× decoding)
- Differential-drive style kinematics with odometry
- Velocity PID control per wheel
- Serial command (`<V,lin,ang>`) and telemetry (`<O,x,y,θ,v,ω>`) protocol

## Hardware Setup

### Components
- **ESP32 WROOM32 DevKit** (30-pin)
- **2× L298N** dual-H-bridge modules
- **4× GA25 metal gear motors** (97 mm mecanum wheels)
- Quadrature encoders (Hall, yellow/A, white/B)

### Pinout (`include/Config.h`)
```cpp
// Front-Right (IN1/IN2)
constexpr uint8_t FR_PWM_PIN = 18;  // PWM
constexpr uint8_t FR_DIR_PIN = 19;  // DIR
constexpr bool    FR_INVERT  = true;

// Front-Left (IN3/IN4)
constexpr uint8_t FL_PWM_PIN = 16;  // PWM
constexpr uint8_t FL_DIR_PIN = 17;  // DIR

// Rear-Left (IN1/IN2)
constexpr uint8_t RL_PWM_PIN = 25;
constexpr uint8_t RL_DIR_PIN = 26;

// Rear-Right (IN3/IN4)
constexpr uint8_t RR_PWM_PIN = 27;
constexpr uint8_t RR_DIR_PIN = 14;

// Encoders (yellow = A, white = B)
constexpr uint8_t FL_ENC_A_PIN = 23;
constexpr uint8_t FL_ENC_B_PIN = 22;
constexpr uint8_t FR_ENC_A_PIN = 21;
constexpr uint8_t FR_ENC_B_PIN =  2;
constexpr uint8_t RL_ENC_A_PIN = 34;
constexpr uint8_t RL_ENC_B_PIN = 35;
constexpr uint8_t RR_ENC_A_PIN = 32;
constexpr uint8_t RR_ENC_B_PIN = 33;

// Communication & control
constexpr uint32_t CONTROL_HZ       = 100;
constexpr uint32_t CONTROL_PERIOD   = 1000/CONTROL_HZ;
constexpr uint32_t DBG_BAUD         = 115200;
constexpr uint32_t UP_BAUD          = 1500000;
constexpr uint8_t  UP_RX_PIN        = 5;
constexpr uint8_t  UP_TX_PIN        = 4;
```

### Power and Wiring
- **5 V rail** (from L298N jumper) → ESP32 VIN
- **Common GND** between ESP32, L298N, and Jetson NX
- **Serial2**: GPIO 5 (RX) ← Jetson TX, GPIO 4 (TX) → Jetson RX
- **USB-CDC**: Debug logging and optional control via USB

## Mechanical Configuration

See `include/MechConfig.h`:
```cpp
constexpr float WHEEL_DIAMETER_M      = 0.097f;
constexpr uint16_t ENCODER_CPR_MOTOR  = 12;
constexpr uint8_t  GEARBOX_RATIO      = 34;
constexpr uint32_t ENCODER_TICKS_PER_REV = ENCODER_CPR_MOTOR * GEARBOX_RATIO * 4; // quadrature
constexpr float TRACK_WIDTH_M         = 0.20f;
constexpr float WHEEL_BASE_M          = 0.20f;
```

## Control Parameters

See `include/ControlConfig.h`:
```cpp
constexpr uint32_t ODOM_HZ            = 50;
constexpr uint32_t VEL_CTRL_HZ        = 100;
constexpr float    VEL_PID_KP         = 1.0f;
constexpr float    VEL_PID_KI         = 0.1f;
constexpr float    VEL_PID_KD         = 0.01f;
```

## Software Structure

- `include/`
  - `Config.h` — electrical pin & comm settings
  - `MechConfig.h` — mechanical dimensions
  - `ControlConfig.h` — loop rates & PID gains
  - `Encoder.h` — quadrature decoding
  - `MotorDriver.h` — L298N 2-wire driver
  - `PIDController.h` — generic PID
  - `VelocityController.h` — wheel-speed PID wrapper
  - `Odometry.h` — differential-drive odometry
  - `RobotController.h` — high-level control orchestration
  - `CommProtocol.h` — serial framing for commands & telemetry
  - `Logger.h` — debug logging helper

- `src/`
  - Implementation files for each module
  - `main.cpp` — integrates everything, handles timing

## Build & Upload

```bash
# Build in release mode
pio run

# Upload to ESP32
pio run -t upload

# Open serial monitor (USB CDC for debug)
pio device monitor --baud 115200
```

## Running & Testing

1. **Debug over USB**  
   Define `DEBUG_USB_COMMANDS` in `main.cpp` to use Serial for `<V,…>` / `<O,…>`.

2. **Example Commands**  
   - Drive forward at 0.3 m/s:
     ```
     <V,0.30,0.00>
     ```
   - Spin in place at 90°/s:
     ```
     <V,0.00,1.57>
     ```
   - Stop:
     ```
     <V,0.00,0.00>
     ```

3. **Wheel Tests**  
   Use the provided `src/*_ramp_dynamic_brake.cpp` sketches for FL, FR, RL, RR and `src/all_wheels_ramp_test.cpp`.

## Next Steps
- Tune PID gains in `ControlConfig.h`
- Integrate with your high-level planner on Jetson
- Add sensor fusion (IMU, etc.) for improved odometry

---

