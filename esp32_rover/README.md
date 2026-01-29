# ESP32 Rover - micro-ROS Firmware

Firmware ESP32 untuk rover dengan kontrol via micro-ROS over WiFi.

## Arsitektur

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32 Firmware                        │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌─────────────────┐      ┌─────────────────────────┐  │
│  │ MicroROSHandler │      │    MotorController      │  │
│  │   - WiFi       │──────▶│   - Differential Drive  │  │
│  │   - cmd_vel    │       │   - PWM Control         │  │
│  │   - watchdog   │       │   - Direction Control   │  │
│  └─────────────────┘      └─────────────────────────┘  │
│                                    │                     │
│                                    ▼                     │
│                           ┌───────────────┐             │
│                           │ Motor Driver  │             │
│                           │   (L298N)     │             │
│                           └───────────────┘             │
│                                    │                     │
│                           ┌───────┴───────┐             │
│                           ▼               ▼             │
│                       Motor L         Motor R           │
└─────────────────────────────────────────────────────────┘
```

## Konfigurasi

### 1. WiFi & Agent

Edit `platformio.ini` atau `src/main.cpp`:

```cpp
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define AGENT_IP "192.168.1.100"  // IP komputer yang running micro-ROS agent
#define AGENT_PORT 8888
```

### 2. Pin Motor

Edit di `src/main.cpp`:

```cpp
// Left Motor
#define LEFT_PWM_PIN    25  // ENA on L298N
#define LEFT_DIR1_PIN   26  // IN1 on L298N
#define LEFT_DIR2_PIN   27  // IN2 on L298N

// Right Motor
#define RIGHT_PWM_PIN   14  // ENB on L298N
#define RIGHT_DIR1_PIN  12  // IN3 on L298N
#define RIGHT_DIR2_PIN  13  // IN4 on L298N
```

## Build & Upload

```bash
# Install PlatformIO jika belum
pip install platformio

# Build
cd esp32_rover
pio run

# Upload ke ESP32
pio run --target upload

# Monitor serial
pio device monitor
```

## Running micro-ROS Agent

Di komputer (dengan ROS2 Humble):

```bash
# Install micro-ROS agent
sudo apt install ros-humble-micro-ros-agent

# Run agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Testing

```bash
# Test publish cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Stop motors
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Wiring Diagram (L298N)

```
ESP32          L298N
─────          ─────
GPIO 25 ─────▶ ENA (Left PWM)
GPIO 26 ─────▶ IN1 (Left Dir1)
GPIO 27 ─────▶ IN2 (Left Dir2)
GPIO 14 ─────▶ ENB (Right PWM)
GPIO 12 ─────▶ IN3 (Right Dir1)
GPIO 13 ─────▶ IN4 (Right Dir2)
GND ──────────▶ GND
5V ───────────▶ 5V (logic power)

Power Supply ─▶ +12V (motor power)
Power Supply ─▶ GND
```

## Troubleshooting

1. **WiFi tidak connect**: Cek SSID dan password
2. **Agent tidak terdeteksi**: Pastikan agent running dan IP sudah benar
3. **Motor tidak bergerak**: Cek wiring dan power supply motor driver
4. **Timeout/disconnect**: Pastikan WiFi stabil, cek firewall
