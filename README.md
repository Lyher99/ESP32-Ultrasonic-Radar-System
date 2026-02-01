# ESP32 Ultrasonic Radar System

A radar-style object detection system using ESP32, HC-SR04 ultrasonic sensor, and SG90 servo motor with real-time visualization in Processing.

## Features

- 180° scanning radar with ultrasonic distance measurement
- Real-time radar visualization with Processing
- BFS (Breadth-First Search) object clustering algorithm
- Object detection and centroid tracking
- Median filtering for stable distance readings

## Hardware Requirements

| Component | Quantity | Description |
|-----------|----------|-------------|
| ESP32 | 1 | ESP32 Development Board |
| HC-SR04 | 1 | Ultrasonic Distance Sensor |
| SG90 | 1 | Micro Servo Motor (9g) |
| Jumper Wires | Several | Male-to-Female recommended |
| Breadboard | 1 | Optional, for prototyping |
| USB Cable | 1 | For programming and power |

## Wiring Diagram

### Pin Connections

```
ESP32          HC-SR04 (Ultrasonic)
─────          ────────────────────
GPIO 5   ───►  TRIG
GPIO 18  ◄───  ECHO
3.3V/5V  ───►  VCC
GND      ───►  GND

ESP32          SG90 (Servo)
─────          ─────────────
GPIO 13  ───►  Signal (Orange/Yellow)
5V       ───►  VCC (Red)
GND      ───►  GND (Brown)
```

### Visual Wiring

```
                    ┌─────────────┐
                    │   HC-SR04   │
                    │ ┌─┐ ┌─┐     │
                    │ │ │ │ │     │
                    └─┴─┴─┴─┴─────┘
                      │ │ │ │
                    VCC│ │TRIG
                      ECHO│GND
                      │ │ │ │
    ┌─────────────────┼─┼─┼─┼─────────────────┐
    │                 │ │ │ │                 │
    │  ┌──────────────┴─┼─┼─┴──────────────┐  │
    │  │ 5V           GPIO18 GPIO5    GND  │  │
    │  │                                   │  │
    │  │            ESP32                  │  │
    │  │                                   │  │
    │  │ 5V            GPIO13         GND  │  │
    │  └───┬─────────────┬─────────────┬───┘  │
    │      │             │             │      │
    └──────┼─────────────┼─────────────┼──────┘
           │             │             │
         ┌─┴─────────────┴─────────────┴─┐
         │  VCC       Signal          GND │
         │         ┌───────┐              │
         │         │ SG90  │              │
         │         │ Servo │              │
         │         └───────┘              │
         └────────────────────────────────┘
```

## Software Requirements

### For ESP32

- [Arduino IDE](https://www.arduino.cc/en/software) (1.8.x or 2.x)
- ESP32 Board Package
- **ESP32Servo** library

### For Visualization

- [Processing](https://processing.org/download) (3.x or 4.x)

## Installation

### Step 1: Install Arduino IDE & ESP32 Board

1. Download and install Arduino IDE
2. Open Arduino IDE → **File** → **Preferences**
3. Add this URL to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to **Tools** → **Board** → **Boards Manager**
5. Search for "ESP32" and install **esp32 by Espressif Systems**

### Step 2: Install ESP32Servo Library

1. Open Arduino IDE → **Sketch** → **Include Library** → **Manage Libraries**
2. Search for "ESP32Servo"
3. Install **ESP32Servo** by Kevin Harrington

### Step 3: Upload ESP32 Code

1. Open `ESP32.ino` in Arduino IDE
2. Select your board: **Tools** → **Board** → **ESP32 Dev Module**
3. Select the correct COM port: **Tools** → **Port** → **COMx**
4. Click **Upload** button

### Step 4: Install Processing

1. Download and install [Processing](https://processing.org/download)
2. Open `Radar_UI_Processing..pde` in Processing

### Step 5: Configure Serial Port

In `Radar_UI_Processing..pde`, check this line:

```java
esp = new Serial(this, Serial.list()[0], 115200);
```

- Run the Processing sketch once to see available serial ports in the console
- Change `[0]` to the correct index for your ESP32's COM port
- Example: If ESP32 is on COM5 and it's listed as index 2, use `Serial.list()[2]`

## Usage

1. **Connect Hardware**: Wire all components as shown in the wiring diagram
2. **Upload ESP32 Code**: Upload `ESP32.ino` to your ESP32
3. **Run Processing**: Open and run `Radar_UI_Processing..pde`
4. **View Radar**: The radar display will show:
   - **Green sweep line**: Current scanning angle
   - **Red dots**: Detected objects (raw readings)
   - **Yellow dot**: Detected object cluster centroid (BFS result)

## Configuration

### ESP32 Settings (ESP32.ino)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ANGLE_MIN` | 0 | Minimum scan angle (degrees) |
| `ANGLE_MAX` | 180 | Maximum scan angle (degrees) |
| `MAX_DISTANCE_CM` | 200 | Maximum detection range (cm) |
| `SERVO_STEP_DEG` | 2 | Angle step per movement |
| `MOVE_MS` | 60 | Delay between servo movements (ms) |
| `GRID` | 32 | Grid size for BFS algorithm |
| `CELL_CM` | 10.0 | Grid cell size in cm |

### Processing Settings (Radar_UI_Processing..pde)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `maxCM` | 200 | Maximum display range (match ESP32) |
| `GRID` | 32 | Grid size (match ESP32) |
| `CELL_CM` | 10.0 | Cell size (match ESP32) |

## Serial Communication Protocol

The ESP32 sends two types of messages via Serial at **115200 baud**:

### Distance Data
```
angle,distance
```
- `angle`: Current servo angle (0-180)
- `distance`: Measured distance in cm (-1 if out of range)

Example: `45,123` (angle 45°, distance 123cm)

### Object Detection Data
```
OBJ,size,cx,cy
```
- `size`: Number of grid cells in detected cluster
- `cx`: Centroid X position in grid units
- `cy`: Centroid Y position in grid units

Example: `OBJ,15,8.50,12.30`

## How It Works

### Scanning Process

1. Servo sweeps from 0° to 180° and back continuously
2. At each angle, ultrasonic sensor measures distance (median of 3 readings)
3. Distance data is sent to Processing for real-time display
4. Points are mapped to a 32x32 grid for object detection

### Object Detection (BFS Algorithm)

1. Each distance reading is converted to X,Y coordinates
2. Points are stored in a 2D grid (32x32, each cell = 10cm)
3. At the end of each sweep, BFS finds connected clusters
4. Largest cluster is identified as the main object
5. Cluster centroid is calculated and sent to Processing

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No serial data | Check COM port selection in Processing |
| Servo not moving | Verify GPIO 13 connection and 5V power |
| Incorrect distances | Check TRIG/ECHO wiring (GPIO 5, 18) |
| Erratic readings | Ensure stable power supply to ultrasonic sensor |
| Processing crashes | Close Arduino Serial Monitor (conflicts with Processing) |

## Project Structure

```
RaderProject/
├── ESP32.ino                    # ESP32 Arduino code
├── Radar_UI_Processing..pde     # Processing visualization
└── README.md                    # This file
```

## License

This project is open source and available for educational purposes.

## Contributing

Feel free to submit issues and pull requests for improvements!
