# PSO - Propulsion System Optimizer

[![Platform](https://img.shields.io/badge/platform-TM4C123-blue.svg)](https://www.ti.com/product/TM4C123GH6PM)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-active-success.svg)]()

A real-time embedded data acquisition system for motor and propeller testing, built on the Texas Instruments TM4C123GH6PM microcontroller (Tiva C LaunchPad).

## Features

- **Multi-Channel Data Acquisition**: 6-channel ADC sampling at 5 kHz
- **Real-Time RPM Measurement**: Hardware-based edge counting with 100ms resolution
- **Flexible PWM Control**: Configurable profiles (trapezoid, linear, step sequences)
- **High-Speed Communication**: UART streaming at 115200 baud with CRC validation
- **Debug Capabilities**: Hardware timing pins for oscilloscope analysis
- **SD Card Logging**: Optional FatFS-based data storage

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Getting Started](#getting-started)
  - [Clone the Repository](#1-clone-the-repository)
  - [Install Dependencies](#2-install-dependencies)
  - [Configure Code Composer Studio](#3-configure-code-composer-studio)
  - [Build the Project](#4-build-the-project)
  - [Flash the Firmware](#5-flash-the-firmware)
- [Hardware Setup](#hardware-setup)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Contributing](#contributing)
- [License](#license)

## Hardware Requirements

### Required Components

- **TM4C123GH6PM LaunchPad** (EK-TM4C123GXL)
- **USB Cable** (Micro-B for programming and power)
- **Sensors** (optional for testing):
  - Accelerometers (3-axis)
  - Optical encoder (for RPM)
  - Current/voltage sensors
  - Strain gauge or load cell (for thrust)

### Pin Connections

Refer to the detailed pin assignment table below for connecting your sensors:

| Pin  | Function         | Description                    |
|------|------------------|--------------------------------|
| PA0  | UART RX          | USB serial receive             |
| PA1  | UART TX          | USB serial transmit            |
| PC6  | RPM Input        | Hall sensor/encoder input      |
| PC7  | PWM Output       | Motor ESC control              |
| PD0  | ADC (AIN7)       | Thrust sensor                  |
| PD1  | ADC (AIN6)       | Acceleration X-axis            |
| PD2  | ADC (AIN5)       | Acceleration Y-axis            |
| PD3  | ADC (AIN4)       | Acceleration Z-axis            |
| PE1  | ADC (AIN2)       | Motor voltage                  |
| PE2  | ADC (AIN1)       | Motor current                  |
| PF0  | Button (SW2)     | Start/stop control             |
| PF1  | Red LED          | Error indication               |
| PF2  | Blue LED         | Standby mode                   |
| PF3  | Green LED        | Streaming active               |

## Software Requirements

### Development Tools

1. **Code Composer Studio (CCS)**
   - Version: 12.x or earlier (Eclipse-based IDE)
   - Download: [CCS Download Page](https://www.ti.com/tool/CCSTUDIO)
   - Platform: Windows, Linux, or macOS

2. **TivaWare SDK**
   - Version: Tested on v2.2.x
   - Included with CCS or download separately
   - Contains peripheral driver library

3. **Serial Terminal** (for data monitoring)
   - Settings: 115200 baud, 8N1

### Optional Tools

- **MATLAB/Python**: For data analysis and visualization

## Getting Started

### 1. Clone the Repository

```bash
# Clone the repository
git clone git@github.com:rlima-rogerio/pso.git
cd pso

# If submodules are used
git submodule update --init --recursive
```

### 2. Install Dependencies


#### Linux

```bash
# Install CCS (example for Ubuntu/Debian)
chmod +x CCS<version>_linux-x64.run
sudo ./CCS<version>_linux-x64.run

# Install required libraries
sudo apt-get install libusb-1.0-0-dev libusb-1.0-0
sudo apt-get install libc6:i386  # If using 64-bit OS

# Add USB udev rules for LaunchPad
sudo cp /opt/ti/ccs/install_scripts/99-stellaris-launchpad.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

### 3. Configure Code Composer Studio

#### Import the Project

1. **Launch CCS**
   ```
   Start → Code Composer Studio → CCS 12.x
   ```

2. **Create/Select Workspace**
   ```
   File → Switch Workspace → Select or create a workspace directory
   ```

3. **Import Project**
   ```
   File → Import → Code Composer Studio → CCS Projects → Next
   Select search directory: <cloned-repo-directory>
   ✓ Select "Copy projects into workspace" (optional)
   Finish
   ```

#### Configure TivaWare Path

1. **Set TivaWare Installation Path**
   ```
   Right-click project → Properties
   Resource → Linked Resources → Path Variables
   Add Variable:
     Name: TIVAWARE_ROOT
     Location: C:\ti\TivaWare_C_Series-2.2.x.xxxxx  (Windows)
              /opt/ti/TivaWare_C_Series-2.2.x.xxxxx  (Linux)
   ```

2. **Verify Include Paths**
   ```
   Properties → Build → ARM Compiler → Include Options
   Should contain:
     "${TIVAWARE_ROOT}"
     "${TIVAWARE_ROOT}/inc"
     "${TIVAWARE_ROOT}/driverlib"
   ```

3. **Verify Library Paths**
   ```
   Properties → Build → ARM Linker → File Search Path
   Should contain:
     "${TIVAWARE_ROOT}/driverlib/ccs/Debug/driverlib.lib"
   ```

#### Configure Build Settings

1. **Compiler Optimization**
   ```
   Properties → Build → ARM Compiler → Optimization
   Set to: -O2 (for release) or -Og (for debug)
   ```

2. **Target Configuration**
   ```
   Properties → General → Device
   Device: TM4C123GH6PM
   Connection: Stellaris In-Circuit Debug Interface
   ```

3. **Stack Size** (if needed)
   ```
   Properties → Build → ARM Linker → Basic Options
   Stack Size: 0x800 (2048 bytes - default is usually sufficient)
   ```

### 4. Build the Project

#### GUI Method

```
Project → Build All
or
Click the hammer icon in toolbar
or
Press Ctrl+B
```

#### Command Line Method (Linux/macOS)

```bash
# Navigate to CCS installation
cd /opt/ti/ccs/eclipse

# Build project
./ccstudio -noSplash -data <workspace> -application com.ti.ccstudio.apps.projectBuild -project pso-project -config Debug
```

#### Expected Output

```
**** Build Finished ****
**** Build of configuration Debug for project PSO ****

Finished building target: PSO.out
   text     data     bss     dec     hex filename
  45678     1234    5678   52590    cd6e PSO.out
```

### 5. Flash the Firmware using CCS GUI (Recommended)

1. **Connect LaunchPad** via USB
2. **Select Debug Configuration**
   ```
   Run → Debug Configurations → TM4C123GH6PM → Debug
   ```
3. **Flash and Run**
   ```
   The firmware will automatically flash and start debugging
   Press F8 or click Resume button to run
   ```


## Hardware Setup

### Minimal Test Setup

For initial testing without sensors:

```
TM4C123 LaunchPad
├── USB Cable → PC (power + programming + serial)
└── No additional connections required
```

The system will run with default/test values for sensors.

### Full Sensor Setup

1. **Power Supply**
   - LaunchPad: 5V via USB
   - External sensors: 3.3V from LaunchPad (max 50mA total)
   - Motor ESC: External power supply (according to motor specs)

2. **ADC Connections** (0-3.3V range)
   ```
   Accelerometer X → PD1 (AIN6)
   Accelerometer Y → PD2 (AIN5)
   Accelerometer Z → PD3 (AIN4)
   Voltage Divider → PE1 (AIN2)  [Motor voltage sensing]
   Current Sensor  → PE2 (AIN1)  [Motor current sensing]
   Strain Gauge    → PD0 (AIN7)  [Thrust measurement]
   ```

3. **RPM Input**
   ```
   Hall Sensor / Encoder → PC6 (WT1CCP0)
   - 3.3V logic level
   - Rising edge triggered
   - Internal pull-up enabled
   ```

4. **PWM Output**
   ```
   PC7 (WT1CCP1) → ESC Signal Input
   - 50 Hz frequency
   - 1-2 ms pulse width
   - Standard RC servo/ESC protocol
   ```

5. **Debug Pins** (optional)
   ```
   PD4-PD7 → Oscilloscope probes
   - Monitor timing and execution
   - 3.3V logic levels
   ```

### Safety Considerations

**Important Safety Notes**:

- Always remove propellers during initial testing
- Use proper motor mounting and shielding
- Ensure adequate cooling for motor and ESC
- Keep loose objects away from rotating parts
- Use emergency stop button (SW2) within reach
- Never exceed motor/ESC voltage ratings

## Usage

### Basic Operation Flow

1. **Power On**
   - Connect LaunchPad via USB
   - Blue LED starts blinking (standby mode)

2. **Start Acquisition**
   - Press SW2 button (PF0)
   - System transitions through states:
     ```
     INIT → IDLE → TIMING → PROCESSING → STREAMING
     ```
   - Green LED blinks (streaming active)

3. **Monitor Data**
   - Open serial terminal (115200 baud, 8N1)
   - Data packets stream continuously
   - Format: 21-byte packets with CRC

4. **Stop Acquisition**
   - Press SW2 button again
   - System stops gracefully
   - White LED flashes (completion indicator)

### Serial Terminal Setup

#### Windows (PuTTY)

```
Connection Type: Serial
Serial line: COM# (check Device Manager)
Speed: 115200
Data bits: 8
Stop bits: 1
Parity: None
Flow control: None
```

#### Linux/macOS

```bash
# Find device
ls /dev/tty*  # Look for /dev/ttyACM0 or similar

# Option 1: screen
screen /dev/ttyACM0 115200

# Option 2: minicom
minicom -D /dev/ttyACM0 -b 115200

# Exit: Ctrl+A, then K (screen) or Ctrl+A, X (minicom)
```

### PWM Profile Configuration

Profiles are configured at compile time in `main.c`:

```c
/* Select profile type (uncomment one) */
// #define PWM_PROFILE_TRAPEZOID_SELECTED
#define PWM_PROFILE_LINEAR_SELECTED
// #define PWM_PROFILE_STEP_SELECTED

/* Configure profile parameters */
linear_config_t linear_config = {
    .duration_ms = 10000,      // 10 second ramp
    .start_value = 0,          // Start at 0%
    .end_value = 100,          // End at 100%
    .cycles = 1,               // Run once
    .bidirectional = false,    // One direction only
    .slew_rate = 0.01          // 1% per 100ms
};
```

Rebuild and reflash after changing configuration.

### Data Packet Format

Each packet contains 21 bytes:

```
Byte 0:      STX (0xFE)
Byte 1:      Length (19)
Bytes 2-3:   Packet index
Bytes 4-5:   Accel X (int16)
Bytes 6-7:   Accel Y (int16)
Bytes 8-9:   Accel Z (int16)
Bytes 10-11: RPM (uint16)
Bytes 12-13: Motor current (int16)
Bytes 14-15: Motor voltage (int16)
Bytes 16-17: Thrust (int16)
Byte 18:     Throttle (0-100)
Bytes 19-20: CRC-16
```

### LED Status Indicators

| LED Color | Pattern       | Meaning                        |
|-----------|---------------|--------------------------------|
| Blue      | Slow blink    | Standby/Idle - ready to start  |
| Green     | Fast blink    | Streaming data                 |
| Red       | Fast blink    | Error condition                |
| White     | Flash         | Operation complete             |

## Project Structure

```
pso-propulsion-optimizer/
├── src/
│   ├── main.c                 # Main application and state machine
│   ├── pso_init.c             # Hardware initialization
│   ├── pso_uart.c             # UART communication
│   ├── pso_pwm.c              # PWM profile generation
│   ├── pso_rpm.c              # RPM measurement
│   ├── pso_timing.c           # System timing utilities
│   ├── pso_led.c              # LED control functions
│   ├── pso_data.c             # Data packet handling
│   ├── pso_debug.c            # Debug GPIO utilities
│   ├── pso_isr.c              # Interrupt service routines
│   ├── pso_system.c           # System status indicators
│   ├── fifo.c                 # Circular FIFO buffer
│   ├── ulink.c                # Communication protocol
│   └── checksum.c             # CRC calculation
├── inc/
│   ├── *.h                    # Header files for all modules
│   └── tm4c123gh6pm.h         # MCU register definitions
├── docs/
│   ├── PSO_TECHNICAL_DOCUMENTATION.md
│   ├── pin_diagram.pdf
│   └── schematic.pdf
├── tools/
│   ├── matlab/                # MATLAB analysis scripts
│   └── python/                # Python data parser
├── .cproject                  # Eclipse/CCS project file
├── .project                   # Eclipse/CCS project file
├── tm4c123gh6pm.lds           # Linker script
├── README.md                  # This file
└── LICENSE                    # Project license
```

## Configuration

### Compile-Time Configuration

#### PWM Frequency (pso_pwm.h)

```c
#define PWM_FREQUENCY 50  // Hz (default for RC servos/ESCs)
// Change to 400 for high-frequency ESCs
```

#### ADC Sampling Rate (pso_init.c)

```c
// In PSO_Timers() function
TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 5000);
// 5000 = 5 kHz sampling
// Change divisor to adjust rate
```

#### RPM Blade Count (pso_rpm.c)

```c
#define BLADE_NUMBER 2  // Pulses per revolution
// 2 for 2-pole motor with hall sensor
// Adjust based on your motor/sensor
```

#### UART Baud Rate (pso_init.c)

```c
UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, ...);
// Change 115200 to desired baud rate
```

### Runtime Configuration

Some parameters can be modified without recompilation:

- **Start/Stop**: SW2 button (PF0)
- **Mode Select**: SW1 button (PF4) - future feature
- **Emergency Stop**: SW2 during operation

## Troubleshooting

### Common Issues

#### 1. LaunchPad Not Detected

**Windows:**
```
- Check Device Manager for "Stellaris ICDI"
- Reinstall drivers from CCS installation
- Try different USB port/cable
```

**Linux:**
```bash
# Check if device appears
lsusb | grep "1cbe"

# Add udev rules if missing
sudo cp 99-stellaris-launchpad.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

#### 2. Build Errors

**Missing TivaWare:**
```
Error: cannot find -ldriverlib

Solution:
- Verify TIVAWARE_ROOT path variable
- Check library path in linker settings
- Rebuild driverlib if necessary
```

**Include Path Issues:**
```
Error: tm4c123gh6pm.h not found

Solution:
- Check include paths in project properties
- Ensure TivaWare is properly installed
```

#### 3. Programming Fails

```
Error: Flash verification failed

Solution:
- Update LaunchPad firmware (using LM Flash Programmer)
- Try lower programming speed in debug configuration
- Check for proper USB connection
```

#### 4. No Serial Output

```
Problem: Terminal shows no data

Checks:
✓ Correct COM port selected
✓ Baud rate: 115200
✓ SW2 pressed to start streaming
✓ Green LED blinking (indicates streaming)
✓ USB cable supports data (not just power)
```

### Debug Techniques

#### Using Debug Pins

Connect oscilloscope to PD6 and PD7:
```c
// In your ISR or function
DEBUG_ADC_TOGGLE();  // Toggle PD6
// Measure pulse width = execution time
```

#### UART Debug Messages

Enable debug output:
```c
// In pso_debug.c
#define DEBUG_UART_ENABLED
```

#### CCS Debugger

Set breakpoints and inspect variables:
```
1. Run → Debug (F11)
2. Set breakpoint (double-click line number)
3. Run → Resume (F8)
4. View → Variables (inspect values)
```

## Contributing

Contributions are welcome! Please follow these guidelines:

1. **Fork the repository**
2. **Create a feature branch** (`git checkout -b feature/amazing-feature`)
3. **Commit changes** (`git commit -m 'Add amazing feature'`)
4. **Push to branch** (`git push origin feature/amazing-feature`)
5. **Open a Pull Request**

### Code Style

- Follow existing code formatting
- Add comments for complex logic
- Update documentation as needed
- Test on hardware before submitting

### Reporting Issues

Include in bug reports:
- CCS version
- TivaWare version
- Hardware setup description
- Steps to reproduce
- Expected vs actual behavior

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Original implementation by Rogerio Lima (2015)
- Texas Instruments for TivaWare peripheral library
- ARM Cortex-M4 architecture
- Community contributors and testers


<!-- ## Roadmap

- [ ] Add USB device support for direct PC connection
- [ ] Implement dynamic sampling rate configuration
- [ ] Complete SD card data logging functionality
- [ ] Create MATLAB/Python analysis toolkit
- [ ] Add real-time graphing over UART
- [ ] Implement configuration storage in EEPROM
- [ ] Add support for additional sensor types
- [ ] Create web-based configuration interface -->

---

**Developed with  for embedded systems and propulsion testing**

Last Updated: December 2025
