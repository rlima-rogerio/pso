# PSO Quick Reference Card

## Pin Assignments

### ADC Inputs (0-3.3V)
```
PD1 (AIN6)  →  Acceleration X-axis
PD2 (AIN5)  →  Acceleration Y-axis
PD3 (AIN4)  →  Acceleration Z-axis
PE1 (AIN2)  →  Motor Voltage
PE2 (AIN1)  →  Motor Current
PD0 (AIN7)  →  Thrust / Strain Gauge
```

### Digital I/O
```
PC6  →  RPM Input (WT1CCP0) - Hall sensor/encoder
PC7  →  PWM Output (WT1CCP1) - Motor ESC control
PF0  →  SW2 Button (start/stop)
PF4  →  SW1 Button (mode select)
```

### LEDs
```
PF1  →  Red LED (error)
PF2  →  Blue LED (standby)
PF3  →  Green LED (streaming)
```

### UART
```
PA0/PA1  →  UART0 (115200 baud) - Main communication
PD6/PD7  →  UART2 (9600 baud) - Alternative
```

### Debug Pins
```
PD4-PD7  →  Timing measurement (oscilloscope)
```

## System Parameters

### Timing
- **System Clock**: 40 MHz
- **ADC Sample Rate**: 5 kHz (200 μs period)
- **RPM Update Rate**: 10 Hz (100 ms period)
- **PWM Frequency**: 50 Hz (20 ms period)
- **UART Baud**: 115200 bps

### PWM Control
- **Pulse Range**: 1000-2000 μs (1-2 ms)
- **Resolution**: 0-100% (10 μs steps)
- **0% Throttle**: 1000 μs pulse
- **50% Throttle**: 1500 μs pulse
- **100% Throttle**: 2000 μs pulse

### Memory
- **Flash**: 256 KB
- **RAM**: 32 KB
- **Stack**: 2 KB
- **FIFO Buffers**: 512 bytes (2x 256 bytes)

## Configuration Macros

### pso_config.h
```c
#define PWM_FREQUENCY         50U      // Hz
#define STREAMING_RATE_HZ     125000U  // Max data rate
#define FIFO_BUFFER_SIZE      256U     // Bytes
```

### main.c - Profile Selection
```c
// Uncomment ONE profile type:
// #define PWM_PROFILE_TRAPEZOID_SELECTED
#define PWM_PROFILE_LINEAR_SELECTED
// #define PWM_PROFILE_STEP_SELECTED
```

### pso_rpm.h
```c
#define BLADE_NUMBER  2  // Pulses per revolution
#define RPM_MIN_VALID 10
#define RPM_MAX_VALID 50000
```

## Data Packet Format

### UART Packet Structure (21 bytes)
```
┌────┬────┬─────┬─────┬─────┬─────┬─────┬─────┬──────┬─────┐
│STX │Len │Idx_H│Idx_L│Acc_X│Acc_Y│Acc_Z│ RPM │I_mot │V_mot│
│0xFE│0x13│     │     │     │     │     │     │      │     │
├────┼────┼─────┼─────┼─────┼─────┼─────┼─────┼──────┼─────┤
│ 1B │ 1B │  1B │  1B │  2B │  2B │  2B │  2B │  2B  │  2B │
└────┴────┴─────┴─────┴─────┴─────┴─────┴─────┴──────┴─────┘

┌───────┬──────┬──────┐
│Thrust │Throt │ CRC  │
│       │      │      │
├───────┼──────┼──────┤
│  2B   │  1B  │  2B  │
└───────┴──────┴──────┘

Total: 21 bytes
```

### Data Types
```c
typedef struct {
    uint16_t index;      // Packet counter
    int16_t  accel[3];   // X, Y, Z acceleration
    uint16_t rpm;        // Revolutions per minute
    int16_t  i_motor;    // Motor current [A]
    int16_t  v_motor;    // Motor voltage [V]
    int16_t  thrust;     // Thrust force [N]
    uint8_t  throttle;   // PWM duty cycle (0-100)
} ulink_pso_data_t;
```

## State Machine

### States
```
0: SYS_STATE_INIT          - Initialize system
1: SYS_STATE_IDLE          - Wait for start
2: SYS_STATE_TIMING        - Configure sampling
3: SYS_STATE_DATA_PROCESSING - Prepare packets
4: SYS_STATE_STREAMING     - Send data via UART
5: SYS_STATE_PWM_CONTROL   - Execute PWM profile
6: SYS_STATE_STOPPING      - Graceful shutdown
7: SYS_STATE_ERROR         - Error handling
```

### Typical Flow
```
INIT → IDLE → [SW2] → TIMING → PROCESSING → STREAMING ←→ PWM_CONTROL
                                                ↓
                                           STOPPING → INIT
```

## PWM Profiles

### Trapezoid Config
```c
trapezoid_config_t config = {
    .duration_ms = 20000,    // 20s total
    .ramp_up_ms = 5000,      // 5s ramp up
    .hold_ms = 10000,        // 10s hold
    .ramp_down_ms = 5000,    // 5s ramp down
    .min_value = 0,          // 0% min
    .max_value = 100,        // 100% max
    .cycles = 1,             // 1 cycle
    .auto_repeat = false     // No repeat
};
```

### Linear Config
```c
linear_config_t config = {
    .duration_ms = 10000,    // 10s duration
    .start_value = 0,        // 0% start
    .end_value = 100,        // 100% end
    .cycles = 1,             // 1 cycle
    .bidirectional = false,  // One direction
    .slew_rate = 0.01        // 1%/100ms
};
```

### Step Config
```c
step_config_t config = {
    .step_interval_ms = 5000,  // 5s per step
    .num_steps = 5,            // 5 steps
    .steps = {0, 25, 50, 75, 100},
    .cycles = 1,               // 1 cycle
    .ping_pong = false         // Forward only
};
```

## 🔍 Debug Macros

### Debug Pins (pso_debug.h)
```c
DEBUG_STREAM_TOGGLE()   // PD4 - Streaming timing
DEBUG_LOOP_TOGGLE()     // PD5 - Main loop timing
DEBUG_ADC_TOGGLE()      // PD6 - ADC ISR timing
DEBUG_STATE_TOGGLE()    // PD7 - State indicator
```

### Usage in Code
```c
DEBUG_ADC_HIGH();       // Set pin high
// ... code to measure ...
DEBUG_ADC_LOW();        // Set pin low
// Measure pulse width on oscilloscope
```

## Formula Reference

### RPM Calculation
```
RPM = (pulse_count × 600) / BLADE_NUMBER

Where:
- pulse_count = pulses in 100ms
- 600 = conversion (100ms → min: ×10 ×60)
- BLADE_NUMBER = pulses per revolution
```

### PWM Duty Calculation
```
pulse_width_us = 1000 + (throttle × 10)
timer_counts = pulse_width_us × 40

Where:
- throttle = 0-100
- 40 = clock cycles per μs (40 MHz)
```

### ADC Voltage Conversion
```
voltage = (adc_value × 3.3) / 4095

Where:
- adc_value = 0-4095 (12-bit)
- 3.3 = reference voltage
```

## LED Patterns

| Color  | Pattern      | State          |
|--------|--------------|----------------|
| Blue   | Slow blink   | Idle/Standby   |
| Green  | Fast blink   | Streaming      |
| Red    | Fast blink   | Error          |
| White  | 3× Flash     | Complete       |

## ⚡ Interrupt Priorities

| Interrupt | Priority | Frequency | Handler            |
|-----------|----------|-----------|-------------------|
| ADC0/1    | 1 (High) | 5 kHz     | ADC0SS1IntHandler |
| Timer3A   | 2        | 10 Hz     | Timer3AIntHandler |
| UART0     | 3        | Async     | UART0IntHandler   |
| WTimer1   | 4 (Low)  | Disabled  | Error handler     |

## Timing Measurements

| Operation              | Duration  | CPU Load |
|------------------------|-----------|----------|
| ADC ISR (6 channels)   | 15-20 μs  | 0.01%    |
| RPM calculation        | 8-12 μs   | 0.001%   |
| Packet assembly        | 50-100 μs | 0.05%    |
| UART TX (21 bytes)     | ~2 ms     | 2%       |

## Build Commands

### CCS GUI
```
Project → Build All (Ctrl+B)
```

### Command Line
```bash
# Linux/macOS
cd /opt/ti/ccs/eclipse
./ccstudio -noSplash -data ~/workspace \
  -application com.ti.ccstudio.apps.projectBuild \
  -project PSO -config Debug
```

### Flash
```bash
# Using lm4flash
lm4flash PSO.out

# Using OpenOCD
openocd -f board/ek-tm4c123gxl.cfg \
  -c "program PSO.out verify reset exit"
```

## 🔌 Serial Terminal

### Settings
```
Baud Rate:    115200
Data Bits:    8
Parity:       None
Stop Bits:    1
Flow Control: None
```

### Linux/macOS Commands
```bash
# screen
screen /dev/ttyACM0 115200

# minicom
minicom -D /dev/ttyACM0 -b 115200

# Python pyserial
python3 -m serial.tools.miniterm /dev/ttyACM0 115200
```

## Key Files

### Core Application
- `main.c` - State machine and main loop
- `pso_init.c` - Hardware initialization
- `pso_isr.c` - Interrupt handlers

### Peripherals
- `pso_uart.c` - UART communication
- `pso_pwm.c` - PWM generation
- `pso_rpm.c` - RPM measurement
- `pso_led.c` - LED control

### Data Handling
- `fifo.c` - Circular buffer
- `ulink.c` - Protocol implementation
- `pso_data.c` - Packet assembly

### Utilities
- `pso_timing.c` - System timing
- `pso_debug.c` - Debug pins
- `pso_system.c` - Status indicators

## 🔗 Useful Links

- **Datasheet**: [TM4C123GH6PM](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf)
- **TivaWare**: [Peripheral Driver Library](https://www.ti.com/tool/SW-TM4C)
- **LaunchPad**: [User Guide](https://www.ti.com/lit/ug/spmu296/spmu296.pdf)
- **CCS**: [Download](https://www.ti.com/tool/CCSTUDIO)
- **E2E Forum**: [Support](https://e2e.ti.com)

---

**Quick Start**: Clone → Import to CCS → Build → Flash → Press SW2 to start
