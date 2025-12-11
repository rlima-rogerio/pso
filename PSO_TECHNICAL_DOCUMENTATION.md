# PSO (Propulsion System Optimizer) - Technical Documentation

## Project Overview

The Propulsion System Optimizer (PSO) is a comprehensive embedded data acquisition system designed for real-time motor and propeller testing. Built on the Texas Instruments TM4C123GH6PM microcontroller (Tiva C LaunchPad), it provides high-speed sensor data collection, PWM motor control, and UART streaming capabilities.

### Key Features

- **Real-time Data Acquisition**: Multi-channel ADC sampling at 5 kHz
- **RPM Measurement**: Hardware-based edge counting with 100ms resolution
- **PWM Motor Control**: Configurable profiles (trapezoid, linear, step)
- **UART Streaming**: 115200 baud data transmission with CRC validation
- **SD Card Logging**: FatFS-based data storage (optional)
- **Debug Interface**: Hardware timing pins for oscilloscope analysis

---

## Hardware Architecture

### Microcontroller Specifications

- **MCU**: TM4C123GH6PM (ARM Cortex-M4F)
- **Clock Speed**: 40 MHz
- **Flash Memory**: 256 KB
- **RAM**: 32 KB
- **Development Board**: Tiva C LaunchPad (EK-TM4C123GXL)

### Peripheral Configuration

#### ADC Configuration (6-Channel Acquisition)

**ADC0 Sample Sequencer 1**:
- **Channel 0** (PD1/AIN6): X-axis acceleration
- **Channel 1** (PD0/AIN7): Strain gauge / Thrust sensor
- **Channel 2** (PE1/AIN2): Motor voltage

**ADC1 Sample Sequencer 1**:
- **Channel 0** (PD2/AIN5): Y-axis acceleration
- **Channel 1** (PD3/AIN4): Z-axis acceleration
- **Channel 2** (PE2/AIN1): Motor current

**Sampling Configuration**:
- Trigger: Timer0A (5 kHz periodic)
- Resolution: 12-bit (0-4095)
- Hardware averaging: Optional
- Interrupt-driven acquisition

#### UART Configuration

**UART0 (Primary Communication)**:
- **Pins**: PA0 (RX), PA1 (TX)
- **Baud Rate**: 115200
- **Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **FIFO**: Enabled (16-byte depth)
- **Interrupts**: RX and RX timeout enabled
- **Usage**: Data streaming to host PC

**UART2 (Alternative/Debug)**:
- **Pins**: PD6 (RX), PD7 (TX)
- **Baud Rate**: 9600
- **Format**: 8N1
- **Usage**: Secondary communication channel

#### PWM Configuration (Motor Control)

**Wide Timer 1B (WTIMER1B)**:
- **Output Pin**: PC7 (WT1CCP1)
- **Frequency**: 50 Hz (20ms period)
- **Pulse Width Range**: 1000-2000 μs (1-2 ms)
- **Resolution**: 0-100% (maps to 1-2ms linearly)
- **Control Modes**: 
  - Trapezoid profile
  - Linear ramp
  - Step sequence
  - Custom profiles

**PWM Timing Calculation**:
```c
// 40 MHz clock → 25 ns period
// Position 0% → 1000 μs pulse
// Position 100% → 2000 μs pulse
// Formula: pulse_us = 1000 + (position × 10)
```

#### RPM Measurement System

**Wide Timer 1A (WTIMER1A)**:
- **Input Pin**: PC6 (WT1CCP0)
- **Mode**: Edge counter (rising edge)
- **Counter Width**: 32-bit
- **Interrupts**: Disabled (polling-based)

**Timer 3A (Measurement Period)**:
- **Period**: 100 ms (10 Hz)
- **Function**: Triggers RPM calculation
- **Algorithm**: 
  ```c
  RPM = (pulse_count × 600) / BLADE_NUMBER
  // 600 = conversion factor (100ms → min: ×10 ×60)
  ```

#### GPIO Pin Assignments

**Port F (User Interface)**:
- **PF0**: SW2 button (start/stop control)
- **PF1**: Red LED (error indication)
- **PF2**: Blue LED (standby mode)
- **PF3**: Green LED (streaming active)
- **PF4**: SW1 button (mode selection)

**Port D (Debug Pins)**:
- **PD4**: Debug timing pin 4
- **PD5**: Debug timing pin 5
- **PD6**: ADC timing measurement
- **PD7**: State machine indicator

**Port C (Timer/PWM)**:
- **PC6**: RPM input (WT1CCP0)
- **PC7**: PWM output (WT1CCP1)

#### SPI Configuration (SD Card Interface)

**SPI0**:
- **Pins**: PA2 (CLK), PA3 (CS), PA4 (MISO), PA5 (MOSI)
- **Clock Speed**: 400 kHz (initialization), up to 12.5 MHz (data transfer)
- **Mode**: SPI Mode 0 (CPOL=0, CPHA=0)
- **Usage**: FatFS SD card communication

---

## Software Architecture

### State Machine Design

The system operates as a finite state machine with eight states:

```
┌─────────────┐
│  SYS_INIT   │ ← System initialization
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  SYS_IDLE   │ ← Waiting for start command
└──────┬──────┘
       │ (SW2 pressed)
       ▼
┌─────────────┐
│ SYS_TIMING  │ ← Configure timing/sampling
└──────┬──────┘
       │
       ▼
┌─────────────┐
│SYS_DATA_PROC│ ← Packet preparation
└──────┬──────┘
       │
       ▼
┌─────────────┐     ┌──────────────┐
│SYS_STREAMING│ ←──→│ SYS_PWM_CTRL │
└──────┬──────┘     └──────┬───────┘
       │                   │
       │ (SW2 or complete) │
       ▼                   ▼
┌─────────────┐     ┌──────────────┐
│SYS_STOPPING │     │  SYS_ERROR   │
└──────┬──────┘     └──────┬───────┘
       │                   │
       └───────────┬───────┘
                   ▼
           Back to SYS_INIT
```

#### State Descriptions

1. **SYS_STATE_INIT**: 
   - Initializes FIFOs
   - Resets flags
   - Sets up initial conditions

2. **SYS_STATE_IDLE**: 
   - LED blinking (blue)
   - Waits for SW2 button press
   - Low power state

3. **SYS_STATE_TIMING**: 
   - Configures ADC trigger timing
   - Enables Timer0 for 5 kHz sampling
   - Prepares data acquisition

4. **SYS_STATE_DATA_PROCESSING**: 
   - Scales ADC values
   - Calculates RPM
   - Prepares data packets
   - Applies filters (if enabled)

5. **SYS_STATE_STREAMING**: 
   - Transmits data via UART
   - LED indication (green)
   - Packet serialization with CRC

6. **SYS_STATE_PWM_CONTROL**: 
   - Executes selected PWM profile
   - Updates motor throttle
   - Monitors profile completion

7. **SYS_STATE_STOPPING**: 
   - Graceful shutdown
   - Flushes FIFO buffers
   - Stops PWM output
   - Completion indication

8. **SYS_STATE_ERROR**: 
   - Error handling
   - Disables all operations
   - LED error pattern

### Module Structure

```
PSO Project
├── main.c                 # Main application and state machine
├── pso_init.c/h          # Hardware initialization
├── pso_uart.c/h          # UART communication
├── pso_pwm.c/h           # PWM profile generation
├── pso_rpm.c/h           # RPM measurement
├── pso_timing.c/h        # System timing utilities
├── pso_led.c/h           # LED control functions
├── pso_data.c/h          # Data packet handling
├── pso_debug.c/h         # Debug GPIO utilities
├── pso_isr.c/h           # Interrupt service routines
├── pso_system.c/h        # System status indicators
├── fifo.c/h              # Circular FIFO implementation
├── ulink.c/h             # Communication protocol
└── ulink_pso.c/h         # PSO-specific protocol
```

### Data Flow Architecture

```
┌──────────────────────────────────────────────────────┐
│                  SENSOR INPUTS                        │
├──────────┬───────────┬──────────┬────────────────────┤
│  ADC0/1  │   RPM     │  PWM FB  │  User Buttons      │
│ (6-ch)   │  Input    │          │  (SW1/SW2)         │
└────┬─────┴─────┬─────┴────┬─────┴──────┬─────────────┘
     │           │          │            │
     │           │          │            │
     ▼           ▼          ▼            ▼
┌────────────────────────────────────────────────────┐
│          INTERRUPT SERVICE ROUTINES                │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐            │
│  │ ADC ISR │  │Timer3 ISR│  │UART ISR │            │
│  │(5 kHz)  │  │ (10 Hz)  │  │ (async) │            │
│  └────┬────┘  └────┬─────┘  └────┬────┘            │
└───────┼────────────┼─────────────┼─────────────────┘
        │            │             │
        ▼            ▼             ▼
┌────────────────────────────────────────────────────┐
│              DATA PROCESSING LAYER                  │
│  ┌────────────────────────────────────────┐        │
│  │ • ADC value scaling                    │        │
│  │ • RPM calculation (pulse_diff × 600)   │        │
│  │ • Data packet assembly                 │        │
│  │ • CRC checksum generation              │        │
│  └────────────────┬───────────────────────┘        │
└────────────────────┼──────────────────────────────┘
                     │
                     ▼
┌────────────────────────────────────────────────────┐
│                  FIFO BUFFERS                       │
│  ┌──────────────┐      ┌──────────────┐           │
│  │ PING Buffer  │  ←→  │ PONG Buffer  │           │
│  │ (256 bytes)  │      │ (256 bytes)  │           │
│  └──────┬───────┘      └──────┬───────┘           │
└─────────┼──────────────────────┼───────────────────┘
          │                      │
          ▼                      ▼
┌────────────────────────────────────────────────────┐
│                 OUTPUT LAYER                        │
│  ┌──────────────┐      ┌──────────────┐           │
│  │ UART Stream  │      │  SD Card Log │           │
│  │ (115200 bps) │      │  (Optional)  │           │
│  └──────────────┘      └──────────────┘           │
└────────────────────────────────────────────────────┘
```

### Communication Protocol (ULINK)

**Packet Structure** (21 bytes total):

```
┌───────┬────────┬────────┬────────┬────────┬─────────┬────────┐
│ STX   │ Length │ Index  │ Index  │ Data   │   ...   │  CRC   │
│ (1B)  │  (1B)  │ High   │  Low   │ (16B)  │         │  (2B)  │
│ 0xFE  │  0x13  │ (1B)   │  (1B)  │        │         │        │
└───────┴────────┴────────┴────────┴────────┴─────────┴────────┘
```

**Data Payload** (16 bytes):
- **Bytes 0-1**: Index (packet counter)
- **Bytes 2-3**: Accel X (int16)
- **Bytes 4-5**: Accel Y (int16)
- **Bytes 6-7**: Accel Z (int16)
- **Bytes 8-9**: RPM (uint16)
- **Bytes 10-11**: Motor current (int16)
- **Bytes 12-13**: Motor voltage (int16)
- **Bytes 14-15**: Thrust (int16)
- **Byte 16**: Throttle (0-100%)
- **Bytes 17-18**: CRC-16 checksum

**CRC Algorithm**: CRC-16-CCITT
- Polynomial: 0x1021
- Initial value: 0xFFFF
- XOR out: 0x0000

---

## PWM Profile System

### Profile Types

#### 1. Trapezoid Profile

Generates a three-segment waveform: ramp up → hold → ramp down

```
Throttle (%)
100% ├────────────────────┐
     │                    │
 75% │                    │
     │                    │
 50% │                    │
     │                    │
 25% │                    │
     │                    │
  0% ┴────┬───────────────┬────┬───→ Time
     0    5s             15s  20s

Configuration:
- ramp_up_ms: 5000 ms
- hold_ms: 10000 ms  
- ramp_down_ms: 5000 ms
- min_value: 0%
- max_value: 100%
```

#### 2. Linear Profile

Generates a linear ramp from start to end value

```
Throttle (%)
100% ├─────────────────────┐
     │                    ╱│
 75% │                 ╱  │
     │              ╱     │
 50% │           ╱        │
     │        ╱           │
 25% │     ╱              │
     │  ╱                 │
  0% ┴─────────────────────┴───→ Time
     0                    30s

Configuration:
- duration_ms: 30000 ms
- start_value: 0%
- end_value: 100%
- bidirectional: false
- slew_rate: 0 (unlimited)
```

#### 3. Step Profile

Generates discrete step changes at specified intervals

```
Throttle (%)
100% ├─────┐
     │     │        ┌─────┐
 75% │     │  ┌─────┘     └─────┐
     │     │  │                 │
 50% │     └──┘                 └──┐
     │                             │
 25% │  ┌─────┐                    │
     │  │     │                    │
  0% ┴──┘     └────────────────────┴──→ Time
     0   5s  10s  15s  20s  25s  30s

Configuration:
- step_interval_ms: 5000 ms
- steps[]: {0, 25, 50, 75, 100, 75, 50, 25, 0}
- num_steps: 9
- cycles: 1
```

### Profile Configuration Example

```c
/* Linear ramp: 0% → 100% over 10 seconds */
linear_config_t my_profile = {
    .duration_ms = 10000,
    .start_value = 0,
    .end_value = 100,
    .cycles = 1,
    .bidirectional = false,
    .slew_rate = 0.01  // 1% per 100ms
};

/* Execute profile in main loop */
uint32_t elapsed = get_systick_ms() - start_time;
execute_linear_profile(elapsed, &my_profile);
```

---

## Interrupt System

### Priority Configuration

| Interrupt      | Priority | Handler Function      | Frequency | Purpose                |
|----------------|----------|-----------------------|-----------|------------------------|
| ADC0/1 SS1     | High (1) | ADC0SS1IntHandler     | 5 kHz     | Sensor data capture    |
| Timer3A        | Med (2)  | Timer3AIntHandler     | 10 Hz     | RPM calculation        |
| UART0 RX       | Med (3)  | UART0IntHandler       | Async     | Command reception      |
| WTimer1A       | Low (4)  | WTimer1AIntHandler    | Disabled  | (Error handler only)   |
| WTimer1B       | Low (5)  | WTimer1BIntHandler    | Disabled  | (Reserved)             |

### ISR Execution Times

Measured with oscilloscope on debug pins (40 MHz clock):

- **ADC0SS1IntHandler**: 15-20 μs
- **Timer3AIntHandler**: 8-12 μs
- **UART0IntHandler**: 2-5 μs (per character)

### Critical Sections

The following operations require interrupt protection:

```c
/* Example: Clearing RPM ready flag */
IntMasterDisable();
g_rpm_ready_flag = 0;
IntMasterEnable();

/* Example: FIFO access from ISR */
// Use atomic operations when possible
// FIFOs designed for single-producer, single-consumer
```

---

## Memory Map

### Flash Memory Layout

```
0x0000_0000  ┌────────────────────────┐
             │  Interrupt Vectors     │  (1 KB)
0x0000_0400  ├────────────────────────┤
             │                        │
             │   Application Code     │  (~200 KB)
             │                        │
0x0003_2000  ├────────────────────────┤
             │   Constant Data        │  (~20 KB)
0x0003_7000  ├────────────────────────┤
             │   Reserved / Unused    │  (~36 KB)
0x0003_FFFF  └────────────────────────┘
```

### RAM Layout

```
0x2000_0000  ┌────────────────────────┐
             │  Stack (grows down)    │  (2 KB)
0x2000_0800  ├────────────────────────┤
             │  Global Variables      │
             │  - ADC buffers         │
             │  - FIFO structures     │
             │  - UART buffers        │  (~8 KB)
             │  - System state        │
0x2000_2800  ├────────────────────────┤
             │  Heap (grows up)       │  (~20 KB)
0x2000_7800  ├────────────────────────┤
             │  Unused                │  (2 KB)
0x2000_7FFF  └────────────────────────┘
```

### Key Data Structures

```c
/* ADC buffers - 24 bytes total */
volatile uint32_t adc0_buffer[3];  // 12 bytes
volatile uint32_t adc1_buffer[3];  // 12 bytes

/* FIFO structures - 520 bytes each */
typedef struct {
    int16_t front;
    int16_t rear;
    uint8_t data[256];
} fifo_t;
fifo_t g_fifo_ping;  // 520 bytes
fifo_t g_fifo_pong;  // 520 bytes

/* UART buffers - 42 bytes */
uint16_t uart_tx_buffer[21];  // 42 bytes
```

---

## Performance Metrics

### System Throughput

- **ADC Sampling Rate**: 5 kHz (200 μs period)
- **UART Data Rate**: ~10 kHz (100 μs/packet @ 115200 baud)
- **RPM Update Rate**: 10 Hz (100 ms period)
- **PWM Update Rate**: Varies by profile (typically 10 Hz)
- **State Machine Rate**: ~10 kHz (limited by UART streaming)

### Timing Measurements

| Operation                    | Duration   | CPU Usage |
|------------------------------|------------|-----------|
| ADC acquisition (6 channels) | 15-20 μs   | 0.01%     |
| RPM calculation              | 8-12 μs    | 0.001%    |
| Packet serialization         | 50-100 μs  | 0.05%     |
| UART transmission (21 bytes) | ~2 ms      | 2%        |
| **Total overhead**           | **~2.1 ms**| **~2%**   |

### Latency Analysis

- **Sensor → ADC buffer**: <100 ns (hardware)
- **ADC ISR response**: ~2 μs (interrupt latency)
- **Data → UART buffer**: 50-100 μs
- **UART buffer → transmission**: Immediate (FIFO-based)
- **End-to-end latency**: ~200 μs + transmission time

---

## Error Handling

### Error Detection

1. **FIFO Overflow**:
   - Condition: `fifo_is_full()` returns true
   - Action: Drop incoming data, increment error counter
   - Recovery: Automatic when FIFO drains

2. **CRC Mismatch**:
   - Condition: Received CRC ≠ calculated CRC
   - Action: Discard packet, request retransmission
   - Recovery: Protocol-level retry

3. **UART Overrun**:
   - Condition: UART_FR_OE flag set
   - Action: Clear error, log event
   - Recovery: Automatic

4. **ADC Timeout**:
   - Condition: No ADC interrupt within expected period
   - Action: Transition to ERROR state
   - Recovery: System reset required

### Debug Capabilities

**Hardware Debug Pins** (Port D):
- PD4-PD7 provide real-time signal output
- Use oscilloscope for timing analysis
- Example: Toggle PD6 in ADC ISR to measure execution time

**LED Status Codes**:
- Blue blinking: Standby/Idle
- Green blinking: Streaming active
- Red blinking: Error condition
- White flash: Operation complete

**UART Debug Messages** (optional):
```c
// Enable debug output in pso_debug.c
#define DEBUG_UART_ENABLED
// Example output:
// "Debug GPIO PD4-PD7 initialized\r\n"
```

---

## Power Consumption

### Current Measurements

| Operating Mode      | Current (mA) | Notes                        |
|---------------------|--------------|------------------------------|
| Idle (LEDs off)     | 15-20        | CPU in WFI, peripherals off  |
| Streaming (no PWM)  | 35-45        | UART active, ADC sampling    |
| Full operation      | 50-60        | All peripherals active       |
| Peak (UART TX)      | 70-80        | During burst transmission    |

### Power Optimization

- Use WFI (Wait For Interrupt) in idle state
- Disable unused peripherals
- Lower ADC sampling rate if possible
- Use FIFO for burst transmission

---

## Testing and Validation

### Unit Test Procedures

#### ADC Calibration
```
1. Connect known voltage source to ADC inputs
2. Verify readings match expected values (±1% tolerance)
3. Test all 6 channels independently
4. Verify sampling rate with oscilloscope
```

#### RPM Measurement
```
1. Connect signal generator to PC6 (WT1CCP0)
2. Set frequency: 100 Hz
3. Expected RPM (2-blade): (100 × 60) / 2 = 3000 RPM
4. Verify ±0.5% accuracy
5. Test range: 50 Hz - 10 kHz
```

#### PWM Output
```
1. Connect oscilloscope to PC7 (PWM output)
2. Measure pulse width at 0%, 50%, 100% throttle
3. Expected: 1000 μs, 1500 μs, 2000 μs (±10 μs)
4. Verify 50 Hz frequency
5. Check rise/fall times (<1 μs)
```

#### UART Communication
```
1. Connect USB-to-serial adapter to PA0/PA1
2. Configure terminal: 115200 8N1
3. Verify packet reception
4. Check CRC validation (inject errors)
5. Measure packet loss rate (<0.01%)
```

### Integration Test Scenarios

1. **Full Acquisition Test**:
   - Enable all sensors
   - Run for 60 seconds
   - Verify continuous streaming
   - Check for data loss or corruption

2. **PWM Profile Test**:
   - Execute each profile type
   - Monitor motor response
   - Verify throttle accuracy
   - Check profile timing

3. **Stress Test**:
   - Maximum sampling rate
   - Continuous streaming
   - All peripherals active
   - Monitor for >5 minutes

---

## Known Issues and Limitations

### Current Limitations

1. **Fixed Sampling Rate**: ADC sampling fixed at 5 kHz
   - Workaround: Modify Timer0 period in `pso_init.c`

2. **UART Buffer Size**: 256-byte FIFO may overflow at high rates
   - Workaround: Implement flow control or increase buffer

3. **SD Card Support**: FatFS partially implemented
   - Status: Code present but requires testing/completion

4. **Return Values**: Some functions return placeholder values
   - Example: `uartBatchWrite()` always returns last value
   - Impact: Limited error detection

### Future Improvements

- [ ] Implement dynamic sampling rate configuration
- [ ] Add flow control to UART streaming
- [ ] Complete SD card data logging
- [ ] Add USB device support for direct PC connection
- [ ] Implement adaptive filtering algorithms
- [ ] Add configuration storage in EEPROM
- [ ] Create MATLAB/Python analysis tools
- [ ] Add real-time graphing capability

---

## Appendix A: Pin Summary Table

| Pin  | Function      | Direction | Peripheral | Description                    |
|------|---------------|-----------|------------|--------------------------------|
| PA0  | UART0 RX      | Input     | UART0      | USB serial receive             |
| PA1  | UART0 TX      | Output    | UART0      | USB serial transmit            |
| PA2  | SPI0 CLK      | Output    | SPI0       | SD card clock                  |
| PA3  | SPI0 CS       | Output    | GPIO       | SD card chip select            |
| PA4  | SPI0 MISO     | Input     | SPI0       | SD card data in                |
| PA5  | SPI0 MOSI     | Output    | SPI0       | SD card data out               |
| PC6  | WT1CCP0       | Input     | WTIMER1    | RPM sensor input               |
| PC7  | WT1CCP1       | Output    | WTIMER1    | PWM motor control              |
| PD0  | AIN7          | Analog    | ADC0       | Strain gauge / Thrust          |
| PD1  | AIN6          | Analog    | ADC0       | Acceleration X                 |
| PD2  | AIN5          | Analog    | ADC1       | Acceleration Y                 |
| PD3  | AIN4          | Analog    | ADC1       | Acceleration Z                 |
| PD4  | GPIO          | Output    | GPIO       | Debug timing pin 4             |
| PD5  | GPIO          | Output    | GPIO       | Debug timing pin 5             |
| PD6  | GPIO/UART2 RX | I/O       | GPIO/UART2 | Debug ADC timing / UART2 RX    |
| PD7  | GPIO/UART2 TX | I/O       | GPIO/UART2 | Debug state / UART2 TX         |
| PE1  | AIN2          | Analog    | ADC0       | Motor voltage                  |
| PE2  | AIN1          | Analog    | ADC1       | Motor current                  |
| PF0  | SW2           | Input     | GPIO       | User button (start/stop)       |
| PF1  | LED Red       | Output    | GPIO       | Error indicator                |
| PF2  | LED Blue      | Output    | GPIO       | Standby indicator              |
| PF3  | LED Green     | Output    | GPIO       | Streaming indicator            |
| PF4  | SW1           | Input     | GPIO       | User button (mode select)      |

---

## Appendix B: Compilation Flags

Available compile-time options in `main.c`:

```c
/* PWM Profile Selection (choose one) */
#define PWM_PROFILE_NONE_SELECTED         // No PWM output
#define PWM_PROFILE_TRAPEZOID_SELECTED    // Trapezoid profile
#define PWM_PROFILE_LINEAR_SELECTED       // Linear ramp (default)
#define PWM_PROFILE_STEP_SELECTED         // Step sequence
#define PWM_PROFILE_CUSTOM_SELECTED       // Custom profile
#define PWM_PROFILE_SINE_SELECTED         // Sine wave (future)
#define PWM_PROFILE_EXPONENTIAL_SELECTED  // Exponential (future)
```

---

## Appendix C: References

### TI Documentation
- [TM4C123GH6PM Datasheet](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf)
- [TivaWare Peripheral Driver Library User's Guide](https://www.ti.com/lit/ug/spmu298e/spmu298e.pdf)
- [Tiva C LaunchPad User's Guide](https://www.ti.com/lit/ug/spmu296/spmu296.pdf)

### Application Notes
- SPMA074: ADC Configuration Guide
- SPMA073: PWM Generation on Tiva C
- SPMA075: Timer Capture Configuration

### Development Tools
- Code Composer Studio (CCS) v12.x or later
- TivaWare SDK v2.2.x or later
- Lm4flash (for programming via ICDI)

---

## Document History

| Version | Date       | Author       | Changes                           |
|---------|------------|--------------|-----------------------------------|
| 1.0     | 2025-12-11 | ROG3R10      | Initial documentation creation    |
| 0.9     | 2015-08-31 | Rogerio Lima | Original code implementation      |

---

**Last Updated**: December 11, 2025  
**Document Status**: Complete  
**Maintainer**: ROG3R10
