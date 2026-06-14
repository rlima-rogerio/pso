# PSO (Propulsion System Optimizer) - Technical Documentation

**Version**: 2.0 Final  
**Last Updated**: 2025-12-15  
**Status**: Complete and Consolidated

---

## 📋 Table of Contents

1. [Project Overview](#project-overview)
2. [Hardware Architecture](#hardware-architecture)
3. [ADC & Sensor Configuration](#adc--sensor-configuration)
4. [Voltage/Current Measurement System](#voltagecurrent-measurement-system)
5. [RPM Measurement System](#rpm-measurement-system)
6. [PWM Motor Control](#pwm-motor-control)
7. [Communication Protocol (ULINK)](#communication-protocol-ulink)
8. [Software Architecture](#software-architecture)
9. [Performance Metrics](#performance-metrics)
10. [Calibration & Testing](#calibration--testing)
11. [Troubleshooting Guide](#troubleshooting-guide)
12. [Configuration Reference](#configuration-reference)
13. [Appendices](#appendices)

---

## Project Overview

The **Propulsion System Optimizer (PSO)** is a comprehensive embedded data acquisition system designed for real-time motor and propeller testing. Built on the Texas Instruments TM4C123GH6PM microcontroller (Tiva C LaunchPad), it provides high-speed sensor data collection, PWM motor control, and UART streaming capabilities with integrated voltage/current measurement.

### Key Features

- ✅ **Real-time Data Acquisition**: Multi-channel ADC sampling at 5 kHz
- ✅ **RPM Measurement**: Hardware edge-period measurement with 25ns resolution
- ✅ **Voltage/Current Monitoring**: INA169-based current sensing (0-60A) + voltage divider (0-33.4V)
- ✅ **Direct Physical Units**: Voltage in mV, Current in mA (no MATLAB scaling needed)
- ✅ **PWM Motor Control**: Configurable profiles (trapezoid, linear, step, sine, exponential, custom)
- ✅ **UART Streaming**: 115200 baud with CRC-16 validation
- ✅ **SD Card Logging**: FatFS-based storage (optional)
- ✅ **Debug Interface**: Hardware timing pins for oscilloscope analysis

### Version History

| Version | Date | Key Changes |
|---------|------|-------------|
| **2.0** | 2025-12-15 | Added pso_iv module (V/I scaling), simplified MATLAB |
| **1.1** | 2025-12-12 | Updated to edge-period RPM method |
| **1.0** | 2025-11-30 | Initial release with edge-count RPM |

---

## Hardware Architecture

### Microcontroller Specifications

- **MCU**: TM4C123GH6PM (ARM Cortex-M4F with FPU)
- **Core**: ARM Cortex-M4 @ 40 MHz
- **Flash Memory**: 256 KB
- **SRAM**: 32 KB
- **EEPROM**: 2 KB
- **Development Board**: Tiva C LaunchPad (EK-TM4C123GXL)

### Pin Assignment Summary

#### ADC Inputs (6 channels)
| ADC | Channel | Pin | Function | Output Format |
|-----|---------|-----|----------|---------------|
| ADC0 | AIN6 | PD1 | X-axis acceleration | Raw ADC (0-4095) |
| ADC0 | AIN7 | PD0 | Thrust sensor | Raw ADC (0-4095) |
| ADC0 | **AIN2** | **PE1** | **Battery voltage** | **mV (0-33400)** |
| ADC1 | AIN5 | PD2 | Y-axis acceleration | Raw ADC (0-4095) |
| ADC1 | AIN4 | PD3 | Z-axis acceleration | Raw ADC (0-4095) |
| ADC1 | **AIN1** | **PE2** | **Motor current** | **mA (0-60000)** |

#### Communication & Control
| Peripheral | Pin | Function |
|------------|-----|----------|
| UART0 TX | PA1 | Data streaming (115200 baud) |
| UART0 RX | PA0 | Command reception |
| UART2 TX | PD7 | Debug output (9600 baud) |
| UART2 RX | PD6 | Debug input |
| PWM Out | PC7 (WT1CCP1) | Motor control (50Hz) |
| RPM In | PC6 (WT1CCP0) | Hall sensor input |
| SPI0 SCK | PA2 | SD card clock |
| SPI0 MOSI | PA5 | SD card data out |
| SPI0 MISO | PA4 | SD card data in |
| SPI0 CS | PA3 | SD card chip select |

#### Debug Pins
| Pin | Function |
|-----|----------|
| PD4 | Debug timing pulse #1 |
| PD5 | Debug timing pulse #2 |
| PD6 | Debug timing pulse #3 |
| PD7 | Debug timing pulse #4 |

#### Status LEDs (Built-in RGB)
| Pin | LED Color | Usage |
|-----|-----------|-------|
| PF1 | Red | Error/fault indication |
| PF2 | Blue | Standby mode |
| PF3 | Green | Active streaming |

---

## ADC & Sensor Configuration

### ADC Configuration Details

**ADC0 Sample Sequencer 1**:
- **Channels**: 3 (AIN6, AIN7, AIN2)
- **Trigger Source**: Timer0A
- **Priority**: 1 (high priority)
- **Interrupt**: Enabled on sequence completion
- **Buffer**: `adc0_buffer[3]`

**ADC1 Sample Sequencer 1**:
- **Channels**: 3 (AIN5, AIN4, AIN1)
- **Trigger Source**: Timer0A
- **Priority**: 1 (high priority)
- **Interrupt**: Enabled on sequence completion
- **Buffer**: `adc1_buffer[3]`

**Common Settings**:
- **Sampling Rate**: 5000 Hz (5 kHz)
- **Resolution**: 12-bit (0-4095)
- **Reference Voltage**: 3.3V (internal)
- **Sample Time**: ~8 μs per channel
- **Total Acquisition Time**: ~24 μs for 3 channels
- **Hardware Averaging**: Disabled (done in software if needed)

**Timer0A Configuration** (ADC Trigger):
```c
Frequency: 5000 Hz
Period: 200 μs
Mode: Periodic
Trigger: ADC SS1 for both ADC0 and ADC1
```

### Sensor Data Flow

```
Timer0A (5kHz)
    ↓
ADC Trigger
    ↓
    ├─→ ADC0SS1 (3 channels) → adc0_buffer[3]
    │       ↓
    │   ADC0SS1IntHandler()
    │
    └─→ ADC1SS1 (3 channels) → adc1_buffer[3]
            ↓
        ADC1SS1IntHandler()
            ↓
        Set g_data_ready_flag
            ↓
        Main loop processes data
            ↓
        packet_data() scales and packages
            ↓
        UART transmission
```

---

## Voltage/Current Measurement System

### Overview

The PSO system includes **integrated voltage and current measurement** using:
- **Voltage**: Resistive voltage divider (0-33.4V range)
- **Current**: INA169 high-side current monitor (0-60A range)

Both measurements are **pre-scaled in firmware** to physical units (mV and mA), eliminating the need for complex MATLAB scaling.

---

### Voltage Measurement (Battery Voltage)

#### Hardware Circuit

**Components**:
- Input Pin: **PE1 (AIN2)**
- R3 (upper resistor): **1.5 kΩ**
- R4 (lower resistor): **13.7 kΩ**
- Input voltage range: **0 - 33.4V**
- ADC input range: **0 - 3.296V**

**Circuit Diagram**:
```
    Vbat (0-33.4V)
         │
         ├─── R3 (1.5kΩ) ───┬─── PE1 (AIN2) → ADC0
         │                  │
         └─── R4 (13.7kΩ) ──┴─── GND

Voltage Divider Ratio:
  V_adc = V_bat × [R4 / (R3 + R4)]
  V_adc = V_bat × [13700 / 15200]
  V_adc = V_bat × 0.09868

Maximum ADC voltage:
  V_adc_max = 33.4V × 0.09868 = 3.296V ✓ (within 3.3V limit)
```

#### Conversion Formula

```c
V(mV) = (ADC × 33400) / 4095

Where:
  ADC       = raw ADC value (0-4095)
  V(mV)     = voltage in millivolts (0-33400)
  33400     = VBAT_MAX_MV constant (33.4V in mV)
  4095      = ADC_MAX_VALUE (12-bit maximum)
```

#### Implementation (pso_iv.c)

```c
uint16_t voltage_adc_to_mv(uint32_t adc_value)
{
    // Clamp to valid range
    if (adc_value > ADC_MAX_VALUE) {
        adc_value = ADC_MAX_VALUE;
    }
    
    // Direct conversion: V(mV) = (ADC × 33400) / 4095
    uint32_t voltage_mv = (adc_value * VBAT_MAX_MV) / ADC_MAX_VALUE;
    
    return (uint16_t)voltage_mv;
}
```

#### Conversion Table

| ADC Value | Voltage (mV) | Voltage (V) | % Full Scale |
|-----------|--------------|-------------|--------------|
| 0         | 0            | 0.000       | 0%           |
| 410       | 3344         | 3.344       | 10%          |
| 819       | 6678         | 6.678       | 20%          |
| 1229      | 10022        | 10.022      | 30%          |
| 2048      | 16704        | 16.704      | 50%          |
| 3072      | 25056        | 25.056      | 75%          |
| 4095      | 33400        | 33.400      | 100%         |

#### Specifications

- **Range**: 0 - 33.4V
- **Resolution**: 8.16 mV/step (33400mV / 4096 steps)
- **Accuracy**: ±1% (limited by resistor tolerance)
- **Data Type**: `uint16_t` (0-65535, fits 0-33400)
- **Units Transmitted**: Millivolts (mV)
- **MATLAB Conversion**: `V = v_motor / 1000` (mV → V)

---

### Current Measurement (Motor Current)

#### Hardware Circuit (INA169)

**Components**:
- IC: **INA169** High-Side Current Monitor
- Shunt Resistor: **Rs = 0.5 mΩ** (0.0005 Ω)
- Load Resistor: **RL = 110 kΩ** (110000 Ω)
- Transconductance: **gm = 1000 μA/V** (0.001 A/V)
- Input Pin: **PE2 (AIN1)**

**Circuit Diagram**:
```
V+ ──┬── Motor/Load ──┬── INA169 ─┐
     │                │            │
     │          Rs(0.5mΩ)      RL(110kΩ)
     │                │            │
    GND              GND    PE2 (AIN1) → ADC1
                                   │
                                  GND

INA169 measures voltage drop across Rs,
converts to proportional output voltage via gm and RL
```

#### INA169 Transfer Function

**Basic Equation**:
```
Vout = (Is × Rs) × gm × RL

Where:
  Is  = shunt current (load current in Amperes)
  Rs  = shunt resistance = 0.0005 Ω
  gm  = transconductance = 0.001 A/V
  RL  = load resistor = 110000 Ω
  
Vout = Is × (0.0005 × 0.001 × 110000)
Vout = Is × 0.055 V/A

Simplified:
  Is = Vout / 0.055
```

**Maximum Current**:
```
Vadc_max = 3.3V (ADC reference voltage)
Is_max = 3.3V / 0.055 V/A = 60A
```

#### Conversion Formula

```c
I(mA) = (ADC × 60000) / 4095

Where:
  ADC       = raw ADC value (0-4095)
  I(mA)     = current in milliamps (0-60000)
  60000     = IMAX_MA constant (60A in mA)
  4095      = ADC_MAX_VALUE (12-bit maximum)
```

**Mathematical Derivation**:
```
Step 1: ADC voltage
  Vadc = (ADC / 4095) × 3.3V

Step 2: Current from INA169
  Is = Vadc / 0.055

Step 3: Substitute and simplify
  Is = [(ADC / 4095) × 3.3] / 0.055
  Is = (ADC × 3.3) / (4095 × 0.055)
  Is = (ADC × 3.3) / 225.225
  Is(A) = ADC × 0.014652
  Is(mA) = ADC × 14.652

Step 4: Equivalent form
  Is(mA) = (ADC × 60000) / 4095
```

#### Implementation (pso_iv.c)

```c
uint16_t current_adc_to_ma(uint32_t adc_value)
{
    // Clamp to valid range
    if (adc_value > ADC_MAX_VALUE) {
        adc_value = ADC_MAX_VALUE;
    }
    
    // Direct conversion: I(mA) = (ADC × 60000) / 4095
    uint32_t current_ma = (adc_value * IMAX_MA) / ADC_MAX_VALUE;
    
    return (uint16_t)current_ma;
}
```

#### Conversion Table

| ADC Value | Current (mA) | Current (A) | % Full Scale |
|-----------|--------------|-------------|--------------|
| 0         | 0            | 0.00        | 0%           |
| 410       | 6010         | 6.01        | 10%          |
| 682       | 10000        | 10.00       | ~17%         |
| 819       | 12005        | 12.00       | 20%          |
| 1024      | 15015        | 15.01       | 25%          |
| 2048      | 30015        | 30.01       | 50%          |
| 3072      | 45023        | 45.02       | 75%          |
| 4095      | 60000        | 60.00       | 100%         |

#### Specifications

- **Range**: 0 - 60A
- **Resolution**: 14.65 mA/step (60000mA / 4096 steps)
- **Accuracy**: ±2% (INA169 + shunt tolerance)
- **Data Type**: `uint16_t` (0-65535, fits 0-60000)
- **Units Transmitted**: Milliamps (mA)
- **MATLAB Conversion**: `A = i_motor / 1000` (mA → A)

---

### pso_iv Module (Software Implementation)

#### Module Structure

**Files**:
- `pso_iv.h` - Constants, macros, and API declarations
- `pso_iv.c` - Conversion function implementations

#### Key Constants (pso_iv.h)

```c
/* Voltage Divider Configuration */
#define R3_OHM               1500U        // Upper resistor: 1.5kΩ
#define R4_OHM               13700U       // Lower resistor: 13.7kΩ
#define VBAT_MAX_MV          33400UL      // Max voltage: 33.4V

/* INA169 Current Monitor Configuration */
#define RSHUNT_MOHM          0.5f         // Shunt: 0.5 mΩ
#define RL_OHM               110000UL     // Load: 110 kΩ
#define INA169_GM            0.001f       // gm: 1000 μA/V (0.001 A/V)
#define IMAX_MA              60000UL      // Max current: 60A

/* ADC Configuration */
#define ADC_VREF_MV          3300U        // Reference: 3.3V
#define ADC_MAX_VALUE        4095U        // 12-bit max: 2^12 - 1
#define ADC_RESOLUTION       4096U        // 12-bit: 2^12

/* Derived Constants */
#define VOUT_PER_AMP         0.055f       // INA169: 0.055 V/A
```

#### API Functions

**Primary Conversions** (ADC → Physical Units):
```c
uint16_t voltage_adc_to_mv(uint32_t adc_value);  // ADC → mV
uint16_t current_adc_to_ma(uint32_t adc_value);  // ADC → mA
```

**Reverse Conversions** (Physical Units → ADC, for testing):
```c
uint32_t voltage_mv_to_adc(uint16_t voltage_mv); // mV → ADC
uint32_t current_ma_to_adc(uint16_t current_ma); // mA → ADC
```

**Validation Functions**:
```c
bool voltage_mv_is_valid(uint16_t voltage_mv);   // Check 0-33400 range
bool current_ma_is_valid(uint16_t current_ma);   // Check 0-60000 range
```

**Conversion Macros**:
```c
#define MV_TO_V(mv)   ((float)(mv) / 1000.0f)     // mV → V
#define MA_TO_A(ma)   ((float)(ma) / 1000.0f)     // mA → A
#define V_TO_MV(v)    ((uint16_t)((v) * 1000.0f)) // V → mV
#define A_TO_MA(a)    ((uint16_t)((a) * 1000.0f)) // A → mA
```

#### Usage in packet_data()

```c
/**
 * @brief Fill ULINK packet with sensor data
 * 
 * TRANSMITTED VALUES:
 *   - accel[0-2]: Raw ADC (0-4095)
 *   - thrust:     Raw ADC (0-4095)
 *   - v_motor:    Millivolts (0-33400 mV)
 *   - i_motor:    Milliamps (0-60000 mA)
 *   - rpm:        Revolutions per minute
 *   - throttle:   PWM percentage (0-100%)
 */
uint8_t packet_data(ulink_pso_data_t* dp)
{
    uint8_t returnval = 0U;
    
    /* Acceleration sensors (raw ADC) */
    dp->accel[0] = adc0_buffer[0];  // X-axis
    dp->accel[1] = adc1_buffer[0];  // Y-axis
    dp->accel[2] = adc1_buffer[1];  // Z-axis
    
    /* Thrust sensor (raw ADC) */
    dp->thrust = adc0_buffer[1];
    
    /* RPM from edge-period measurement */
    dp->rpm = g_scaled_rpm;
    
    /* VOLTAGE in millivolts (0-33400 mV) */
    dp->v_motor = voltage_adc_to_mv(adc0_buffer[2]);
    
    /* CURRENT in milliamps (0-60000 mA) */
    dp->i_motor = current_adc_to_ma(adc1_buffer[2]);
    
    /* PWM throttle position (0-100%) */
    dp->throttle = g_pwm_value;
    
    return returnval;
}
```

#### Benefits of This Approach

✅ **Simplified MATLAB Processing**:
```matlab
% OLD (v1.x) - Complex scaling in MATLAB
V_adc = (data.v_motor * 3.3) / 4095;
V_battery = V_adc / 0.09868;  % Need to know hardware!

% NEW (v2.0) - Simple conversion
V_battery = data.v_motor / 1000;  % Just mV → V!
```

✅ **Hardware Knowledge Encapsulated**:
- All resistor values in firmware
- No need to know circuit details in MATLAB
- Easy to change hardware without MATLAB updates

✅ **Integer Arithmetic**:
- Fast execution on embedded system
- No floating-point overhead in ISRs
- Full precision maintained

✅ **Clear Units**:
- Transmitted values have obvious meaning
- mV and mA are intuitive
- Consistent with engineering standards

✅ **Maintainability**:
- Hardware constants centralized in pso_iv.h
- Easy to adjust for different circuits
- Well-documented formulas

---

## RPM Measurement System

### Overview

The PSO system uses a **hardware edge-period measurement** method for RPM calculation, providing:
- **Instant updates** (per rising edge)
- **High precision** (25ns timer resolution)
- **Low latency** (~5-8 μs ISR execution)
- **Wide range** (15 RPM to 300,000 RPM)

### Hardware Configuration

**Wide Timer 1A (WTIMER1A)**:
- **Input Pin**: PC6 (WT1CCP0)
- **Mode**: Edge capture on rising edge
- **Timer Width**: 32-bit
- **Clock**: 40 MHz (25 ns resolution)
- **Interrupts**: **Enabled** (triggers on each edge)
- **Free-running**: Continuous counting

**Timer 3A (Timeout Detection)**:
- **Period**: 100 ms (10 Hz)
- **Function**: Detects motor stopped condition
- **Interrupts**: Enabled
- **Role**: Safety/timeout monitoring only

### Edge-Period Measurement Algorithm

**Concept**:
Instead of counting edges over a fixed period, measure the **time between consecutive edges** and calculate RPM directly from each period.

**ISR Execution** (WTimer1AIntHandler):
```c
void WTimer1AIntHandler(void)
{
    // 1. Capture current timer value (25ns resolution)
    current_capture = WTIMER1_TAR_R;
    
    // 2. Calculate period between edges (in timer ticks)
    period_ticks = current_capture - last_capture;
    last_capture = current_capture;
    
    // 3. Convert ticks to microseconds (40 ticks = 1 μs)
    period_us = period_ticks / 40;
    
    // 4. Validate period range (noise filter + timeout check)
    if (period_us >= RPM_PERIOD_MIN_US &&
        period_us <= RPM_PERIOD_MAX_US) {
        
        // 5. Calculate RPM from period
        // Formula: RPM = 60,000,000 μs/min ÷ (period × blades)
        g_rpm_value = 60000000UL / (period_us * BLADE_NUMBER);
        
        // 6. Update moving average filter
        rpm_update_filter(g_rpm_value);
        
        // 7. Set data ready flag
        g_rpm_ready_flag ^= 0xFF;  // Toggle flag
        
        // 8. Reset timeout counter
        g_edge_timeout_counter = 0;
    }
    
    // Clear interrupt flag
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;
}
```

**RPM Formula**:
```c
RPM = 60,000,000 / (period_μs × BLADE_NUMBER)

Where:
  60,000,000 = microseconds per minute (60 sec × 1,000,000 μs)
  period_μs  = time between edges in microseconds
  BLADE_NUMBER = pulses per revolution (typically 2)
  
Example:
  period = 30,000 μs (30 ms)
  BLADE_NUMBER = 2
  RPM = 60,000,000 / (30,000 × 2) = 1000 RPM
```

### Timeout Detection (Timer3A)

**Purpose**: Detect when motor has stopped (no edges received)

```c
void Timer3AIntHandler(void)
{
    // Increment timeout counter every 100ms
    g_edge_timeout_counter++;
    
    // If no edges for RPM_STOP_TIMEOUT_MS → motor stopped
    if (g_edge_timeout_counter >= (RPM_STOP_TIMEOUT_MS / 100)) {
        g_rpm_value = 0;
        g_rpm_ready_flag ^= 0xFF;
    }
    
    // Clear interrupt
    TIMER3_ICR_R = TIMER_ICR_TATOCINT;
}
```

### Configuration Parameters

```c
/* pso_rpm.h */
#define BLADE_NUMBER            2       // Pulses per revolution
#define RPM_STOP_TIMEOUT_MS     2000    // 2 seconds timeout
#define RPM_HIGH_MIN_US         50      // Minimum accepted pulse width
#define RPM_PERIOD_MIN_US       (60000000UL / (RPM_MAX_OPERATING * BLADE_NUMBER))
#define RPM_PERIOD_MAX_US       (60000000UL / (RPM_MIN_OPERATING * BLADE_NUMBER))
#define RPM_FILTER_SAMPLES      1       // Moving average size
```

**Calculating Minimum RPM**:
```
For 1 RPM with BLADE_NUMBER = 2:
  Period = 60,000,000 μs / (1 RPM × 2) = 30,000,000 μs = 30 seconds
  
Therefore RPM_STOP_TIMEOUT_MS should be > 30 seconds for 1 RPM
For 1 RPM: set RPM_STOP_TIMEOUT_MS = 35000 (35 seconds with margin)
For 15 RPM: set RPM_STOP_TIMEOUT_MS = 2000 (2 seconds) ✓ Current
```

### Performance Comparison

| Metric | Edge-Period (v1.1+) | Edge-Count (v1.0) |
|--------|---------------------|-------------------|
| **Update Rate** | Per edge (instant) | 10 Hz (100ms fixed) |
| **ISR Latency** | 5-8 μs | 100 ms delay |
| **Min RPM** | ~15 RPM (configurable to 1 RPM) | ~60 RPM |
| **Max RPM** | 300,000 RPM | ~100,000 RPM |
| **Resolution @ 1000 RPM** | ~0.001 RPM | ~0.1 RPM |
| **CPU Load** | <0.5% | <1% |
| **Accuracy** | ±0.1% | ±1% |

### RPM Range and Resolution

**Measurement Range**:
```
Minimum RPM (with 2s timeout):
  RPM_min = 60,000,000 / (2,000,000 × 2) = 15 RPM

Maximum RPM (with 100μs min period):
  RPM_max = 60,000,000 / (100 × 2) = 300,000 RPM
```

**Resolution Examples**:
| Target RPM | Period (μs) | Next Period | Next RPM | Resolution |
|------------|-------------|-------------|----------|------------|
| 100        | 300,000     | 299,975     | 100.008  | 0.008 RPM  |
| 1,000      | 30,000      | 29,975      | 1000.83  | 0.83 RPM   |
| 10,000     | 3,000       | 2,975       | 10,084   | 84 RPM     |

### Filtering and Noise Rejection

**Moving Average Filter**:
```c
// pso_rpm.c
static uint16_t rpm_filter_buffer[RPM_FILTER_SAMPLES];
static uint8_t rpm_filter_index = 0;

void rpm_update_filter(uint16_t new_rpm)
{
    // Add new sample to circular buffer
    rpm_filter_buffer[rpm_filter_index] = new_rpm;
    rpm_filter_index = (rpm_filter_index + 1) % RPM_FILTER_SAMPLES;
}

uint16_t rpm_get_filtered(void)
{
    // Calculate average of last N samples
    uint32_t sum = 0;
    for (uint8_t i = 0; i < RPM_FILTER_SAMPLES; i++) {
        sum += rpm_filter_buffer[i];
    }
    return (uint16_t)(sum / RPM_FILTER_SAMPLES);
}
```

**Noise Rejection**:
- Min period filter: Reject edges < 100 μs (debouncing)
- Max period filter: Reject periods > 60 seconds (disconnected sensor)
- Range validation: Only accept physically plausible RPM values

### API Functions

**Read RPM**:
```c
uint16_t rpm_get_value(void);           // Get latest RPM
uint16_t rpm_get_filtered(void);        // Get filtered RPM (recommended)
uint32_t rpm_get_edge_interval_us(void);// Get period in μs
```

**Status Checks**:
```c
bool rpm_is_ready(void);                // Check if new data available
bool rpm_is_stopped(void);              // Check if motor stopped
bool rpm_is_valid(uint16_t rpm);        // Validate RPM range
```

**Utility Functions**:
```c
void rpm_clear_ready_flag(void);        // Clear data ready flag
void rpm_reset(void);                   // Reset all RPM state
void rpm_update_filter(uint16_t rpm);   // Add to filter
void rpm_reset_filter(void);            // Clear filter buffer
```

**Conversion Functions**:
```c
uint16_t rpm_from_period_us(uint32_t period_us);  // Period → RPM
uint16_t rpm_from_frequency(float freq_hz);       // Freq → RPM
float rpm_to_frequency(uint16_t rpm);             // RPM → Freq
```

---

## PWM Motor Control

### Hardware Configuration

**Wide Timer 1B (WTIMER1B)**:
- **Output Pin**: PC7 (WT1CCP1)
- **Frequency**: 50 Hz (20 ms period)
- **Pulse Width Range**: 1000-2000 μs (1-2 ms)
- **Resolution**: 0-100% throttle position
- **Update Rate**: Configurable (typically 10 Hz during profiles)

**PWM Signal Characteristics**:
```
Period: 20 ms (50 Hz)
│◄───────────────────────────►│
├─────┐                        ├─────┐
│     │                        │     │
│     │                        │     │
└─────┘                        └─────┘
◄─────►                        ◄─────►
Pulse Width                    Pulse Width
1.0 - 2.0 ms                  1.0 - 2.0 ms

Position Mapping:
  0% throttle   → 1.0 ms pulse
  50% throttle  → 1.5 ms pulse
  100% throttle → 2.0 ms pulse
  
Formula: pulse_μs = 1000 + (position × 10)
```

### PWM Control Modes

#### 1. Direct Throttle Control
```c
void pwm_set_throttle(uint8_t position);
// position: 0-100%
// Immediately sets PWM to specified throttle
```

#### 2. Trapezoid Profile
**Profile Stages**:
1. **Ramp Up**: Linear increase from start to peak
2. **Hold**: Maintain peak throttle
3. **Ramp Down**: Linear decrease to end position

**Configuration**:
```c
typedef struct {
    uint8_t start_position;     // Initial throttle (0-100%)
    uint8_t peak_position;      // Maximum throttle (0-100%)
    uint8_t end_position;       // Final throttle (0-100%)
    uint16_t ramp_up_ms;        // Time to reach peak (ms)
    uint16_t hold_ms;           // Time at peak (ms)
    uint16_t ramp_down_ms;      // Time to descend (ms)
    uint8_t update_rate_hz;     // Update frequency (Hz)
} trapezoid_config_t;

// Usage
void pwm_set_trapezoid_config(trapezoid_config_t* config);
```

**Example**:
```c
trapezoid_config_t trap = {
    .start_position = 0,
    .peak_position = 80,
    .end_position = 0,
    .ramp_up_ms = 3000,     // 3 seconds ramp up
    .hold_ms = 5000,        // 5 seconds at 80%
    .ramp_down_ms = 2000,   // 2 seconds ramp down
    .update_rate_hz = 10    // 10 Hz updates
};
pwm_set_trapezoid_config(&trap);
pwm_profile_start(PROFILE_TRAPEZOID);
```

#### 3. Linear Ramp Profile
**Characteristics**:
- Single linear ramp between start and end positions
- Configurable duration and direction
- Bidirectional support (up or down)

**Configuration**:
```c
typedef struct {
    uint8_t start_position;     // Initial throttle
    uint8_t end_position;       // Final throttle
    uint16_t duration_ms;       // Ramp duration
    uint8_t update_rate_hz;     // Update frequency
} linear_config_t;

// Usage
void pwm_set_linear_config(linear_config_t* config);
```

#### 4. Step Profile
**Characteristics**:
- Discrete throttle steps with dwell times
- Up to 10 steps per profile
- Useful for efficiency mapping

**Configuration**:
```c
typedef struct {
    uint8_t positions[10];      // Throttle positions
    uint16_t durations_ms[10];  // Dwell time per step
    uint8_t num_steps;          // Number of steps (1-10)
} step_config_t;

// Usage
void pwm_set_step_config(step_config_t* config);
```

**Example** (Efficiency curve):
```c
step_config_t steps = {
    .positions = {0, 20, 40, 60, 80, 100, 80, 60, 40, 20},
    .durations_ms = {1000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 1000},
    .num_steps = 10
};
pwm_set_step_config(&steps);
pwm_profile_start(PROFILE_STEP);
```

### Profile Control API

```c
// Start a profile
void pwm_profile_start(profile_type_t type);

// Stop current profile
void pwm_profile_stop(void);

// Check if profile is running
bool pwm_profile_is_running(void);

// Get current throttle position
uint8_t pwm_get_current_throttle(void);

// Execute one profile step (called from timer ISR)
void pwm_profile_execute(void);
```

### Profile State Machine

```
IDLE
  ↓ pwm_profile_start()
RUNNING
  ├→ Trapezoid: RAMP_UP → HOLD → RAMP_DOWN → COMPLETE
  ├→ Linear:    RAMPING → COMPLETE
  └→ Step:      STEP_0 → STEP_1 → ... → STEP_N → COMPLETE
        ↓
     COMPLETE
        ↓ pwm_profile_stop()
      IDLE
```

### Timing and Updates

**Profile Update Rate**:
- Controlled by Timer3A interrupt
- Configurable: 1-100 Hz (typically 10 Hz)
- Higher rates = smoother transitions

**PWM Output Rate**:
- Fixed at 50 Hz (motor control standard)
- Independent of profile update rate

---

## Communication Protocol (ULINK)

### Packet Structure

**Format**: Binary packet with CRC-16 validation  
**Total Size**: 21 bytes  
**Byte Order**: Little-endian for multi-byte fields

| Byte | Field | Type | Description | Units | Range |
|------|-------|------|-------------|-------|-------|
| 0 | STX | uint8_t | Start byte (magic number) | - | 0xAA |
| 1 | Length | uint8_t | Packet length | bytes | 21 |
| 2-3 | Index | uint16_t | Packet sequence number | - | 0-65535 |
| 4-5 | accel[0] | uint16_t | X-axis acceleration | ADC | 0-4095 |
| 6-7 | accel[1] | uint16_t | Y-axis acceleration | ADC | 0-4095 |
| 8-9 | accel[2] | uint16_t | Z-axis acceleration | ADC | 0-4095 |
| 10-11 | thrust | uint16_t | Thrust sensor | ADC | 0-4095 |
| 12-13 | **v_motor** | uint16_t | **Battery voltage** | **mV** | **0-33400** |
| 14-15 | **i_motor** | uint16_t | **Motor current** | **mA** | **0-60000** |
| 16-17 | rpm | uint16_t | Motor RPM | RPM | 0-65535 |
| 18 | throttle | uint8_t | PWM throttle | % | 0-100 |
| 19-20 | CRC | uint16_t | CRC-16 checksum | - | 0-65535 |

**Important Notes**:
- ✅ **v_motor** and **i_motor** are **pre-scaled** (v2.0+)
- ❌ **accel** and **thrust** remain as **raw ADC** (sensor-specific calibration needed)
- ⚠️ Byte order is **little-endian** on TM4C123 (LSB first)

### CRC-16 Calculation

**Algorithm**: CRC-16-CCITT (polynomial 0x1021)  
**Initial Value**: 0xFFFF  
**Bytes Covered**: All bytes except CRC field (bytes 0-18)

```c
uint16_t create_checksum(uint8_t* data, uint8_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint8_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    
    return crc;
}
```

### MATLAB Data Processing

#### Complete MATLAB Example (v2.0)

```matlab
function data = parse_pso_packet(packet)
    % Parse ULINK binary packet (21 bytes)
    
    % Validate length
    if length(packet) ~= 21
        error('Invalid packet length');
    end
    
    % Check start byte
    if packet(1) ~= 0xAA
        error('Invalid start byte');
    end
    
    % Verify CRC
    crc_calculated = crc16_ccitt(packet(1:19));
    crc_received = typecast(uint8(packet(20:21)), 'uint16');
    if crc_calculated ~= crc_received
        warning('CRC mismatch');
    end
    
    % Parse fields (little-endian)
    data.index = typecast(uint8(packet(3:4)), 'uint16');
    data.accel_x = typecast(uint8(packet(5:6)), 'uint16');   % raw ADC
    data.accel_y = typecast(uint8(packet(7:8)), 'uint16');   % raw ADC
    data.accel_z = typecast(uint8(packet(9:10)), 'uint16');  % raw ADC
    data.thrust = typecast(uint8(packet(11:12)), 'uint16');  % raw ADC
    data.v_motor = typecast(uint8(packet(13:14)), 'uint16'); % mV
    data.i_motor = typecast(uint8(packet(15:16)), 'uint16'); % mA
    data.rpm = typecast(uint8(packet(17:18)), 'uint16');     % RPM
    data.throttle = packet(19);                              % %
    
    % Convert to physical units (SIMPLE!)
    data.voltage_V = data.v_motor / 1000;   % mV → V
    data.current_A = data.i_motor / 1000;   % mA → A
    data.power_W = data.voltage_V * data.current_A;  % Watts
    
    % Acceleration and thrust need sensor-specific calibration
    % (depends on your specific sensors)
end
```

#### Old vs New Comparison

**OLD (v1.x) - Complex Scaling**:
```matlab
% Required hardware knowledge
v_adc = (data.v_motor * 3.3) / 4095;
V_battery = v_adc / 0.09868;  % Voltage divider ratio

i_adc = (data.i_motor * 3.3) / 4095;
I_motor = i_adc / 0.055;  % INA169 transfer function

% Result: V_battery in Volts, I_motor in Amperes
```

**NEW (v2.0) - Simple Conversion**:
```matlab
% No hardware knowledge needed!
V_battery = data.v_motor / 1000;  % mV → V
I_motor = data.i_motor / 1000;    % mA → A

% Result: Same units, much simpler!
```

### Transmission Rate and Bandwidth

**UART Configuration**:
- Baud Rate: 115200 bps
- Bits per byte: 10 (8 data + 1 start + 1 stop)
- Bytes per packet: 21
- Bits per packet: 210

**Theoretical Limits**:
```
Max packets/second = 115200 / 210 = 548 packets/s
Actual sustainable rate = ~200 packets/s (with processing overhead)
Data rate = 200 packets/s × 21 bytes = 4.2 KB/s
```

**Practical Rates**:
- 5 Hz: Slow logging
- 10 Hz: Standard testing
- 50 Hz: High-speed data collection
- 200 Hz: Maximum sustainable rate

---

## Software Architecture

### File Structure

```
PSO Project/
├── Core/
│   ├── main.c                  State machine & main loop
│   ├── pso_init.c/h           Peripheral initialization
│   ├── pso_system.c/h         System control & LED indicators
│   └── pso_isr.c/h            Interrupt service routines
│
├── Sensors/
│   ├── pso_rpm.c/h            RPM measurement (edge-period)
│   ├── pso_iv.c/h             Voltage/current scaling (NEW v2.0)
│   └── pso_data.c/h           Data handling
│
├── Control/
│   └── pso_pwm.c/h            PWM generation & profiles
│
├── Communication/
│   ├── pso_uart.c/h           UART configuration
│   └── ulink.c/h              Protocol encoder/decoder
│
├── Utilities/
│   ├── fifo.c/h               Circular buffer
│   ├── pso_timing.c/h         System timing & delays
│   └── pso_debug.c/h          Debug GPIO & timing measurement
│
├── Drivers/ (TivaWare)
│   └── [TI peripheral drivers]
│
└── Config/
    ├── pso_config.h           System-wide configuration
    └── tm4c123gh6pm.h         MCU register definitions
```

### State Machine (main.c)

**System States**:
```c
typedef enum {
    PSO_STATE_INIT,        // System initialization
    PSO_STATE_IDLE,        // Standby (blue LED)
    PSO_STATE_TIMING,      // Timing calibration
    PSO_STATE_PROCESSING,  // Data processing
    PSO_STATE_STREAMING,   // Active data streaming (green LED)
    PSO_STATE_PWM_CONTROL, // PWM profile execution
    PSO_STATE_STOPPING,    // Shutdown sequence
    PSO_STATE_FINISH       // Test complete (white LED)
} pso_system_state_t;
```

**State Transitions**:
```
Power On → INIT
    ↓
   IDLE (blue LED, waiting for button press)
    ↓ Button SW1 pressed
  TIMING (calibrate timing)
    ↓
PROCESSING (prepare data structures)
    ↓
STREAMING (green LED, transmit data via UART)
    ↓ Button SW2 pressed
PWM_CONTROL (execute motor profile)
    ↓ Profile complete
 STOPPING (decelerate motor)
    ↓
  FINISH (white LED, motors off)
    ↓ Button SW1 pressed
   IDLE
```

**Button Functions**:
- **SW1** (PF4): Start/restart sequence
- **SW2** (PF0): Trigger PWM profile (during streaming)

### Data Flow Architecture

```
┌─────────────┐
│  Timer0A    │ 5 kHz trigger
│   (5 kHz)   │
└──────┬──────┘
       │
       ├──────────────┬──────────────┐
       │              │              │
       ▼              ▼              ▼
┌───────────┐  ┌───────────┐  ┌───────────┐
│ ADC0 SS1  │  │ ADC1 SS1  │  │ WTimer1A  │
│ 3 channels│  │ 3 channels│  │  (RPM)    │
└─────┬─────┘  └─────┬─────┘  └─────┬─────┘
      │              │              │
      ▼              ▼              ▼
┌─────────────────────────────────────┐
│        ADC ISR Handlers             │
│  - Store to buffers                 │
│  - Set data ready flag              │
└──────────────┬──────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│        Main Loop (if flag set)       │
│  - Call packet_data()                │
│  - Scale V/I (pso_iv module) ← NEW  │
│  - Package into ULINK structure      │
└──────────────┬───────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│        ULINK Encoder                 │
│  - Add STX, length, index            │
│  - Calculate CRC-16                  │
│  - Push to FIFO                      │
└──────────────┬───────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│        UART0 TX                      │
│  - FIFO → UART transmitter           │
│  - 115200 baud, 8N1                  │
│  - Interrupt-driven                  │
└──────────────┬───────────────────────┘
               │
               ▼
         HOST PC / MATLAB
```

### Interrupt Priority Table

| Priority | Interrupt | ISR Handler | Purpose |
|----------|-----------|-------------|---------|
| **0** | SysTick | SysTick_Handler | 1ms system tick |
| **1** | ADC0 SS1 | ADC0SS1IntHandler | Acceleration + thrust + voltage |
| **1** | ADC1 SS1 | ADC1SS1IntHandler | Disabled; ADC1 FIFO drained by ADC0 SS1 |
| **2** | WTimer1A | WTimer1AIntHandler | RPM edge capture |
| **3** | Timer3A | Timer3AIntHandler | RPM timeout check |
| **4** | UART0 RX | UART0IntHandler | Receive commands |
| **5** | Timer0A | Timer0AIntHandler | ADC trigger (no ISR) |

**Priority Levels**:
- **0** = Highest priority (most critical)
- **7** = Lowest priority

### Memory Usage

**Flash (256 KB total)**:
```
Code (.text):       ~48 KB  (18.8%)
Constants (.rodata): ~2 KB   (0.8%)
Vectors:             ~1 KB   (0.4%)
─────────────────────────────────
Total Used:         ~51 KB  (19.9%)
Available:         ~205 KB  (80.1%)
```

**RAM (32 KB total)**:
```
Stack:              2 KB
ADC Buffers:        24 bytes (6ch × 2B × 2 ADCs)
FIFO Buffer:       512 bytes
RPM Filter:          8 bytes
Packet Structure:   21 bytes
Global Variables:   ~1 KB
─────────────────────────────────
Total Used:        ~3.5 KB (10.9%)
Available:        ~28.5 KB (89.1%)
```

---

## Performance Metrics

### System Timing

**ADC Acquisition**:
- Sample Rate: 5000 Hz (200 μs period)
- Channels per ADC: 3
- Sample Time: ~2 μs per channel
- Total Acquisition: ~6 μs for 3 channels
- ISR Execution: ~5-10 μs
- **CPU Load**: ~2.5-5%

**RPM Measurement**:
- Update Rate: Per edge (instant)
- ISR Execution: 5-8 μs
- Timeout Check: Every 100ms (Timer3A)
- **CPU Load**: <0.5%

**UART Transmission**:
- Baud Rate: 115200 bps
- Packet Size: 21 bytes
- Transmission Time: ~1.8 ms/packet
- Max Rate: ~548 packets/second
- Typical Rate: 10-200 packets/second
- **CPU Load**: ~1-2% (at 200 pkt/s)

**PWM Generation**:
- Frequency: 50 Hz (20 ms period)
- Update Rate: 10 Hz (profile execution)
- **CPU Load**: <1%

**Total System CPU Load**: ~10-15% (leaves 85-90% headroom)

### Latency Analysis

| Event | Latency | Notes |
|-------|---------|-------|
| ADC trigger → ISR | ~5 μs | Hardware + ISR entry |
| ADC ISR → Data ready | ~10 μs | Store to buffer |
| RPM edge → ISR | ~5 μs | Hardware capture |
| RPM ISR → Calculation | ~8 μs | Period → RPM |
| Data ready → UART TX | ~50 μs | Main loop processing |
| UART TX → Complete | ~1.8 ms | 21 bytes @ 115200 |

**End-to-End Latency** (sensor → UART): ~2 ms typical

---

## Calibration & Testing

### Voltage Calibration

**Equipment**:
- Calibrated multimeter (±0.1% accuracy or better)
- Variable DC power supply (0-35V, 1A min)
- Test leads

**Procedure**:

1. **Setup**:
   - Connect power supply to voltage divider input
   - Connect multimeter in parallel (measure actual voltage)
   - Connect PSO system

2. **Test Points**:
   | Supply (V) | Expected ADC | Expected (mV) | Tolerance |
   |-----------|--------------|---------------|-----------|
   | 0.0       | 0            | 0             | ±10 mV    |
   | 5.0       | 614          | 5000          | ±50 mV    |
   | 10.0      | 1226         | 10000         | ±100 mV   |
   | 15.0      | 1839         | 15000         | ±150 mV   |
   | 20.0      | 2453         | 20000         | ±200 mV   |
   | 25.0      | 3066         | 25000         | ±250 mV   |
   | 30.0      | 3679         | 30000         | ±300 mV   |
   | 33.4      | 4095         | 33400         | ±334 mV   |

3. **Verification**:
   ```c
   // Read ADC and convert
   uint32_t adc = read_adc_channel(2);  // PE1 (AIN2)
   uint16_t v_mv = voltage_adc_to_mv(adc);
   float v = v_mv / 1000.0f;
   
   // Compare with multimeter
   float error_percent = (multimeter_reading - v) / multimeter_reading * 100.0f;
   
   // Should be < ±1% (limited by resistor tolerance)
   ```

4. **Adjustment** (if needed):
   - Measure actual R3 and R4 values with precision multimeter
   - Calculate new `VBAT_MAX_MV`:
     ```
     VBAT_MAX = [(R3 + R4) / R4] × 3.3V × 1000
     ```
   - Update constant in `pso_iv.h`
   - Recompile and retest

**Expected Accuracy**: ±1% (limited by resistor tolerance ±1%)

---

### Current Calibration

**Equipment**:
- Electronic load (0-60A capability, programmable)
- Calibrated ammeter (±0.5% accuracy or better, 60A range)
- DC power supply (>60V, >60A capability)
- Heavy-duty wiring (16 AWG or better)

**Procedure**:

1. **Setup**:
   ```
   Power Supply (+) ──┬── Ammeter ──┬── Motor/Load ──┐
                      │             │                │
                   INA169        Ground          Ground
                   Shunt Rs
                      │
   Power Supply (-) ──┴─────────────┴────────────────┘
   ```

2. **Test Points**:
   | Load (A) | Expected ADC | Expected (mA) | Tolerance |
   |----------|--------------|---------------|-----------|
   | 0.0      | 0            | 0             | ±100 mA   |
   | 5.0      | 341          | 5000          | ±100 mA   |
   | 10.0     | 682          | 10000         | ±200 mA   |
   | 15.0     | 1024         | 15000         | ±300 mA   |
   | 20.0     | 1365         | 20000         | ±400 mA   |
   | 30.0     | 2048         | 30000         | ±600 mA   |
   | 40.0     | 2730         | 40000         | ±800 mA   |
   | 50.0     | 3413         | 50000         | ±1000 mA  |
   | 60.0     | 4095         | 60000         | ±1200 mA  |

3. **Verification**:
   ```c
   // Read ADC and convert
   uint32_t adc = read_adc_channel(1);  // PE2 (AIN1)
   uint16_t i_ma = current_adc_to_ma(adc);
   float i = i_ma / 1000.0f;
   
   // Compare with ammeter
   float error_percent = (ammeter_reading - i) / ammeter_reading * 100.0f;
   
   // Should be < ±2% (INA169 + shunt tolerance)
   ```

4. **Adjustment** (if needed):
   - Verify Rs = 0.5 mΩ (measure with milliohm meter)
   - Verify RL = 110 kΩ (measure with multimeter)
   - Calculate new `IMAX_MA`:
     ```
     IMAX = 3.3V / (Rs × gm × RL) × 1000
     Where: gm = 0.001 A/V
     ```
   - Update constant in `pso_iv.h`
   - Recompile and retest

**Expected Accuracy**: ±2% (limited by INA169 ±1% and shunt ±1%)

---

### RPM Calibration and Testing

**Equipment**:
- Function generator (0.1 Hz - 100 kHz capability)
- Frequency counter (for verification)
- Oscilloscope (optional, for waveform verification)

**Test Setup**:
```
Function Generator TTL Output → PC6 (WT1CCP0)
                                 │
                              TM4C123
                                 │
                            UART → PC
```

**Test Points**:
| Input Freq | Expected RPM | Formula |
|------------|--------------|---------|
| 1 Hz       | 30 RPM       | 1 × 60 / 2 |
| 5 Hz       | 150 RPM      | 5 × 60 / 2 |
| 10 Hz      | 300 RPM      | 10 × 60 / 2 |
| 16.67 Hz   | 500 RPM      | 16.67 × 60 / 2 |
| 33.33 Hz   | 1000 RPM     | 33.33 × 60 / 2 |
| 50 Hz      | 1500 RPM     | 50 × 60 / 2 |
| 83.33 Hz   | 2500 RPM     | 83.33 × 60 / 2 |
| 166.67 Hz  | 5000 RPM     | 166.67 × 60 / 2 |
| 333.33 Hz  | 10000 RPM    | 333.33 × 60 / 2 |
| 833.33 Hz  | 25000 RPM    | 833.33 × 60 / 2 |

**Formula**: `RPM = (Frequency × 60) / BLADE_NUMBER`  
(Assuming BLADE_NUMBER = 2)

**Procedure**:
1. Set function generator to square wave, 3.3V, 50% duty cycle
2. Start at 1 Hz, verify PSO reports 30 RPM
3. Increase frequency through test points
4. Record actual vs reported RPM
5. Calculate error: `error = (reported - expected) / expected × 100%`

**Expected Accuracy**: ±0.1% (limited by 40 MHz clock precision)

---

## Troubleshooting Guide

### Voltage Measurement Issues

**Problem**: Voltage reads 0 or very low
- ❌ Check voltage divider connections (R3, R4)
- ❌ Verify R3 = 1.5kΩ, R4 = 13.7kΩ
- ❌ Check ADC pin PE1 connection
- ❌ Measure voltage at PE1 (should be 0-3.3V max)
- ❌ Check for open circuit in divider

**Problem**: Voltage reads maximum (33400 mV) constantly
- ❌ Input voltage exceeds 33.4V (ADC saturated)
- ❌ Short circuit in voltage divider
- ❌ R3 and R4 may be swapped
- ❌ Check for solder bridges

**Problem**: Voltage inaccurate
- ⚠️ Resistor tolerance (typically ±1%, ±5% for standard)
- ⚠️ ADC reference voltage drift (check 3.3V rail)
- ⚠️ Temperature effects on resistors
- ⚠️ Recalibrate `VBAT_MAX_MV` constant
- ⚠️ Check for high-impedance voltage source

---

### Current Measurement Issues

**Problem**: Current reads 0
- ❌ INA169 not powered (check V+ supply)
- ❌ Shunt resistor disconnected or damaged
- ❌ Check PE2 (AIN1) connection to INA169 output
- ❌ Verify RL (110kΩ) is present
- ❌ Check INA169 enable pin (if used)

**Problem**: Current reads maximum (60000 mA) constantly
- ❌ Current exceeds 60A (ADC saturated)
- ❌ INA169 output saturated (measure voltage at PE2)
- ❌ Short circuit in load
- ❌ Shunt resistor value incorrect (should be 0.5mΩ)

**Problem**: Current noisy or unstable
- ⚠️ Add 0.1μF capacitor across INA169 output (Vout to GND)
- ⚠️ Check shunt connections (must be very low resistance)
- ⚠️ Verify good ground reference between INA169 and ADC
- ⚠️ Check for ground loops in circuit
- ⚠️ Ensure heavy-duty wiring for high currents
- ⚠️ Move INA169 away from noisy switching components

**Problem**: Current reads incorrectly but stable
- ⚠️ Shunt resistance wrong (verify 0.5mΩ)
- ⚠️ RL value wrong (verify 110kΩ ±1%)
- ⚠️ INA169 transconductance differs from datasheet
- ⚠️ Recalibrate `IMAX_MA` constant

---

### RPM Measurement Issues

**Problem**: RPM reads 0
- ❌ No signal at PC6 (WT1CCP0) - check Hall sensor
- ❌ Hall sensor not powered (check 5V or 3.3V)
- ❌ Signal level too low (should be 0-3.3V or 0-5V with divider)
- ❌ WTimer1A not configured correctly
- ❌ Check for pull-up resistor on Hall sensor output

**Problem**: RPM unstable or noisy
- ⚠️ Electrical noise on Hall sensor signal
- ⚠️ Add 100Ω series resistor + 0.1μF capacitor at PC6
- ⚠️ Reduce `MIN_EDGE_INTERVAL_US` if too aggressive
- ⚠️ Increase `RPM_FILTER_SAMPLES` for smoother output
- ⚠️ Check for loose Hall sensor mounting

**Problem**: RPM reads too high
- ❌ `BLADE_NUMBER` configured incorrectly
- ❌ Multiple pulses per magnet pass
- ❌ Hall sensor detecting stray magnetic fields

**Problem**: RPM reads too low
- ❌ `BLADE_NUMBER` too high
- ❌ Missing edges (weak signal)
- ❌ `RPM_PERIOD_MAX_US` too restrictive

**Problem**: RPM not updating
- ❌ WTimer1A interrupt not enabled
- ❌ Interrupt priority too low
- ❌ Check `g_rpm_ready_flag` in code
- ❌ Main loop not processing RPM data

---

### UART Communication Issues

**Problem**: No data received on PC
- ❌ Check TX pin (PA1) connection
- ❌ Verify baud rate (115200)
- ❌ Check GND connection between PSO and PC
- ❌ Wrong COM port selected in terminal
- ❌ UART0 not initialized

**Problem**: Corrupted data
- ❌ Baud rate mismatch
- ❌ Incorrect frame format (should be 8N1)
- ❌ CRC errors (check CRC implementation)
- ❌ Buffer overflow (reduce streaming rate)
- ❌ Electrical noise on UART lines

**Problem**: Data transmission stops
- ❌ FIFO overflow (main loop not processing fast enough)
- ❌ UART TX interrupt not firing
- ❌ Check `g_data_ready_flag` logic
- ❌ System entered error state

---

### PWM Control Issues

**Problem**: Motor not responding
- ❌ Check PWM output pin PC7 (WT1CCP1)
- ❌ Verify ESC/motor powered
- ❌ Check PWM frequency (should be 50 Hz)
- ❌ Verify pulse width range (1-2 ms)
- ❌ ESC may need calibration (full throttle → zero throttle procedure)

**Problem**: Motor runs at full speed
- ❌ PWM signal stuck high
- ❌ Check `g_pwm_value` variable
- ❌ Profile configuration incorrect
- ❌ ESC calibration wrong

**Problem**: Erratic motor behavior
- ❌ Noisy PWM signal (add ground plane, shorter wires)
- ❌ Power supply voltage drop under load
- ❌ ESC incompatible with signal (try different ESC)
- ❌ Check PWM frequency (must be 50 Hz for most ESCs)

---

### System-Level Issues

**Problem**: System crashes or hangs
- ❌ Stack overflow (increase stack size)
- ❌ Interrupt priority conflict
- ❌ Infinite loop in ISR
- ❌ Watch for division by zero (RPM calculation)
- ❌ Check for buffer overruns

**Problem**: Data acquisition stops
- ❌ ADC trigger (Timer0A) not running
- ❌ ADC SS1 interrupts disabled
- ❌ Check `g_data_ready_flag` handling
- ❌ FIFO full condition not handled

**Problem**: High CPU load
- ❌ ISR execution too long (optimize code)
- ❌ Main loop blocking (use non-blocking delays)
- ❌ Too high data rate (reduce sampling rate)
- ❌ Debug timing enabled (disable if not needed)

---

## Configuration Reference

### System Configuration (pso_config.h)

```c
/* Clock Configuration */
#define SYSTEM_CLOCK_HZ          40000000UL    // 40 MHz

/* ADC Configuration */
#define ADC_SAMPLE_RATE_HZ       5000U         // 5 kHz
#define ADC_VREF_MV              3300U         // 3.3V reference
#define ADC_MAX_VALUE            4095U         // 12-bit max

/* Voltage Measurement (pso_iv.h) */
#define R3_OHM                   1500U         // 1.5kΩ upper resistor
#define R4_OHM                   13700U        // 13.7kΩ lower resistor
#define VBAT_MAX_MV              33400UL       // 33.4V max voltage

/* Current Measurement (pso_iv.h) */
#define RSHUNT_MOHM              0.5f          // 0.5mΩ shunt
#define RL_OHM                   110000UL      // 110kΩ load resistor
#define INA169_GM                0.001f        // 1000 μA/V
#define IMAX_MA                  60000UL       // 60A max current

/* RPM Configuration (pso_rpm.h) */
#define BLADE_NUMBER             2             // Pulses per revolution
#define RPM_STOP_TIMEOUT_MS      2000          // 2 second timeout
#define RPM_FILTER_SAMPLES       1             // Moving average size
#define RPM_HIGH_MIN_US          50            // Minimum accepted pulse width
#define RPM_PERIOD_MAX_US        (60000000UL / (RPM_MIN_OPERATING * BLADE_NUMBER))

/* UART Configuration */
#define UART_BAUD_RATE           115200UL      // Baud rate
#define UART_TX_BUFFER_SIZE      512U          // TX buffer (bytes)

/* PWM Configuration */
#define PWM_FREQUENCY_HZ         50U           // 50 Hz (20 ms period)
#define PWM_MIN_PULSE_US         1000U         // 1.0 ms (0% throttle)
#define PWM_MAX_PULSE_US         2000U         // 2.0 ms (100% throttle)

/* FIFO Configuration */
#define FIFO_BUFFER_SIZE         256U          // Circular buffer size

/* Debug Configuration */
#define DEBUG_TIMING_ENABLED     1             // Enable debug pins
#define DEBUG_UART_ENABLED       0             // Enable UART2 debug
```

---

## Appendices

### Appendix A: Hardware Schematics

**Voltage Divider Circuit**:
```
      Vbat (0-33.4V)
           │
           ├─── R3 (1.5kΩ) ───┬─── PE1 (AIN2) → ADC0
           │                  │
           └─── R4 (13.7kΩ) ──┴─── GND

Vout = Vin × [R4 / (R3 + R4)]
Vout = Vin × 0.09868
Max ADC voltage = 33.4V × 0.09868 = 3.296V ✓
```

**INA169 Current Monitor Circuit**:
```
V+ ─┬─── Motor/Load ──┬── INA169 ─┐
    │                 │            │
    │           Rs(0.5mΩ)      RL(110kΩ)
    │                 │            │
   GND               GND    PE2 (AIN1) → ADC1
                                   │
                                  GND

Transfer Function:
Vout = (Is × Rs) × gm × RL
Vout = Is × 0.055 V/A
Is = Vout / 0.055
```

---

### Appendix B: Bill of Materials (BOM)

**Critical Components**:

| Component | Part Number | Value | Tolerance | Qty |
|-----------|-------------|-------|-----------|-----|
| MCU | TM4C123GH6PM | - | - | 1 |
| R3 (Voltage Divider) | - | 1.5kΩ | ±1% | 1 |
| R4 (Voltage Divider) | - | 13.7kΩ | ±1% | 1 |
| Rs (Shunt) | - | 0.5mΩ | ±1% | 1 |
| RL (INA169 Load) | - | 110kΩ | ±1% | 1 |
| INA169 IC | INA169NA | - | - | 1 |
| C1 (INA169 Filter) | - | 0.1μF | 10% | 1 |
| C2 (Bypass) | - | 10μF | 10% | 2 |
| Hall Sensor | A3144 (or equivalent) | - | - | 1 |

---

### Appendix C: Glossary

- **ADC**: Analog-to-Digital Converter
- **CRC**: Cyclic Redundancy Check
- **ESC**: Electronic Speed Controller
- **FIFO**: First-In-First-Out buffer
- **ISR**: Interrupt Service Routine
- **LSB**: Least Significant Bit (little-endian byte order)
- **PWM**: Pulse Width Modulation
- **RPM**: Revolutions Per Minute
- **SysTick**: System tick timer (1ms periodic interrupt)
- **UART**: Universal Asynchronous Receiver/Transmitter
- **ULINK**: PSO custom communication protocol

---

### Appendix D: Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| **2.0** | 2025-12-15 | Engineering Team | • Added pso_iv module (voltage/current scaling)<br>• Updated MATLAB processing (simplified)<br>• Added INA169 circuit analysis<br>• Consolidated documentation |
| **1.1** | 2025-12-12 | Engineering Team | • Updated RPM to edge-period method<br>• Added performance comparison<br>• Improved ISR timing analysis |
| **1.0** | 2025-11-30 | Engineering Team | • Initial release<br>• Edge-count RPM method<br>• Basic ADC configuration |

---

### Appendix E: References

1. **TM4C123GH6PM Datasheet** - Texas Instruments
2. **INA169 High-Side Current Monitor** - Texas Instruments
3. **TivaWare Peripheral Driver Library** - Texas Instruments
4. **CRC-16-CCITT Algorithm** - ITU-T Recommendation V.41

---

**Document Maintained By**: Embedded Systems Engineering Team  
**Next Review Date**: 2026-01-15  
**For Questions or Issues**: See project repository or contact engineering team

---

**END OF DOCUMENT**
