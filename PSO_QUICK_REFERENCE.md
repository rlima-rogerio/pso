# PSO Quick Reference Card v2.0

**Version**: 2.0 (Edge-Period RPM + V/I Scaling)  
**Last Updated**: 2025-12-15

---

## Pin Assignments

### ADC Inputs (0-3.3V)
```
PD1 (AIN6)  →  Acceleration X-axis (raw ADC)
PD2 (AIN5)  →  Acceleration Y-axis (raw ADC)
PD3 (AIN4)  →  Acceleration Z-axis (raw ADC)
PD0 (AIN7)  →  Thrust sensor (raw ADC)
PE1 (AIN2)  →  Motor Voltage (scaled to mV)
PE2 (AIN1)  →  Motor Current (scaled to mA)
```

### Digital I/O
```
PC6  →  RPM Input (WT1CCP0) - Edge capture with INTERRUPT
PC7  →  PWM Output (WT1CCP1) - Motor ESC control (50Hz)
PF0  →  SW2 Button (PWM profile trigger)
PF4  →  SW1 Button (start/stop streaming)
```

### Status LEDs (RGB)
```
PF1  →  Red LED (error/fault)
PF2  →  Blue LED (standby mode)
PF3  →  Green LED (active streaming)
```

### Communication
```
PA0/PA1  →  UART0 (115200 baud, 8N1) - Main data streaming
PD6/PD7  →  UART2 (9600 baud, 8N1) - Debug/alternative
```

### Debug Pins (Oscilloscope)
```
PD4  →  Streaming timing toggle
PD5  →  Main loop timing toggle
PD6  →  ADC ISR timing toggle
PD7  →  State machine indicator
```

---

## System Parameters

### Timing Configuration
| Parameter | Value | Notes |
|-----------|-------|-------|
| System Clock | 40 MHz | ARM Cortex-M4 |
| ADC Sample Rate | 5 kHz (200 μs) | Timer0A trigger |
| RPM Update | **Per edge** (instant) | NOT fixed rate |
| RPM Timeout Check | 10 Hz (100 ms) | Timer3A |
| PWM Frequency | 50 Hz (20 ms) | Standard ESC |
| UART Baud | 115200 bps | 8N1 format |

### RPM Measurement (Edge-Period Method) ⚡
| Spec | Value |
|------|-------|
| Method | Period measurement between edges |
| Timer Resolution | 25 ns (40 MHz clock) |
| Update Mode | Interrupt on each rising edge |
| Formula | `RPM = 60,000,000 / (period_μs × BLADE_NUMBER)` |
| Min RPM | ~15 RPM (2s timeout) |
| Max RPM | ~300,000 RPM (100μs min period) |
| Resolution @ 1000 RPM | ~0.001 RPM (theoretical) |
| ISR Latency | 5-8 μs |

### Voltage/Current Measurement
| Parameter | Range | Resolution | Units |
|-----------|-------|------------|-------|
| Voltage | 0 - 33.4V | 8.16 mV/step | **mV** (transmitted) |
| Current | 0 - 60A | 14.65 mA/step | **mA** (transmitted) |
| Voltage Circuit | Resistive divider | R3=1.5kΩ, R4=13.7kΩ | - |
| Current Circuit | INA169 + 0.5mΩ shunt | RL=110kΩ | - |

### PWM Control
```
Pulse Range:    1000-2000 μs (1-2 ms)
Resolution:     0-100% (10 μs steps)
0% Throttle:    1000 μs pulse
50% Throttle:   1500 μs pulse
100% Throttle:  2000 μs pulse

Formula: pulse_μs = 1000 + (throttle × 10)
```

### Memory Usage
```
Flash:          256 KB total → ~51 KB used (19.9%)
RAM:            32 KB total → ~3.5 KB used (10.9%)
Stack:          2 KB
FIFO Buffer:    512 bytes (2× 256B circular buffers)
ADC Buffers:    24 bytes (6ch × 2B × 2 ADCs)
```

---

## Configuration Macros

### pso_rpm.h (Edge-Period Configuration)
```c
#define BLADE_NUMBER            2U      // Pulses per revolution
#define RPM_STOP_TIMEOUT_MS     2000U   // Motor stopped timeout (2s)
#define MIN_EDGE_INTERVAL_US    100U    // Min period = noise filter
#define MAX_EDGE_INTERVAL_MS    60000U  // Max period (60 seconds)
#define RPM_FILTER_SAMPLES      4U      // Moving average size
/* Select measurement method  */
#define RPM_EDGE_COUNT_METHOD           // Edge counting (default) */
// #define RPM_EDGE_PERIOD_METHOD          // Period measurement */
```

### pso_iv.h (Voltage/Current Scaling) 
```c
/* Voltage Divider */
#define R3_OHM                  1500U   // Upper resistor: 1.5kΩ
#define R4_OHM                  13700U  // Lower resistor: 13.7kΩ
#define VBAT_MAX_MV             33400UL // Max voltage: 33.4V

/* INA169 Current Monitor */
#define RSHUNT_MOHM             0.5f    // Shunt: 0.5 mΩ
#define RL_OHM                  110000UL// Load resistor: 110kΩ
#define IMAX_MA                 60000UL // Max current: 60A

/* ADC */
#define ADC_VREF_MV             3300U   // Reference: 3.3V
#define ADC_MAX_VALUE           4095U   // 12-bit max
```

### pso_config.h (System)
```c
#define SYSTEM_CLOCK_HZ         40000000UL
#define PWM_FREQUENCY           50U        // Hz
#define ADC_SAMPLE_RATE_HZ      5000U      // Hz
#define FIFO_BUFFER_SIZE        256U       // Bytes
#define UART_BAUD_RATE          115200UL
```

### main.c (Profile Selection)
```c
// Uncomment ONE profile type:
// #define PWM_PROFILE_TRAPEZOID_SELECTED
#define PWM_PROFILE_LINEAR_SELECTED
// #define PWM_PROFILE_STEP_SELECTED
```

---

## Data Packet Format

### UART Packet Structure (21 bytes)
```
┌────┬────┬─────┬─────┬─────┬─────┬─────┬─────┬──────┬─────┐
│STX │Len │Idx_H│Idx_L│Acc_X│Acc_Y│Acc_Z│Thrst│V_mot │I_mot│
│0xAA│0x15│     │     │     │     │     │     │      │     │
├────┼────┼─────┼─────┼─────┼─────┼─────┼─────┼──────┼─────┤
│ 1B │ 1B │  2B │  2B │  2B │  2B │  2B │  2B │  2B  │ RPM │
└────┴────┴─────┴─────┴─────┴─────┴─────┴─────┴──────┴─────┘

┌─────┬──────┬──────┐
│ RPM │Throt │ CRC  │
│     │      │      │
├─────┼──────┼──────┤
│ 2B  │  1B  │  2B  │
└─────┴──────┴──────┘

Total: 21 bytes
Byte order: Little-endian (LSB first)
```

### Field Details (UPDATED)
```c
typedef struct {
    uint16_t index;      // Packet counter (0-65535)
    uint16_t accel[3];   // X, Y, Z acceleration (raw ADC 0-4095)
    uint16_t thrust;     // Thrust sensor (raw ADC 0-4095)
    uint16_t v_motor;    // Voltage in mV (0-33400) ⭐ SCALED
    uint16_t i_motor;    // Current in mA (0-60000) ⭐ SCALED
    uint16_t rpm;        // RPM (updated per edge, 0-65535)
    uint8_t  throttle;   // PWM duty cycle (0-100%)
} ulink_pso_data_t;
```

**Key Changes in v2.0**:
- `v_motor`: Now in **millivolts** (was raw ADC)
- `i_motor`: Now in **milliamps** (was raw ADC)
- `accel[3]` and `thrust`: Still **raw ADC** (sensor-specific)

---

## State Machine

### System States
```
0: PSO_STATE_INIT          - System initialization
1: PSO_STATE_IDLE          - Standby (blue LED, wait for SW1)
2: PSO_STATE_TIMING        - Configure timing parameters
3: PSO_STATE_PROCESSING    - Prepare data structures
4: PSO_STATE_STREAMING     - Active data transmission (green LED)
5: PSO_STATE_PWM_CONTROL   - Execute PWM profile (SW2 triggered)
6: PSO_STATE_STOPPING      - Graceful motor shutdown
7: PSO_STATE_FINISH        - Test complete (white LED)
```

### State Transitions
```
Power On → INIT
    ↓
   IDLE (blue LED)
    ↓ Press SW1
  TIMING
    ↓
PROCESSING
    ↓
STREAMING (green LED) ←────┐
    ↓ Press SW2            │
PWM_CONTROL ───────────────┘
    ↓ Profile complete
 STOPPING
    ↓
  FINISH (white LED)
    ↓ Press SW1
   IDLE
```

---

## ⚡ RPM Measurement - Edge-Period Method

### WTimer1A ISR (Triggered on Each Edge)
```c
void WTimer1AIntHandler(void)
{
    // 1. Capture timer value (32-bit, 25ns resolution)
    current_capture = WTIMER1_TAR_R;
    
    // 2. Calculate period since last edge
    period_ticks = current_capture - last_capture;
    last_capture = current_capture;
    
    // 3. Convert to microseconds (40 ticks = 1 μs)
    period_us = period_ticks / 40;
    
    // 4. Validate period (100μs to 60s)
    if (period_us >= MIN_EDGE_INTERVAL_US && 
        period_us <= MAX_EDGE_INTERVAL_MS * 1000) {
        
        // 5. Calculate RPM instantly
        g_rpm_value = 60000000UL / (period_us * BLADE_NUMBER);
        
        // 6. Update filter
        rpm_update_filter(g_rpm_value);
        
        // 7. Signal new data ready
        g_rpm_ready_flag ^= 0xFF;
        
        // 8. Reset timeout counter
        g_edge_timeout_counter = 0;
    }
    
    // Clear interrupt flag
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;
}
```

### Timer3A ISR (Timeout Detection - 100ms)
```c
void Timer3AIntHandler(void)
{
    // Increment timeout counter
    g_edge_timeout_counter++;
    
    // Check if motor stopped (no edges for 2000ms)
    if (g_edge_timeout_counter >= (RPM_STOP_TIMEOUT_MS / 100)) {
        g_rpm_value = 0;        // Motor stopped
        g_rpm_ready_flag ^= 0xFF;
    }
    
    // Clear interrupt
    TIMER3_ICR_R = TIMER_ICR_TATOCINT;
}
```

### RPM Formulas
```
Period to RPM:
  RPM = 60,000,000 / (period_μs × BLADE_NUMBER)

Example (1000 RPM, 2-blade):
  Period = 60,000,000 / (1000 × 2) = 30,000 μs = 30 ms
  Check:   60,000,000 / (30,000 × 2) = 1000 RPM ✓

Frequency to RPM:
  Frequency_Hz = (RPM × BLADE_NUMBER) / 60
  
  Example: 50 Hz, 2-blade
    RPM = (50 × 60) / 2 = 1500 RPM
```

### Range Table (BLADE_NUMBER = 2)
| Period | Frequency | RPM | Notes |
|--------|-----------|-----|-------|
| 100 μs | 10 kHz | 300,000 | Max (noise filter limit) |
| 1 ms | 1 kHz | 30,000 | High speed |
| 10 ms | 100 Hz | 3,000 | Medium speed |
| 30 ms | 33.3 Hz | 1,000 | Common test speed |
| 100 ms | 10 Hz | 300 | Low speed |
| 2 s | 0.5 Hz | 15 | Min (timeout limit) |

### RPM API Functions
```c
// Core functions
uint16_t rpm_get_value(void);            // Latest RPM
uint16_t rpm_get_filtered(void);         // Filtered RPM (4-sample avg)
uint32_t rpm_get_edge_interval_us(void); // Period in μs

// Status functions
bool rpm_is_ready(void);                 // New data available?
bool rpm_is_stopped(void);               // Motor stopped?
bool rpm_is_valid(uint16_t rpm);         // Valid range?

// Control functions
void rpm_clear_ready_flag(void);         // Clear flag
void rpm_reset(void);                    // Reset all state
void rpm_update_filter(uint16_t rpm);    // Add to filter

// Conversion functions
uint16_t rpm_from_period_us(uint32_t period);
uint16_t rpm_from_frequency(float freq_hz);
float rpm_to_frequency(uint16_t rpm);
```

---

## 🔌 Voltage/Current API (NEW v2.0)

### pso_iv.h Functions
```c
/* Primary conversions (ADC → Physical Units) */
uint16_t voltage_adc_to_mv(uint32_t adc_value);  // ADC → mV
uint16_t current_adc_to_ma(uint32_t adc_value);  // ADC → mA

/* Reverse conversions (Physical Units → ADC) */
uint32_t voltage_mv_to_adc(uint16_t voltage_mv); // mV → ADC
uint32_t current_ma_to_adc(uint16_t current_ma); // mA → ADC

/* Validation */
bool voltage_mv_is_valid(uint16_t voltage_mv);   // 0-33400?
bool current_ma_is_valid(uint16_t current_ma);   // 0-60000?

/* Conversion macros */
#define MV_TO_V(mv)   ((float)(mv) / 1000.0f)    // mV → V
#define MA_TO_A(ma)   ((float)(ma) / 1000.0f)    // mA → A
#define V_TO_MV(v)    ((uint16_t)((v) * 1000.0f))  // V → mV
#define A_TO_MA(a)    ((uint16_t)((a) * 1000.0f))  // A → mA
```

### Conversion Formulas
```
Voltage:
  V(mV) = (ADC × 33400) / 4095
  Example: ADC=2048 → 16704 mV = 16.704 V

Current:
  I(mA) = (ADC × 60000) / 4095
  Example: ADC=2048 → 30015 mA = 30.015 A

Power (derived):
  P(W) = V(V) × I(A)
  P(W) = (v_motor / 1000) × (i_motor / 1000)
```

### Usage in packet_data()
```c
// In packet_data() - ulink.c
dp->v_motor = voltage_adc_to_mv(adc0_buffer[2]);  // mV
dp->i_motor = current_adc_to_ma(adc1_buffer[2]);  // mA

// In MATLAB - simple!
V_battery = data.v_motor / 1000;  % mV → V
I_motor = data.i_motor / 1000;    % mA → A
P_motor = V_battery * I_motor;    % Watts
```

---

## PWM Profiles

### Trapezoid Profile
```c
trapezoid_config_t config = {
    .start_position = 0,        // 0% initial
    .peak_position = 80,        // 80% peak
    .end_position = 0,          // 0% final
    .ramp_up_ms = 3000,         // 3s ramp up
    .hold_ms = 5000,            // 5s hold at peak
    .ramp_down_ms = 2000,       // 2s ramp down
    .update_rate_hz = 10        // 10 Hz updates
};
pwm_set_trapezoid_config(&config);
pwm_profile_start(PROFILE_TRAPEZOID);
```

### Linear Ramp Profile
```c
linear_config_t config = {
    .start_position = 0,        // 0% start
    .end_position = 100,        // 100% end
    .duration_ms = 10000,       // 10s duration
    .update_rate_hz = 10        // 10 Hz updates
};
pwm_set_linear_config(&config);
pwm_profile_start(PROFILE_LINEAR);
```

### Step Profile
```c
step_config_t config = {
    .positions = {0, 20, 40, 60, 80, 100},  // 6 steps
    .durations_ms = {1000, 2000, 2000, 2000, 2000, 1000},
    .num_steps = 6              // Total steps
};
pwm_set_step_config(&config);
pwm_profile_start(PROFILE_STEP);
```

---

## 🐛 Debug Macros

### Debug Pins (pso_debug.h)
```c
DEBUG_STREAM_HIGH()     // Set PD4 high
DEBUG_STREAM_LOW()      // Set PD4 low
DEBUG_STREAM_TOGGLE()   // Toggle PD4

DEBUG_LOOP_TOGGLE()     // Toggle PD5 - main loop
DEBUG_ADC_TOGGLE()      // Toggle PD6 - ADC ISR
DEBUG_STATE_TOGGLE()    // Toggle PD7 - state changes
```

### Usage Example
```c
void ADC0SS1IntHandler(void)
{
    DEBUG_ADC_HIGH();           // Start timing
    
    // Read ADC channels
    adc0_buffer[0] = ADC0_SSFIFO1_R;
    adc0_buffer[1] = ADC0_SSFIFO1_R;
    adc0_buffer[2] = ADC0_SSFIFO1_R;
    
    DEBUG_ADC_LOW();            // End timing
    // Measure pulse width on oscilloscope
    
    ADC0_ISC_R = ADC_ISC_IN1;   // Clear interrupt
}
```

---

## Formula Reference

### RPM Calculation
```
Edge-Period Method:
  RPM = 60,000,000 / (period_μs × BLADE_NUMBER)

Where:
  60,000,000 = microseconds per minute
  period_μs  = time between edges (μs)
  BLADE_NUMBER = pulses per revolution

Minimum RPM (for 1 RPM with BLADE_NUMBER=2):
  Timeout_needed = 60,000,000 / (1 × 2) = 30,000,000 μs = 30s
  Set RPM_STOP_TIMEOUT_MS = 35000 (with margin)
```

### PWM Duty Calculation
```
Pulse width:
  pulse_μs = 1000 + (throttle × 10)
  
Timer match value:
  match = (pulse_μs × 40) - 1
  
Where:
  throttle = 0-100%
  40 = clock cycles per μs (40 MHz)

Example (50% throttle):
  pulse_μs = 1000 + (50 × 10) = 1500 μs
  match = (1500 × 40) - 1 = 59999
```

### Voltage/Current Conversion
```
Voltage:
  V(mV) = (ADC × 33400) / 4095
  V(V) = V(mV) / 1000

Current:
  I(mA) = (ADC × 60000) / 4095
  I(A) = I(mA) / 1000

Power:
  P(W) = V(V) × I(A)
  P(W) = (v_motor / 1000) × (i_motor / 1000)
```

---

## LED Patterns

| LED Color | Pattern | State | Trigger |
|-----------|---------|-------|---------|
| Blue | Slow blink (1Hz) | Idle/Standby | Waiting for SW1 |
| Green | Fast blink (5Hz) | Streaming | Active data transmission |
| Red | Fast blink (10Hz) | Error/Fault | System error |
| White | 3× flash | Complete | Test finished |
| All Off | - | Init/Off | Power up or shutdown |

---

## Interrupt Priority Table

| Priority | Interrupt | Handler | Frequency | Purpose |
|----------|-----------|---------|-----------|---------|
| **0** (Highest) | SysTick | SysTick_Handler | 1 kHz | System tick |
| **1** | ADC0 SS1 | ADC0SS1IntHandler | 5 kHz | Voltage + accel |
| **1** | ADC1 SS1 | ADC1SS1IntHandler | 5 kHz | Current + accel |
| **2** | WTimer1A | WTimer1AIntHandler | Variable | RPM edge capture |
| **3** | Timer3A | Timer3AIntHandler | 10 Hz | RPM timeout |
| **4** | UART0 RX | UART0IntHandler | Async | RX data |
| **5** (Lowest) | Timer0A | - | 5 kHz | ADC trigger (no ISR) |

---

## Timing Measurements

| Operation | Duration | CPU Load @ 5kHz | Notes |
|-----------|----------|-----------------|-------|
| ADC0 ISR (3 channels) | 8-12 μs | ~0.005% | Store to buffer |
| ADC1 ISR (3 channels) | 8-12 μs | ~0.005% | Store to buffer |
| WTimer1A ISR (RPM) | 5-8 μs | <0.5% | Per edge (variable) |
| Timer3A ISR (timeout) | 3-5 μs | <0.0005% | Every 100ms |
| packet_data() | 50-80 μs | ~0.03% | Data assembly |
| Voltage scaling | 2-3 μs | - | voltage_adc_to_mv() |
| Current scaling | 2-3 μs | - | current_adc_to_ma() |
| UART TX (21 bytes) | ~1.8 ms | ~2% @ 200Hz | Transmission time |
| **Total CPU Load** | - | **~10-15%** | Plenty of headroom |

---

## Build & Flash Commands

### Code Composer Studio (GUI)
```
Project → Build All (Ctrl+B)
Run → Debug (F11)
Run → Resume (F8)
```

### Command Line (Linux/macOS)
```bash
# Build
cd /opt/ti/ccs/eclipse
./ccstudio -noSplash -data ~/workspace \
  -application com.ti.ccstudio.apps.projectBuild \
  -project PSO -config Debug

# Flash with lm4flash
lm4flash PSO.bin

# Flash with OpenOCD
openocd -f board/ek-tm4c123gxl.cfg \
  -c "program PSO.out verify reset exit"
```

### Makefile (if available)
```bash
make clean
make all
make flash
```

---

## 📟 Serial Terminal Setup

### Connection Settings
```
Port:         /dev/ttyACM0 (Linux) or COM3 (Windows)
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

# Check device
ls -l /dev/ttyACM*
```

### Windows Commands
```cmd
# PuTTY
putty -serial COM3 -sercfg 115200,8,n,1,N

# PowerShell
$port = new-Object System.IO.Ports.SerialPort COM3,115200,None,8,one
$port.Open()
```

---

## Key Files

### Core Application
```
main.c          - State machine, main loop
pso_init.c/h    - Peripheral initialization
pso_isr.c/h     - Interrupt service routines
pso_system.c/h  - System control, LED indicators
```

### Sensor Modules
```
pso_rpm.c/h     - RPM measurement (edge-period)
pso_iv.c/h      - Voltage/current scaling ⭐ NEW
pso_data.c/h    - Data handling and buffering
```

### Control & Communication
```
pso_pwm.c/h     - PWM generation and profiles
pso_uart.c/h    - UART configuration
ulink.c/h       - Protocol encoder/decoder
```

### Utilities
```
fifo.c/h        - Circular buffer implementation
pso_timing.c/h  - System timing, delays, SysTick
pso_debug.c/h   - Debug GPIO pins, timing measurement
pso_led.c/h     - RGB LED control
```

### Configuration
```
pso_config.h    - System-wide configuration constants
```

---

## 🔗 Quick Links

### Datasheets & Guides
- [TM4C123GH6PM Datasheet](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf)
- [TivaWare Peripheral Library](https://www.ti.com/tool/SW-TM4C)
- [Tiva C LaunchPad User Guide](https://www.ti.com/lit/ug/spmu296/spmu296.pdf)
- [INA169 Datasheet](https://www.ti.com/lit/ds/symlink/ina169.pdf)

### Tools & Software
- [Code Composer Studio](https://www.ti.com/tool/CCSTUDIO)
- [OpenOCD](https://openocd.org/)
- [lm4flash Utility](https://github.com/utzig/lm4tools)

### Support
- [TI E2E Forums](https://e2e.ti.com)
- [Project Repository](#)

---

## 🚀 Quick Start

```
1. Clone repository
2. Open in Code Composer Studio
3. Build project (Ctrl+B)
4. Connect Tiva C LaunchPad via USB
5. Flash to board (F11)
6. Press SW1 to start streaming
7. Press SW2 to trigger PWM profile
8. Monitor data via serial terminal (115200 baud)
```

---

## Test Commands

### RPM Test (Function Generator)
```
Freq (Hz)   →   Expected RPM (BLADE_NUMBER=2)
1           →   30
5           →   150
10          →   300
16.67       →   500
33.33       →   1000
50          →   1500
83.33       →   2500
166.67      →   5000
```

### MATLAB Packet Parsing
```matlab
% v2.0 - Simplified!
data = parse_uart_packet(raw_bytes);

V = data.v_motor / 1000;  % mV → V
A = data.i_motor / 1000;  % mA → A
P = V * A;                % Watts
```

---

**Version**: 2.0 Final  
**Last Updated**: 2025-12-15  
**Maintainer**: Embedded Systems Team

**Changes in v2.0**:
- ✅ Added pso_iv module (voltage/current scaling)
- ✅ v_motor now in mV (0-33400), i_motor in mA (0-60000)
- ✅ Simplified MATLAB processing (no hardware constants needed)
- ✅ Updated packet format documentation
- ✅ Added V/I API functions and formulas
