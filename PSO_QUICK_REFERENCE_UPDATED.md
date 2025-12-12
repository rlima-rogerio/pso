# PSO Quick Reference Card (Edge-Period Method)

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
PC6  →  RPM Input (WT1CCP0) - **Edge capture (INTERRUPT ENABLED)**
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
- **RPM Update Rate**: **Per edge (instant)** - NOT fixed 10 Hz
- **Timeout Check**: 10 Hz (100 ms) - for motor stopped detection
- **PWM Frequency**: 50 Hz (20 ms period)
- **UART Baud**: 115200 bps

### RPM Measurement (Edge-Period Method)
- **Method**: Period measurement between edges
- **Timer Resolution**: 25 ns (40 MHz clock)
- **Update Mode**: Interrupt-driven (each rising edge)
- **Formula**: `RPM = 60,000,000 / (period_μs × BLADE_NUMBER)`
- **Min Measurable**: ~15 RPM (2000ms timeout)
- **Max Measurable**: ~300,000 RPM (100μs min period)
- **Resolution @ 1000 RPM**: ~0.001 RPM (theoretical)
- **Latency**: 5-8 μs (ISR execution time)

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

### pso_rpm.h (Edge-Period Configuration)
```c
#define BLADE_NUMBER            2U      // Pulses per revolution
#define RPM_CALC_PERIOD_MS      100U    // Timeout check period
#define RPM_STOP_TIMEOUT_MS     2000U   // Motor stopped timeout
#define MIN_EDGE_INTERVAL_US    100U    // Min valid period (noise filter)
#define MAX_EDGE_INTERVAL_MS    60000U  // Max valid period (60s)
#define RPM_FILTER_SAMPLES      4U      // Moving average filter size
```

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
RPM field: Updated on each edge (instant measurement)
```

### Data Types
```c
typedef struct {
    uint16_t index;      // Packet counter
    int16_t  accel[3];   // X, Y, Z acceleration
    uint16_t rpm;        // RPM (updated per edge)
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
3: SYS_STATE_DATA_PROCESSING - Prepare packets (read latest RPM)
4: SYS_STATE_STREAMING     - Send data via UART
5: SYS_STATE_PWM_CONTROL   - Execute PWM profile
6: SYS_STATE_STOPPING      - Graceful shutdown
7: SYS_STATE_ERROR         - Error handling
```

### Typical Flow
```
INIT → IDLE → [SW1] → TIMING → PROCESSING → STREAMING ←→ PWM_CONTROL
                                                ↓
                                           STOPPING → INIT
```

## ⚡ RPM Measurement - Edge-Period Method

### Algorithm (WTimer1A ISR)
```c
// Triggered on each rising edge at PC6
void WTimer1AIntHandler(void)
{
    // 1. Capture timer value
    current_capture = WTIMER1_TAR_R;
    
    // 2. Calculate period (handle overflow)
    period_ticks = current_capture - last_capture;
    
    // 3. Convert to microseconds
    // 40 MHz = 25 ns/tick = 40 ticks/μs
    period_us = period_ticks / 40;
    
    // 4. Validate period (100μs to 60s)
    if (valid_range(period_us)) {
        // 5. Calculate RPM instantly
        g_rpm_value = 60,000,000 / (period_us × BLADE_NUMBER);
        
        // 6. Signal new data
        g_rpm_ready_flag = 1;
    }
    
    // 7. Save for next edge
    last_capture = current_capture;
}
```

### Timer3A (Timeout Detection)
```c
// Called every 100ms
void Timer3AIntHandler(void)
{
    timeout_counter += 100;
    
    // Check if motor stopped (no edges for 2000ms)
    if (timeout_counter >= 2000) {
        g_rpm_value = 0;
        g_rpm_ready_flag = 1;
    }
}
```

### Key Formulas
```
Period to RPM:
  RPM = 60,000,000 / (period_μs × BLADE_NUMBER)

Example (1000 RPM, 2-blade):
  Period = 60,000,000 / (1000 × 2) = 30,000 μs = 30 ms
  Verify: 60,000,000 / (30,000 × 2) = 1000 RPM ✓

Frequency to RPM:
  Frequency = RPM × BLADE_NUMBER / 60
  50 Hz, 2-blade → (50 × 60) / 2 = 1500 RPM
```

### Range and Resolution
```
BLADE_NUMBER = 2 (example)

Minimum RPM:
  Timeout = 2000 ms → Period = 2,000,000 μs
  RPM_min = 60,000,000 / (2,000,000 × 2) = 15 RPM

Maximum RPM:
  Min period = 100 μs (noise filter)
  RPM_max = 60,000,000 / (100 × 2) = 300,000 RPM

Resolution @ 1000 RPM:
  Timer res = 25 ns
  ΔRPM ≈ 0.001 RPM (with 4-sample filter: ~0.01 RPM)
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

## Debug Macros

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

## RPM API Functions

### Core Functions
```c
// Get current RPM value
uint32_t rpm_get_value(void);

// Check if new RPM data available
bool rpm_is_ready(void);

// Clear ready flag after reading
void rpm_clear_ready_flag(void);

// Get edge period in microseconds
uint32_t rpm_get_edge_interval_us(void);

// Check if motor is stopped (timeout)
bool rpm_is_stopped(void);

// Get filtered RPM (4-sample moving average)
uint32_t rpm_get_filtered(void);

// Update filter with new RPM value
void rpm_update_filter(uint32_t new_rpm);

// Reset RPM system
void rpm_reset(void);
```

### Conversion Functions
```c
// Calculate RPM from period
uint32_t rpm_from_period_us(uint32_t period_us, 
                             uint32_t pulses_per_rev);

// Convert RPM to frequency
uint32_t rpm_to_frequency(uint32_t rpm, 
                           uint32_t pulses_per_rev);

// Convert frequency to RPM
uint32_t rpm_from_frequency(uint32_t frequency_hz, 
                             uint32_t pulses_per_rev);
```

### Usage Example
```c
// In main loop
if (rpm_is_ready()) {
    uint32_t rpm = rpm_get_value();
    uint32_t rpm_filtered = rpm_get_filtered();
    uint32_t period_us = rpm_get_edge_interval_us();
    
    printf("RPM: %lu (filtered: %lu), Period: %lu us\r\n",
           rpm, rpm_filtered, period_us);
    
    rpm_clear_ready_flag();
}

// Check for stopped motor
if (rpm_is_stopped()) {
    printf("Motor stopped (no edges for 2000ms)\r\n");
}
```

## LED Patterns

| Color  | Pattern      | State          |
|--------|--------------|----------------|
| Blue   | Slow blink   | Idle/Standby   |
| Green  | Fast blink   | Streaming      |
| Red    | Fast blink   | Error          |
| White  | 3× Flash     | Complete       |

## ⚡ Interrupt Priorities

| Interrupt | Priority | Frequency | Handler              |
|-----------|----------|-----------|----------------------|
| ADC0/1    | 1 (High) | 5 kHz     | ADC0SS1IntHandler    |
| **WTimer1A** | **2 (High)** | **Per edge** | **WTimer1AIntHandler** |
| Timer3A   | 3        | 10 Hz     | Timer3AIntHandler    |
| UART0     | 4        | Async     | UART0IntHandler      |

## Timing Measurements

| Operation              | Duration  | CPU Load  |
|------------------------|-----------|-----------|
| ADC ISR (6 channels)   | 15-20 μs  | 0.01%     |
| **RPM edge capture**   | **5-8 μs**| **<0.001%** |
| Timeout check (100ms)  | 3-5 μs    | <0.001%   |
| Packet assembly        | 50-100 μs | 0.05%     |
| UART TX (21 bytes)     | ~2 ms     | 2%        |

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

## Serial Terminal

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

## RPM Testing Procedure

### Signal Generator Test
```
1. Connect signal generator to PC6 (WT1CCP0)
2. Configure: Square wave, 3.3V, 50% duty cycle
3. Test frequencies and expected RPM (BLADE_NUMBER=2):

   Freq    Period      Expected RPM    Formula Check
   ----    ------      ------------    -------------
   1 Hz    1,000 ms    30 RPM          60,000,000/(1,000,000×2)=30 ✓
   10 Hz   100 ms      300 RPM         60,000,000/(100,000×2)=300 ✓
   50 Hz   20 ms       1,500 RPM       60,000,000/(20,000×2)=1,500 ✓
   100 Hz  10 ms       3,000 RPM       60,000,000/(10,000×2)=3,000 ✓
   1 kHz   1 ms        30,000 RPM      60,000,000/(1,000×2)=30,000 ✓
   10 kHz  100 μs      300,000 RPM     60,000,000/(100×2)=300,000 ✓

4. Verify with oscilloscope:
   - Probe PC6: Measure period
   - Probe PD6: Check ISR execution (should see pulse on each edge)
   - Verify calculated RPM matches expected (±0.1%)

5. Test noise rejection:
   - Add 50 μs glitches → should be filtered
   - Check MIN_EDGE_INTERVAL_US = 100 μs filter

6. Test timeout:
   - Stop signal → RPM should go to 0 after 2000 ms
   - Restart signal → RPM should update on first edge
```

### Motor Test
```
1. Connect Hall sensor to PC6
2. Manually spin motor slowly
3. Expected behavior:
   - RPM updates on each pulse
   - Low latency (<10ms)
   - Smooth readings with filter
4. Monitor UART output for RPM values
5. Verify zero RPM on motor stop (2s timeout)
```

## Key Files

### Core Application
- `main.c` - State machine and main loop
- `pso_init.c` - Hardware initialization
- `pso_isr.c` - **Interrupt handlers (WTimer1A edge capture)**

### Peripherals
- `pso_uart.c` - UART communication
- `pso_pwm.c` - PWM generation
- `pso_rpm.c` - **RPM measurement (edge-period method)**
- `pso_timing.c` - System timing and SysTick
- `pso_led.c` - LED control

### Data Handling
- `fifo.c` - Circular buffer
- `ulink.c` - Protocol implementation
- `pso_data.c` - Packet assembly

### Utilities
- `pso_debug.c` - Debug pins
- `pso_system.c` - Status indicators

## Useful Links

- **Datasheet**: [TM4C123GH6PM](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf)
- **TivaWare**: [Peripheral Driver Library](https://www.ti.com/tool/SW-TM4C)
- **LaunchPad**: [User Guide](https://www.ti.com/lit/ug/spmu296/spmu296.pdf)
- **Timer Capture**: [SPMA075 - Timer Capture Configuration](https://www.ti.com/lit/an/spma075/spma075.pdf)
- **CCS**: [Download](https://www.ti.com/tool/CCSTUDIO)
- **E2E Forum**: [Support](https://e2e.ti.com)

## Method Comparison

|   Feature   | Edge-Count (Old)  |    Edge-Period (New)  |
|-------------|-------------------|-----------------------|
| Update rate | 10 Hz fixed       | Per edge (instant)    |
| Latency     | 100 ms            | 5-8 μs                |
| Low RPM min | ~60 RPM           | ~15 RPM               |
| Resolution  | ~0.1 RPM          | ~0.001 RPM            |
| Timer use   | Free-run counter  | Capture + timeout     |
| ISR         | Timer3 only       | **WTimer1A + Timer3** |
| CPU load    | 0.001%            | <0.001%               |
| Accuracy    | Good              | Excellent             |

---

**Quick Start**: Clone → Import to CCS → Build → Flash → Press SW1 to start  
**RPM Test**: Signal gen @ 50Hz → Expect 1500 RPM (2-blade) → Verify ±0.1%
