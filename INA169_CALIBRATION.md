# INA169 Current Sensor Empirical Calibration

## Summary

This document describes the empirical calibration procedure and results for the INA169 current sensing circuit used in the PSO (Propulsion System Observer) data acquisition system.

## Purpose

The INA169 current sensor required calibration to establish an accurate conversion factor between ADC readings and actual current measurements. This calibration was performed using precision measurement equipment to ensure reliable current monitoring across the operational range.

## Calibration Procedure

### Test Equipment

- **Reference Ammeter**: Calibrated DC ammeter (assumed ±0.5% accuracy)
- **ADC**: 12-bit (TM4C123GH6PM), Vref = 3.3V
- **Measurement Method**: Direct comparison between reference ammeter and ADC voltage readings

### Hardware Configuration

| Component | Value | Notes |
|-----------|-------|-------|
| Shunt Resistor (Rs) | 0.5 mΩ | Current sensing element |
| Load Resistor (RL) | 55 kΩ | Two 110kΩ ±0.1% in parallel |
| INA169 Transconductance (gm) | 1000 µA/V | Per datasheet specification |
| ADC Reference Voltage | 3300 mV | TM4C123 internal reference |
| ADC Resolution | 12-bit (4096 steps) | Range: 0-4095 counts |

### Measurement Data

Eleven measurement points were collected across the operational current range:

| Current (A) | ADC Voltage (mV) | ADC Counts |
|-------------|------------------|------------|
| 0.057       | 3.55             | 4          |
| 0.33        | 16.5             | 20         |
| 0.51        | 25.3             | 31         |
| 1.02        | 50.2             | 62         |
| 2.08        | 101              | 125        |
| 3.08        | 151              | 187        |
| 4.20        | 203              | 252        |
| 5.00        | 244              | 303        |
| 7.36        | 358              | 444        |
| 8.12        | 397              | 493        |
| 9.75        | 475              | 589        |

**ADC Counts Calculation:**
```
ADC_counts = round((ADC_voltage_mV / 3300) × 4095)
```

## Calibration Results

### Linear Regression Analysis

The relationship between ADC counts and current was analyzed using linear regression:

**Regression Equation:**
```
I(mA) = 16.5402 × ADC_counts - 0.7827
```

**Statistical Quality:**
- **R² = 0.999973** (excellent linear correlation)
- **Slope**: 16.5402 mA/count
- **Offset**: -0.7827 mA (negligible, within noise floor)

**Error Analysis:**

| ADC  | I_measured (mA) | I_fit (mA) | Error (%) |
|------|-----------------|------------|-----------|
| 4    | 57.2            | 65.4       | -14.30    |
| 20   | 330.0           | 330.0      | -0.01     |
| 31   | 510.0           | 512.0      | -0.38     |
| 62   | 1020.0          | 1024.7     | -0.46     |
| 125  | 2080.0          | 2066.7     | +0.64     |
| 187  | 3080.0          | 3092.2     | -0.40     |
| 252  | 4200.0          | 4167.3     | +0.78     |
| 303  | 5000.0          | 5010.9     | -0.22     |
| 444  | 7360.0          | 7343.0     | +0.23     |
| 493  | 8120.0          | 8153.5     | -0.41     |
| 589  | 9750.0          | 9741.4     | +0.09     |

**Performance Summary:**
- **Typical error**: < 1% across most of range
- **Maximum error**: 14.30% at lowest measurement point (57 mA)
- **Recommended operating range**: > 100 mA for best accuracy

### Conversion Factor for Firmware

For integer-only implementation in embedded systems, the conversion formula is:

```c
I(mA) = (ADC_counts × 67732) / 4095
```

Where:
- **IMAX_MA = 67732** is the maximum current in milliamperes at full-scale ADC reading
- This corresponds to **67.7 A** maximum measurable current

**Derivation:**
```
At ADC = 4095 (full scale):
I_max = 16.5402 × 4095 - 0.7827 ≈ 67732 mA

Neglecting offset for integer math:
I(mA) ≈ (ADC × 67732) / 4095
```

The offset term (-0.7827 mA) is omitted as it represents only 0.001% of full scale and is within the measurement noise floor.

## Firmware Implementation

### Constants (pso_iv.h)

```c
/* Current Monitor - INA169 Configuration (Calibrated 2025-02-07) */
#define RSHUNT_MOHM          0.5f         /* Shunt resistance: 0.5 mΩ */
#define RL_OHM               55000UL      /* Load resistor: 55 kΩ (parallel) */
#define INA169_GM            0.001f       /* Transconductance: 1000 µA/V */

/* Calibrated maximum current */
#define IMAX_MA              67732UL      /* 67.7 A = 67732 mA */

/* Calibrated conversion factor (for reference) */
#define CALIBRATED_MA_PER_ADC  16.5402f   /* mA per ADC count */
```

### Conversion Function (pso_iv.c)

```c
uint16_t current_adc_to_ma(uint32_t adc_value)
{
    if (adc_value > ADC_MAX_VALUE) {
        adc_value = ADC_MAX_VALUE;
    }
    
    /*
     * Calibrated conversion: I(mA) = (ADC × 67732) / 4095
     * 
     * Based on empirical calibration (2025-02-07):
     *   - 11 measurement points from 57 mA to 9.75 A
     *   - Linear regression: I = 16.5402×ADC - 0.7827
     *   - R² = 0.999973
     *   - Typical error < 1%
     * 
     * Maximum current at ADC = 4095: 67.7 A
     */
    uint32_t current_ma = (adc_value * IMAX_MA) / ADC_MAX_VALUE;
    
    return (uint16_t)current_ma;
}
```

## Verification Points

Use these reference values to verify correct implementation:

| ADC Value | Expected Current (mA) | Expected Current (A) |
|-----------|-----------------------|----------------------|
| 0         | 0                     | 0.000                |
| 100       | 1654                  | 1.654                |
| 500       | 8270                  | 8.270                |
| 1000      | 16540                 | 16.540               |
| 1500      | 24810                 | 24.810               |
| 2000      | 33080                 | 33.080               |
| 2500      | 41350                 | 41.350               |
| 3000      | 49621                 | 49.621               |
| 3500      | 57891                 | 57.891               |
| 4095      | 67732                 | 67.732               |

## Calibration Maintenance

### Recommended Re-calibration Schedule

- **Initial**: After any hardware modifications
- **Periodic**: Every 6 months for critical applications
- **Triggered**: If drift > 2% is observed in known-current tests

### Re-calibration Procedure

1. Set up precision reference ammeter in series with load
2. Apply known current loads: 0.5A, 1A, 2A, 5A, 10A, 20A, 30A
3. Record ADC voltage readings for each current point
4. Perform linear regression analysis (see MATLAB script)
5. Update `IMAX_MA` constant if new calibration differs by > 1%

## Supporting Files

### Analysis Tools

- **`ina169_calibration_analysis.m`**: MATLAB script for regression analysis
  - Performs linear fit
  - Generates calibration plots
  - Calculates error statistics
  - Exports calibration parameters

### Modified Source Files

- **`pso_iv.h`**: Header with updated `IMAX_MA` constant
- **`pso_iv.c`**: Implementation with calibrated conversion formula

## Usage in MATLAB/Octave

After data acquisition, convert raw ADC counts to current:

```matlab
% Raw data from firmware (already converted to mA)
current_mA = data.current;  % Firmware applies: (ADC × 67732) / 4095

% Convert to amperes for analysis
current_A = current_mA / 1000;

% Plot
plot(time, current_A);
ylabel('Current (A)');
```

If working with raw ADC values:

```matlab
% Apply calibration
IMAX_MA = 67732;
ADC_MAX = 4095;
current_mA = (adc_counts * IMAX_MA) / ADC_MAX;
current_A = current_mA / 1000;
```

## Notes

- The offset term from regression (-0.7827 mA) is omitted in firmware for computational efficiency
- This introduces negligible error: 0.001% at full scale, 1.4% at 57 mA
- For currents below 100 mA, expect reduced accuracy due to ADC resolution limits
- Maximum error of 14.3% occurs at the lowest measurement point (57 mA)
- For critical low-current measurements (< 100 mA), consider higher-resolution ADC or different sensor

## Calibration History

| Date       | Version | Performed By | IMAX_MA | Notes                          |
|------------|---------|--------------|---------|--------------------------------|
| 2025-02-07 | 1.0     | ROG3R10      | 67732   | Initial empirical calibration  |
|            |         |              |         | 11-point measurement           |
|            |         |              |         | Range: 57 mA - 9.75 A          |

---

**Calibration Status**: ✓ Valid  
**Next Calibration Due**: 2025-08-07 (recommended)  
**Calibration Method**: Empirical measurement with linear regression  
**Quality**: R² = 0.999973
