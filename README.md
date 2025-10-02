# System Calibration Script

This calibration script automatically tests your control system to solve for the plant parameters τ (tau) and K1 from the transfer function K1/(s(τs+1)).

## How to Use

1. **Upload to Arduino**: Compile and upload the code to your Arduino
2. **Monitor Serial Output**: Open the Serial Monitor at 115200 baud
3. **Wait for Completion**: The script will run automatically and output results

## What the Script Does

### Test Sequence
- Tests **8 different Kp values** from 18 to 24
- For each Kp, tests **10 different setpoints** from π/8 to π/4 radians
- Each test runs for **0.75 seconds** with **0.25 seconds settling** between tests
- Total test time: approximately **80 seconds**

### Data Collection
For each test, the script measures:
- **Percentage Overshoot**: How much the system overshoots the target
- **Peak Time**: Time to reach the first peak
- **Steady State Value**: Final settled value

### Parameter Calculation
The script uses the collected data to solve for:
- **τ (tau)**: Time constant of the plant
- **K1**: Gain of the plant

## Output Format

### During Testing
```
time,angle,kp,setpoint,test_phase
1.234,0.456,30.0,0.785,test
```

### Final Results
```
=== Calibration Results ===
Kp,Setpoint,PO%,PeakTime(s),Valid
20.0,0.000,0.00,0.000,0
20.0,0.196,15.23,0.456,1
...
Calculated τ (tau): 0.1234
Calculated K1: 2.5678
```

## Understanding the Results

The script calculates parameters for a second-order system with transfer function:
```
G(s) = K1/(s(τs+1))
```

With proportional controller Kp, the closed-loop system becomes:
```
T(s) = Kp*K1/(τs² + s + Kp*K1)
```

The parameters are calculated using:
- **Damping ratio (ζ)** from percentage overshoot
- **Natural frequency (ωn)** from peak time
- **τ = 1/(2ζωn)**
- **K1 = (ωn²τ)/Kp**

## Troubleshooting

- **No valid data**: Ensure setpoints are non-zero for overshoot analysis
- **Inconsistent results**: Check that the system is properly connected and calibrated
- **Oscillation issues**: The script includes settling time between tests to allow the system to stabilize
