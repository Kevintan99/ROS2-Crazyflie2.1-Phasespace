# 🚀 Crazyflie Fast PID Optimization System

## Overview

This system is specifically designed for rapidly obtaining optimal PID control parameters for Crazyflie, solving the efficiency problem of traditional Bayesian optimization where each experiment requires restarting the Crazyflie.

## Core Optimization Strategy

### 1. Time Optimization
- **Fast Experiments**: 15-second experiment time (vs 20-second standard)
- **Quick Takeoff/Landing**: 2-second takeoff/landing (vs 3-second standard)
- **Reduced Iterations**: 15 iterations (vs 30 standard)
- **Expected Total Time**: 15-25 minutes (vs 60 minutes standard)

### 2. Smart Initialization
- **Physical Model**: Calculate theoretical optimal values based on Crazyflie mass and dynamics characteristics
- **Experience Database**: Automatically learn and store successful parameter combinations
- **Adaptive Boundaries**: Dynamically adjust search ranges based on historical data

### 3. Parameter Space Optimization
- **Optimize Kp Only**: Fix Ki and Kd to empirically optimal values, optimize only 3 Kp parameters
- **Reduced Search Range**: Limit to 1.2-2.8 range based on Crazyflie characteristics
- **Early Stopping**: Automatically stop when target error is reached

## File Structure

```
bayes_opt_ros2/
├── launch_fast_pid_optimization.py    # Fast optimization launch script
├── config/
│   └── fast_pid_optimization_config.yaml  # Fast optimization configuration
├── scripts/
│   ├── bayesian_opt_node.py               # Bayesian optimization node
│   ├── fast_pid_experiment_client.py      # Fast experiment client
│   ├── flight_logger.py                   # Flight data logger
│   └── phasespace_wrapper.py              # PhaseSpace integration
├── cache/                                 # Optimization cache files
├── flight_logs/                           # Flight data logs
└── README_FAST_PID_OPTIMIZATION.md        # This document
```

## Quick Start

### Method 1: Using Fast Launch Script (Recommended)

```bash
cd bayes_opt_ros2
python3 launch_fast_pid_optimization.py
```

### Method 2: Manual Component Startup

Terminal 1 - Start Bayesian optimization node:
```bash
cd /home/fann/crazyswarm_phasespace/src/bayes_opt_ros2
python3 scripts/bayesian_opt_node.py --ros-args --params-file config/fast_pid_optimization_config.yaml
```

Terminal 2 - Start fast PID experiment client:
```bash
cd /home/fann/crazyswarm_phasespace/src/bayes_opt_ros2
python3 scripts/fast_pid_experiment_client.py
```

### Method 3: Using Launch File (Recommended)

```bash
cd /home/fann/crazyswarm_phasespace/src/bayes_opt_ros2
python3 launch_fast_pid_optimization.py
```

## System Requirements

### Hardware Requirements
- ✅ Crazyflie 2.0/2.1 connected and ready to fly
- ✅ PhaseSpace motion capture system running normally
- ✅ Safe flight area (minimum 2x2x2 meters)

### Software Requirements
- ✅ ROS2 (Humble/Iron)
- ✅ Python 3.8+
- ✅ Dependencies: `bayesian-optimization`, `numpy`, `scipy`

## Configuration

### Fast Optimization Configuration (fast_pid_optimization_config.yaml)

```yaml
bayes_opt_node:
  ros__parameters:
    # Optimize only 3 Kp parameters
    param_names: ["kp_x", "kp_y", "kp_z"]
    
    # Reduced search ranges (based on experience)
    lower_bounds: [1.2, 1.2, 1.4]
    upper_bounds: [2.8, 2.8, 2.6]
    
    # Fast optimization settings
    n_iter: 15          # 15 iterations
    init_points: 5      # 5 initial points
    
    # Use EI acquisition function (robust to noise)
    acquisition_type: "ei"
    xi: 0.05           # Faster convergence
    
    # Reduced timeout for faster operation
    timeout_sec: 90.0   # 90 second timeout
```

### Smart Initialization Features

The smart initializer automatically:

1. **Theoretical Calculation**: Calculate theoretical optimal Kp values based on Crazyflie physical parameters
2. **Experience Query**: Select best parameters from successful case database
3. **Adaptive Boundaries**: Adjust search ranges based on historical data
4. **Strategy Recommendations**: Recommend optimization strategies based on experience data

## Experiment Workflow

Each PID parameter test includes the following steps:

1. **Device Connection** (2s)
   - Connect PhaseSpace system
   - Connect Crazyflie

2. **Parameter Setup** (3s)
   - Set PID parameters
   - Configure controller and estimator
   - Reset state estimator

3. **Fast Flight Experiment** (15s)
   - 2s quick takeoff to 0.5m
   - 11s hover data collection
   - 2s quick landing

4. **Result Calculation** (1s)
   - Calculate position tracking error
   - Update experience database

**Total per experiment: ~21 seconds**

## Optimization Results

### Performance Metrics
- **Position Tracking Error**: Target < 0.1m RMSE
- **Response Time**: Settling time < 2 seconds
- **Oscillation Level**: Minimize overshoot

### Output Parameters
After optimization completion, you will get:
```
Optimal PID Parameters:
- posCtlPid.xKp = 2.123
- posCtlPid.yKp = 2.098  
- posCtlPid.zKp = 2.245

Fixed Parameters (Recommended Values):
- posCtlPid.xKi = 0.1
- posCtlPid.yKi = 0.1
- posCtlPid.zKi = 0.1
- posCtlPid.xKd = 0.35
- posCtlPid.yKd = 0.35
- posCtlPid.zKd = 0.45
```

## Troubleshooting

### Common Issues

1. **Crazyflie Connection Failure**
   ```bash
   # Check URI configuration
   export CRAZYFLIE_URI=radio://0/81/2M/E7E7E7E7E7
   
   # Check USB permissions
   sudo chmod 666 /dev/ttyUSB0
   ```

2. **PhaseSpace Data Anomalies**
   ```bash
   # Check PhaseSpace system status
   # Ensure rigid body ID is correctly configured
   # Verify coordinate transformation settings
   ```

3. **Slow Optimization Convergence**
   - Check if flight environment is stable (no airflow)
   - Confirm PhaseSpace tracking quality
   - Consider adjusting search ranges

4. **High Experiment Failure Rate**
   - Increase takeoff wait time
   - Check battery level
   - Verify safe area settings

### Debug Mode

Enable detailed logging:
```bash
export ROS_LOG_LEVEL=DEBUG
python3 scripts/fast_pid_experiment_client.py
```

## Performance Comparison

| Metric | Standard Optimization | Fast Optimization | Improvement |
|--------|---------------------|-------------------|-------------|
| Experiment Time | 35s | 21s | 40% ↓ |
| Total Iterations | 30 | 15 | 50% ↓ |
| Total Time | 60min | 20min | 67% ↓ |
| Parameters | 9D | 3D | 67% ↓ |
| Search Space | Large | Small | Intelligent Reduction |

## Advanced Usage

### Custom Experience Database

Edit `cache/pid_experience_db.json`:
```json
{
  "successful_params": [
    {"kp": [2.0, 2.0, 2.2], "error": 0.08, "conditions": "indoor_no_wind"}
  ],
  "best_known": {"kp": [2.0, 2.0, 2.2], "error": 0.06}
}
```

### Adjust Optimization Strategy

Modify optimization strategy parameters (in configuration file):
```python
# More aggressive exploration
xi = 0.1  # Increase exploration parameter

# More conservative convergence
xi = 0.01  # Decrease exploration parameter
```

### Extend to Other Parameters

To optimize more parameters, modify configuration:
```yaml
param_names: ["kp_x", "kp_y", "kp_z", "ki_x", "ki_y", "ki_z"]
lower_bounds: [1.2, 1.2, 1.4, 0.05, 0.05, 0.05]
upper_bounds: [2.8, 2.8, 2.6, 0.3, 0.3, 0.3]
```

## Safety Reminders

⚠️ **Important Safety Considerations**:

1. **Flight Area**: Ensure minimum 2x2x2 meter safe flight space
2. **Emergency Stop**: Always be ready to press 'q' key or Ctrl+C for emergency stop
3. **Battery Monitoring**: Ensure sufficient battery voltage (>3.7V)
4. **Personnel Safety**: Maintain safe distance during optimization
5. **Backup Plan**: Have manual controller ready as backup

## Support and Feedback

If you encounter problems or need improvement suggestions, please:

1. Check error messages in log files
2. Verify hardware connection status
3. Check if experience database is updating normally
4. Record specific error scenarios and parameter settings

---

🚀 **Start your fast PID optimization journey!** 