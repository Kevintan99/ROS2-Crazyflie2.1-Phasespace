# BAYES_OPT_ROS2 — Bayesian Optimization for ROS 2

A comprehensive Python package that performs Bayesian Optimization over N-dimensional parameter spaces, with specialized support for Crazyflie PID parameter optimization. Uses ROS 2 messaging for communication but runs directly with Python.

## Overview

This package provides a flexible Bayesian optimization framework that can be used for various parameter optimization tasks, with particular emphasis on rapid PID tuning for Crazyflie quadcopters. It publishes parameter suggestions for real experiments, then waits for measured results and continuation flags before proceeding to the next iteration.

## Key Features

### 🚀 Fast PID Optimization System
- **Rapid Optimization**: 15-25 minutes total time (vs 60 minutes standard)
- **Smart Initialization**: Physics-based parameter estimation and experience database
- **Reduced Dimensions**: Optimize only 3 Kp parameters instead of 9 full PID parameters
- **Quick Experiments**: 21-second experiment cycles (vs 35 seconds standard)

### 🔧 General Bayesian Optimization
- **Flexible Parameter Spaces**: Support for any N-dimensional optimization
- **Multiple Acquisition Functions**: UCB, EI, POI with configurable exploration
- **Robust Experiment Control**: Timeout handling and safety mechanisms
- **Real-time Monitoring**: Live status updates and progress tracking

## Package Structure

```
bayes_opt_ros2/
├─ CMakeLists.txt
├─ package.xml
├─ README.md
├─ README_FAST_PID_OPTIMIZATION.md    # Detailed PID optimization guide
├─ launch_fast_pid_optimization.py    # PID optimization launch file
├─ config/
│  ├─ fast_pid_optimization_config.yaml  # Fast PID optimization settings
│  └─ my_opt.yaml                        # Example general optimization config
├─ scripts/
│  ├─ bayesian_opt_node.py               # Core Bayesian optimization node
│  ├─ fast_pid_experiment_client.py      # Fast PID experiment client
│  ├─ flight_logger.py                   # Flight data logging utility
│  └─ phasespace_wrapper.py              # PhaseSpace motion capture integration
├─ cache/                                # Optimization cache files
└─ flight_logs/                          # Flight data logs
```

## Requirements

### Core Requirements
- Python 3.8+
- ROS 2 (rclpy) — e.g., Humble / Iron / Jazzy (for messaging)
- ROS messages: std_msgs
- PyPI packages:
  ```bash
  pip install bayesian-optimization numpy scipy
  ```

### Fast PID Optimization Requirements
- Crazyflie 2.0/2.1 quadcopter
- PhaseSpace motion capture system
- Safe flight area (minimum 2x2x2 meters)

## Installation

### Method 1: Direct Python Usage (Recommended)

Clone or download the package and run directly:

```bash
# Navigate to the package directory
cd bayes_opt_ros2

# Make scripts executable
chmod +x scripts/bayesian_opt_node.py
chmod +x scripts/fast_pid_experiment_client.py
```

### Method 2: ROS 2 Package Installation (Optional)

If you want to install as a ROS 2 package:

From your colcon workspace root (e.g., ~/ros2_ws):
```bash
colcon build --packages-select bayes_opt_ros2
```

Then source the overlay:
```bash
# Linux/macOS (Bash)
source install/setup.bash

# Linux/macOS (Zsh)
source install/setup.zsh

# Windows PowerShell
.\install\setup.ps1
```

## Quick Start

### 1. General Bayesian Optimization

Minimal run (uses defaults):
```bash
cd bayes_opt_ros2
python3 scripts/bayesian_opt_node.py
```

With custom configuration:
```bash
cd bayes_opt_ros2
python3 scripts/bayesian_opt_node.py --ros-args --params-file config/my_opt.yaml
```

### 2. Fast PID Optimization (Recommended)

**Method 1: Launch File (Easiest)**
```bash
cd bayes_opt_ros2
python3 launch_fast_pid_optimization.py
```

**Method 2: Manual Component Startup**

Terminal 1 - Start Bayesian optimization node:
```bash
cd bayes_opt_ros2
python3 scripts/bayesian_opt_node.py --ros-args --params-file config/fast_pid_optimization_config.yaml
```

Terminal 2 - Start fast PID experiment client:
```bash
cd bayes_opt_ros2
python3 scripts/fast_pid_experiment_client.py
```

### 3. Alternative: Direct Python Execution

You can also run the scripts directly without ROS 2 packaging:

```bash
# Set Python path to include the scripts directory
export PYTHONPATH=$PYTHONPATH:$(pwd)/scripts

# Run the optimization node
python3 scripts/bayesian_opt_node.py --ros-args --params-file config/fast_pid_optimization_config.yaml
```

## Configuration

### General Optimization Configuration

Create `config/my_opt.yaml`:
```yaml
bayes_opt_node:
  ros__parameters:
    # Search space definition
    param_names: ["x1", "x2"]
    lower_bounds: [-2.0, -2.0]
    upper_bounds: [10.0, 10.0]

    # Optimization controls
    n_iter: 15
    init_points: 3
    acquisition_type: "ucb"   # ucb | ei | poi
    kappa: 0.1                # for UCB
    xi: 0.01                  # for EI/POI
    random_state: 987234
    objective_mode: "max"     # or "min"
    timeout_sec: 0.0          # 0 disables timeout

    # Topic configuration
    topics.params_out: "/bayes_opt/params_to_test"
    topics.result_in: "/bayes_opt/experiment_result"
    topics.allow_in: "/bayes_opt/allow_step"
    topics.status: "/bayes_opt/status"
    topics.best_params: "/bayes_opt/best_params"
    topics.best_value: "/bayes_opt/best_value"
```

### Fast PID Optimization Configuration

The `config/fast_pid_optimization_config.yaml` is pre-configured for rapid Crazyflie PID tuning:

```yaml
bayes_opt_node:
  ros__parameters:
    # Optimize only 3 Kp parameters (Ki, Kd fixed to optimal values)
    param_names: ["kp_x", "kp_y", "kp_z"]
    
    # Optimized search ranges based on Crazyflie characteristics
    lower_bounds: [1.2, 1.2, 1.4]
    upper_bounds: [2.8, 2.8, 2.6]
    
    # Fast optimization settings
    n_iter: 15          # 15 iterations
    init_points: 5      # 5 initial random points
    
    # EI acquisition function (robust to noise)
    acquisition_type: "ei"
    xi: 0.05           # Faster convergence
    
    # Reduced timeout for faster operation
    timeout_sec: 90.0   # 90 second timeout
```

## Fast PID Optimization Features

### Performance Improvements
| Metric | Standard | Fast PID | Improvement |
|--------|----------|----------|-------------|
| Experiment Time | 35s | 21s | 40% ↓ |
| Total Iterations | 30 | 15 | 50% ↓ |
| Total Time | 60min | 20min | 67% ↓ |
| Parameters | 9D | 3D | 67% ↓ |

### Smart Initialization
- **Physics-based estimation**: Calculates theoretical optimal Kp values
- **Experience database**: Learns from successful parameter combinations
- **Adaptive boundaries**: Dynamically adjusts search ranges
- **Strategy recommendations**: Suggests optimization strategies

### Experiment Workflow
1. **Device Connection** (2s): Connect PhaseSpace and Crazyflie
2. **Parameter Setup** (3s): Configure PID parameters and estimators
3. **Flight Experiment** (15s): Quick takeoff, hover data collection, landing
4. **Result Processing** (1s): Calculate tracking error, update database

**Total per experiment: ~21 seconds**

## Topics

### Published Topics
- `/bayes_opt/params_to_test` (std_msgs/Float64MultiArray)
  - Suggested next parameters; order matches param_names
- `/bayes_opt/status` (std_msgs/String)
  - Progress logs and summary events
- `/bayes_opt/best_params` (std_msgs/Float64MultiArray)
  - Best-so-far parameter vector after completion
- `/bayes_opt/best_value` (std_msgs/Float64)
  - Best-so-far objective value

### Subscribed Topics
- `/bayes_opt/experiment_result` (std_msgs/Float64)
  - Measured objective for the last suggested parameters
- `/bayes_opt/allow_step` (std_msgs/Bool)
  - Set true to allow optimizer to proceed to next suggestion

## Interaction Flow

```
[Optimizer Node] --(Float64MultiArray: params)--> [Your Experiment]
[Your Experiment] --(Float64: result)-----------> [Optimizer Node]
[Your Experiment] --(Bool: allow_step=true)-----> [Optimizer Node]
                                     (then optimizer computes next suggestion)
```

The optimizer blocks per iteration until it receives BOTH the numeric result and allow_step=true.

## Quick Test (No Real Experiment)

Start the optimizer:
```bash
cd bayes_opt_ros2
python3 scripts/bayesian_opt_node.py --ros-args --params-file config/my_opt.yaml
```

Watch suggestions:
```bash
ros2 topic echo /bayes_opt/params_to_test
```

When a vector arrives, publish dummy result and allow:
```bash
ros2 topic pub /bayes_opt/experiment_result std_msgs/Float64 "{data: 1.23}" --once
ros2 topic pub /bayes_opt/allow_step std_msgs/Bool "{data: true}" --once
```

## Component Details

### Core Components
- **bayesian_opt_node.py**: Main Bayesian optimization engine
- **fast_pid_experiment_client.py**: Specialized client for Crazyflie PID experiments
- **flight_logger.py**: Comprehensive flight data logging utility
- **phasespace_wrapper.py**: PhaseSpace motion capture system integration

### Configuration Files
- **fast_pid_optimization_config.yaml**: Optimized settings for rapid PID tuning
- **my_opt.yaml**: Example configuration for general optimization tasks
- **launch_fast_pid_optimization.py**: Automated launch script for PID optimization

### Data Directories
- **cache/**: Stores optimization progress and experience database
- **flight_logs/**: Contains detailed flight data and experiment logs

## Safety Guidelines

⚠️ **Important Safety Considerations**:

1. **Flight Area**: Ensure minimum 2x2x2 meter safe flight space
2. **Emergency Stop**: Always be ready to press 'q' or Ctrl+C for emergency stop
3. **Battery Monitoring**: Ensure sufficient battery voltage (>3.7V)
4. **Personnel Safety**: Maintain safe distance during optimization
5. **Backup Plan**: Have manual controller ready as backup

## Troubleshooting

### Common Issues

1. **No next suggestion appears**:
   - Ensure both `/bayes_opt/experiment_result` and `/bayes_opt/allow_step` were published
   - Check timeout settings

2. **Dimension mismatch**:
   - Verify param_names, lower_bounds, and upper_bounds have same length

3. **Python import errors**:
   ```bash
   # Set Python path to include scripts directory
   export PYTHONPATH=$PYTHONPATH:$(pwd)/scripts
   ```

4. **Crazyflie connection issues**:
   ```bash
   export CRAZYFLIE_URI=radio://0/81/2M/E7E7E7E7E7
   sudo chmod 666 /dev/ttyUSB0
   ```

5. **PhaseSpace data problems**:
   - Check PhaseSpace system status
   - Verify rigid body ID configuration
   - Validate coordinate transformation settings

### Debug Mode

Enable detailed logging:
```bash
export ROS_LOG_LEVEL=DEBUG
cd bayes_opt_ros2
python3 scripts/fast_pid_experiment_client.py
```

## Advanced Usage

### Custom Experience Database

For PID optimization, edit experience database:
```json
{
  "successful_params": [
    {"kp": [2.0, 2.0, 2.2], "error": 0.08, "conditions": "indoor_no_wind"}
  ],
  "best_known": {"kp": [2.0, 2.0, 2.2], "error": 0.06}
}
```

### Parameter Space Extension

To optimize more parameters, modify configuration:
```yaml
param_names: ["kp_x", "kp_y", "kp_z", "ki_x", "ki_y", "ki_z"]
lower_bounds: [1.2, 1.2, 1.4, 0.05, 0.05, 0.05]
upper_bounds: [2.8, 2.8, 2.6, 0.3, 0.3, 0.3]
```

### Acquisition Function Tuning

Adjust exploration parameters:
```yaml
# More aggressive exploration
acquisition_type: "ucb"
kappa: 0.2

# More conservative convergence
acquisition_type: "ei"
xi: 0.01
```

## Performance Optimization

### For Fast PID Optimization
- Use EI acquisition function for noise robustness
- Limit to 3 Kp parameters only
- Set appropriate search ranges (1.2-2.8)
- Use 15 iterations with 5 initial points

### For General Optimization
- Choose acquisition function based on noise level
- Adjust exploration parameters (kappa/xi) for desired balance
- Set appropriate timeout values
- Consider objective mode (max/min)

## Development Notes

### Running Without ROS 2 Packaging

The scripts can be run directly without building as a ROS 2 package:

```bash
# Set up environment
export PYTHONPATH=$PYTHONPATH:$(pwd)/scripts

# Run optimization node
python3 scripts/bayesian_opt_node.py --ros-args --params-file config/fast_pid_optimization_config.yaml
```

### ROS 2 Dependencies

While the code runs with Python directly, it still requires:
- ROS 2 installation (for rclpy and message types)
- ROS 2 environment sourced (for message definitions)

## License

MIT License (or adapt to your project's license).

## Acknowledgments

- Uses the excellent 'bayesian-optimization' Python package for BO internals
- Built for ROS 2 ecosystem compatibility
- Specialized for Crazyflie quadcopter optimization

---

🚀 **Start optimizing your parameters efficiently with Bayesian Optimization!**
