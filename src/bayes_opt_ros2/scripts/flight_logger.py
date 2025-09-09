#!/usr/bin/env python3
"""
Flight logger
Record detailed data for each PID test: parameters, trajectory, error, etc.
Generate visualizations and detailed reports
"""

import json
import time
import os
from datetime import datetime
from typing import List, Dict, Any, Optional
import numpy as np
import matplotlib.pyplot as plt

class NumpyEncoder(json.JSONEncoder):
    """Handle NumPy type JSON encoder"""
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, np.bool_):
            return bool(obj)
        return super(NumpyEncoder, self).default(obj)

class FlightLogger:
    def __init__(self, log_dir: str = "flight_logs"):
        """Initialize flight logger"""
        self.log_dir = log_dir
        self.current_session_id = None
        self.current_experiment_data = {}
        
        # Create log directory
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Set matplotlib Chinese font
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial']
        plt.rcParams['axes.unicode_minus'] = False
        
        print(f"📊 Flight logger initialized, log directory: {self.log_dir}")
    
    def start_new_session(self, optimization_config: Dict[str, Any]) -> str:
        """Start new optimization session"""
        timestamp = datetime.now()
        self.current_session_id = timestamp.strftime("%Y%m%d_%H%M%S")
        
        # Create session directory
        session_dir = os.path.join(self.log_dir, f"session_{self.current_session_id}")
        os.makedirs(session_dir, exist_ok=True)
        
        # Save optimization configuration
        config_file = os.path.join(session_dir, "optimization_config.json")
        with open(config_file, 'w', encoding='utf-8') as f:
            json.dump({
                'session_id': self.current_session_id,
                'start_time': timestamp.isoformat(),
                'optimization_config': optimization_config
            }, f, indent=2, ensure_ascii=False, cls=NumpyEncoder)
        
        print(f"🆕 Start new optimization session: {self.current_session_id}")
        return self.current_session_id
    
    def start_experiment(self, iteration: int, pid_params: List[float]) -> Dict[str, Any]:
        """Start new experiment record"""
        experiment_id = f"exp_{iteration:03d}"
        
        self.current_experiment_data = {
            'experiment_id': experiment_id,
            'iteration': iteration,
            'start_time': datetime.now().isoformat(),
            'pid_parameters': {
                'kp_x': pid_params[0],
                'kp_y': pid_params[1],
                'kp_z': pid_params[2],
                'ki_x': 0.1,  # Fixed value
                'ki_y': 0.1,
                'ki_z': 0.1,
                'kd_x': 0.35, # Fixed value
                'kd_y': 0.35,
                'kd_z': 0.45
            },
            'flight_data': {
                'crazyflie_positions': [],
                'phasespace_positions': [],
                'position_errors': [],
                'timestamps': []
            },
            'experiment_config': {
                'hover_height': 0.3,
                'target_position': [0.0, 0.0, 0.3],
                'experiment_duration': 15.0,
                'takeoff_duration': 2.0,
                'landing_duration': 2.0
            }
        }
        
        print(f"🧪 Start experiment {experiment_id}: Kp=[{pid_params[0]:.3f}, {pid_params[1]:.3f}, {pid_params[2]:.3f}]")
        return self.current_experiment_data
    
    def log_position_data(self, cf_pos: List[float], ps_pos: Optional[List[float]] = None, 
                         error: float = None, timestamp: float = None):
        """Record position data"""
        if not self.current_experiment_data:
            return
        
        if timestamp is None:
            timestamp = time.time()
        
        flight_data = self.current_experiment_data['flight_data']
        flight_data['crazyflie_positions'].append(cf_pos.copy())
        flight_data['timestamps'].append(timestamp)
        
        if ps_pos is not None:
            flight_data['phasespace_positions'].append(ps_pos.copy())
        
        if error is not None:
            flight_data['position_errors'].append(error)
    
    def finish_experiment(self, final_error: float, success: bool = True, 
                         notes: str = "") -> str:
        """Finish experiment record and generate report"""
        if not self.current_experiment_data:
            print("⚠️  No ongoing experiment")
            return ""
        
        # Finish experiment data
        self.current_experiment_data.update({
            'end_time': datetime.now().isoformat(),
            'final_error': final_error,
            'success': success,
            'notes': notes
        })
        
        # Calculate statistics
        self._calculate_statistics()
        
        # Save experiment data
        experiment_file = self._save_experiment_data()
        
        # Generate visualizations
        self._generate_plots()
        
        # Generate report
        report_file = self._generate_report()
        
        experiment_id = self.current_experiment_data['experiment_id']
        print(f"✅ Experiment {experiment_id} completed, error: {final_error:.4f}m")
        print(f"📄 Report saved: {report_file}")
        
        # Clear current experiment data
        current_data = self.current_experiment_data.copy()
        self.current_experiment_data = {}
        
        return experiment_file
    
    def _calculate_statistics(self):
        """Calculate flight statistics"""
        flight_data = self.current_experiment_data['flight_data']
        errors = flight_data['position_errors']
        
        if len(errors) == 0:
            return
        
        # Basic statistics
        errors_array = np.array(errors)
        statistics = {
            'total_samples': len(errors),
            'mean_error': float(np.mean(errors_array)),
            'rmse': float(np.sqrt(np.mean(errors_array**2))),
            'max_error': float(np.max(errors_array)),
            'min_error': float(np.min(errors_array)),
            'std_error': float(np.std(errors_array)),
            'median_error': float(np.median(errors_array))
        }
        
        # Steady state analysis (last 70% data)
        steady_start = int(len(errors) * 0.3)
        if steady_start < len(errors):
            steady_errors = errors_array[steady_start:]
            statistics.update({
                'steady_state_mean': float(np.mean(steady_errors)),
                'steady_state_rmse': float(np.sqrt(np.mean(steady_errors**2))),
                'steady_state_std': float(np.std(steady_errors))
            })
        
        # Trajectory analysis
        cf_positions = np.array(flight_data['crazyflie_positions'])
        if len(cf_positions) > 0:
            target_pos = np.array(self.current_experiment_data['experiment_config']['target_position'])
            
            # Calculate axis errors
            axis_errors = cf_positions - target_pos
            statistics.update({
                'x_axis_rmse': float(np.sqrt(np.mean(axis_errors[:, 0]**2))),
                'y_axis_rmse': float(np.sqrt(np.mean(axis_errors[:, 1]**2))),
                'z_axis_rmse': float(np.sqrt(np.mean(axis_errors[:, 2]**2))),
                'max_displacement': float(np.max(np.linalg.norm(axis_errors, axis=1)))
            })
        
        self.current_experiment_data['statistics'] = statistics
    
    def _save_experiment_data(self) -> str:
        """Save experiment data to JSON file"""
        if not self.current_session_id:
            return ""
        
        session_dir = os.path.join(self.log_dir, f"session_{self.current_session_id}")
        experiment_id = self.current_experiment_data['experiment_id']
        
        # Save detailed data
        data_file = os.path.join(session_dir, f"{experiment_id}_data.json")
        with open(data_file, 'w', encoding='utf-8') as f:
            json.dump(self.current_experiment_data, f, indent=2, ensure_ascii=False, cls=NumpyEncoder)
        
        # Save simplified summary to session summary file
        summary_file = os.path.join(session_dir, "experiments_summary.json")
        summary_data = []
        
        # Read existing summary
        if os.path.exists(summary_file):
            try:
                with open(summary_file, 'r', encoding='utf-8') as f:
                    summary_data = json.load(f)
            except:
                summary_data = []
        
        # Add current experiment summary
        experiment_summary = {
            'experiment_id': experiment_id,
            'iteration': self.current_experiment_data['iteration'],
            'start_time': self.current_experiment_data['start_time'],
            'end_time': self.current_experiment_data['end_time'],
            'pid_parameters': self.current_experiment_data['pid_parameters'],
            'final_error': self.current_experiment_data['final_error'],
            'success': self.current_experiment_data['success'],
            'statistics': self.current_experiment_data.get('statistics', {})
        }
        
        summary_data.append(experiment_summary)
        
        # Save summary
        with open(summary_file, 'w', encoding='utf-8') as f:
            json.dump(summary_data, f, indent=2, ensure_ascii=False, cls=NumpyEncoder)
        
        return data_file
    
    def _generate_plots(self):
        """Generate visualizations"""
        if not self.current_session_id or not self.current_experiment_data:
            return
        
        session_dir = os.path.join(self.log_dir, f"session_{self.current_session_id}")
        experiment_id = self.current_experiment_data['experiment_id']
        
        flight_data = self.current_experiment_data['flight_data']
        cf_positions = np.array(flight_data['crazyflie_positions'])
        errors = np.array(flight_data['position_errors'])
        timestamps = np.array(flight_data['timestamps'])
        
        if len(cf_positions) == 0:
            return
        
        # Convert timestamps to relative time
        start_time = timestamps[0]
        relative_times = timestamps - start_time
        
        # Create plots
        fig = plt.figure(figsize=(16, 12))
        
        # 1. 3D trajectory plot
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        target_pos = self.current_experiment_data['experiment_config']['target_position']
        
        ax1.plot(cf_positions[:, 0], cf_positions[:, 1], cf_positions[:, 2], 
                'b-', linewidth=2, label='Crazyflie Trajectory')
        ax1.scatter(*target_pos, color='red', s=100, label='Target Position')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title(f'3D Flight Trajectory - {experiment_id}')
        ax1.legend()
        ax1.grid(True)
        
        # 2. Position error time series
        ax2 = fig.add_subplot(2, 3, 2)
        if len(errors) > 0:
            ax2.plot(relative_times[:len(errors)], errors, 'r-', linewidth=2)
            ax2.axhline(y=np.mean(errors), color='g', linestyle='--', 
                       label=f'Mean Error: {np.mean(errors):.4f}m')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Position Error (m)')
            ax2.set_title('Position Tracking Error')
            ax2.grid(True)
            ax2.legend()
        
        # 3. XYZ position time series
        ax3 = fig.add_subplot(2, 3, 3)
        ax3.plot(relative_times, cf_positions[:, 0], 'r-', label='X Position')
        ax3.plot(relative_times, cf_positions[:, 1], 'g-', label='Y Position')
        ax3.plot(relative_times, cf_positions[:, 2], 'b-', label='Z Position')
        ax3.axhline(y=target_pos[0], color='r', linestyle='--', alpha=0.5)
        ax3.axhline(y=target_pos[1], color='g', linestyle='--', alpha=0.5)
        ax3.axhline(y=target_pos[2], color='b', linestyle='--', alpha=0.5)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Position (m)')
        ax3.set_title('Position vs Time')
        ax3.legend()
        ax3.grid(True)
        
        # 4. XY plane trajectory
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.plot(cf_positions[:, 0], cf_positions[:, 1], 'b-', linewidth=2)
        ax4.scatter(target_pos[0], target_pos[1], color='red', s=100, marker='x')
        ax4.set_xlabel('X (m)')
        ax4.set_ylabel('Y (m)')
        ax4.set_title('XY Plane Trajectory')
        ax4.grid(True)
        ax4.axis('equal')
        
        # 5. Error distribution histogram
        ax5 = fig.add_subplot(2, 3, 5)
        if len(errors) > 0:
            ax5.hist(errors, bins=20, alpha=0.7, edgecolor='black')
            ax5.axvline(x=np.mean(errors), color='r', linestyle='--', 
                       label=f'Mean: {np.mean(errors):.4f}m')
            ax5.axvline(x=np.median(errors), color='g', linestyle='--', 
                       label=f'Median: {np.median(errors):.4f}m')
            ax5.set_xlabel('Position Error (m)')
            ax5.set_ylabel('Frequency')
            ax5.set_title('Error Distribution')
            ax5.legend()
            ax5.grid(True)
        
        # 6. PID parameters display
        ax6 = fig.add_subplot(2, 3, 6)
        ax6.axis('off')
        
        pid_params = self.current_experiment_data['pid_parameters']
        stats = self.current_experiment_data.get('statistics', {})
        
        info_text = f"""PID Parameters:
Kp: [{pid_params['kp_x']:.3f}, {pid_params['kp_y']:.3f}, {pid_params['kp_z']:.3f}]
Ki: [{pid_params['ki_x']:.3f}, {pid_params['ki_y']:.3f}, {pid_params['ki_z']:.3f}]
Kd: [{pid_params['kd_x']:.3f}, {pid_params['kd_y']:.3f}, {pid_params['kd_z']:.3f}]

Experiment Results:
Final Error: {self.current_experiment_data['final_error']:.4f}m
Mean Error: {stats.get('mean_error', 0):.4f}m
RMSE: {stats.get('rmse', 0):.4f}m
Max Error: {stats.get('max_error', 0):.4f}m
Std Dev: {stats.get('std_error', 0):.4f}m

Experiment Config:
Hover Height: {self.current_experiment_data['experiment_config']['hover_height']}m
Duration: {self.current_experiment_data['experiment_config']['experiment_duration']}s
Data Points: {stats.get('total_samples', 0)}
"""
        
        ax6.text(0.1, 0.9, info_text, transform=ax6.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
        
        plt.tight_layout()
        
        # Save plots
        plot_file = os.path.join(session_dir, f"{experiment_id}_analysis.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"📊 Analysis plot saved: {plot_file}")
    
    def _generate_report(self) -> str:
        """Generate experiment report"""
        if not self.current_session_id or not self.current_experiment_data:
            return ""
        
        session_dir = os.path.join(self.log_dir, f"session_{self.current_session_id}")
        experiment_id = self.current_experiment_data['experiment_id']
        
        report_file = os.path.join(session_dir, f"{experiment_id}_report.md")
        
        pid_params = self.current_experiment_data['pid_parameters']
        stats = self.current_experiment_data.get('statistics', {})
        config = self.current_experiment_data['experiment_config']
        
        report_content = f"""# Flight experiment report - {experiment_id}

## Basic information
- **Experiment ID**: {experiment_id}
- **Iteration**: {self.current_experiment_data['iteration']}
- **Start time**: {self.current_experiment_data['start_time']}
- **End time**: {self.current_experiment_data['end_time']}
- **Experiment status**: {'Success' if self.current_experiment_data['success'] else 'Failed'}

## PID control parameters
| Axis | Kp | Ki | Kd |
|------|----|----|----| 
| X axis  | {pid_params['kp_x']:.3f} | {pid_params['ki_x']:.3f} | {pid_params['kd_x']:.3f} |
| Y axis  | {pid_params['kp_y']:.3f} | {pid_params['ki_y']:.3f} | {pid_params['kd_y']:.3f} |
| Z axis  | {pid_params['kp_z']:.3f} | {pid_params['ki_z']:.3f} | {pid_params['kd_z']:.3f} |

## Experiment configuration
- **Target position**: [{config['target_position'][0]:.1f}, {config['target_position'][1]:.1f}, {config['target_position'][2]:.1f}] m
- **Hover height**: {config['hover_height']} m  
- **Experiment duration**: {config['experiment_duration']} s
- **Takeoff duration**: {config['takeoff_duration']} s
- **Landing duration**: {config['landing_duration']} s

## Performance statistics

### Overall performance
- **Final error**: {self.current_experiment_data['final_error']:.4f} m
- **Mean error**: {stats.get('mean_error', 0):.4f} m
- **Root mean square error (RMSE)**: {stats.get('rmse', 0):.4f} m
- **Maximum error**: {stats.get('max_error', 0):.4f} m
- **Minimum error**: {stats.get('min_error', 0):.4f} m
- **Standard deviation of error**: {stats.get('std_error', 0):.4f} m
- **Median error**: {stats.get('median_error', 0):.4f} m

### Steady state performance (last 70% data)
- **Steady state mean error**: {stats.get('steady_state_mean', 0):.4f} m
- **Steady state RMSE**: {stats.get('steady_state_rmse', 0):.4f} m
- **Steady state standard deviation**: {stats.get('steady_state_std', 0):.4f} m

### Performance of each axis
- **X axis RMSE**: {stats.get('x_axis_rmse', 0):.4f} m
- **Y axis RMSE**: {stats.get('y_axis_rmse', 0):.4f} m  
- **Z axis RMSE**: {stats.get('z_axis_rmse', 0):.4f} m
- **Maximum displacement**: {stats.get('max_displacement', 0):.4f} m

## Data statistics
- **Total sampling points**: {stats.get('total_samples', 0)}
- **Sampling frequency**: ~{stats.get('total_samples', 0) / config['experiment_duration']:.1f} Hz

## Performance rating
"""
        
        # Add performance rating
        final_error = self.current_experiment_data['final_error']
        if final_error < 0.05:
            rating = "🌟🌟🌟🌟🌟 Excellent"
        elif final_error < 0.1:
            rating = "🌟🌟🌟🌟 Good"
        elif final_error < 0.15:
            rating = "🌟🌟🌟 Average"
        elif final_error < 0.2:
            rating = "🌟🌟 Poor"
        else:
            rating = "🌟 Need improvement"
        
        report_content += f"**Control accuracy**: {rating}\n\n"
        
        # Add notes
        if self.current_experiment_data.get('notes'):
            report_content += f"## Notes\n{self.current_experiment_data['notes']}\n\n"
        
        report_content += f"""## File references
- **Detailed data**: {experiment_id}_data.json
- **Analysis plots**: {experiment_id}_analysis.png
- **This report**: {experiment_id}_report.md

---
*Report generation time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}*
"""
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(report_content)
        
        return report_file
    
    def generate_session_summary(self) -> str:
        """Generate summary report for the entire optimization session"""
        if not self.current_session_id:
            return ""
        
        session_dir = os.path.join(self.log_dir, f"session_{self.current_session_id}")
        summary_file = os.path.join(session_dir, "experiments_summary.json")
        
        if not os.path.exists(summary_file):
            return ""
        
        # Read all experiment data
        with open(summary_file, 'r', encoding='utf-8') as f:
            experiments = json.load(f)
        
        if len(experiments) == 0:
            return ""
        
        # Generate optimization progress plot
        self._generate_optimization_progress_plot(experiments)
        
        # Generate session summary report
        return self._generate_session_report(experiments)
    
    def _generate_optimization_progress_plot(self, experiments: List[Dict]):
        """Generate optimization progress plot"""
        session_dir = os.path.join(self.log_dir, f"session_{self.current_session_id}")
        
        # Extract data
        iterations = [exp['iteration'] for exp in experiments]
        errors = [exp['final_error'] for exp in experiments]
        kp_x = [exp['pid_parameters']['kp_x'] for exp in experiments]
        kp_y = [exp['pid_parameters']['kp_y'] for exp in experiments]
        kp_z = [exp['pid_parameters']['kp_z'] for exp in experiments]
        
        # Create plots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # 1. Optimization progress
        ax1.plot(iterations, errors, 'bo-', linewidth=2, markersize=6)
        ax1.set_xlabel('Iteration')
        ax1.set_ylabel('Tracking Error (m)')
        ax1.set_title('Bayesian Optimization Progress')
        ax1.grid(True)
        
        # Mark best point
        best_idx = np.argmin(errors)
        ax1.plot(iterations[best_idx], errors[best_idx], 'ro', markersize=10, 
                label=f'Best: {errors[best_idx]:.4f}m')
        ax1.legend()
        
        # 2. Kp parameter change
        ax2.plot(iterations, kp_x, 'r.-', label='Kp_x', linewidth=2)
        ax2.plot(iterations, kp_y, 'g.-', label='Kp_y', linewidth=2)
        ax2.plot(iterations, kp_z, 'b.-', label='Kp_z', linewidth=2)
        ax2.set_xlabel('Iteration')
        ax2.set_ylabel('Kp Value')
        ax2.set_title('Kp Parameter Evolution')
        ax2.legend()
        ax2.grid(True)
        
        # 3. Parameter space exploration (Kp_x vs Kp_y)
        scatter = ax3.scatter(kp_x, kp_y, c=errors, cmap='viridis_r', s=60, alpha=0.7)
        ax3.set_xlabel('Kp_x')
        ax3.set_ylabel('Kp_y')
        ax3.set_title('Parameter Space Exploration (Kp_x vs Kp_y)')
        ax3.grid(True)
        plt.colorbar(scatter, ax=ax3, label='Tracking Error (m)')
        
        # Mark best point
        ax3.plot(kp_x[best_idx], kp_y[best_idx], 'r*', markersize=15, 
                label='Best Parameters')
        ax3.legend()
        
        # 4. Error distribution
        ax4.hist(errors, bins=min(10, len(errors)), alpha=0.7, edgecolor='black')
        ax4.axvline(x=np.mean(errors), color='r', linestyle='--', 
                   label=f'Mean: {np.mean(errors):.4f}m')
        ax4.axvline(x=np.min(errors), color='g', linestyle='--', 
                   label=f'Best: {np.min(errors):.4f}m')
        ax4.set_xlabel('Tracking Error (m)')
        ax4.set_ylabel('Frequency')
        ax4.set_title('Error Distribution')
        ax4.legend()
        ax4.grid(True)
        
        plt.tight_layout()
        
        # Save plots
        plot_file = os.path.join(session_dir, f"optimization_progress_{self.current_session_id}.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"📈 Optimization progress plot saved: {plot_file}")
    
    def _generate_session_report(self, experiments: List[Dict]) -> str:
        """Generate session summary report"""
        session_dir = os.path.join(self.log_dir, f"session_{self.current_session_id}")
        
        # Find best experiment
        best_exp = min(experiments, key=lambda x: x['final_error'])
        errors = [exp['final_error'] for exp in experiments]
        
        report_file = os.path.join(session_dir, f"session_summary_{self.current_session_id}.md")
        
        report_content = f"""# Bayesian optimization session summary report

## Session information
- **Session ID**: {self.current_session_id}
- **Start time**: {experiments[0]['start_time']}
- **End time**: {experiments[-1]['end_time']}
- **Total experiments**: {len(experiments)}

## Optimization results

### Best parameters
- **Experiment ID**: {best_exp['experiment_id']}
- **Iteration**: {best_exp['iteration']}
- **Best error**: {best_exp['final_error']:.4f} m

| Axis | Kp | Ki | Kd |
|------|----|----|----| 
| X axis | {best_exp['pid_parameters']['kp_x']:.3f} | {best_exp['pid_parameters']['ki_x']:.3f} | {best_exp['pid_parameters']['kd_x']:.3f} |
| Y axis | {best_exp['pid_parameters']['kp_y']:.3f} | {best_exp['pid_parameters']['ki_y']:.3f} | {best_exp['pid_parameters']['kd_y']:.3f} |
| Z axis | {best_exp['pid_parameters']['kp_z']:.3f} | {best_exp['pid_parameters']['ki_z']:.3f} | {best_exp['pid_parameters']['kd_z']:.3f} |

### Optimization statistics
- **Best error**: {np.min(errors):.4f} m
- **Mean error**: {np.mean(errors):.4f} m
- **Worst error**: {np.max(errors):.4f} m
- **Error improvement**: {(np.max(errors) - np.min(errors)):.4f} m ({((np.max(errors) - np.min(errors))/np.max(errors)*100):.1f}%)

## Experiment list

| Iteration | Experiment ID | Kp_x | Kp_y | Kp_z | Error (m) | Status |
|------|--------|------|------|------|---------|------|
"""
        
        for exp in experiments:
            status = "✅" if exp['success'] else "❌"
            report_content += f"| {exp['iteration']} | {exp['experiment_id']} | {exp['pid_parameters']['kp_x']:.3f} | {exp['pid_parameters']['kp_y']:.3f} | {exp['pid_parameters']['kp_z']:.3f} | {exp['final_error']:.4f} | {status} |\n"
        
        report_content += f"""
## Recommended settings

Based on the optimization results, the recommended PID parameter settings are:

```
posCtlPid.xKp = {best_exp['pid_parameters']['kp_x']:.3f}
posCtlPid.yKp = {best_exp['pid_parameters']['kp_y']:.3f}  
posCtlPid.zKp = {best_exp['pid_parameters']['kp_z']:.3f}
posCtlPid.xKi = {best_exp['pid_parameters']['ki_x']:.3f}
posCtlPid.yKi = {best_exp['pid_parameters']['ki_y']:.3f}
posCtlPid.zKi = {best_exp['pid_parameters']['ki_z']:.3f}
posCtlPid.xKd = {best_exp['pid_parameters']['kd_x']:.3f}
posCtlPid.yKd = {best_exp['pid_parameters']['kd_y']:.3f}
posCtlPid.zKd = {best_exp['pid_parameters']['kd_z']:.3f}
```

## File references
- **Optimization progress plot**: optimization_progress_{self.current_session_id}.png
- **Experiments summary**: experiments_summary.json
- **Experiment detailed report**: exp_XXX_report.md

---
*Report generation time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}*
"""
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(report_content)
        
        print(f"📋 Session summary report saved: {report_file}")
        return report_file


# Example and test code
if __name__ == "__main__":
    # Create logger
    logger = FlightLogger()
    
    # Simulate optimization session
    config = {
        'param_names': ['kp_x', 'kp_y', 'kp_z'],
        'n_iter': 5,
        'acquisition_type': 'ei'
    }
    
    session_id = logger.start_new_session(config)
    
    # Simulate several experiments
    for i in range(3):
        # Simulate PID parameters
        kp_params = [2.0 + np.random.normal(0, 0.2), 
                     2.0 + np.random.normal(0, 0.2),
                     2.2 + np.random.normal(0, 0.2)]
        
        # Start experiment
        logger.start_experiment(i+1, kp_params)
        
        # Simulate flight data
        for j in range(100):
            # Simulate position data
            cf_pos = [np.random.normal(0, 0.1), 
                     np.random.normal(0, 0.1), 
                     0.3 + np.random.normal(0, 0.05)]
            
            error = np.linalg.norm(np.array(cf_pos) - np.array([0, 0, 0.3]))
            logger.log_position_data(cf_pos, error=error)
        
        # Finish experiment
        final_error = np.random.uniform(0.05, 0.2)
        logger.finish_experiment(final_error, success=True, 
                               notes=f"Simulated experiment {i+1}")
    
    # Generate session summary
    logger.generate_session_summary()
    
    print("✅ Flight logger test completed!") 