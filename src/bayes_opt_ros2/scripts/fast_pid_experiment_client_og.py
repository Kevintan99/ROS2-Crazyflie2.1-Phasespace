#!/usr/bin/env python3
"""
Fast PID controller optimization experiment client
Optimize experiment process, reduce experiment time, improve optimization efficiency

Main improvements:
1. Shorten experiment time (15 seconds vs 20 seconds)
2. Faster takeoff/landing speed
3. Pre-set experience values as initial guesses
4. Simplify data collection logic
5. More aggressive parameter search strategy
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Bool, String
import numpy as np
import time
import threading
from collections import deque
import sys
import os

# Add crazyflie_python path
sys.path.append('/home/fann/crazyswarm_phasespace/src/crazyflie_python')

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
from phasespace_wrapper import PhaseSpaceWrapper

import signal
import atexit

# Add flight logger
from flight_logger import FlightLogger


class FastPIDOptimizationClient(Node):
    def __init__(self):
        super().__init__('fast_pid_optimization_client')
        
        # Initialize cflib driver
        try:
            cflib.crtp.init_drivers()
            self.get_logger().info("✅ Crazyflie driver initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Crazyflie driver initialization failed: {e}")
        
        # Initialize flight logger
        self.flight_logger = FlightLogger()
        self.session_id = None
        
        # Clean up flag
        self._shutdown_requested = False
        self._cleanup_done = False
        
        # Register cleanup function
        atexit.register(self.cleanup_resources)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Subscribe to fast PID parameter suggestions
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/fast_pid_opt/params_to_test',
            self.pid_params_callback,
            10
        )
        
        # Publish experiment results
        self.result_publisher = self.create_publisher(
            Float64,
            '/fast_pid_opt/experiment_result',
            10
        )
        
        # Publish allow signal
        self.allow_publisher = self.create_publisher(
            Bool,
            '/fast_pid_opt/allow_step',
            10
        )
        
        # Subscribe to status information
        self.status_sub = self.create_subscription(
            String, '/fast_pid_opt/status', self.status_callback, 10
        )
        
        self.best_value_sub = self.create_subscription(
            Float64, '/fast_pid_opt/best_value', self.best_value_callback, 10
        )
        
        # Experiment state
        self.iteration = 0
        self.current_experiment_running = False
        
        # Data collection
        self.position_errors = deque(maxlen=500)  # reduce buffer size
        self.data_lock = threading.Lock()
        
        # Crazyflie configuration
        self.uri = uri_helper.uri_from_env(default='radio://0/81/2M/E7E7E7E7E7')
        self.phasespace_rigid_body_id = 6
        self.ENABLE_COORDINATE_TRANSFORM = True
        self.send_full_pose = False
        
        # Fast experiment parameters
        self.experiment_duration = 15.0  # reduce to 15 seconds
        self.hover_height = 0.3
        self.target_position = [0.0, 0.0, self.hover_height]
        
        # Fast takeoff/landing parameters
        self.takeoff_duration = 2.0  # faster takeoff (2 seconds vs 3 seconds)
        self.landing_duration = 2.0  # faster landing (2 seconds vs 3 seconds)
        
        # Emergency stop flag
        self.emergency_stop_flag = False
        self.emergency_stop_lock = threading.Lock()
        
        # PhaseSpace wrapper - keep running during entire optimization
        self.phasespace_wrapper = None
        self.phasespace_initialized = False
        
        # Experience database (for intelligent initialization)
        self.experience_db = {
            'good_params': [
                [2.0, 2.0, 2.2],  # experience value 1
                [1.8, 1.8, 2.0],  # experience value 2
                [2.2, 2.2, 2.4],  # experience value 3
            ],
            'best_error': float('inf'),
            'best_params': None
        }
        
        # Start new optimization session
        optimization_config = {
            'param_names': ['kp_x', 'kp_y', 'kp_z'],
            'experiment_duration': self.experiment_duration,
            'hover_height': self.hover_height,
            'acquisition_type': 'ei',
            'node_type': 'fast_pid_optimization'
        }
        self.session_id = self.flight_logger.start_new_session(optimization_config)
        
        # Initialize PhaseSpace connection (only initialize once at startup)
        self._initialize_phasespace()
        
        self.get_logger().info("🚀 Fast PID optimization experiment client started")
        self.get_logger().info("⚡ Optimization settings: 15 seconds experiment, fast takeoff/landing, intelligent initialization")
        self.get_logger().info(f"📊 Flight logger session: {self.session_id}")
        self.get_logger().info("⏳ Waiting for Bayesian optimization node to send PID parameter suggestions...")
        
        # Start keyboard listener thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
    
    def _initialize_phasespace(self):
        """Initialize PhaseSpace connection (only initialize once at startup)"""
        try:
            print("🔗 Initializing PhaseSpace wrapper for entire optimization session...")
            
            # Import PhaseSpace wrapper
            from phasespace_wrapper import PhaseSpaceWrapper
            
            self.phasespace_wrapper = PhaseSpaceWrapper(
                self.phasespace_rigid_body_id,
                enable_coordinate_transform=self.ENABLE_COORDINATE_TRANSFORM,
                send_full_pose=self.send_full_pose
            )
            
            # Wait for initialization to complete
            time.sleep(1.0)
            
            self.phasespace_initialized = True
            print("✅ PhaseSpace wrapper initialized for session")
            
        except Exception as e:
            print(f"❌ Failed to initialize PhaseSpace wrapper: {e}")
            self.phasespace_initialized = False
    
    def signal_handler(self, signum, frame):
        """Signal handler - graceful exit"""
        if not self._shutdown_requested:
            self._shutdown_requested = True
            print(f"\n🛑 Received exit signal ({signum}), shutting down gracefully...")
            self.emergency_stop()
            self.cleanup_resources()
    
    def cleanup_resources(self):
        """Clean up all resources"""
        if self._cleanup_done:
            return
        
        try:
            print("🧹 Cleaning up resources...")
            
            # Now stop PhaseSpace wrapper (only when optimization is complete)
            if self.phasespace_wrapper is not None and self.phasespace_initialized:
                try:
                    print("🛑 Stopping PhaseSpace wrapper (optimization complete)...")
                    self.phasespace_wrapper.on_pose = None
                    time.sleep(0.2)
                    self.phasespace_wrapper.stop()
                    time.sleep(1.0)  # Give enough time to stop
                    self.phasespace_wrapper = None
                    self.phasespace_initialized = False
                    print("✅ PhaseSpace wrapper stopped")
                except Exception as e:
                    print(f"⚠️ Error stopping PhaseSpace wrapper: {e}")
            
            # Generate session summary report
            if self.flight_logger and self.session_id:
                try:
                    print("📋 Generating optimization session summary report...")
                    self.flight_logger.generate_session_summary()
                except Exception as e:
                    print(f"⚠️ Error generating summary report: {e}")
            
            # Force garbage collection
            import gc
            gc.collect()
            
            # 设置清理完成标志
            self._cleanup_done = True
            print("✅ Resources cleaned up")
            
        except Exception as e:
            print(f"⚠️ Error cleaning up resources: {e}")
    
    def emergency_stop(self):
        """Emergency stop function"""
        with self.emergency_stop_lock:
            self.emergency_stop_flag = True
        print("\n🚨 Emergency stop activated!")
    
    def check_emergency_stop(self):
        """Check emergency stop status"""
        with self.emergency_stop_lock:
            return self.emergency_stop_flag or self._shutdown_requested
    
    def keyboard_listener(self):
        """Keyboard listener thread"""
        print("🎮 Emergency stop switch started! Press 'q' + Enter or Ctrl+C to stop immediately")
        
        try:
            while not self.check_emergency_stop():
                try:
                    user_input = input().strip().lower()
                    if user_input in ['q', 'quit', 'kill', 'stop', 'emergency']:
                        self.emergency_stop()
                        break
                except (EOFError, KeyboardInterrupt):
                    self.emergency_stop()
                    break
                except Exception:
                    # Ignore other input errors
                    continue
        except Exception as e:
            if not self._shutdown_requested:
                print(f"Keyboard listener error: {e}")
            self.emergency_stop()
    
    def emergency_kill_switch(self, cf):
        """Emergency stop switch"""
        try:
            print("🚨 Emergency stop! Disconnecting motors immediately!")
            cf.loc.send_emergency_stop()
            print("⚡ Motors disconnected")
        except Exception as e:
            print(f"Emergency stop error: {e}")
    
    def setup_crazyflie_fast(self, cf):
        """Fast setup Crazyflie (optimized version)"""
        try:
            # 1. Set estimator
            cf.param.set_value('stabilizer.estimator', '2')
            
            # 2. Set external data trust
            cf.param.set_value('locSrv.extQuatStdDev', 100)
            cf.param.set_value('locSrv.extPosStdDev', 0.02)
            
            # 3. Set controller
            cf.param.set_value('stabilizer.controller', '1')
            
            # 4. Optimize control parameters, reduce oscillation
            cf.param.set_value('posCtlPid.xVelMax', 0.8)  # slightly increase maximum speed
            cf.param.set_value('posCtlPid.yVelMax', 0.8)
            cf.param.set_value('posCtlPid.zVelMax', 0.6)
            cf.param.set_value('posCtlPid.rLimit', 25.0)  # slightly relax angle limit
            cf.param.set_value('posCtlPid.pLimit', 25.0)
            
            return True
        except Exception as e:
            print(f"Fast setup failed: {e}")
            return False
    
    def set_pid_parameters_fast(self, cf, params):
        """Fast set PID parameters"""
        if len(params) != 3:
            self.get_logger().error(f"Expected 3 Kp parameters, received {len(params)}")
            return False
        
        kp_x, kp_y, kp_z = params
        # Use optimized fixed ki and kd values
        ki_x = ki_y = ki_z = 0.1  # slightly reduce integral gain, reduce overshoot
        kd_x = kd_y = 0.35        # slightly reduce derivative gain, reduce noise sensitivity
        kd_z = 0.45
        
        try:
            # Set parameters in batch
            cf.param.set_value('posCtlPid.xKp', float(kp_x))
            cf.param.set_value('posCtlPid.xKi', float(ki_x))
            cf.param.set_value('posCtlPid.xKd', float(kd_x))
            
            cf.param.set_value('posCtlPid.yKp', float(kp_y))
            cf.param.set_value('posCtlPid.yKi', float(ki_y))
            cf.param.set_value('posCtlPid.yKd', float(kd_y))
            
            cf.param.set_value('posCtlPid.zKp', float(kp_z))
            cf.param.set_value('posCtlPid.zKi', float(ki_z))
            cf.param.set_value('posCtlPid.zKd', float(kd_z))
            
            # Reduce waiting time
            time.sleep(0.5)
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to set PID parameters: {e}")
            return False
    
    def setup_fast_logging(self, cf):
        """Setup fast logging"""
        time.sleep(1.0)  # Reduce waiting time
        
        try:
            log_config = LogConfig(name='FastPosition', period_in_ms=50)  # 20Hz，更高频率
            log_config.add_variable('stateEstimate.x', 'float')
            log_config.add_variable('stateEstimate.y', 'float')
            log_config.add_variable('stateEstimate.z', 'float')
            
            log_config.data_received_cb.add_callback(self.fast_position_callback)
            
            def log_error_callback(logconfig, msg):
                print(f"Log error: {msg}")
            log_config.error_cb.add_callback(log_error_callback)
            
            cf.log.add_config(log_config)
            log_config.start()
            print("✅ Fast position log started")
            
            time.sleep(0.2)
            return log_config
            
        except Exception as e:
            print(f"Failed to set up logging: {e}")
            return None
    
    def fast_position_callback(self, timestamp, data, logconfig):
        """Fast position data callback"""
        if self.current_experiment_running:
            try:
                x = data['stateEstimate.x']
                y = data['stateEstimate.y'] 
                z = data['stateEstimate.z']
                
                cf_pos = [x, y, z]
                
                with self.data_lock:
                    # Calculate position error
                    error = np.linalg.norm(
                        np.array([x, y, z]) - np.array(self.target_position)
                    )
                    self.position_errors.append({
                        'timestamp': time.time(),
                        'error': error
                    })
                    
                    # Record to flight logger
                    self.flight_logger.log_position_data(
                        cf_pos=cf_pos, 
                        error=error, 
                        timestamp=timestamp/1000.0  # Convert to seconds
                    )
                        
            except Exception as e:
                print(f"Position callback error: {e}")
    
    def perform_fast_hover_experiment(self, cf):
        """Perform fast hover experiment"""
        commander = cf.high_level_commander
        
        try:
            print(f"🚀 Taking off to {self.hover_height}m...")
            commander.takeoff(self.hover_height, self.takeoff_duration)
            
            # Takeoff wait (reduce check frequency)
            for i in range(int(self.takeoff_duration * 10)):  # 100ms interval
                if self.check_emergency_stop():
                    print("🚨 Emergency stop during takeoff!")
                    self.emergency_kill_switch(cf)
                    return False
                time.sleep(0.1)
            
            # Fast move to hover position
            if not self.check_emergency_stop():
                commander.go_to(0.0, 0.0, self.hover_height, 0.0, 0.8, relative=False)
                print(f"⚡ Hovering for {self.experiment_duration} seconds...")
                
                # Hover data collection
                hover_start_time = time.time()
                while time.time() - hover_start_time < self.experiment_duration:
                    if self.check_emergency_stop():
                        print("🚨 Emergency stop during hover!")
                        self.emergency_kill_switch(cf)
                        return False
                    time.sleep(0.1)  # 100ms check interval
            
            # Fast landing
            if not self.check_emergency_stop():
                print("📉 Landing...")
                commander.land(0.0, self.landing_duration)
                
                # Landing wait
                for i in range(int(self.landing_duration * 10)):
                    if self.check_emergency_stop():
                        print("🚨 Emergency stop during landing!")
                        self.emergency_kill_switch(cf)
                        return False
                    time.sleep(0.1)
                
                print("✅ Fast landing completed")
            
            commander.stop()
            return True
            
        except Exception as e:
            print(f"Fast hover experiment error: {e}")
            return False
    
    def calculate_fast_result(self):
        """Fast calculate experiment result"""
        with self.data_lock:
            if len(self.position_errors) == 0:
                return float('inf')
            
            errors = [e['error'] for e in self.position_errors]
            
            # Simplify calculation, only use steady state part (last 70% of data)
            steady_start = int(len(errors) * 0.3)
            steady_errors = errors[steady_start:] if steady_start < len(errors) else errors
            
            if len(steady_errors) == 0:
                return float('inf')
            
            # Use RMSE as main indicator (more sensitive)
            rmse = np.sqrt(np.mean(np.square(steady_errors)))
            mae = np.mean(steady_errors)
            
            # Simplified composite indicator
            composite_error = 0.7 * rmse + 0.3 * mae
            
            print(f"📊 Fast result: RMSE={rmse:.4f}m, MAE={mae:.4f}m, Composite={composite_error:.4f}m")
            
            return composite_error
    
    def run_fast_experiment(self, params):
        """Run fast experiment"""
        try:
            # Check if shutdown is requested
            if self._shutdown_requested:
                return float('inf')
            
            # Reset emergency stop
            with self.emergency_stop_lock:
                if not self._shutdown_requested:
                    self.emergency_stop_flag = False
            
            # Check if PhaseSpace is initialized
            if not self.phasespace_initialized or self.phasespace_wrapper is None:
                print("❌ PhaseSpace wrapper not initialized")
                return float('inf')
            
            print("🔗 Using existing PhaseSpace wrapper...")
            
            with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                if self.check_emergency_stop():
                    return float('inf')
                
                cf = scf.cf
                
                # Set PhaseSpace callback
                def pose_callback(pose_data):
                    if self.check_emergency_stop():
                        return
                    try:
                        x, y, z, quaternion = pose_data
                        if self.send_full_pose:
                            cf.extpos.send_extpose(x, y, z, quaternion.x, quaternion.y, quaternion.z, quaternion.w)
                        else:
                            cf.extpos.send_extpos(x, y, z)
                    except Exception as e:
                        if not self._shutdown_requested:
                            print(f"Position data sending error: {e}")
                
                # Set PhaseSpace callback
                try:
                    self.phasespace_wrapper.on_pose = pose_callback
                    print("✅ PhaseSpace callback set")
                except Exception as e:
                    print(f"❌ Failed to set PhaseSpace callback: {e}")
                    return float('inf')
                
                # Wait for PhaseSpace data
                print("⏳ Waiting for PhaseSpace data...")
                for i in range(20):  # 2 seconds
                    if self.check_emergency_stop():
                        return float('inf')
                    time.sleep(0.1)
                
                print("✅ PhaseSpace data wait completed")
                
                # Fast setup Crazyflie
                if not self.setup_crazyflie_fast(cf):
                    return float('inf')
                
                if self.check_emergency_stop():
                    return float('inf')
                
                # Set PID parameters
                if not self.set_pid_parameters_fast(cf, params):
                    return float('inf')
                
                if self.check_emergency_stop():
                    return float('inf')
                
                # Set logging
                log_config = self.setup_fast_logging(cf)
                if log_config is None:
                    return float('inf')
                
                if self.check_emergency_stop():
                    return float('inf')
                
                # Fast reset estimator
                print("🔄 Fast estimator reset...")
                reset_estimator(cf)
                
                # Wait for estimator convergence, check stop signal
                for i in range(30):  # 3 seconds, check every 100ms
                    if self.check_emergency_stop():
                        return float('inf')
                    time.sleep(0.1)
                
                # Arm
                if not self.check_emergency_stop():
                    cf.platform.send_arming_request(True)
                    time.sleep(0.5)
                    print("✅ Armed")
                    
                    # Clear data
                    with self.data_lock:
                        self.position_errors.clear()
                    
                    # Start experiment
                    self.current_experiment_running = True
                    
                    try:
                        # Perform fast hover experiment
                        success = self.perform_fast_hover_experiment(cf)
                    finally:
                        self.current_experiment_running = False
                    
                    if success and not self.check_emergency_stop():
                        result = self.calculate_fast_result()
                        return result
                    else:
                        return float('inf')
                else:
                    return float('inf')
                    
        except Exception as e:
            if not self._shutdown_requested:
                print(f"Fast experiment error: {e}")
            return float('inf')
        finally:
            # After experiment, do not stop PhaseSpace, only clear callback
            if self.phasespace_wrapper is not None and self.phasespace_initialized:
                try:
                    # Only clear callback, do not stop wrapper
                    self.phasespace_wrapper.on_pose = None
                    print("✅ PhaseSpace callback cleared (wrapper still running)")
                except Exception as e:
                    print(f"⚠️  Warning clearing PhaseSpace callback: {e}")
            
            print("✅ Experiment completed, PhaseSpace wrapper remains active")

    def pid_params_callback(self, msg):
        """PID parameter callback"""
        if self.current_experiment_running:
            self.get_logger().warn("Experiment is running, skipping this parameter set")
            return
        
        self.iteration += 1
        params = list(msg.data)
        
        print(f"\n{'='*60}")
        print(f"⚡ Fast PID Optimization Experiment {self.iteration}")
        print(f"📥 Kp parameters: X={params[0]:.3f}, Y={params[1]:.3f}, Z={params[2]:.3f}")
        print(f"{'='*60}")
        
        # Start flight logging
        self.flight_logger.start_experiment(self.iteration, params)
        
        # Run fast experiment
        print("🚀 Starting fast experiment...")
        start_time = time.time()
        
        try:
            tracking_error = self.run_fast_experiment(params)
            
            experiment_time = time.time() - start_time
            print(f"⏱️   Fast experiment time: {experiment_time:.1f} seconds")
            print(f"📊 Tracking error: {tracking_error:.4f}m")
            
            # Finish flight logging
            success = tracking_error < float('inf')
            notes = f"Experiment time: {experiment_time:.1f}s"
            if not success:
                notes += " (Experiment failed)"
            
            log_file = self.flight_logger.finish_experiment(
                final_error=tracking_error,
                success=success,
                notes=notes
            )
            
            # Publish result
            result_msg = Float64()
            result_msg.data = tracking_error
            self.result_publisher.publish(result_msg)
            
            # Fast send allow signal
            time.sleep(1.0)  # Reduce waiting time
            allow_msg = Bool()
            allow_msg.data = True
            self.allow_publisher.publish(allow_msg)
            print("✅ Results sent")
            
        except Exception as e:
            self.get_logger().error(f"Fast experiment failed: {e}")
            
            # Record failed experiment
            self.flight_logger.finish_experiment(
                final_error=float('inf'),
                success=False,
                notes=f"Experiment exception: {str(e)}"
            )
            
            result_msg = Float64()
            result_msg.data = float('inf')
            self.result_publisher.publish(result_msg)
            
            time.sleep(0.5)
            allow_msg = Bool()
            allow_msg.data = True
            self.allow_publisher.publish(allow_msg)
    
    def status_callback(self, msg):
        """Handle status message"""
        print(f"📢 Optimization status: {msg.data}")
    
    def best_value_callback(self, msg):
        """Handle best result"""
        print(f"\n🎉 Best result updated!")
        print(f"🏆 Best error: {msg.data:.4f}m")
        
        if msg.data < 0.05:
            print("✨ Optimization effect: Excellent!")
        elif msg.data < 0.1:
            print("👍 Optimization effect: Very good")
        elif msg.data < 0.2:
            print("👌 Optimization effect: Good")
        else:
            print("📈 Optimization effect: Continue improving")


def main():
    print("="*60)
    print("⚡ Fast Crazyflie PID Optimization System")
    print("="*60)
    print("🚀 Optimization Features:")
    print("  • 15-second fast experiment (vs 20-second standard experiment)")
    print("  • 2-second fast takeoff/landing (vs 3-second standard)")
    print("  • Smart parameter range narrowing")
    print("  • Simplified data processing")
    print("  • 15 iterations for fast convergence")
    print("")
    print("⚠️  Estimated total optimization time: 15-25 minutes")
    print("")
    
    # Improved ROS2 initialization and cleanup
    rclpy.init()
    
    client = None
    try:
        client = FastPIDOptimizationClient()
        
        # Use more robust executor
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(client)
        
        print("🚀 Node started, waiting for PID parameter suggestions...")
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            print("\n🛑 Received keyboard interrupt")
        finally:
            # Clean up executor
            try:
                executor.remove_node(client)
                executor.shutdown()
            except Exception:
                pass  # Ignore executor cleanup error
                
    except Exception as e:
        print(f"❌ Client startup failed: {e}")
    finally:
        # Clean up client
        if client is not None:
            try:
                client.cleanup_resources()
                client.destroy_node()
            except Exception:
                pass  # Ignore node cleanup error
        
        # Clean up ROS2
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore ROS2 cleanup error
        
        print("✅ Program exited safely")


if __name__ == '__main__':
    main() 