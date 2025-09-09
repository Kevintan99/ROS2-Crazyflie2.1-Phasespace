#!/usr/bin/env python3
"""
Integrated Crazyflie mission with PhaseSpace tracking and flight logging
Executes the water collection trajectory from simulation with real-world feedback
"""

import time
import sys
import os
import numpy as np
from threading import Thread, Lock
from collections import deque
import rclpy  # Import rclpy at the top

# Add your crazyflie_python path
sys.path.append('/home/fann/crazyswarm_phasespace/src/crazyflie_python')

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

# Import your existing modules
from phasespace_wrapper import PhaseSpaceWrapper
from flight_logger import FlightLogger

class IntegratedMissionController:
    def __init__(self):
        # Initialize ROS2 FIRST before anything that might need it
        rclpy.init()
        
        # Initialize cflib
        cflib.crtp.init_drivers()
        
        # Configuration
        self.uri = uri_helper.uri_from_env(default='radio://0/81/2M/E7E7E7E7E7')
        self.phasespace_rigid_body_id = 2
        self.ENABLE_COORDINATE_TRANSFORM = True
        self.send_full_pose = False
        
        # Mission waypoints from your simulation (adjusted for real coordinates)
        self.waypoints = [
            {'name': 'Above Water', 'position': [0.5, 0.5, 0.5], 'duration': 6.0, 'type': 'drone'},
            {'name': 'Water Target', 'position': [0.5, 0.5, 0.25], 'duration': 5.0, 'type': 'bucket'},
            {'name': 'Above Water (Return)', 'position': [0.5, 0.5, 0.5], 'duration': 5.0, 'type': 'drone'},
            {'name': 'Lift Target', 'position': [0.0, 0.0, 0.5], 'duration': 7.0, 'type': 'drone'},
        ]
        
        # PhaseSpace wrapper
        self.phasespace_wrapper = None
        self.current_position = [0, 0, 0]
        self.position_lock = Lock()
        
        # Flight logger
        self.flight_logger = FlightLogger()
        
        # Data collection
        self.position_errors = deque(maxlen=1000)
        self.trajectory_data = []
        
        # Initialize session
        mission_config = {
            'mission_type': 'water_collection_trajectory',
            'waypoints': self.waypoints,
            'source': 'simulation_export'
        }
        self.session_id = self.flight_logger.start_new_session(mission_config)
        
    def setup_phasespace(self):
        """Initialize PhaseSpace tracking"""
        print("🔗 Initializing PhaseSpace...")
        
        self.phasespace_wrapper = PhaseSpaceWrapper(
            self.phasespace_rigid_body_id,
            enable_coordinate_transform=self.ENABLE_COORDINATE_TRANSFORM,
            send_full_pose=self.send_full_pose
        )
        
        time.sleep(1.0)
        print("✅ PhaseSpace initialized")
        
    def setup_crazyflie(self, cf):
        """Configure Crazyflie for mission"""
        # Set estimator
        cf.param.set_value('stabilizer.estimator', '2')
        
        # External position trust
        cf.param.set_value('locSrv.extQuatStdDev', 100)
        cf.param.set_value('locSrv.extPosStdDev', 0.02)
        
        # Set controller
        cf.param.set_value('stabilizer.controller', '1')
        
        # Set your optimized PID parameters (use best from optimization)
        # These should be replaced with your best values from optimization
        cf.param.set_value('posCtlPid.xKp', 2.0)  # Replace with your best Kp_x
        cf.param.set_value('posCtlPid.yKp', 2.0)  # Replace with your best Kp_y
        cf.param.set_value('posCtlPid.zKp', 2.2)  # Replace with your best Kp_z
        
        cf.param.set_value('posCtlPid.xKi', 0.1)
        cf.param.set_value('posCtlPid.yKi', 0.1)
        cf.param.set_value('posCtlPid.zKi', 0.1)
        
        cf.param.set_value('posCtlPid.xKd', 0.35)
        cf.param.set_value('posCtlPid.yKd', 0.35)
        cf.param.set_value('posCtlPid.zKd', 0.45)
        
        # Velocity limits
        cf.param.set_value('posCtlPid.xVelMax', 0.5)
        cf.param.set_value('posCtlPid.yVelMax', 0.5)
        cf.param.set_value('posCtlPid.zVelMax', 1.0)
        
        print("✅ Crazyflie configured")
        
    def setup_logging(self, cf):
        """Setup position logging"""
        log_config = LogConfig(name='Position', period_in_ms=50)
        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        log_config.add_variable('stateEstimate.z', 'float')
        
        log_config.data_received_cb.add_callback(self.position_callback)
        cf.log.add_config(log_config)
        log_config.start()
        
        return log_config
        
    def position_callback(self, timestamp, data, logconfig):
        """Handle position data from Crazyflie"""
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        
        with self.position_lock:
            self.current_position = [x, y, z]
        
        # Log trajectory data
        self.trajectory_data.append({
            'timestamp': timestamp / 1000.0,
            'position': [x, y, z]
        })
        
    def execute_waypoint(self, cf, waypoint, previous_position):
        """Execute movement to a single waypoint"""
        commander = cf.high_level_commander
        
        target_pos = waypoint['position']
        duration = waypoint['duration']
        name = waypoint['name']
        wp_type = waypoint.get('type', 'drone')
        
        print(f"\n🎯 Flying to {name} ({wp_type} target)")
        print(f"   Target: [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]")
        
        # Log waypoint start
        self.flight_logger.start_experiment(
            iteration=self.waypoints.index(waypoint),
            pid_params=[target_pos[0], target_pos[1], target_pos[2]]  # Using position as params for logging
        )
        
        # Execute movement
        commander.go_to(target_pos[0], target_pos[1], target_pos[2], 0, duration/2, relative=False)
        
        # Monitor during movement
        start_time = time.time()
        errors = []
        
        while time.time() - start_time < duration:
            with self.position_lock:
                current_pos = self.current_position.copy()
            
            # Calculate error based on waypoint type
            if wp_type == 'drone':
                error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
            else:  # bucket type
                # Approximate bucket position (0.25m below drone)
                bucket_pos = [current_pos[0], current_pos[1], current_pos[2] - 0.25]
                error = np.linalg.norm(np.array(bucket_pos) - np.array(target_pos))
            
            errors.append(error)
            
            # Log to flight logger
            self.flight_logger.log_position_data(
                cf_pos=current_pos,
                error=error,
                timestamp=time.time()
            )
            
            time.sleep(0.05)
        
        # Calculate waypoint performance
        mean_error = np.mean(errors) if errors else float('inf')
        final_error = errors[-1] if errors else float('inf')
        
        print(f"   Achieved: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
        print(f"   Mean error: {mean_error:.3f}m, Final error: {final_error:.3f}m")
        
        # Finish logging for this waypoint
        self.flight_logger.finish_experiment(
            final_error=final_error,
            success=True,
            notes=f"Waypoint: {name}"
        )
        
        return current_pos
        
    def run_mission(self):
        """Execute the complete mission"""
        try:
            # Setup PhaseSpace
            self.setup_phasespace()
            
            with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                cf = scf.cf
                
                # Set PhaseSpace callback
                def pose_callback(pose_data):
                    x, y, z, quaternion = pose_data
                    if self.send_full_pose:
                        cf.extpos.send_extpose(x, y, z, quaternion.x, quaternion.y, quaternion.z, quaternion.w)
                    else:
                        cf.extpos.send_extpos(x, y, z)
                    
                    with self.position_lock:
                        self.current_position = [x, y, z]
                
                self.phasespace_wrapper.on_pose = pose_callback
                
                # Wait for PhaseSpace data
                print("⏳ Waiting for PhaseSpace data...")
                time.sleep(2)
                
                # Setup Crazyflie
                self.setup_crazyflie(cf)
                
                # Setup logging
                log_config = self.setup_logging(cf)
                
                # Reset estimator
                print("🔄 Resetting estimator...")
                reset_estimator(cf)
                time.sleep(3)
                
                # Arm
                cf.platform.send_arming_request(True)
                time.sleep(0.5)
                print("✅ Armed and ready")
                
                # Take off
                commander = cf.high_level_commander
                print("🚁 Taking off...")
                commander.takeoff(0.5, 2.0)  # Take off to 0.5m
                time.sleep(3.5)
                
                # Execute waypoints
                print("\n" + "="*50)
                print("Starting Water Collection Mission")
                print("="*50)
                
                previous_pos = [0, 0, 0.5]  # Starting position
                
                for waypoint in self.waypoints:
                    previous_pos = self.execute_waypoint(cf, waypoint, previous_pos)
                    time.sleep(0.5)  # Brief pause between waypoints
                
                # Land
                print("\n🛬 Landing...")
                commander.land(0.0, 3.0)
                time.sleep(3.5)
                
                commander.stop()
                print("✅ Mission completed!")
                
                # Generate final report
                self.generate_mission_report()
                
        except Exception as e:
            print(f"❌ Mission failed: {e}")
            import traceback
            traceback.print_exc()
            
        finally:
            # Clean up PhaseSpace
            if self.phasespace_wrapper:
                self.phasespace_wrapper.on_pose = None
                time.sleep(0.5)
                self.phasespace_wrapper.close()  # Use close() method instead of stop()
            
            # Generate session summary
            self.flight_logger.generate_session_summary()
    
    def generate_mission_report(self):
        """Generate mission performance report"""
        print("\n" + "="*50)
        print("Mission Performance Report")
        print("="*50)
        
        print("\nSimulation vs Reality Comparison:")
        print("-" * 40)
        
        # Compare with simulation results from your waypoints file
        simulation_results = {
            'Above Water': {'drone': [0.415, 0.416, 0.807], 'bucket': [0.416, 0.417, 0.557]},
            'Water Target': {'drone': [0.491, 0.493, 0.278], 'bucket': [0.492, 0.493, 0.028]},
            'Above Water (Return)': {'drone': [0.501, 0.498, 0.786], 'bucket': [0.501, 0.498, 0.536]},
            'Lift Target': {'drone': [0.791, 0.319, 0.558], 'bucket': [0.791, 0.319, 0.308]},
        }
        
        # Print comparison
        for waypoint in self.waypoints:
            name = waypoint['name']
            target = waypoint['position']
            sim_achieved = simulation_results.get(name, {}).get('drone', [0, 0, 0])
            
            print(f"\n{name}:")
            print(f"  Target:     {target}")
            print(f"  Simulation: {sim_achieved}")
            # Reality results will be filled from actual flight data in logs
        
        print("="*50)
    
    def cleanup(self):
        """Clean up resources"""
        try:
            # Clean up PhaseSpace
            if self.phasespace_wrapper:
                self.phasespace_wrapper.on_pose = None
                self.phasespace_wrapper.close()
            
            # Shutdown ROS2
            if rclpy.ok():
                rclpy.shutdown()
                
        except Exception as e:
            print(f"Warning during cleanup: {e}")


def main():
    print("="*60)
    print("🚁 Crazyflie Water Collection Mission")
    print("    Executing Simulated Trajectory")
    print("="*60)
    
    controller = None
    try:
        controller = IntegratedMissionController()
        controller.run_mission()
    except KeyboardInterrupt:
        print("\n🛑 Mission interrupted by user")
    except Exception as e:
        print(f"❌ Fatal error: {e}")
    finally:
        if controller:
            controller.cleanup()
        print("✅ Program exited safely")


if __name__ == '__main__':
    main()