#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PhaseSpace position and attitude coordinate transformation test program
Test if PhaseSpace coordinate system to Crazyflie ENU coordinate system position and rotation transformations are correct
"""

import rclpy
from rclpy.node import Node
from phasespace_msgs.msg import Rigids, Rigid
import numpy as np
import time
from threading import Thread
from scipy.spatial.transform import Rotation as R

# PhaseSpace rigid body ID
RIGID_BODY_ID = 1

# Coordinate transformation switch
ENABLE_COORDINATE_TRANSFORM = True

class PoseTransformTester(Node):
    def __init__(self):
        super().__init__('pose_transform_tester')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Rigids,
            '/phasespace_rigids',
            self.phasespace_callback,
            10
        )
        
        # Data storage
        self.raw_position = [0.0, 0.0, 0.0]
        self.raw_quaternion = [0.0, 0.0, 0.0, 1.0]  # [x,y,z,w]
        self.raw_euler = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]
        
        self.transformed_position = [0.0, 0.0, 0.0]
        self.transformed_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.transformed_euler = [0.0, 0.0, 0.0]
        
        self.last_update_time = time.time()
        
        # Statistics data
        self.pose_history = []
        self.max_history_length = 100
        
        print("=== PhaseSpace Position and Attitude Transformation Test Program ===")
        print("PhaseSpace coordinate system: X(left) Y(up) Z(forward)")
        print("Crazyflie ENU coordinate system: X(east) Y(north) Z(up)")
        print("Transformation rules:")
        print("  Position: PhaseSpace[X,Y,Z] -> ENU[X,Z,Y] (left-up-forward -> east-north-up)")
        print("  Attitude: Corresponding quaternion coordinate transformation")
        print("\nWaiting for PhaseSpace data...")
        
        # Start data display thread
        self.display_thread = Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

    def phasespace_callback(self, msg):
        """Process PhaseSpace data"""
        for rigid in msg.rigids:
            if rigid.id == RIGID_BODY_ID and rigid.cond > 0:
                # Save raw position (mm to m)
                self.raw_position = [
                    rigid.x / 1000.0,
                    rigid.y / 1000.0, 
                    rigid.z / 1000.0
                ]
                
                # Save raw quaternion
                self.raw_quaternion = [rigid.qx, rigid.qy, rigid.qz, rigid.qw]
                
                # Calculate raw Euler angles
                try:
                    r_raw = R.from_quat(self.raw_quaternion)
                    self.raw_euler = r_raw.as_euler('xyz', degrees=True)
                except:
                    self.raw_euler = [0.0, 0.0, 0.0]
                
                # Coordinate and attitude transformation
                if ENABLE_COORDINATE_TRANSFORM:
                    # === Position transformation ===
                    # PhaseSpace: X(left) Y(up) Z(forward) -> ENU: X(east) Y(north) Z(up)
                    x_enu = rigid.x / 1000.0   # left -> east
                    y_enu = -rigid.z / 1000.0   # forward -> north  
                    z_enu = rigid.y / 1000.0   # up -> up
                    
                    # If any axis direction is wrong, you can add negative sign to adjust:
                    # x_enu = -rigid.x / 1000.0  # Flip X axis
                    # y_enu = -rigid.z / 1000.0  # Flip Y axis
                    
                    # === Attitude transformation ===
                    # Create coordinate system transformation matrix (PhaseSpace -> ENU)
                    T_ps_to_enu = np.array([
                        [1,  0,  0],  # left -> east (X remains)
                        [0,  0,  -1],  # forward -> north (Z -> Y) 
                        [0,  1,  0]   # up -> up (Y -> Z)
                    ])
                    
                    # Convert quaternion to rotation matrix
                    q_ps = np.array([rigid.qx, rigid.qy, rigid.qz, rigid.qw])
                    try:
                        R_ps = R.from_quat(q_ps).as_matrix()
                        
                        # Apply coordinate system transformation
                        R_enu = T_ps_to_enu @ R_ps @ T_ps_to_enu.T
                        
                        # Convert back to quaternion
                        r_enu = R.from_matrix(R_enu)
                        q_enu = r_enu.as_quat()  # [x,y,z,w]
                        
                        self.transformed_quaternion = q_enu.tolist()
                        self.transformed_euler = r_enu.as_euler('xyz', degrees=True)
                        
                    except Exception as e:
                        print(f"Attitude transformation error: {e}")
                        self.transformed_quaternion = [0.0, 0.0, 0.0, 1.0]
                        self.transformed_euler = [0.0, 0.0, 0.0]
                    
                else:
                    # No transformation, use raw data directly
                    x_enu = rigid.x / 1000.0
                    y_enu = -rigid.y / 1000.0
                    z_enu = rigid.z / 1000.0
                    self.transformed_quaternion = self.raw_quaternion.copy()
                    self.transformed_euler = self.raw_euler.copy()
                
                self.transformed_position = [x_enu, y_enu, z_enu]
                self.last_update_time = time.time()
                
                # Save historical data for statistics
                self.pose_history.append({
                    'time': time.time(),
                    'raw_pos': self.raw_position.copy(),
                    'raw_quat': self.raw_quaternion.copy(),
                    'raw_euler': self.raw_euler.copy(),
                    'transformed_pos': self.transformed_position.copy(),
                    'transformed_quat': self.transformed_quaternion.copy(),
                    'transformed_euler': self.transformed_euler.copy()
                })
                
                # Limit historical data length
                if len(self.pose_history) > self.max_history_length:
                    self.pose_history.pop(0)

    def display_loop(self):
        """Data display loop"""
        while True:
            time.sleep(0.5)  # Update display every 0.5 seconds
            
            # Check data update status
            time_since_update = time.time() - self.last_update_time
            if time_since_update > 2.0:
                print(f"\n[Warning] No data update for {time_since_update:.1f}s")
                continue
            
            # Clear screen and display current data
            print("\033[2J\033[H", end="")  # Clear screen
            print("=== PhaseSpace Position and Attitude Transformation Real-time Monitor ===")
            print(f"Rigid body ID: {RIGID_BODY_ID}")
            print(f"Coordinate transformation: {'Enabled' if ENABLE_COORDINATE_TRANSFORM else 'Disabled'}")
            print(f"Data update: {time_since_update:.2f}s ago")
            print("=" * 80)
            
            # === Display position information ===
            print("📍 Position Information:")
            print(f"  PhaseSpace Raw: X={self.raw_position[0]:+7.3f} Y={self.raw_position[1]:+7.3f} Z={self.raw_position[2]:+7.3f}")
            print(f"  ENU Transformed: X={self.transformed_position[0]:+7.3f} Y={self.transformed_position[1]:+7.3f} Z={self.transformed_position[2]:+7.3f}")
            
            # === Display attitude information ===
            print("\n🧭 Attitude Information:")
            print("  Quaternion (x, y, z, w):")
            print(f"    PhaseSpace: ({self.raw_quaternion[0]:+6.3f}, {self.raw_quaternion[1]:+6.3f}, {self.raw_quaternion[2]:+6.3f}, {self.raw_quaternion[3]:+6.3f})")
            print(f"    ENU Transformed: ({self.transformed_quaternion[0]:+6.3f}, {self.transformed_quaternion[1]:+6.3f}, {self.transformed_quaternion[2]:+6.3f}, {self.transformed_quaternion[3]:+6.3f})")
            
            print("  Euler Angles (Roll, Pitch, Yaw) [degrees]:")
            print(f"    PhaseSpace: ({self.raw_euler[0]:+7.2f}, {self.raw_euler[1]:+7.2f}, {self.raw_euler[2]:+7.2f})")
            print(f"    ENU Transformed: ({self.transformed_euler[0]:+7.2f}, {self.transformed_euler[1]:+7.2f}, {self.transformed_euler[2]:+7.2f})")
            
            # Calculate position and attitude changes
            if len(self.pose_history) >= 2:
                current_pos = np.array(self.transformed_position)
                previous_pos = np.array(self.pose_history[-2]['transformed_pos'])
                pos_delta = current_pos - previous_pos
                pos_speed = np.linalg.norm(pos_delta)
                
                current_euler = np.array(self.transformed_euler)
                previous_euler = np.array(self.pose_history[-2]['transformed_euler'])
                euler_delta = current_euler - previous_euler
                
                # Handle Euler angle jump issues
                euler_delta = np.where(euler_delta > 180, euler_delta - 360, euler_delta)
                euler_delta = np.where(euler_delta < -180, euler_delta + 360, euler_delta)
                
                print(f"\n📊 Change Information:")
                print(f"  Position change: ΔX={pos_delta[0]:+7.3f} ΔY={pos_delta[1]:+7.3f} ΔZ={pos_delta[2]:+7.3f}")
                print(f"  Position speed: {pos_speed:.4f} m/s")
                print(f"  Attitude change: ΔR={euler_delta[0]:+7.2f} ΔP={euler_delta[1]:+7.2f} ΔY={euler_delta[2]:+7.2f} [deg/s]")
            
            # Display statistics
            if len(self.pose_history) > 20:
                recent_poses = self.pose_history[-50:]  # Recent 50 points
                positions = np.array([p['transformed_pos'] for p in recent_poses])
                eulers = np.array([p['transformed_euler'] for p in recent_poses])
                
                pos_std = np.std(positions, axis=0)
                pos_mean = np.mean(positions, axis=0)
                euler_std = np.std(eulers, axis=0)
                euler_mean = np.mean(eulers, axis=0)
                
                print(f"\n📈 Statistics (Recent {len(recent_poses)} data points):")
                print(f"  Position mean: X={pos_mean[0]:+7.3f} Y={pos_mean[1]:+7.3f} Z={pos_mean[2]:+7.3f}")
                print(f"  Position std dev: X={pos_std[0]:7.3f} Y={pos_std[1]:7.3f} Z={pos_std[2]:7.3f}")
                print(f"  Attitude mean: R={euler_mean[0]:+7.2f} P={euler_mean[1]:+7.2f} Y={euler_mean[2]:+7.2f}")
                print(f"  Attitude std dev: R={euler_std[0]:7.2f} P={euler_std[1]:7.2f} Y={euler_std[2]:7.2f}")
                
                pos_noise_level = "Low" if max(pos_std) < 0.002 else "Medium" if max(pos_std) < 0.01 else "High"
                rot_noise_level = "Low" if max(euler_std) < 0.5 else "Medium" if max(euler_std) < 2.0 else "High"
                print(f"  Noise level: Position={pos_noise_level} Attitude={rot_noise_level}")
            
            # Display test instructions
            print("\n" + "=" * 80)
            print("🧪 Test Instructions:")
            print("【Position Test】")
            print("1. Move along PhaseSpace X-axis (left-right) -> Observe ENU X value change")
            print("2. Move along PhaseSpace Z-axis (forward-back) -> Observe ENU Y value change") 
            print("3. Move along PhaseSpace Y-axis (up-down) -> Observe ENU Z value change")
            print("【Attitude Test】")
            print("4. Rotate around PhaseSpace axes, observe Euler angle changes:")
            print("   - Rotate around X-axis (roll) -> Observe Roll change")
            print("   - Rotate around Y-axis (pitch) -> Observe Pitch change") 
            print("   - Rotate around Z-axis (yaw) -> Observe Yaw change")
            print("\nPress Ctrl+C to exit program")

    def analyze_coordinate_mapping(self):
        """Analyze coordinate mapping relationships"""
        if len(self.pose_history) < 10:
            return
        
        recent_data = self.pose_history[-10:]
        
        # Analyze dominant axis of position changes
        pos_changes = []
        for i in range(1, len(recent_data)):
            current = np.array(recent_data[i]['transformed_pos'])
            previous = np.array(recent_data[i-1]['transformed_pos'])
            pos_changes.append(current - previous)
        
        if pos_changes:
            total_change = np.sum(pos_changes, axis=0)
            if np.max(np.abs(total_change)) > 0.05:  # Movement > 5cm
                dominant_axis = np.argmax(np.abs(total_change))
                axis_names = ['X(East)', 'Y(North)', 'Z(Up)']
                direction = 'Positive' if total_change[dominant_axis] > 0 else 'Negative'
                print(f"  Movement detected: {axis_names[dominant_axis]} {direction} direction ({total_change[dominant_axis]:+.3f}m)")


def test_coordinate_transform_theory():
    """Test coordinate transformation theory correctness"""
    print("\n=== Coordinate Transformation Theory Verification ===")
    
    # Define transformation matrix
    T_ps_to_enu = np.array([
        [1,  0,  0],  # left -> east
        [0,  0,  -1],  # forward -> north  
        [0,  1,  0]   # up -> up
    ])
    
    # Test vectors
    test_vectors = {
        "PhaseSpace Right(+X)": np.array([1, 0, 0]),
        "PhaseSpace Up(+Y)": np.array([0, 1, 0]), 
        "PhaseSpace Forward(+Z)": np.array([0, 0, 1]),
    }
    
    print("PhaseSpace -> ENU Position Mapping:")
    for name, vec in test_vectors.items():
        transformed = T_ps_to_enu @ vec
        enu_dirs = ["East(+X)", "North(+Y)", "Up(+Z)"]
        result_dir = enu_dirs[np.argmax(np.abs(transformed))]
        sign = "+" if transformed[np.argmax(np.abs(transformed))] > 0 else "-"
        print(f"  {name} -> ENU {sign}{result_dir}")
    
    # Test rotation transformation
    print("\nAttitude Transformation Test:")
    test_rotations = [
        ("Rotate around X-axis 90°", R.from_euler('x', 90, degrees=True)),
        ("Rotate around Y-axis 90°", R.from_euler('y', 90, degrees=True)),
        ("Rotate around Z-axis 90°", R.from_euler('z', 90, degrees=True)),
    ]
    
    for name, rotation in test_rotations:
        R_ps = rotation.as_matrix()
        R_enu = T_ps_to_enu @ R_ps @ T_ps_to_enu.T
        euler_enu = R.from_matrix(R_enu).as_euler('xyz', degrees=True)
        print(f"  {name} -> ENU Euler angles: R={euler_enu[0]:.1f}° P={euler_enu[1]:.1f}° Y={euler_enu[2]:.1f}°")


def main():
    """Main function"""
    rclpy.init()
    
    # Run theory verification
    test_coordinate_transform_theory()
    
    try:
        tester = PoseTransformTester()
        
        print("\nTest program started, please move and rotate the PhaseSpace rigid body for testing...")
        print("Recommended test sequence:")
        print("1. Keep rigid body still to observe noise level")
        print("2. Test position separately (translation)")
        print("3. Test attitude separately (rotation)")
        print("4. Combined test of position and attitude")
        
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Program error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()
        print("Program exited")


if __name__ == '__main__':
    main()