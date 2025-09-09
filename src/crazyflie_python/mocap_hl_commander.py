# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2023 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Example of how to connect to a PhaseSpace motion capture system and feed the position to a
Crazyflie. This script uses ROS2 to subscribe to PhaseSpace data and provides
position and orientation data to the Crazyflie's state estimator.
The script uses the high level commander to upload a trajectory to fly a figure 8.

Set the uri to the radio settings of the Crazyflie and modify the
PhaseSpace rigid body ID matching your system.

坐标系说明:
- PhaseSpace坐标系: X(左) Y(上) Z(前) - 右手坐标系
- Crazyflie严格使用ENU坐标系: X(东) Y(北) Z(上) - 右手坐标系
- 本脚本自动进行坐标系转换，确保位置和姿态数据正确传递给Crazyflie
"""
import time
from threading import Thread
import threading
import rclpy
from rclpy.node import Node
from phasespace_msgs.msg import Rigids, Rigid
from geometry_msgs.msg import Quaternion

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import os
from collections import deque
from std_msgs.msg import Float64, Float64MultiArray, Bool

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/81/2M/E7E7E7E7E7')

# PhaseSpace rigid body ID that represents the Crazyflie
phasespace_rigid_body_id = 1

# PhaseSpace到ENU坐标系转换
# 启用此选项确保PhaseSpace数据正确转换为Crazyflie要求的ENU坐标系
# PhaseSpace: X(左) Y(上) Z(前) -> ENU: X(东) Y(北) Z(上)
ENABLE_COORDINATE_TRANSFORM = True

# True: 发送位置和姿态; False: 仅发送位置
# 建议使用完整姿态数据以获得更好的飞行性能
# 调试阶段先禁用姿态数据，仅使用位置数据
send_full_pose = False  # 临时设为False进行调试

# 姿态数据标准差 - 当偏航角接近±90度时，估计器可能对姿态噪声敏感
# 如果遇到问题，可以适当增加此值。固件默认值为4.5e-3
orientation_std_dev = 100

# 悬停配置
HOVER_HEIGHT = 0.3      # 悬停高度 (米) - 降低到0.3米进行调试
HOVER_DURATION = 0     # 自动悬停时长 (秒)，设置为0则无限悬停直到手动停止
ENABLE_AUTO_LAND = True # 是否启用自动降落

phasespace_wrapper_global = None

class AdaptiveFilter:
    def __init__(self, static_alpha=0.3, moving_alpha=0.7, motion_threshold=0.01):
        self.static_alpha = static_alpha    # Heavy filtering when still
        self.moving_alpha = moving_alpha    # Light filtering when moving
        self.motion_threshold = motion_threshold
        self.last_raw = None
        self.last_filtered = None
        
    def filter(self, raw_value):
        if self.last_raw is None:
            self.last_raw = raw_value
            self.last_filtered = raw_value
            return raw_value
        
        # Detect if we're moving
        motion = abs(raw_value - self.last_raw)
        
        # Adaptive alpha based on motion
        if motion > self.motion_threshold:
            alpha = self.moving_alpha  # Less filtering during movement
        else:
            alpha = self.static_alpha  # More filtering when stationary
        
        # Apply filter
        filtered = alpha * raw_value + (1 - alpha) * self.last_filtered
        
        self.last_raw = raw_value
        self.last_filtered = filtered
        
        return filtered
        
# Add this class before the main code
class PIDOptimizationEvaluator(Node):
    def __init__(self, cf):
        super().__init__('pid_optimization_evaluator')
        
        self.cf = cf
        self.current_params = None
        self.evaluation_active = False
        self.position_errors = deque(maxlen=500)
        self.evaluation_start_time = None
        self.evaluation_duration = 5.0
        
        # Connection monitoring
        self.connection_lost = False
        self.last_connection_check = time.time()
        
        # Target positions for hover
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = HOVER_HEIGHT

        # Subscribe to Bayesian optimization parameters
        self.param_sub = self.create_subscription(
            Float64MultiArray, 
            '/bayes_opt/params_to_test',
            self.receive_params_callback,
            10
        )
        
        # Publishers for results
        self.result_pub = self.create_publisher(Float64, '/bayes_opt/experiment_result', 10)
        self.allow_pub = self.create_publisher(Bool, '/bayes_opt/allow_step', 10)
        
        # Timer for evaluation
        self.eval_timer = self.create_timer(0.1, self.evaluation_loop)
        
        self.get_logger().info("PID Optimization Evaluator initialized")
    
    def check_connection(self):
        """Check if drone is still connected"""
        try:
            # Try to read a parameter to check connection
            test_param = self.cf.param.get_value('stabilizer.estimator')
            return True
        except:
            return False
    
    def receive_params_callback(self, msg):
        """Receive new PID parameters to test"""
        if not self.evaluation_active:
            # Check connection first
            if not self.check_connection():
                self.get_logger().error("Connection lost! Cannot apply parameters")
                # Send high cost to skip this iteration
                self.result_pub.publish(Float64(data=1000.0))
                time.sleep(0.5)
                self.allow_pub.publish(Bool(data=True))
                return
            
            self.current_params = msg.data
            self.get_logger().info(f"Received PID params to test: {self.current_params}")
            self.apply_pid_params()
            self.start_evaluation()
    
    def apply_pid_params(self):
        """Apply the PID parameters to the Crazyflie"""
        if self.current_params is None:
            return
        
        try:
            # Check connection before applying
            if not self.check_connection():
                raise Exception("Connection lost before applying parameters")
            
            # Apply parameters with error checking for each one
            params_to_set = [
                ('posCtlPid.xKp', self.current_params[0]),
                ('posCtlPid.yKp', self.current_params[1]),
                ('posCtlPid.zKp', self.current_params[2]),
                ('posCtlPid.xKi', self.current_params[3]),
                ('posCtlPid.yKi', self.current_params[4]),
                ('posCtlPid.zKi', self.current_params[5])
            ]
            
            for param_name, value in params_to_set:
                try:
                    self.cf.param.set_value(param_name, float(value))
                except Exception as e:
                    self.get_logger().error(f"Failed to set {param_name}: {e}")
                    # Try to reconnect
                    raise
            
            self.get_logger().info(f"Applied PID params successfully")
            
            # Give time for parameters to take effect
            time.sleep(0.5)
            
        except Exception as e:
            self.get_logger().error(f"Failed to apply PID params: {e}")
            self.connection_lost = True
            raise

class PhaseSpaceWrapper(Thread):
    def __init__(self, rigid_body_id):
        Thread.__init__(self)
        self.daemon = True  # Make thread daemon so it won't prevent program exit

        self.rigid_body_id = rigid_body_id
        self.on_pose = None
        self._stay_open = True
        self.current_position = [-0.006, 0.014, 0.041]
        self.node = None

        # Add filtering
        self.filter_alpha = 0.3  # Lower = more filtering (0.1-0.5)
        self.filtered_position = [0.0, 0.0, 0.0]
        self.filtered_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.first_reading = True
        
        # Add filters for adaptive filtering
        self.filter_x = None
        self.filter_y = None
        self.filter_z = None

        # Create ROS2 node (don't initialize ROS2 here, it's done in main)
        try:
            self.node = Node('phasespace_wrapper')
            
            # Subscribe to PhaseSpace data
            self.phasespace_sub = self.node.create_subscription(
                Rigids, '/phasespace_rigids', self.phasespace_callback, 10)

            self.start()
        except Exception as e:
            print(f"Failed to create PhaseSpace node: {e}")


    def close(self):
        """Clean shutdown of the PhaseSpace wrapper"""
        self._stay_open = False
        # Give a moment for the thread to exit its loop
        time.sleep(0.2)
        
        # Destroy the node if it exists
        if self.node:
            try:
                self.node.destroy_node()
            except:
                pass

    def low_pass_filter(self, new_value, old_value, alpha):
        """Simple exponential moving average filter"""
        if self.first_reading:
            self.first_reading = False
            return new_value
        return alpha * new_value + (1 - alpha) * old_value
    

    def phasespace_callback(self, msg):
        """Handle PhaseSpace rigid body data with position and rotation"""
        for rigid in msg.rigids:
            if rigid.id == self.rigid_body_id and rigid.cond > 0:
                # === STEP 1: COORDINATE TRANSFORMATION (MUST COME FIRST!) ===
                if ENABLE_COORDINATE_TRANSFORM:
                    # Position transformation (millimeters to meters)
                    x = rigid.x / 1000.0    # left -> east
                    y = -rigid.z / 1000.0   # forward -> north
                    z = rigid.y / 1000.0    # up -> up
                    
                    # Quaternion transformation
                    q_ps = np.array([rigid.qx, rigid.qy, rigid.qz, rigid.qw])
                    
                    # Create coordinate system transformation matrix (PhaseSpace -> ENU)
                    T_ps_to_enu = np.array([
                        [ 1,  0,  0],  # left -> east
                        [ 0,  0, -1],  # forward -> north
                        [ 0,  1,  0]   # up -> up
                    ])
                    
                    try:
                        # Transform rotation
                        R_ps = R.from_quat(q_ps).as_matrix()
                        R_enu = T_ps_to_enu @ R_ps @ T_ps_to_enu.T
                        q_enu = R.from_matrix(R_enu).as_quat()
                        
                        if send_full_pose:
                            qx, qy, qz, qw = q_enu
                        else:
                            qx = qy = qz = 0.0
                            qw = 1.0
                    except:
                        # Fallback if rotation transformation fails
                        qx = qy = qz = 0.0
                        qw = 1.0
                    
                else:
                    # No coordinate transformation
                    x = rigid.x / 1000.0
                    y = rigid.y / 1000.0
                    z = rigid.z / 1000.0
                    qx = rigid.qx
                    qy = rigid.qy
                    qz = rigid.qz
                    qw = rigid.qw
                
                # === STEP 2: APPLY LOW-PASS FILTER (AFTER TRANSFORMATION!) ===
                if self.first_reading:
                    # Initialize filtered values on first reading
                    self.filtered_position = [x, y, z]
                    self.filtered_quaternion = [qx, qy, qz, qw]
                    self.first_reading = False
                
                # Filter position
                self.filter_x = AdaptiveFilter(static_alpha=0.3, moving_alpha=0.7, motion_threshold=0.01)
                self.filter_y = AdaptiveFilter(static_alpha=0.2, moving_alpha=0.6, motion_threshold=0.01)  # Stronger filtering for Y
                self.filter_z = AdaptiveFilter(static_alpha=0.3, moving_alpha=0.7, motion_threshold=0.01)

                x_filtered = self.filter_x.filter(x)
                y_filtered = self.filter_y.filter(y)
                z_filtered = self.filter_z.filter(z)

                self.filtered_position = [x_filtered, y_filtered, z_filtered]
                self.current_position = self.filtered_position
                
                # Filter quaternion if using full pose
                if send_full_pose:
                    qx_filtered = self.low_pass_filter(qx, self.filtered_quaternion[0], self.filter_alpha)
                    qy_filtered = self.low_pass_filter(qy, self.filtered_quaternion[1], self.filter_alpha)
                    qz_filtered = self.low_pass_filter(qz, self.filtered_quaternion[2], self.filter_alpha)
                    qw_filtered = self.low_pass_filter(qw, self.filtered_quaternion[3], self.filter_alpha)
                    
                    # Normalize quaternion after filtering
                    norm = np.sqrt(qx_filtered**2 + qy_filtered**2 + qz_filtered**2 + qw_filtered**2)
                    if norm > 0:
                        qx_filtered /= norm
                        qy_filtered /= norm
                        qz_filtered /= norm
                        qw_filtered /= norm
                    else:
                        qx_filtered = qy_filtered = qz_filtered = 0.0
                        qw_filtered = 1.0
                    
                    self.filtered_quaternion = [qx_filtered, qy_filtered, qz_filtered, qw_filtered]
                else:
                    qx_filtered = qy_filtered = qz_filtered = 0.0
                    qw_filtered = 1.0
                
                # === STEP 3: SEND FILTERED DATA ===
                if self.on_pose:
                    quaternion = Quaternion()
                    quaternion.x = qx_filtered
                    quaternion.y = qy_filtered
                    quaternion.z = qz_filtered
                    quaternion.w = qw_filtered
                    self.on_pose([x_filtered, y_filtered, z_filtered, quaternion])


    def run(self):
        """Main thread loop with clean shutdown handling"""
        while self._stay_open:
            try:
                # Check if ROS2 is still initialized and OK before spinning
                if rclpy.ok():
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                else:
                    break
            except rclpy.executors.ExternalShutdownException:
                # This is expected when shutting down, just exit the loop
                break
            except Exception as e:
                if self._stay_open:
                    # Only print error if we're not shutting down
                    print(f"Error in PhaseSpace wrapper: {e}")
                break


def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(x, y, z)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', 100)
    cf.param.set_value('locSrv.extPosStdDev', 0.001)

def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')
    # cf.param.set_value('ctrlMel.mass', 0.036)
    # cf.param.set_value('ctrlMel.massThrust', 0.036 * 9.81)

def activate_pid_controller(cf):
    cf.param.set_value('stabilizer.controller', '1')
 

def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    trajectory_mem.write_data_sync()
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration

def reduce_oscillation_pid(cf):
     """
     Balanced PID parameters for 36g Crazyflie
     Strategy: Keep thrust capability while reducing oscillation
     """
     print("Applying balanced PID parameters for stable hovering...")
     print("Balanced parameters applied!")
 

def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(0.5, 2.0)
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()


if __name__ == '__main__':
    # Initialize ROS2 only once at the beginning
    try:
        rclpy.init()
    except RuntimeError as e:
        if "Context.init() must only be called once" in str(e):
            print("ROS2 already initialized, continuing...")
        else:
            raise
    
    cflib.crtp.init_drivers()

    # Connect to the PhaseSpace system
    phasespace_wrapper = PhaseSpaceWrapper(phasespace_rigid_body_id)
    phasespace_wrapper_global = phasespace_wrapper  # Store globally

    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf
            trajectory_id = 1

            pid_evaluator = PIDOptimizationEvaluator(cf)
            # Set up a callback to handle data from the PhaseSpace system
            phasespace_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])

            # 统一的参数设置
            print("正在配置Crazyflie参数...")
            
            # 1. 设置估计器
            activate_kalman_estimator(cf)
            print("估计器设置为Kalman")

            # 2. 设置外部数据信任度
            adjust_orientation_sensitivity(cf)
            
            # 3. 设置控制器
            activate_pid_controller(cf)
            print("控制器设置为PID")
            
            reduce_oscillation_pid(cf)
            
            # 6. 等待参数设置生效
            time.sleep(1.0)
            print("参数配置完成")
            
            reset_estimator(cf)
            
            # Arm the Crazyflie
            cf.platform.send_arming_request(True)
            time.sleep(1.0)
            print("Armed and ready for PID optimization")
            
            # 悬停飞行序列
            commander = cf.high_level_commander
            
            initial_position = phasespace_wrapper.current_position
            print(f"Initial position: x={initial_position[0]:.3f}, y={initial_position[1]:.3f}, z={initial_position[2]:.3f}")

            # Calculate absolute target position
            target_x = initial_position[0]  # Keep same X position
            target_y = initial_position[1]  # Keep same Y position
            target_z = HOVER_HEIGHT         # Use absolute height directly

            # Update evaluator target positions
            pid_evaluator.target_x = target_x
            pid_evaluator.target_y = target_y
            pid_evaluator.target_z = target_z

            print(f"Taking off to hover position: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
            commander.go_to(target_x, target_y, target_z, 0.0, 3.0, relative=False)
            time.sleep(4.0)  # Wait for movement to complete

            # Keep hovering and let the optimization run
            print("Hovering... Bayesian optimization will test different PID parameters")
            print("The drone will evaluate each parameter set for 10 seconds")
            print("Press Ctrl+C to stop the optimization and land")

            # Create executor for ROS2
            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(pid_evaluator)
            
            # Spin in a separate thread
            executor_thread = threading.Thread(target=executor.spin, daemon=True)
            executor_thread.start()

            
            # Main hover loop - run indefinitely or for HOVER_DURATION
            try:
                if HOVER_DURATION > 0:
                    print(f"Running optimization for {HOVER_DURATION} seconds...")
                    start_time = time.time()
                    while time.time() - start_time < HOVER_DURATION:
                        # Re-send position command periodically for stability
                        commander.go_to(target_x, target_y, target_z, 0.0, 0.5, relative=False)
                        time.sleep(2.0)
                else:
                    print("Running optimization indefinitely... Press Ctrl+C to stop")
                    while True:
                        # Re-send position command periodically for stability
                        commander.go_to(target_x, target_y, target_z, 0.0, 0.5, relative=False)
                        time.sleep(2.0)
                        
            except KeyboardInterrupt:
                print("\nOptimization interrupted by user")
            
            # Landing sequence
            if ENABLE_AUTO_LAND:
                print("Starting landing sequence...")
                commander.land(0.0, 3.0)
                time.sleep(4.0)
                print("Landing complete")
            else:
                print("Auto-land disabled. Maintaining hover.")
            
            commander.stop()
            
            # Disarm
            cf.platform.send_arming_request(False)
            print("Drone disarmed")
                
    except Exception as e:
        print(f"Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        if phasespace_wrapper:
            phasespace_wrapper.close()
        if rclpy.ok():
            rclpy.shutdown()
        print("Program terminated")