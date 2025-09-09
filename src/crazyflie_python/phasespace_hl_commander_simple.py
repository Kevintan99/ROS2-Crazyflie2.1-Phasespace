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
PhaseSpace Motion Capture Integration with Crazyflie (Simplified Version)

This script connects to a PhaseSpace motion capture system and feeds the position data to a Crazyflie.
For rotation data, it uses a simple identity quaternion (no rotation) as a placeholder.
This is a simplified version that focuses on position tracking only.

The script uses the high level commander to upload a trajectory to fly a figure 8.

Set the uri to the radio settings of the Crazyflie and modify the
PhaseSpace settings matching your system.
"""
import time
import math
from threading import Thread

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

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/81/2M/E7E7E7E7E7')

# PhaseSpace rigid body ID that represents the Crazyflie
phasespace_rigid_body_id = 1

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3

# PhaseSpace coordinate transformation parameters
# PhaseSpace uses XZY coordinate system, convert to XYZ
# Also convert from millimeters to meters
ENABLE_COORDINATE_TRANSFORM = True

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]


class PhaseSpaceWrapper(Thread):
    def __init__(self, rigid_body_id):
        Thread.__init__(self)

        self.rigid_body_id = rigid_body_id
        self.on_pose = None
        self._stay_open = True
        self.current_position = [0.0, 0.0, 0.0]

        # Initialize ROS2 node
        rclpy.init()
        self.node = Node('phasespace_wrapper')
        
        # Subscribe to PhaseSpace data
        self.phasespace_sub = self.node.create_subscription(
            Rigids, '/phasespace_rigids', self.phasespace_callback, 10)

        self.start()

    def close(self):
        self._stay_open = False
        rclpy.shutdown()

    def phasespace_callback(self, msg):
        """Handle PhaseSpace rigid body data with position and rotation"""
        for rigid in msg.rigids:
            if rigid.id == self.rigid_body_id and rigid.cond > 0:
                # Transform coordinates if enabled
                if ENABLE_COORDINATE_TRANSFORM:
                    # Phase Space uses XZY coordinate system, convert to XYZ
                    # Also convert from millimeters to meters
                    x = rigid.x / 1000.0  # mm to m
                    y = rigid.z / 1000.0  # Z -> Y
                    z = rigid.y / 1000.0  # Y -> Z
                    
                    # Transform quaternion (XZY to XYZ)
                    qx = rigid.qx
                    qy = rigid.qz  # Z -> Y
                    qz = -rigid.qy  # Y -> -Z (due to coordinate system change)
                    qw = rigid.qw
                else:
                    # No transformation
                    x = rigid.x / 1000.0
                    y = rigid.y / 1000.0
                    z = rigid.z / 1000.0
                    qx = rigid.qx
                    qy = rigid.qy
                    qz = rigid.qz
                    qw = rigid.qw

                self.current_position = [x, y, z]
                
                if self.on_pose:
                    # Create quaternion object
                    quaternion = Quaternion()
                    quaternion.w = qw
                    quaternion.x = qx
                    quaternion.y = qy
                    quaternion.z = qz
                    
                    # print(f"PhaseSpace数据: 位置=({x:.3f}, {y:.3f}, {z:.3f}), 四元数=({qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f})")
                    self.on_pose(self.current_position, quaternion)

    def run(self):
        """Main thread loop"""
        while self._stay_open:
            rclpy.spin_once(self.node, timeout_sec=0.1)


def send_extpose_quat(cf, position, quaternion):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if send_full_pose:
        cf.extpos.send_extpose(position[0], position[1], position[2], 
                              quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    else:
        cf.extpos.send_extpos(position[0], position[1], position[2])


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')


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


def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    relative = True
    # commander.start_trajectory(trajectory_id, 1.0, relative)
    # time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    print("启动PhaseSpace + Crazyflie集成系统...")
    print(f"PhaseSpace刚体ID: {phasespace_rigid_body_id}")
    print(f"坐标转换: {'启用' if ENABLE_COORDINATE_TRANSFORM else '禁用'}")
    print(f"发送完整姿态: {'是' if send_full_pose else '否'}")

    # Connect to the PhaseSpace system
    phasespace_wrapper = PhaseSpaceWrapper(phasespace_rigid_body_id)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        trajectory_id = 1



        # Set up a callback to handle data from the PhaseSpace system
        def pose_callback(position, quaternion):
            # Use PhaseSpace position and rotation with coordinate transformation
            send_extpose_quat(cf, position, quaternion)

        phasespace_wrapper.on_pose = pose_callback

        # 配置Crazyflie以使用PhaseSpace控制
        # configure_crazyflie_for_phasespace(cf)
        adjust_orientation_sensitivity(cf)
        activate_kalman_estimator(cf)
        
        # 重置状态估计器
        reset_estimator(cf)

        # Arm the Crazyflie
        cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # # 简单的悬停测试
        # print("开始悬停测试...")
        commander = cf.high_level_commander
        commander.takeoff(0.2, 2.0)  # 起飞到1米高度
        time.sleep(2.0)              # 悬停5秒
        commander.land(0.0, 2.0)     # 降落
        time.sleep(2.0)
        commander.stop()

    phasespace_wrapper.close() 