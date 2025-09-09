"""
Example of how to connect to a PhaseSpace motion capture system and feed the position to a
Crazyflie. This script uses ROS2 to subscribe to PhaseSpace data and provides
position and orientation data to the Crazyflie's state estimator.
The script uses the high level commander to upload a trajectory to fly a figure 8.

Set the uri to the radio settings of the Crazyflie and modify the
PhaseSpace rigid body ID matching your system.

axes in PhaseSpace:
- X(left) Y(up) Z(front) - right-handed coordinate system
- Crazyflie strictly uses ENU coordinate system: X(east) Y(north) Z(up) - right-handed coordinate system
- This script automatically performs coordinate transformation to ensure position and attitude data are correctly passed to Crazyflie
"""
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from phasespace_msgs.msg import Rigids, Rigid
from geometry_msgs.msg import Quaternion

import numpy as np
from scipy.spatial.transform import Rotation as R


# PhaseSpace rigid body ID that represents the Crazyflie
phasespace_rigid_body_id = 1

# PhaseSpace to ENU coordinate system conversion
# Enable this option to ensure PhaseSpace data is correctly converted to ENU coordinate system required by Crazyflie
# PhaseSpace: X(left) Y(up) Z(front) -> ENU: X(east) Y(north) Z(up)
ENABLE_COORDINATE_TRANSFORM = True

# True: send position and attitude; False: only send position
# It is recommended to use full attitude data for better flight performance
# In the debugging stage, disable attitude data and only use position data
send_full_pose = False  # temporarily set to False for debugging


# Data collection list - used for 3D trajectory drawing
phasespace_positions = []  # PhaseSpace original position data
crazyflie_positions = []   # Crazyflie state estimated position data
timestamps = []            # timestamps

# Backup data collection flag
fallback_data_collection_active = False


class PhaseSpaceWrapper(Thread):
    def __init__(self, rigid_body_id, enable_coordinate_transform=True, send_full_pose=False):
        Thread.__init__(self)

        self.rigid_body_id = rigid_body_id
        self.enable_coordinate_transform = enable_coordinate_transform
        self.send_full_pose = send_full_pose
        self.on_pose = None
        self._stay_open = True
        self.current_position = [0, 0, 0.041]

        # Initialize ROS2 node
        # rclpy.init()
        self.node = Node('phasespace_wrapper')
        
        # Subscribe to PhaseSpace data
        self.phasespace_sub = self.node.create_subscription(
            Rigids, '/phasespace_rigids', self.phasespace_callback, 10)

        self.start()

    def close(self):
        self._stay_open = False
        # rclpy.shutdown()

    def phasespace_callback(self, msg):
        """Handle PhaseSpace rigid body data with position and rotation"""
        for rigid in msg.rigids:
            if rigid.id == self.rigid_body_id and rigid.cond > 0:
                # Transform coordinates if enabled
                if self.enable_coordinate_transform:
                    # PhaseSpace coordinate system: X(left) Y(up) Z(front) - right-handed coordinate system
                    # Crazyflie ENU coordinate system: X(east) Y(north) Z(up) - right-handed coordinate system
                    
                    # position conversion (millimeters to meters)
                    x = rigid.x / 1000.0    # left -> east (adjust symbol according to actual situation)
                    y = -rigid.z / 1000.0   # front -> north (adjust symbol according to actual situation)
                    z = rigid.y / 1000.0    # up -> up
                    
                    # quaternion attitude conversion
                    q_ps = np.array([rigid.qx, rigid.qy, rigid.qz, rigid.qw])
                    
                    # create coordinate transformation matrix (PhaseSpace -> ENU)
                    T_ps_to_enu = np.array([
                        [ 1,  0,  0],  # left -> east (keep original direction, no inversion)
                        [ 0,  0, -1],  # front -> north (Z -> Y, invert)
                        [ 0,  1,  0]   # up -> up (Y -> Z)
                    ])
                    
                    # convert quaternion to rotation matrix
                    R_ps = R.from_quat(q_ps).as_matrix()
                    
                    # apply coordinate transformation: R_enu = T * R_ps * T^T
                    R_enu = T_ps_to_enu @ R_ps @ T_ps_to_enu.T
                    # convert back to quaternion (scipy uses [x, y, z, w] format)
                    q_enu = R.from_matrix(R_enu).as_quat()

                    if self.send_full_pose:
                        qx, qy, qz, qw = q_enu
                    else:
                        qx = qy = qz = 0.0
                        qw = 1.0
                    
                else:
                    # do not perform coordinate transformation, use original data (not recommended)
                    x = rigid.x / 1000.0
                    y = rigid.y / 1000.0
                    z = rigid.z / 1000.0
                    qx = rigid.qx
                    qy = rigid.qy
                    qz = rigid.qz
                    qw = rigid.qw

                self.current_position = [x, y, z]

                if self.on_pose:
                    quaternion = Quaternion()
                    quaternion.w = qw
                    quaternion.x = qx
                    quaternion.y = qy
                    quaternion.z = qz
                    self.on_pose([x, y, z, quaternion])

    def run(self):
        """Main thread loop"""
        print("PhaseSpace thread started...")
        while self._stay_open:
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as e:
                print(f"PhaseSpace thread error: {e}")
        print("PhaseSpace thread stopped")