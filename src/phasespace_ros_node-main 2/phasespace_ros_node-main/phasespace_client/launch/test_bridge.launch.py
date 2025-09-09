from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 桥接节点
    bridge_node = Node(
        package='phasespace_client',
        executable='phasespace_to_mocap_bridge',
        name='phasespace_to_mocap_bridge',
        output='screen',
        parameters=[{
            'rigid_body_mapping': '1:cf1,2:cf2,3:cf3',
            'frame_id': 'mocap'
        }],
    )

    # 测试节点
    test_node = Node(
        package='phasespace_client',
        executable='test_phasespace_bridge.py',
        name='phasespace_bridge_tester',
        output='screen',
    )

    return LaunchDescription([
        bridge_node,
        test_node
    ]) 