import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    
    return LaunchDescription([
        Node(
            package='charger_server_ros2',
            namespace='',
            executable='charger_server_ros2',
            name='charger_service',
            output="screen",
            emulate_tty=True,
            respawn=True,
            parameters=[
                {
                    'PLC_IP_address': '192.168.1.1',
                    'PLC_Port_address': 8501,
                    'timeout': 10.0,
                    'frequency': 2.0,
                }
            ]
        ),

        Node(
            package='charger_fleet_client_ros2',
            namespace='',
            executable='charger_fleet_client_ros2',
            name='fleet_charger_client_node',
            output="screen",
            emulate_tty=True,
            respawn=True,
            parameters=[
                {
                    'fleet_name': 'amr_vdm',
                    'charger_name': 'charger001',
                    'charger_state_topic': '/charger_state',
                    'charging_trigger_server_name': '/charger_server',
                    'dds_domain': 52,
                    'dds_state_topic': 'charger_state',
                    'dds_charger_request_topic': 'charger_request',
                    'update_frequency': 5.0,
                    'publish_frequency': 1.0,
                }
            ]
        ),
    ])
