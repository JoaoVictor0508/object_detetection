from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nó para o primeiro RPLidar
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_node',
        #     name='rplidar_node_1',
        #     output='screen',
        #     parameters=[
        #         {'serial_port': '/dev/rplidar_right'},
        #         {'frame_id': 'laser1_frame'},
        #         {'angle_compensate': True},
        #         {'scan_mode': 'Boost'},
        #         {'serial_baudrate': 115200}],
        #     remappings=[
        # ('/scan', '/scan_right')   # Tópico publicado pelo LIDAR direito
        #     ]
        # ),
        # Nó para o segundo RPLidar
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_2',
            output='screen',
            parameters=[
                {'serial_port': '/dev/rplidar_left'},
                {'frame_id': 'laser2_frame'},
                {'angle_compensate': True},
                {'scan_mode': 'Boost'},
                {'serial_baudrate': 115200}],
            remappings=[
                ('/scan', '/scan_left')   # Tópico publicado pelo LIDAR direito
            ]
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_2',
            output='screen',
            parameters=[
                {'serial_port': '/dev/rplidar_right'},
                {'frame_id': 'laser2_frame'},
                {'angle_compensate': True},
                {'inverted': False},
                {'scan_mode': 'Standard'},
                {'serial_baudrate': 1000000}],
            remappings=[
                ('/scan', '/scan_right')   # Tópico publicado pelo LIDAR direito
            ]
        )
    ])

