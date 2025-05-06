from setuptools import setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dual_lidar_launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'rplidar'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seuemail@example.com',
    description='Pacote para detecção de objetos redondos a partir dos dados do RPLidar',
    license='MIT',
    entry_points={
        'console_scripts': [
            'object_detection_node = object_detection.object_detection_node:main',
        ],
    },
)
