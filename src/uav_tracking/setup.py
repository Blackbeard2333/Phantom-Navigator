from setuptools import setup

package_name = 'uav_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/uav_tracking_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Object detection and tracking from LiDAR point clouds.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector = uav_tracking.detector:main',
            'tracker = uav_tracking.tracker:main',
        ],
    },
)
