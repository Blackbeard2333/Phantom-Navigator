from setuptools import find_packages, setup

package_name = 'camera_lidar_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fusion_projection_launch.py']),
        ('share/' + package_name + '/launch', ['launch/fusion_frustum_filter_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpsl',
    maintainer_email='haocheng.meng@duke.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'projector = camera_lidar_fusion.projector:main',
            'frustum_filter = camera_lidar_fusion.frustum_filter:main',
            'frustum_cluster = camera_lidar_fusion.frustum_cluster:main',
            'point_cloud_tracker = camera_lidar_fusion.point_cloud_tracker:main',
        ],
    },
)
