from setuptools import setup

package_name = 'redirection_commander'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install BOTH launch files (old indoor + new outdoor GPS)
        ('share/' + package_name + '/launch', [
            'launch/redirection_commander.launch.py',
            'launch/redirection_commander_gps.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CPSL',
    maintainer_email='you@example.com',
    description='Closed-loop redirection controller for UAV spoofing acceleration commands.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Indoor (existing)
            'redirection_commander = redirection_commander.redirection_commander_node:main',
            'redirection_target_playlist_sender = redirection_commander.redirection_target_playlist_sender:main',
            # Outdoor GPS (new)
            'redirection_commander_gps = redirection_commander.redirection_commander_gps:main',
            'redirection_socket_server = redirection_commander.redirection_socket_server:main',
        ],
    },
)
