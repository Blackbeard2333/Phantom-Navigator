from setuptools import setup

package_name = 'visual_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/visual_detection_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='haocheng.meng@duke.edu',
    description='YOLO-based visual detection from RGB camera.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detector = visual_detection.yolo_detector:main',
        ],
    },
)
