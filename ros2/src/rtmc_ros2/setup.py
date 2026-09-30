from setuptools import find_packages, setup

package_name = 'rtmc_ros2'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.py', 'launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='RTMC maintainers',
    maintainer_email='maintainer@example.invalid',
    description='Optional ROS 2 adapter for the RTMC simulator',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'bridge = rtmc_ros2.bridge:main',
        'jog = rtmc_ros2.jog:main',
        'jog_for = rtmc_ros2.jog_for:main',
        'smoke = rtmc_ros2.smoke:main',
        'demo_smoke = rtmc_ros2.demo_smoke:main',
    ]},
)
