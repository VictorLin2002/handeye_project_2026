from setuptools import setup

package_name = 'rtde_controller'

setup(
    name='rtde-controller',     # distribution name (keeps dash for metadata)
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/action', ['action/MoveToPose.action']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Victor',
    maintainer_email='example@example.com',
    description='RTDE motion control with ROS2 Action interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtde_action_node = rtde_controller.rtde_node:main',
            'move_client = rtde_controller.move_client:main',
        ],
    },
)