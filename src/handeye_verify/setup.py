from setuptools import find_packages, setup

package_name = 'handeye_verify'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Victor',
    maintainer_email='victor@todo.todo',
    description='Hand-eye verification tools packaged for ROS 2.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'verify_handeye_touch = handeye_verify.verify_touch:main',
            'verify_repeatability = handeye_verify.verify_repeatability:main',
            'verify_dynamic_consistency = handeye_verify.verify_dynamic_consistency:main',
        ],
    },
)
