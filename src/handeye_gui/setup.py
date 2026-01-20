from setuptools import find_packages, setup

package_name = 'handeye_gui'

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
    description='PyQt5 GUI for hand-eye calibration workflows.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'handeye_gui = handeye_gui.app:main',
        ],
    },
)
