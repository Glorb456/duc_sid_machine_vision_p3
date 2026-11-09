from setuptools import find_packages, setup

package_name = 'machine_vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # install calibration.npz next to the scripts so they can load it from their own folder
    package_data={package_name: ['calibration.npz']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siddhant',
    maintainer_email='glorb458@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = machine_vision_pkg.vision_node:main',
            'apriltag_viewer = machine_vision_pkg.apriltag_viewer:main',
            'localization_node = machine_vision_pkg.localization_node:main'
        ],
    },
)
