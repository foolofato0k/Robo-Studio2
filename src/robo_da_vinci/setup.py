from setuptools import find_packages, setup

package_name = 'robo_da_vinci'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Removed launch file section
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='william',
    maintainer_email='william.l.foreman@student.uts.edu.au',
    description='ROS2 package for GUI and planning for Robo-Da-Vinci project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processing_node = robo_da_vinci.py_planning.processing_node:main',
        ],
    },
)
