from setuptools import find_packages, setup

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'haarcascade_frontalface_default.xml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jesse',
    maintainer_email='jesse.gonzalez@student.uts.edu.au',
    description='Camera GUI + photo_publisher node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = gui.gui_node:main'
        ],
    },
)
