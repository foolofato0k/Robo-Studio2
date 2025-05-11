from setuptools import find_packages, setup

package_name = 'robo_da_vinci'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/run_all.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='william',
    maintainer_email='william.l.foreman@student.uts.edu.au',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ← updated to point inside the py_planning subpackage:
            'processing_node = robo_da_vinci.py_planning.processing_node:main',
            # gui_node stays the same, since gui/gui_node.py lives under robo_da_vinci/gui/
            'gui_node        = robo_da_vinci.gui.gui_node:main',
        ],
    },
)
