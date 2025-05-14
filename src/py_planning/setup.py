from setuptools import find_packages, setup

package_name = 'py_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'test_publish_node = py_planning.test_publish_node:main',
            'pose_planner_node = py_planning.pose_planner_node:main',
            'processing_node =   py_planning.processing_node:main'
        ],
    },
)
