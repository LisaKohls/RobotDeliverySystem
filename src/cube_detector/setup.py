from setuptools import find_packages, setup

package_name = 'cube_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['cube_detector/best.onnx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lisa',
    maintainer_email='lisa@todo.todo',
    description='Cube detector ros2 node integration',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		"detector_node = cube_detector.detector_node:main",
        ],
    },
)
