from setuptools import setup
from glob import glob

package_name = 'delrobo_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch files
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),

        # config files (IMPORTANT FIX)
        ('share/' + package_name + '/config',
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='delrobo',
    maintainer_email='delrobo@todo.todo',
    description='Robot bringup package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
