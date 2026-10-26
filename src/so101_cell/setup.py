from glob import glob

from setuptools import setup

package_name = 'so101_cell'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moe Sani',
    maintainer_email='50625788+moe-sani@users.noreply.github.com',
    description='SO-101 arm cell: Gazebo simulation and real-robot driver with the same ROS interface.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_object_poses = so101_cell.sim_object_poses:main',
            'so101_driver = so101_cell.driver:main',
            'so101_calibrate = so101_cell.calibrate:main',
            'cell_camera = so101_cell.cell_camera:main',
        ],
    },
)
