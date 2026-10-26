from glob import glob

from setuptools import setup

package_name = 'llm_commander'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moe Sani',
    maintainer_email='50625788+moe-sani@users.noreply.github.com',
    description='Natural-language control of the SO-101 arm (sim and real) via a local LLM.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'commander = llm_commander.commander:main',
            'spawn_scene = llm_commander.spawn_scene:main',
            'spawn_person = llm_commander.spawn_person:main',
            'safety_check = llm_commander.safety:main',
        ],
    },
)
