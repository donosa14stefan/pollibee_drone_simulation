from setuptools import setup
import os
from glob import glob

package_name = 'pollibee_drone_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'plugins'), glob('plugins/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stefan',
    maintainer_email='donosa14stefan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_drone = pollibee_drone_simulation.scripts.control_drone:main',
            'yolo_detection = pollibee_drone_simulation.scripts.yolo_detection:main',
            'pollination_motor = pollibee_drone_simulation.scripts.pollination_motor:main',
            'visualize_results = pollibee_drone_simulation.scripts.visualize_results:main',
            'data_logging = pollibee_drone_simulation.scripts.data_logging:main',
        ],
    },
)
