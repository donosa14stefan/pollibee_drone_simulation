from setuptools import setup
import os
from glob import glob

package_name = 'pollibee_drone_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Pollibee drone simulation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_drone = scripts.control_drone:main',
            'yolo_detection = scripts.yolo_detection:main',
            'pollination_motor = scripts.pollination_motor:main',
            'visualize_results = scripts.visualize_results:main',
            'data_logging = scripts.data_logging:main',
        ],
    },
)
