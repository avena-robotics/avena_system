from setuptools import setup
from glob import glob
import os

package_name = 'rgb_diff'
yolo_detector = 'rgb_diff/yolo_detector'
models = 'rgb_diff/yolo_detector/models'
utils = 'rgb_diff/yolo_detector/utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, yolo_detector, models, utils],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        # (os.path.join('share', package_name), glob(os.path.join('rgb_diff', 'yolo_detector', 'yolo5s.pt'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_diff = rgb_diff.rgb_diff:main'
        ],
    },
)
