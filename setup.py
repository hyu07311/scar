from setuptools import setup
import os
from glob import glob

package_name = 'scar_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 가중치 파일 설치
        (os.path.join('share', package_name, 'weights'),
            glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scar',
    maintainer_email='scar@todo.todo',
    description='SCAR detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_node = scar_vision.person_node:main',
            'stair_node  = scar_vision.stair_node:main',
        ],
    },
)
