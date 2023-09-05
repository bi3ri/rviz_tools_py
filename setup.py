from setuptools import setup
import os
from glob import glob
package_name = 'rviz_tools_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'config'), glob('config/*.csv')),
        # ('lib/' + package_name, [package_name+'/SimpleOneTrailerSystem.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bi3ri',
    maintainer_email='bi3ri@gmx.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
