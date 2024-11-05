from setuptools import setup

package_name = 'wheels_driver'

setup(
 name=package_name,
 version='0.0.1',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Akshet Patel',
 maintainer_email='akshet.patel@duckietown.com',
 description='ROS2 Wheel Driver Package',
 license='None',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
        "wheels_driver_node = wheels_driver.wheels_driver_node:main"
     ],
   },
)
