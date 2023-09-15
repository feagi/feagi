from setuptools import setup

package_name = 'py_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FEAGI',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'listener = py_topic.ros_laser_scan:main',
             'sonar_sensor = py_topic.HC_SR04_Foxy:main', #This is the one you use the sonar sensor.
		'py_laser_scan = py_topic.ros_laser_scan:main', #This is the original ros_laser_scan
		'micro_ros = py_topic.micro_ros:main',
		'py2arduino = py_topic.py2arduino:main'
        ], #Once you add this in your setup.py in ros_ws/src/py_topic/ and update your workspace.
    }, #Then you can run like ros2 run py_topic 
)
