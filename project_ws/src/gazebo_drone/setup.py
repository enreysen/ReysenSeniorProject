from setuptools import setup

package_name = 'gazebo_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_1.py']),
        ('share/' + package_name + '/launch', ['launch/launch_3.py']),
        ('share/' + package_name + '/launch', ['launch/launch_2.py']),
        ('share/' + package_name + '/worlds', ['worlds/challenge_1.world']),
        ('share/' + package_name + '/worlds', ['worlds/challenge_3.world']),
        ('share/' + package_name + '/worlds', ['worlds/challenge_2.world']),
        ('share/' + package_name + '/launch', ['launch/test.py']),
        ('share/' + package_name + '/worlds', ['worlds/test.world'])
    ],

    zip_safe=True,
    maintainer='enreysen',
    maintainer_email='enreysen@go.olemiss.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # dependencies that need to be installed
    install_requires=['setuptools', 'rclpy', 'numpy', 'opencv-python', 'cv_bridge', 'haversine', 'pyquaternion'],
    entry_points={
        'console_scripts': [
        'challenge_1 = gazebo_drone.challenge_1:main'
        ],
    },
)
