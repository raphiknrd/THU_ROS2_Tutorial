from setuptools import setup

package_name = 'kr_example_python'

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
    maintainer='Martin Vybiralik',
    maintainer_email='mvy@kassowrobots.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state = kr_example_python.robot_state:main',
            'follow_joint = kr_example_python.follow_joint:main',
            'jog_linear = kr_example_python.jog_linear:main',
            'move_joint = kr_example_python.move_joint:main',
            'self_motion = kr_example_python.self_motion:main',
            'move_linear_by_button = kr_example_python.move_linear_by_button:main',
            'vier_gewinnt = kr_example_python.vier_gewinnt:main',
        ],
    },
)
