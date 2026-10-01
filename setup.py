from setuptools import find_packages, setup

package_name = 'tb3_maze_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Damit ROS2 das Package findet
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch-Files mit installieren
        ('share/' + package_name + '/launch',
            ['launch/maze.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dein Name',
    maintainer_email='deine@email.de',
    description='Wall-Following und Pledge für TurtleBot3 Labyrinthlösung',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Format: 'befehlsname = package.modul:funktion'
            'wall_following = tb3_maze_navigation.wall_following:main',
            'pledge         = tb3_maze_navigation.pledge:main',
        ],
    },
)
