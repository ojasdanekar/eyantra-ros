from setuptools import find_packages, setup

package_name = 'ps3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ojas',
    maintainer_email='ojasdanekar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['goal_publisher = ps3.goal_publisher:main',
                            'go_to_goal = ps3.go_to_goal:main',
        ],
    },
)
