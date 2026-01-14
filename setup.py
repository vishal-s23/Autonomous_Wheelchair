from setuptools import find_packages, setup

package_name = 'wheelchair_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/wheelchair_simulation']),
    ('share/wheelchair_simulation', ['package.xml']),
    ('share/wheelchair_simulation/urdf',
        ['urdf/wheelchair.urdf.xacro']),
    ('share/wheelchair_simulation/launch',
        ['launch/spawn_wheelchair.launch.py']),
    ('share/wheelchair_simulation/config',
        ['config/bridge.yaml']),
],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vishal',
    maintainer_email='https://github.com/vishal-s23',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
    'wasd_teleop = wheelchair_simulation.wasd_teleop:main',
],

    },
)
