from setuptools import setup

package_name = 'respeaker_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kide Vuojärvi',
    maintainer_email='kide.vuojarvi@unikie.com',
    description='ROS2 node for audio recording',
    license='BDS',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mic_array = respeaker_node.respeaker_node:main',
            'storage = respeaker_node.audio_storage:main'
        ],
    },
)
