from setuptools import setup

package_name = 'aai4r_camera'

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
    maintainer='msjang',
    maintainer_email='minsu@etri.re.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = aai4r_camera.camera_node:main',
            'robot_info_subscriber = aai4r_camera.robot_info_subscriber:main',
            'showimg = aai4r_camera.showimg:main'
        ],
    },
)
