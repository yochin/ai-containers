from setuptools import setup

package_name = 'dummybot_ipcam'

setup(
    name=package_name,
    version='0.0.1',
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
    description='Image Publisher from IP Camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ip_camera_node = aai4r_camera.ip_camera_node:main',
            'showimg = aai4r_camera.showimg:main',
        ],
    },
)
