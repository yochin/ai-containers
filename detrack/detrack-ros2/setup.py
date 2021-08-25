from setuptools import setup

package_name = 'detrack'

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
    maintainer='Minsu Jang',
    maintainer_email='minsu@etri.re.kr',
    description='Multi-Object Detection and Tracking',
    license='AAI4R License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detrack_node = detrack.detrack_node:main',
            'detrack_client_test_node = detrack.detrack_client_test_node:main',
            'model_download = detrack.model_download:main'
        ],
    },
)
