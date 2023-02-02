from setuptools import setup

package_name = 'facial'

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
    maintainer='Minsu Jang',
    maintainer_email='minsu@etri.re.kr',
    description='AAI4R Facial Attribute Detection Module',
    license='AAI4R License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'facial_node = facial.facial_node:main',
            'facial_model_download = facial.model_download:main'
        ],
    },
)
