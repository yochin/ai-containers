from setuptools import setup

package_name = 'meal'

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
    description='AAI4R Meal Context Understanding',
    license='AAI4R License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'meal_node = meal.meal_node:main',
            'meal_model_download = meal.meal_model_download:main'
        ],
    },
)
