from setuptools import find_packages, setup

package_name = 'ecosort_perception'

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
    maintainer='sebastian',
    maintainer_email='gaiborsebastian@gmail.com',
    description='Detección de objetos por color desde la cámara cenital de EcoSort',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector = ecosort_perception.color_detector:main',
        ],
    },
)
