from setuptools import setup

package_name = 'the_mtt_bringup'

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
    author='Alex Chorel-Campanozzi',
    maintainer='Alex',
    maintainer_email='alex.chorel-campanozzi@forces.gc.ca',
    description='',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'mtt_controller = the_mtt_bringup.mtt_controller:main',
        ],
    },
)