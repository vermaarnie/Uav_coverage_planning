from setuptools import find_packages, setup

package_name = 'my_uav_pkg'

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
    maintainer='arnieverma',
    maintainer_email='arnieverma@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "uav_c = my_uav_pkg.uav_client:main",
            "uav_s = my_uav_pkg.uav_server:main"
        ],
    },
)
