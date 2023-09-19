from setuptools import setup

package_name = 'box_measurement_3d'

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
    maintainer='batman',
    maintainer_email='hanifdiyer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'box_measurement_3d_node = box_measurement_3d.box_measurement_3d_node:main',
             'send_pc = box_measurement_3d.send_pc:main',
             'publish_pointcloud_from_file = box_measurement_3d.publish_pointcloud_from_file:main'
        ],
    },
)
