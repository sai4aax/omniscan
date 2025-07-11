from setuptools import find_packages, setup

package_name = 'omniscan_driver'
package_resource = 'omni_resource'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_resource], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sai',
    maintainer_email='k.sai@aquaairx.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_node = omniscan_driver.omniscan_450fs:main',
            'mock_sensor = omniscan_driver.mock_sensor:main',
            'omni_life = omniscan_driver.omniscan_lifecycle:main',
            'sonar2points = omniscan_driver.sonar_point_cloud_publisher:main',
            'dummy_tf = omniscan_driver.tf_publish:main' # this is not working correctly, use tf2 
        ],
    },
)
