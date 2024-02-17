from setuptools import setup

package_name = 'traffic_light_task'

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
    maintainer='lion',
    maintainer_email='sl.doncel@pm.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_segmentator_node = traffic_light_task.color_segmentator:main',
            'distance_estimator_node = traffic_light_task.distance_estimator:main',
        ],
    },
)
