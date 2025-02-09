from setuptools import setup

package_name = 'slm_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_file.launch.py']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A ROS package that integrates the distilled SLM for inference',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slm_node = slm_ros.slm_node:main',
        ],
    },
)
