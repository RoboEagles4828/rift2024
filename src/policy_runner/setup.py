from setuptools import setup

package_name = 'policy_runner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rl-games'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='sarnga.raj@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'runner = policy_runner.policy_runner:main',
            'odom = policy_runner.odom_test:main'
        ],
    },
)
