from setuptools import setup

package_name = 'dhtt'

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
    maintainer='tylerjbecker',
    maintainer_email='tbecker@nevada.unr.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'experiment_1 = dhtt.experiment_1:main',
            'experiment_2 = dhtt.experiment_2:main',
            'experiment_3 = dhtt.experiment_3:main'
        ],
    },
)
