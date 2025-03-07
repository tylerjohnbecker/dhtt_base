from setuptools import find_packages, setup

package_name = 'dhtt_cooking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'cooking_zoo'
    ],
    zip_safe=True,
    maintainer='stosh',
    maintainer_email='williampeterson@unr.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dhtt_cooking = dhtt_cooking.dhtt_cooking:main'
        ],
    },
)
