from setuptools import find_packages, setup

package_name = 'fusion'

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
    maintainer='blaine',
    maintainer_email='Blaine.Oania@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_pub = fusion.img_pub:main',
            'fuser = fusion.fuser:main',
            'line_follow = fusion.line_follow:main',
            'costmap = fusion.costmap:main',
            'lane_costmap = fusion.lane_costmap:main',
        ],
    },
)
