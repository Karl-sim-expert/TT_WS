from setuptools import find_packages, setup

package_name = 'tt_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tt maintainer',
    maintainer_email='wjsquddn1728@gmail.com',
    description='Stroke trajectory generation and execution for the table tennis robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'stroke_executor = tt_motion.stroke_executor:main',
            'hit_static_ball = tt_motion.hit_static_ball:main',
            'return_ball = tt_motion.return_ball:main',
        ],
    },
)
