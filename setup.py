from setuptools import setup, find_packages

package_name = 'robot_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    package_dir={'': 'scripts'}
    install_requires=['setuptools'],
    zip_safe=True,
    author='Arshad Mehmood',
    author_email='arshad.mehmood@intel.com',
    maintainer='Arshad Mehmood',
    maintainer_email='arshad.mehmood@intel.com',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal subscribers using rclpy.',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            
        ],
    },
)
