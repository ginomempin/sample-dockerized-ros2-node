from setuptools import find_packages
from setuptools import setup

target_packages = list()
target_packages.extend(find_packages(where='src'))

setup(
    name='myapp',
    version='0.0.0',
    packages=target_packages,
    data_files=[
        ('share/myapp', ['package.xml']),
    ],
    py_modules=[
        'myapp.app_node',
        'myapp.app',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Author',
    author_email='someone@somewhere.com',
    maintainer='Maintainer',
    maintainer_email='someone@somewhere.com',
    keywords=['ros', 'ros2'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Technological Advancement',
    ],
    description='ROS-ified App',
    license='MIT',
    entry_points={
        'console_scripts': [
            'myapp_node = myapp.app_node:main'
        ],
    },
)
