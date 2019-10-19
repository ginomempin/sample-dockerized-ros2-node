from setuptools import find_packages
from setuptools import setup

target_packages = list()
target_packages.extend(find_packages(where='src'))

setup(
    name='myapp_tester',
    version='0.0.0',
    packages=target_packages,
    data_files=[
        ('share/myapp_tester', ['package.xml']),
    ],
    py_modules=[
        'myapp_tester.test_node',
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
    description='Communicates with the app node to check commands/services functionality.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'myapp_tester = myapp_tester.test_node:main'
        ],
    },
)
