from setuptools import find_packages
from setuptools import setup

package_name = "myapp_tester"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
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
