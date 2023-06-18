from setuptools import setup
import os

package_name = 'flask_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={package_name: ['templates/*.html', 'static/index.js', 'static/style.css']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flask_node = flask_node.web:main',
        ],
    },
)
