from setuptools import find_packages, setup

package_name = 'tcp_server'

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
    maintainer='pc-meccatronica',
    maintainer_email='pc-meccatronica@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [             #elenco nodi presenti nel pkg
             'tcp_server_node = tcp_server.tcp_server_node:main',    #nomenodo=nomepkg.nomecartella:main
        ],
    },
)
