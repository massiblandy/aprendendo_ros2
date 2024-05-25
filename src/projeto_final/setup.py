from setuptools import find_packages, setup

package_name = 'projeto_final'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2d2 = projeto_final.r2d2:main',
            'robot = projeto_final.robot:main',
            'A_Star = projeto_final.A_Star:main',
            'teste = projeto_final.teste:main'
        ],
    },
)
