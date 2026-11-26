from setuptools import find_packages, setup

package_name = 'maze_navigator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isabel Montenegro',
    maintainer_email='isabel.ruela@sou.inteli.edu.br',
    description='Navegação autônoma no labirinto do culling_games',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parte1 = maze_navigator.parte1:main',
            'parte2 = maze_navigator.parte2:main',
        ],
    },
)
