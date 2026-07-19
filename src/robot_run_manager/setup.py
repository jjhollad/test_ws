from setuptools import find_packages, setup


package_name = 'robot_run_manager'


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
    maintainer='jjhollad',
    maintainer_email='jjhollad@todo.todo',
    description='Desktop GUI for managing robot data-collection runs.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_manager_gui = robot_run_manager.main:main',
            'validate_run = robot_run_manager.dataset:main',
            'install_desktop_launcher = robot_run_manager.desktop:main',
            'frontier_mapper = robot_run_manager.frontier_mapper:main',
        ],
    },
)
