from setuptools import setup

package_name = 'mtc_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_data={package_name: ['resources/*']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mtc_gui_client.launch.py']),
    ],
    install_requires=['pyyaml'],
    maintainer='abondada',
    maintainer_email='abondada@bnl.gov',
    description='Graphical User Interface for beambot pipeline',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mtc_gui_client = mtc_gui.main:main',
        ],
    },
)
