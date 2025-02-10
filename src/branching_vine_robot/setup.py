import os
from glob import glob
from setuptools import find_packages, setup

package_name = "branching_vine_robot"

setup(
    name=package_name,
    version="0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'data'), glob(os.path.join('data', '*'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Liam Bray",
    description="Branching Vine Robot Code",
    license="MIT",
    tests_require=["pytest"],
    entry_points={ # All scripts that run standalone should be here if run through the launch file
        'console_scripts': [
            'state = branching_vine_robot.state:main',
            'server = branching_vine_robot.server:main',
            'cluster = branching_vine_robot.cluster:main',
            'gui = branching_vine_robot.gui:main',
            'plot = branching_vine_robot.plot:main'
        ],
    }
)