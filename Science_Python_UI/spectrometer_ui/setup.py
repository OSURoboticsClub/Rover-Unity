from setuptools import find_packages, setup
import os
from glob import glob

package_name = "spectrometer_ui"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[

        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aaron",
    maintainer_email="aaron1.goyal@gmail.com",
    description="Spectrometer and science mechanism groundstation UI",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "spectrometer_control_ui = "
            "spectrometer_ui.spectrometer_control_ui:main",
        ],
    },
)
