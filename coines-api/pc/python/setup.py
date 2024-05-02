#!/usr/bin/env python3

"""Python wrapper for coinesAPI

`coinespy` allows users to access the Bosch Sensortec Application Board using Python

- Control VDD and VDDIO of sensor
- Configure SPI and I2C bus parameters
- Read and write into registers of sensors from Bosch Sensortec via SPI and I2C
- Read and write digital pins of the Application Board.

"""
import os
import shutil
import setuptools


def check_if_path_exists(path):
    """
    Check if path exists
    Args:
    path(str): Path exist / not
    """
    for out in path:
        if not os.path.exists(os.path.join(os.getcwd(), out)):
            return False
    return True


def copy_folder(source, destination):
    """
    Copy the folder from source to destination
    """
    if os.path.exists(source):
        shutil.copytree(source, destination, dirs_exist_ok=True)


def copy_file(source_file, destination):
    """
    Copy the file from source_file to destination
    """
    if os.path.exists(source_file):
        if not os.path.exists(destination):
            shutil.os.makedirs(destination)
        shutil.copy2(source_file, destination)


def copy_dependencies():
    """
    Copy supporting folders for python package
    """
    copy_folder("../../../firmware/app2.0/coines_bridge", "coinespy/firmware/app2.0/coines_bridge")
    copy_folder("../../../firmware/app3.0/coines_bridge", "coinespy/firmware/app3.0/coines_bridge")
    copy_folder("../../../firmware/app3.1/coines_bridge", "coinespy/firmware/app3.1/coines_bridge")
    copy_folder("../../../firmware/nicla/coines_bridge", "coinespy/firmware/nicla/coines_bridge")
    copy_folder("../../../tools/usb-dfu", "coinespy/tools/usb-dfu")
    copy_file("../../../tools/app20-flash/app20-flash.exe", "coinespy/tools/app20-flash")
    copy_file("../../../tools/app_switch/app_switch.exe", "coinespy/tools/app_switch")
    copy_file("../../../tools/app_switch/README.md", "coinespy/tools/app_switch")
    copy_folder("../../../tools/openocd", "coinespy/tools/openocd")
    copy_folder("../../../coines-api/pc/comm_driver/libusb-1.0", "coinespy/tools/libusb-1.0")
    if check_if_path_exists(["../../../driver"]):
        copy_folder("../../../driver", "coinespy/driver")
    else:
        copy_folder("../../../_installer_/Linux_specific/driver", "coinespy/driver/Linux")
        copy_folder("../../../_installer_/Linux_specific/driver", "coinespy/driver/MacOS")


DOCLINES = (__doc__ or "").split("\n")

copy_dependencies()
setuptools.setup(
    name="coinespy",
    version="1.0.2",
    author="Bosch Sensortec GmbH",
    author_email="contact@bosch-sensortec.com",
    url="https://www.bosch-sensortec.com/",
    packages=["coinespy"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 2",
        "License :: OSI Approved :: BSD License",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS :: MacOS X",
    ],
    python_requires=">=2.6, <4",
)
