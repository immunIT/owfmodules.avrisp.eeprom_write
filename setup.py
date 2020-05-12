#!/usr/bin/env python
# -*- coding:utf-8 -*-


from setuptools import setup, find_packages

__authors__ = "Jordan Ovrè, Paul Duncan"
__copyright__ = "Copyright (c) Jordan Ovrè / Paul Duncan"
__license__ = "GPLv3"
__version__ = "1.0.0"
__contact__ = "Jordan Ovrè / Ghecko <ghecko78@gmail.com>, Paul Duncan / Eresse <eresse@dooba.io>"

description = 'Write the AVR eeprom memory'
name = 'owfmodules.avrisp.eeprom_write'

setup(
    name=name,
    version=__version__,
    packages=find_packages(),
    license=__license__,
    description=description,
    author=__authors__,
    zip_safe=True,
    url='https://bitbucket.org/octowire/' + name,
    install_requires=[
        'hexformat>=0.2',
        'tqdm>=4.45.0'
    ],
    classifiers=[
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Programming Language :: Python :: 3',
        'Development Status :: 5 - Production/Stable'
    ],
    keywords=['octowire', 'framework', 'hardware', 'security', 'AVR', 'ISP', 'write', 'eeprom']
)