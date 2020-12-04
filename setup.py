#!/usr/bin/env python

from setuptools import find_packages, setup


with open('README.md', 'r') as fh:
    long_description = fh.read()

setup(
    name='rfidreader',
    version='1.0.1',
    author='Brian Curnow',
    author_email='brian.curnow+rfidreader@gmail.com',
    description='A wrapper around evdev to read from an RFID reader that shows up like a keyboard.',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/bcurnow/rfid-reader',
    packages=find_packages(),
    classifiers=[
        'Programming Language :: Python :: 3.9',
        'License :: OSI Approved :: Apache Software License'
        'Operating System :: OS Independent',
        'Intended Audience :: Developers',
        'Natural Language :: English',
        'Topic :: Software Development :: Libraries :: Python Modules',

    ],
    python_requires='>=3.9',
    install_requires=[
        'evdev',
    ],
)
