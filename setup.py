#!/usr/bin/env python

from setuptools import find_packages, setup


with open('README.md', 'r') as fh:
    long_description = fh.read()

setup(
    name='rfidreader',
    version='2.0.5',
    author='Brian Curnow',
    author_email='brian.curnow+rfidreader@gmail.com',
    description='A wrapper around various implementations of RFID readers (e.g. evdev, mfrc522).',
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
    python_requires='>=3.6',
    extras_require={
        'evdev': 'evdev',
        'mfrc522': [
            'mfrc522-reader@git+https://github.com/bcurnow/mfrc522-reader.git#egg=mfrc522-reader',
        ]

    },
)
