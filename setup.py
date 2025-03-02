#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

setup(
    name="ros_data_recorder",
    version="0.1.0",
    description="ROS数据记录器和数据处理器",
    author="青龙团队",
    packages=find_packages(),
    # install_requires=[
    #     "rospy>=1.15.0",
    #     "numpy>=1.18.0",
    #     "pandas>=1.0.0",
    #     "h5py>=2.10.0",
    #     "opencv-python>=4.2.0",
    #     "opencv-contrib-python>=4.2.0",
    #     "PyQt5>=5.15.0",
    #     "tqdm>=4.45.0",
    #     "cv_bridge>=1.13.0",
    # ],
    scripts=['scripts/ros_data_recorder_gui'],
    python_requires='>=3.6',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Science/Research',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
    ],
) 