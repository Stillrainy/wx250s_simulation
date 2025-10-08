"""
wx250s_simulation - A Python package for simulating the WX250s robotic arm.

This package provides an interface to control and simulate the WX250s robotic arm
using URDF-based forward kinematics. It supports joint position control,
end-effector pose calculation, and visualization.

Classes:
    wx250s: Main robot simulation class for the WX250s robotic arm
"""

from .robot import wx250s

__version__ = "0.1.0"
__author__ = "Your Name"
__email__ = "your.email@example.com"

# Define what gets imported with "from wx250s_simulation import *"
__all__ = ['wx250s']