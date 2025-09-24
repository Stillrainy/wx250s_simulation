# WX250s Robotic Arm Simulation

This project provides simulation and control capabilities for the Interbotix WX250s robotic arm using Python robotics libraries. The project includes URDF models, 3D visualizations, and forward kinematics calculations.

## Features

- **Robot Model**: Complete URDF model of the WX250s robotic arm with 6 DOF + gripper
- **3D Visualization**: Interactive robot visualization using Urchin
- **Forward Kinematics**: Joint angle to end-effector pose calculations

## Prerequisites

- **Operating System**: macOS, Linux, or Windows
- **Python**: 3.10+ (recommended 3.10 for best compatibility)

## Installation

### 1. Clone/Download the Project

Ensure you have the project files in your working directory with the following structure:
```
wx250s_simulation/
├── environment.yml
├── requirements.txt
├── robot.py
├── test_robot.ipynb
└── assets/
    ├── wx250s.urdf
    └── meshes/
        └── *.stl files
```

### 2. Environment Setup

Choose one of the following installation methods:

#### Option A: Using Conda (Recommended)

Navigate to the project directory and create the conda environment:

```bash
cd path/to/wx250s_simulation
conda env create -f environment.yml
conda activate wx250s_arm
```

This creates a new conda environment named `wx250s_arm` with:
- Python 3.10
- NumPy (<2.0)
- Spatialmath-python
- Jupyter Notebook
- Urchin

#### Option B: Using pip

If you prefer to use pip or don't have conda installed:

```bash
cd path/to/wx250s_simulation
# Optional: create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Example Usage

Activate your environment and run the following:

```python
import numpy as np
from robot import wx250s

robot = wx250s()

robot.set_joint_positions(np.deg2rad([0, -35, 60, 0, 65, 0]))
robot.get_ee_pose()
```

See `test_robot.ipynb` for more examples.

## Resources

- [Interbotix WX250s](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx250s.html)


## Acknowledgements
The URDF was adopted from the [interbotix_ros_manipulators](https://github.com/Interbotix/interbotix_ros_manipulators/tree/rolling/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf).

