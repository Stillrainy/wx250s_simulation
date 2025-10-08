"""Setup configuration for wx250s_simulation package."""

from setuptools import setup, find_packages
from pathlib import Path

# Read the README file
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text(encoding='utf-8')

# Read requirements
requirements = []
with open('requirements.txt', 'r') as f:
    requirements = [line.strip() for line in f if line.strip() and not line.startswith('#')]

setup(
    name="wx250s_simulation",
    version="0.1.0",
    author="Your Name",
    author_email="your.email@example.com",
    description="A Python package for simulating the WX250s robotic arm",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Stillrainy/wx250s_simulation",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
    python_requires=">=3.8",
    install_requires=requirements,
    include_package_data=True,
    package_data={
        'wx250s_simulation': ['assets/*', 'assets/meshes/*'],
    },
    zip_safe=False,
    keywords="robotics simulation wx250s urdf kinematics",
    project_urls={
        "Bug Reports": "https://github.com/Stillrainy/wx250s_simulation/issues",
        "Source": "https://github.com/Stillrainy/wx250s_simulation",
    },
)