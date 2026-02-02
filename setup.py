"""
HAPTOS Platform Setup
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read long description from README
readme_file = Path(__file__).parent / "README.md"
if readme_file.exists():
    with open(readme_file, "r", encoding="utf-8") as f:
        long_description = f.read()
else:
    long_description = "HAPTOS - Simulation-first haptics platform for developers"

# Read version from package
version = "0.3.0"  # Phase 2 complete, Phase 3 in progress

setup(
    name="haptos",
    version=version,
    author="HAPTOS Team",
    author_email="haptos@anthropic.com",
    description="Simulation-first haptics platform for developers",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/anthropics/haptos",
    project_urls={
        "Bug Tracker": "https://github.com/anthropics/haptos/issues",
        "Documentation": "https://haptos.readthedocs.io",
        "Source Code": "https://github.com/anthropics/haptos",
    },
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Physics",
        "Topic :: System :: Hardware",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "torch>=1.10.0",
        "mujoco>=2.3.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=3.0.0",
            "black>=22.0.0",
            "flake8>=4.0.0",
            "mypy>=0.950",
        ],
        "docs": [
            "sphinx>=4.5.0",
            "sphinx-rtd-theme>=1.0.0",
            "sphinx-autodoc-typehints>=1.18.0",
        ],
        "hardware": [
            "pyserial>=3.5",
        ],
        "viz": [
            "matplotlib>=3.5.0",
            "PyQt5>=5.15.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "haptos-demo=haptos.quickstart:demo",
        ],
    },
    include_package_data=True,
    package_data={
        "haptos": ["*.json"],
    },
    zip_safe=False,
)
