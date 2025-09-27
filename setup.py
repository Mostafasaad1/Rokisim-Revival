from setuptools import setup, find_packages

setup(
    name="rokisim-revival",
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "PySide6",
        "numpy",
        "pybotics",
        "lxml",
    ],
    entry_points={
        "console_scripts": [
            "rokisim-gui = rokisim_revival.gui:main",
        ],
    },
    description="Revival of RoKiSim robot simulator GUI",
    author="Mostafa Saad",
    license="MIT",
)
