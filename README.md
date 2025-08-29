# RoKiSim-Revival

A comprehensive PySide6-based GUI application for controlling industrial robots through the RoKiSim simulator. This project provides real-time joint control, advanced motion planning, forward/inverse kinematics calculations, and an integrated robot programming environment.

![Python](https://img.shields.io/badge/Python-3.7%2B-blue)
![GUI](https://img.shields.io/badge/GUI-PySide6-green)
![Robotics](https://img.shields.io/badge/Robotics-Kinematics-orange)

## Features

- **Real-time Joint Control**: Smooth interpolation between joint positions with adjustable speed control
- **Forward Kinematics (FK)**: Calculate end-effector position and orientation from joint angles
- **Inverse Kinematics (IK)**: Calculate joint angles from desired end-effector pose
- **Robot Programming**: Integrated compiler for a custom robot programming language
- **Multiple Motion Types**: Support for joint, linear, and circular interpolation
- **Visual Feedback**: Speed gauges for each joint and real-time FK display
- **XML Robot Definitions**: Load custom robot definitions with DH parameters and joint limits
- **Cross-Platform**: Works on Windows, macOS, and Linux

## Installation

1. Clone the repository:
```bash
git clone https://github.com/Mostafasaad1/Rokisim-Revival.git
cd RoKiSim-Revival
```

2. Install the required dependencies:
```bash
pip install -r requirements.txt
```

## Prerequisites

- Python 3.7 or higher
- RoKiSim simulator (separate installation)
- The following Python packages:
  - PySide6
  - NumPy
  - Pybotics
  - lxml

## Usage

1. Start the RoKiSim simulator
2. Launch the application:
```bash
python joint_control_gui.py
```
3. Load a robot definition XML file using the "Load Robot XML" button
4. Control joints using sliders or direct value input
5. Use the FK section to view current end-effector position
6. Use the IK section to move to a specific pose
7. Write and compile programs in the Program Editor tab

## Robot Programming Language

The integrated compiler supports a custom language with commands including:

- `SETE`: Define positions (e.g., `SETE HOME, [0,0,0,0,0,0]`)
- `MOVJ`: Joint interpolation movement
- `MOVL`: Linear interpolation movement
- `MOVC`: Circular interpolation through a via point
- `FOR/NEXT`: Looping constructs
- `IF/THEN`: Conditional statements
- `SET`: Variable assignment

Example program:
```
SETE HOME, [0,0,0,0,0,0]
SETE P1, [10,20,30,40,50,60]
MOVJ HOME
MOVL P1
FOR i, 1, 3
  MOVJ P1
  MOVJ HOME
NEXT i
```

## Project Structure

```
RoKiSim-Revival/
├── joint_control_gui.py    # Main GUI application
├── rokisim_sender.py       # Core communication module
├── kinematics.py           # FK and IK implementations
├── instructionSet.py       # Robot programming compiler
├── main.py                 # Quick test script
├── requirements.txt        # Python dependencies
└── README.md              # This file
```

## XML Robot Definition Format

The application supports robot definition files in XML format:

```xml
<robot_dk name="TestRobot">
    <axis id="Joint1" alpha="0" a="0" theta="0" d="290" limsup="165" liminf="-165"/>
    <axis id="Joint2" alpha="-90" a="0" theta="0" d="0" limsup="110" liminf="-110"/>
    <!-- Additional joints -->
</robot_dk>
```

## Troubleshooting

- Ensure RoKiSim is running before starting the application
- Verify the XML file format matches expected structure
- Check that all required Python dependencies are installed
- If experiencing connection issues, verify the IP and port settings in `rokisim_sender.py`

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgments

- RoKiSim for providing the robot simulation environment
- Pybotics for kinematics calculations
- Qt/PySide6 for the GUI framework
