# RoKiSim-Revival GUI Usage Guide

# GUI Usage Guide

This guide explains how to use the RoKiSim-Revival graphical user interface for controlling robots through the RoKiSim simulator.

## Launching the Application

1. Ensure you have Python and all dependencies installed
2. Start the RoKiSim simulator
3. Launch the GUI application:
```bash
python joint_control_gui.py
```

## Initial Setup

### Loading a Robot Definition

Before you can control a robot, you need to load its definition:

1. Click the "Load Robot XML" button in the Joint Control tab
2. Navigate to your robot definition XML file
3. Select the file and click "Open"
4. The interface will update with controls for all the robot's joints

## Joint Control Tab

### Global Speed Control

- Use the "Speed Factor" slider to adjust the overall movement speed
- Range: 0.1x to 2.0x of the default speed
- This affects all joint movements

### Individual Joint Control

For each joint, you have three control options:

1. **Slider**: Drag to set the joint angle
2. **Spin Box**: Enter precise angle values
3. **Reset Button**: Returns the joint to 0° position

The interface shows:
- Joint name and number
- Minimum and maximum allowed angles
- Current angle value

### Forward Kinematics Display

The FK section shows the current end-effector position and orientation:
- X, Y, Z coordinates (in mm)
- Roll, Pitch, Yaw angles (in degrees)
- These values update in real-time as you move joints

### Inverse Kinematics Control

To move the robot to a specific pose:

1. Enter the desired position (X, Y, Z) in the first three fields
2. Enter the desired orientation (Roll, Pitch, Yaw) in the next three fields
3. Click "Move to Pose (IK)"
4. The status label will show whether the movement was successful

## Program Editor Tab

### Writing Programs

Use the text editor to write robot programs with the supported language:

1. Define positions with `SETE`:
```
SETE HOME, [0,0,0,0,0,0]
SETE P1, [10,20,30,40,50,60]
```

2. Create movement commands:
```
MOVJ HOME    # Joint interpolation to HOME
MOVL P1      # Linear interpolation to P1
```

3. Add control structures:
```
FOR i, 1, 3  # Loop 3 times
  MOVJ P1
  MOVJ HOME
NEXT i
```

### Compiling and Running Programs

1. Write your program in the editor
2. Click "Compile" to check for errors
3. Adjust the "Step Delay" if needed (time between movements)
4. Click "Run" to execute the program
5. Use "Stop" to halt execution at any time

The output area shows:
- Compilation status
- Generated joint positions
- Execution progress

## Tips for Effective Use

1. **Start Slow**: Use lower speed factors when first testing a robot
2. **Check Limits**: Respect joint limits to avoid errors
3. **Test Programs**: Compile and test small program sections first
4. **Use FK/IK Together**: Use FK to find positions, then use IK to return to them
5. **Save Programs**: Copy your programs to a text file for later use

## Troubleshooting Common Issues

- **No Connection**: Ensure RoKiSim is running before launching the GUI
- **XML Errors**: Verify your robot definition file follows the correct format
- **IK Failures**: Some poses may be unreachable; try slightly different values
- **Program Errors**: Check the compiler output for specific error messages

## Advanced Features

- **Circular Movements**: Use `MOVC` with a via point for arc movements
- **Variables**: Use `SET` to create variables for dynamic programs
- **Conditionals**: Use `IF` statements to create decision-making programs
