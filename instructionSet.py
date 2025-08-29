import re
import numpy as np
from math import cos, sin, pi, sqrt, acos, atan2

class EnhancedRobotCompiler:
    def __init__(self, interpolation_steps=10):
        self.positions = {}      # Stores position definitions
        self.variables = {}      # Stores variable values
        self.output = []         # Compiled joint positions
        self.labels = {}         # Label positions in code
        self.call_stack = []     # For CALL/RET handling
        self.current_pose = None # Current robot pose (position and orientation)
        self.default_interpolation_steps = interpolation_steps  # Default steps for interpolation
        
    def compile(self, source_code):
        # Preprocess the source code
        lines = self._preprocess(source_code)
        
        # First pass: collect labels and position definitions
        self._first_pass(lines)
        
        # Second pass: compile the program
        self._second_pass(lines)
        
        return np.array(self.output)

    def _preprocess(self, source_code):
        lines = []
        for line in source_code.split('\n'):
            # Remove comments
            line = line.split("'")[0].strip()
            if not line:
                continue
                
            # Handle labels
            if line.startswith('*'):
                lines.append(line)
            else:
                # Normalize whitespace in commands
                line = re.sub(r'\s+', ' ', line)
                lines.append(line)
                
        return lines

    def _first_pass(self, lines):
        current_line = 0
        for line in lines:
            if line.startswith('*'):
                # Found a label
                label_name = line[1:].strip()
                self.labels[label_name] = current_line
            else:
                parts = line.split(' ')
                command = parts[0]
                
                if command == 'SETE':
                    # Position definition
                    pos_name = parts[1].rstrip(',')
                    # Extract joint values from the array
                    array_str = ' '.join(parts[2:])
                    joint_values = self._parse_array(array_str)
                    self.positions[pos_name] = joint_values
                
                current_line += 1

    def _second_pass(self, lines):
        pc = 0  # Program counter
        while pc < len(lines):
            line = lines[pc]
            
            if line.startswith('*'):
                # Skip labels in execution
                pc += 1
                continue
                
            parts = line.split(' ')
            command = parts[0]
            
            if command == 'SETE':
                # Already processed in first pass
                pc += 1
                
            elif command == 'SET':
                var_name = parts[1].rstrip(',')
                value = self._evaluate_expression(' '.join(parts[2:]))
                self.variables[var_name] = value
                pc += 1
                
            elif command == 'INC':
                var_name = parts[1]
                self.variables[var_name] += 1
                pc += 1
                
            elif command == 'DEC':
                var_name = parts[1]
                self.variables[var_name] -= 1
                pc += 1
                
            elif command == 'MOVJ':
                # Joint interpolation - move directly to target
                pos_name = parts[1]
                target_pose = self.positions[pos_name]
                
                # If we have a current pose, we could add intermediate points for smooth motion
                if self.current_pose is not None:
                    self._generate_joint_interpolation(self.current_pose, target_pose)
                else:
                    self.output.append(target_pose)
                    
                self.current_pose = target_pose
                pc += 1
                
            elif command == 'MOVL':
                # Linear interpolation - straight line in Cartesian space
                pos_name = parts[1]
                target_pose = self.positions[pos_name]
                
                if self.current_pose is None:
                    raise ValueError("MOVL requires a current position. Add a MOVJ command first.")
                
                self._generate_linear_interpolation(self.current_pose, target_pose)
                self.current_pose = target_pose
                pc += 1
                
            elif command == 'MOVC':
                # Circular interpolation - circular arc through via point
                if len(parts) < 3:
                    raise ValueError("MOVC requires a via point and target point")
                    
                via_name = parts[1]
                target_name = parts[2]
                
                # Check if steps parameter is provided
                steps = self.default_interpolation_steps
                if len(parts) > 3:
                    try:
                        steps = int(parts[3])
                    except ValueError:
                        raise ValueError("MOVC steps parameter must be an integer")
                
                via_pose = self.positions[via_name]
                target_pose = self.positions[target_name]
                
                if self.current_pose is None:
                    raise ValueError("MOVC requires a current position. Add a MOVJ command first.")
                
                self._generate_circular_interpolation(self.current_pose, via_pose, target_pose, steps)
                self.current_pose = target_pose
                pc += 1
                
            elif command == 'JUMP':
                label_name = parts[1]
                pc = self.labels[label_name]
                
            elif command == 'CALL':
                label_name = parts[1]
                self.call_stack.append(pc + 1)
                pc = self.labels[label_name]
                
            elif command == 'RET':
                pc = self.call_stack.pop()
                
            elif command == 'FOR':
                # FOR loop implementation with compile-time unrolling
                loop_var = parts[1].rstrip(',')
                start_val = self._evaluate_expression(parts[2].rstrip(','))
                end_val = self._evaluate_expression(parts[3])
                
                # Find the matching NEXT instruction
                loop_start = pc + 1
                loop_end = self._find_matching_next(lines, pc)
                
                # Execute the loop by unrolling at compile time
                for i in range(start_val, end_val + 1):
                    self.variables[loop_var] = i
                    # Execute loop body
                    self._execute_block(lines[loop_start:loop_end])
                
                pc = loop_end + 1
                
            elif command == 'IF':
                condition = ' '.join(parts[1:])
                if self._evaluate_condition(condition):
                    pc += 1
                else:
                    # Find the next THEN or end of IF block
                    pc = self._find_next_then(lines, pc) + 1
                    
            elif command == 'THEN':
                # Just a marker, skip it
                pc += 1
                
            elif command == 'NEXT':
                # Should be handled by FOR loop
                pc += 1
                
            else:
                raise ValueError(f"Unknown command: {command}")

    def _generate_joint_interpolation(self, start, end, steps=None):
        # For joint interpolation, we can add intermediate points at the joint level
        if steps is None:
            steps = self.default_interpolation_steps
            
        for i in range(1, steps):
            t = i / steps
            intermediate = []
            for j in range(6):  # For each joint
                # Linear interpolation between joint angles
                value = start[j] + t * (end[j] - start[j])
                intermediate.append(value)
            self.output.append(intermediate)
        
        # Add the final target point
        self.output.append(end)

    def _generate_linear_interpolation(self, start, end, steps=None):
        # Convert joint angles to Cartesian coordinates (simplified)
        if steps is None:
            steps = self.default_interpolation_steps
            
        start_cartesian = self._joints_to_cartesian(start)
        end_cartesian = self._joints_to_cartesian(end)
        
        # Generate intermediate points in Cartesian space
        for i in range(1, steps + 1):
            t = i / steps
            intermediate_cartesian = []
            
            # Linear interpolation in Cartesian space
            for j in range(6):  # For each Cartesian coordinate
                value = start_cartesian[j] + t * (end_cartesian[j] - start_cartesian[j])
                intermediate_cartesian.append(value)
            
            # Convert back to joint angles (inverse kinematics)
            intermediate_joints = self._cartesian_to_joints(intermediate_cartesian)
            self.output.append(intermediate_joints)

    def _generate_circular_interpolation(self, start, via, end, steps=None):
        # Convert joint angles to Cartesian coordinates
        if steps is None:
            steps = self.default_interpolation_steps
            
        start_cartesian = self._joints_to_cartesian(start)
        via_cartesian = self._joints_to_cartesian(via)
        end_cartesian = self._joints_to_cartesian(end)
        
        # Calculate the plane and center of the circle
        circle_center, normal_vector, radius = self._calculate_circle_center(
            start_cartesian[:3], via_cartesian[:3], end_cartesian[:3]
        )
        
        if circle_center is None:
            # Fall back to linear interpolation if points are colinear
            self._generate_linear_interpolation(start, end, steps)
            return
        
        # Calculate start and end angles
        start_vector = np.array(start_cartesian[:3]) - np.array(circle_center)
        via_vector = np.array(via_cartesian[:3]) - np.array(circle_center)
        end_vector = np.array(end_cartesian[:3]) - np.array(circle_center)
        
        # Project vectors onto the plane
        normal = np.array(normal_vector)
        start_proj = start_vector - np.dot(start_vector, normal) * normal
        via_proj = via_vector - np.dot(via_vector, normal) * normal
        end_proj = end_vector - np.dot(end_vector, normal) * normal
        
        # Calculate angles
        start_angle = atan2(start_proj[1], start_proj[0])
        via_angle = atan2(via_proj[1], via_proj[0])
        end_angle = atan2(end_proj[1], end_proj[0])
        
        # Ensure we take the shortest path
        angle_diff = end_angle - start_angle
        if angle_diff > pi:
            end_angle -= 2 * pi
        elif angle_diff < -pi:
            end_angle += 2 * pi
            
        # Generate points along the circular arc
        for i in range(1, steps + 1):
            t = i / steps
            angle = start_angle + t * (end_angle - start_angle)
            
            # Calculate point on circle
            point_on_circle = circle_center + radius * np.array([
                cos(angle) * start_proj[0] / np.linalg.norm(start_proj),
                sin(angle) * start_proj[1] / np.linalg.norm(start_proj),
                0  # Simplified for demonstration
            ])
            
            # For simplicity, keep orientation linear
            orientation = []
            for j in range(3, 6):
                value = start_cartesian[j] + t * (end_cartesian[j] - start_cartesian[j])
                orientation.append(value)
            
            # Combine position and orientation
            intermediate_cartesian = list(point_on_circle) + orientation
            
            # Convert back to joint angles
            intermediate_joints = self._cartesian_to_joints(intermediate_cartesian)
            self.output.append(intermediate_joints)

    def _joints_to_cartesian(self, joints):
        # Simplified forward kinematics - in a real implementation, this would use
        # the robot's DH parameters to convert joint angles to Cartesian coordinates
        x = joints[0] * 0.1 + joints[1] * 0.2 + joints[2] * 0.3
        y = joints[1] * 0.1 + joints[2] * 0.2 + joints[3] * 0.3
        z = joints[2] * 0.1 + joints[3] * 0.2 + joints[4] * 0.3
        rx = joints[3]
        ry = joints[4]
        rz = joints[5]
        return [x, y, z, rx, ry, rz]

    def _cartesian_to_joints(self, cartesian):
        # Simplified inverse kinematics - in a real implementation, this would use
        # the robot's DH parameters to convert Cartesian coordinates to joint angles
        j1 = cartesian[0] * 0.5 - cartesian[1] * 0.2 + cartesian[2] * 0.1
        j2 = cartesian[1] * 0.5 - cartesian[2] * 0.2 + cartesian[0] * 0.1
        j3 = cartesian[2] * 0.5 - cartesian[0] * 0.2 + cartesian[1] * 0.1
        j4 = cartesian[3]
        j5 = cartesian[4]
        j6 = cartesian[5]
        return [j1, j2, j3, j4, j5, j6]

    def _calculate_circle_center(self, p1, p2, p3):
        # Convert to numpy arrays for easier math
        p1 = np.array(p1)
        p2 = np.array(p2)
        p3 = np.array(p3)
        
        # Check if points are colinear
        v1 = p2 - p1
        v2 = p3 - p1
        cross = np.cross(v1, v2)
        
        if np.linalg.norm(cross) < 1e-10:
            # Points are colinear, no unique circle
            return None, None, None
        
        # Calculate the plane normal
        normal = cross / np.linalg.norm(cross)
        
        # Calculate midpoints
        mid1 = (p1 + p2) / 2
        mid2 = (p2 + p3) / 2
        
        # Calculate direction vectors perpendicular to the sides
        dir1 = np.cross(v1, normal)
        dir2 = np.cross(v2, normal)
        
        # Solve for the center
        # The center lies at the intersection of the perpendicular bisectors
        A = np.array([dir1, -dir2]).T
        b = mid2 - mid1
        
        try:
            t = np.linalg.solve(A, b)
            center = mid1 + t[0] * dir1
            radius = np.linalg.norm(p1 - center)
            return center, normal, radius
        except np.linalg.LinAlgError:
            # Fallback if the matrix is singular
            return None, None, None

    def _execute_block(self, block_lines):
        # Execute a block of code (used for FOR loops)
        temp_pc = 0
        while temp_pc < len(block_lines):
            line = block_lines[temp_pc]
            
            if line.startswith('*'):
                temp_pc += 1
                continue
                
            parts = line.split(' ')
            command = parts[0]
            
            if command == 'SET':
                var_name = parts[1].rstrip(',')
                value = self._evaluate_expression(' '.join(parts[2:]))
                self.variables[var_name] = value
                temp_pc += 1
                
            elif command == 'INC':
                var_name = parts[1]
                self.variables[var_name] += 1
                temp_pc += 1
                
            elif command == 'DEC':
                var_name = parts[1]
                self.variables[var_name] -= 1
                temp_pc += 1
                
            elif command == 'MOVJ':
                pos_name = parts[1]
                target_pose = self.positions[pos_name]
                
                if self.current_pose is not None:
                    self._generate_joint_interpolation(self.current_pose, target_pose)
                else:
                    self.output.append(target_pose)
                    
                self.current_pose = target_pose
                temp_pc += 1
                
            elif command == 'MOVL':
                pos_name = parts[1]
                target_pose = self.positions[pos_name]
                
                if self.current_pose is None:
                    raise ValueError("MOVL requires a current position. Add a MOVJ command first.")
                
                self._generate_linear_interpolation(self.current_pose, target_pose)
                self.current_pose = target_pose
                temp_pc += 1
                
            elif command == 'MOVC':
                if len(parts) < 3:
                    raise ValueError("MOVC requires a via point and target point")
                    
                via_name = parts[1]
                target_name = parts[2]
                
                # Check if steps parameter is provided
                steps = self.default_interpolation_steps
                if len(parts) > 3:
                    try:
                        steps = int(parts[3])
                    except ValueError:
                        raise ValueError("MOVC steps parameter must be an integer")
                
                via_pose = self.positions[via_name]
                target_pose = self.positions[target_name]
                
                if self.current_pose is None:
                    raise ValueError("MOVC requires a current position. Add a MOVJ command first.")
                
                self._generate_circular_interpolation(self.current_pose, via_pose, target_pose, steps)
                self.current_pose = target_pose
                temp_pc += 1
                
            elif command == 'IF':
                condition = ' '.join(parts[1:])
                if self._evaluate_condition(condition):
                    temp_pc += 1
                else:
                    # Find the next THEN in the block
                    then_pos = self._find_next_then_in_block(block_lines, temp_pc)
                    temp_pc = then_pos + 1
                    
            elif command == 'THEN':
                temp_pc += 1
                
            else:
                # For simplicity, we don't support nested loops in this implementation
                temp_pc += 1

    def _evaluate_expression(self, expr):
        # Simple expression evaluator
        expr = expr.strip()
        
        # Check if it's a variable
        if expr in self.variables:
            return self.variables[expr]
            
        # Check if it's a number
        try:
            return int(expr)
        except ValueError:
            pass
            
        # Check if it's a simple arithmetic expression
        if '+' in expr:
            parts = expr.split('+')
            return self._evaluate_expression(parts[0]) + self._evaluate_expression(parts[1])
        elif '-' in expr:
            parts = expr.split('-')
            return self._evaluate_expression(parts[0]) - self._evaluate_expression(parts[1])
        elif '*' in expr:
            parts = expr.split('*')
            return self._evaluate_expression(parts[0]) * self._evaluate_expression(parts[1])
        elif '/' in expr:
            parts = expr.split('/')
            return self._evaluate_expression(parts[0]) / self._evaluate_expression(parts[1])
            
        raise ValueError(f"Cannot evaluate expression: {expr}")

    def _evaluate_condition(self, condition):
        # Simple condition evaluator
        if '==' in condition:
            parts = condition.split('==')
            return self._evaluate_expression(parts[0]) == self._evaluate_expression(parts[1])
        elif '!=' in condition:
            parts = condition.split('!=')
            return self._evaluate_expression(parts[0]) != self._evaluate_expression(parts[1])
        elif '<' in condition:
            parts = condition.split('<')
            return self._evaluate_expression(parts[0]) < self._evaluate_expression(parts[1])
        elif '>' in condition:
            parts = condition.split('>')
            return self._evaluate_expression(parts[0]) > self._evaluate_expression(parts[1])
        elif '<=' in condition:
            parts = condition.split('<=')
            return self._evaluate_expression(parts[0]) <= self._evaluate_expression(parts[1])
        elif '>=' in condition:
            parts = condition.split('>=')
            return self._evaluate_expression(parts[0]) >= self._evaluate_expression(parts[1])
            
        # If no operator, treat as boolean
        return bool(self._evaluate_expression(condition))

    def _parse_array(self, array_str):
        # Parse array string like [j1,j2,j3,j4,j5,j6]
        array_str = array_str.strip()
        if not (array_str.startswith('[') and array_str.endswith(']')):
            raise ValueError(f"Invalid array format: {array_str}")
            
        # Remove brackets and split by commas
        values_str = array_str[1:-1]
        values = [float(v.strip()) for v in values_str.split(',')]
        
        if len(values) != 6:
            raise ValueError("Joint positions must have exactly 6 values")
            
        return values

    def _find_matching_next(self, lines, for_pc):
        # Find the matching NEXT for a FOR loop
        depth = 1
        pc = for_pc + 1
        
        while pc < len(lines) and depth > 0:
            line = lines[pc]
            if line.startswith('*'):
                pc += 1
                continue
                
            parts = line.split(' ')
            command = parts[0]
            
            if command == 'FOR':
                depth += 1
            elif command == 'NEXT':
                depth -= 1
                
            pc += 1
            
        if depth != 0:
            raise ValueError("Unmatched FOR loop")
            
        return pc - 1

    def _find_next_then(self, lines, if_pc):
        # Find the THEN statement for an IF condition
        pc = if_pc + 1
        
        while pc < len(lines):
            line = lines[pc]
            if line.startswith('*'):
                pc += 1
                continue
                
            parts = line.split(' ')
            if parts[0] == 'THEN':
                return pc
                
            pc += 1
            
        raise ValueError("Missing THEN for IF statement")

    def _find_next_then_in_block(self, block_lines, if_pc):
        # Find the THEN statement for an IF condition within a block
        pc = if_pc + 1
        
        while pc < len(block_lines):
            line = block_lines[pc]
            if line.startswith('*'):
                pc += 1
                continue
                
            parts = line.split(' ')
            if parts[0] == 'THEN':
                return pc
                
            pc += 1
            
        raise ValueError("Missing THEN for IF statement in block")

# Example usage
if __name__ == "__main__":
    compiler = EnhancedRobotCompiler(interpolation_steps=5)
    
    # Example program demonstrating different motion types with custom steps for MOVC
    program = """
    ' Define positions
    SETE HOME, [0,0,0,0,0,0]
    SETE P1, [10,20,30,40,50,60]
    SETE P2, [20,30,40,50,60,70]
    SETE VIA, [15,25,35,45,55,65]
    
    ' Set loop counter
    SET count, 2
    
    ' Move to home position with joint interpolation
    MOVJ HOME
    
    ' Move to P1 with linear interpolation
    MOVL P1
    
    ' Move to P2 with circular interpolation through VIA point with custom steps
    MOVC VIA P2 20  ' 20 steps for smoother arc
    
    ' Loop to move between P1 and HOME with different motion types
    FOR i, 1, count
      MOVJ P1
      MOVL HOME
    NEXT i
    """
    
    # Compile the program
    try:
        joint_positions = compiler.compile(program)
        print("Compiled joint positions:")
        print(joint_positions)
        print(f"Total points generated: {len(joint_positions)}")
    except Exception as e:
        print(f"Compilation error: {e}")