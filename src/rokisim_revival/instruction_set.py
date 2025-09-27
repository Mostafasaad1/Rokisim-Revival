import re
from math import acos, atan2, cos, pi, sin, sqrt
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np


class InstructionSetCompiler:
    """Compiles robot programs; modular with handler registry for easy extension."""

    def __init__(self, interpolation_steps: int = 10):
        self.positions: Dict[str, List[float]] = {}
        self.variables: Dict[str, Any] = {}
        self.output: List[List[float]] = []
        self.labels: Dict[str, int] = {}
        self.call_stack: List[int] = []
        self.current_pose: Optional[List[float]] = None
        self.default_interpolation_steps = interpolation_steps

        # Modular command handlers (easy to add new commands)
        self.command_handlers: Dict[str, Callable[[List[str], int, List[str]], int]] = {
            "SETE": self._handle_sete,
            "SET": self._handle_set,
            "INC": self._handle_inc,
            "DEC": self._handle_dec,
            "MOVJ": self._handle_movj,
            "MOVL": self._handle_movl,
            "MOVC": self._handle_movc,
            "JUMP": self._handle_jump,
            "CALL": self._handle_call,
            "RET": self._handle_ret,
            "FOR": self._handle_for,
            "IF": self._handle_if,
            "THEN": self._handle_then,
            "NEXT": self._handle_next,
        }

    def register_handler(
        self, command: str, handler: Callable[[List[str], int, List[str]], int]
    ) -> None:
        """Register a new command handler for extensibility."""
        self.command_handlers[command.upper()] = handler

    def compile(self, source_code: str) -> np.ndarray:
        lines = self._preprocess(source_code)
        self._first_pass(lines)
        self._second_pass(lines)
        return np.array(self.output)

    def _preprocess(self, source_code: str) -> List[str]:
        lines = []
        for line in source_code.split("\n"):
            line = line.split("'")[0].strip()
            if not line:
                continue
            if not line.startswith("*"):
                line = re.sub(r"\s+", " ", line)
            lines.append(line)
        return lines

    def _first_pass(self, lines: List[str]) -> None:
        current_line = 0
        for line in lines:
            if line.startswith("*"):
                label_name = line[1:].strip()
                self.labels[label_name] = current_line
            else:
                parts = line.split(" ")
                command = parts[0]
                if command == "SETE":
                    pos_name = parts[1].rstrip(",")
                    array_str = " ".join(parts[2:])
                    joint_values = self._parse_array(array_str)
                    self.positions[pos_name] = joint_values
                current_line += 1

    def _second_pass(self, lines: List[str]) -> None:
        pc = 0
        while pc < len(lines):
            line = lines[pc]
            if line.startswith("*"):
                pc += 1
                continue
            parts = line.split(" ")
            command = parts[0]
            if command in self.command_handlers:
                pc = self.command_handlers[command](parts, pc, lines)
            else:
                raise ValueError(f"Unknown command: {command}")

    def _handle_sete(self, parts: List[str], pc: int, lines: List[str]) -> int:
        # Already handled in first pass
        return pc + 1

    def _handle_set(self, parts: List[str], pc: int, lines: List[str]) -> int:
        var_name = parts[1].rstrip(",")
        value = self._evaluate_expression(" ".join(parts[2:]))
        self.variables[var_name] = value
        return pc + 1

    def _handle_inc(self, parts: List[str], pc: int, lines: List[str]) -> int:
        var_name = parts[1]
        self.variables[var_name] += 1
        return pc + 1

    def _handle_dec(self, parts: List[str], pc: int, lines: List[str]) -> int:
        var_name = parts[1]
        self.variables[var_name] -= 1
        return pc + 1

    def _handle_movj(self, parts: List[str], pc: int, lines: List[str]) -> int:
        pos_name = parts[1]
        target_pose = self.positions[pos_name]
        if self.current_pose is not None:
            self._generate_joint_interpolation(self.current_pose, target_pose)
        else:
            self.output.append(target_pose[:])
        self.current_pose = target_pose[:]
        return pc + 1

    def _handle_movl(self, parts: List[str], pc: int, lines: List[str]) -> int:
        pos_name = parts[1]
        target_pose = self.positions[pos_name]
        if self.current_pose is None:
            raise ValueError("MOVL requires a current position.")
        self._generate_linear_interpolation(self.current_pose, target_pose)
        self.current_pose = target_pose[:]
        return pc + 1

    def _handle_movc(self, parts: List[str], pc: int, lines: List[str]) -> int:
        if len(parts) < 3:
            raise ValueError("MOVC requires via and target points.")
        via_name = parts[1]
        target_name = parts[2]
        steps = self.default_interpolation_steps
        if len(parts) > 3:
            steps = int(parts[3])
        via_pose = self.positions[via_name]
        target_pose = self.positions[target_name]
        if self.current_pose is None:
            raise ValueError("MOVC requires a current position.")
        self._generate_circular_interpolation(
            self.current_pose, via_pose, target_pose, steps
        )
        self.current_pose = target_pose[:]
        return pc + 1

    def _handle_jump(self, parts: List[str], pc: int, lines: List[str]) -> int:
        label_name = parts[1]
        return self.labels[label_name]

    def _handle_call(self, parts: List[str], pc: int, lines: List[str]) -> int:
        label_name = parts[1]
        self.call_stack.append(pc + 1)
        return self.labels[label_name]

    def _handle_ret(self, parts: List[str], pc: int, lines: List[str]) -> int:
        return self.call_stack.pop()

    def _handle_for(self, parts: List[str], pc: int, lines: List[str]) -> int:
        loop_var = parts[1].rstrip(",")
        start_val = self._evaluate_expression(parts[2].rstrip(","))
        end_val = self._evaluate_expression(parts[3])
        loop_start = pc + 1
        loop_end = self._find_matching_next(lines, pc)
        for i in range(int(start_val), int(end_val) + 1):
            self.variables[loop_var] = i
            self._execute_block(lines[loop_start:loop_end])
        return loop_end + 1

    def _handle_if(self, parts: List[str], pc: int, lines: List[str]) -> int:
        condition = " ".join(parts[1:])
        if self._evaluate_condition(condition):
            return pc + 1
        else:
            return self._find_next_then(lines, pc) + 1

    def _handle_then(self, parts: List[str], pc: int, lines: List[str]) -> int:
        return pc + 1

    def _handle_next(self, parts: List[str], pc: int, lines: List[str]) -> int:
        return pc + 1

    def _generate_joint_interpolation(
        self, start: List[float], end: List[float], steps: Optional[int] = None
    ) -> None:
        steps = steps or self.default_interpolation_steps
        for i in range(1, steps + 1):
            t = i / steps
            intermediate = [start[j] + t * (end[j] - start[j]) for j in range(6)]
            self.output.append(intermediate)

    def _generate_linear_interpolation(
        self, start: List[float], end: List[float], steps: Optional[int] = None
    ) -> None:
        steps = steps or self.default_interpolation_steps
        for i in range(1, steps + 1):
            t = i / steps
            intermediate = [
                start[j] + t * (end[j] - start[j]) for j in range(6)
            ]  # Note: This is joint-linear; for true Cartesian, need IK at each step
            self.output.append(intermediate)

    def _generate_circular_interpolation(
        self, start: List[float], via: List[float], end: List[float], steps: int
    ) -> None:
        # Simplified circular arc in joint space; for Cartesian, compute center/radius and interpolate
        # Here, assuming two linear segments for approximation
        self._generate_linear_interpolation(start, via, steps // 2)
        self._generate_linear_interpolation(via, end, steps // 2)

    def _evaluate_expression(self, expr: str) -> float:
        expr = expr.strip()
        if expr in self.variables:
            return float(self.variables[expr])
        try:
            return float(expr)
        except ValueError:
            pass
        # Basic arithmetic (expand for more ops)
        for op in ["+", "-", "*", "/"]:
            if op in expr:
                parts = expr.split(op)
                left = self._evaluate_expression(parts[0])
                right = self._evaluate_expression(parts[1])
                if op == "+":
                    return left + right
                if op == "-":
                    return left - right
                if op == "*":
                    return left * right
                if op == "/":
                    return left / right
        raise ValueError(f"Cannot evaluate: {expr}")

    def _evaluate_condition(self, condition: str) -> bool:
        for op in ["==", "!=", "<=", ">=", "<", ">"]:
            if op in condition:
                parts = condition.split(op)
                left = self._evaluate_expression(parts[0].strip())
                right = self._evaluate_expression(parts[1].strip())
                if op == "==":
                    return left == right
                if op == "!=":
                    return left != right
                if op == "<":
                    return left < right
                if op == ">":
                    return left > right
                if op == "<=":
                    return left <= right
                if op == ">=":
                    return left >= right
        return bool(self._evaluate_expression(condition))

    def _parse_array(self, array_str: str) -> List[float]:
        array_str = array_str.strip()
        if not (array_str.startswith("[") and array_str.endswith("]")):
            raise ValueError(f"Invalid array: {array_str}")
        values_str = array_str[1:-1]
        return [float(v.strip()) for v in values_str.split(",")]

    def _find_matching_next(self, lines: List[str], for_pc: int) -> int:
        depth = 1
        pc = for_pc + 1
        while pc < len(lines) and depth > 0:
            line = lines[pc]
            if line.startswith("*"):
                pc += 1
                continue
            parts = line.split(" ")
            command = parts[0]
            if command == "FOR":
                depth += 1
            elif command == "NEXT":
                depth -= 1
            pc += 1
        if depth != 0:
            raise ValueError("Unmatched FOR loop")
        return pc - 1

    def _find_next_then(self, lines: List[str], if_pc: int) -> int:
        pc = if_pc + 1
        while pc < len(lines):
            line = lines[pc]
            if line.startswith("*"):
                pc += 1
                continue
            parts = line.split(" ")
            if parts[0] == "THEN":
                return pc
            pc += 1
        raise ValueError("Missing THEN for IF")

    def _execute_block(self, block_lines: List[str]) -> None:
        temp_pc = 0
        while temp_pc < len(block_lines):
            line = block_lines[temp_pc]
            if line.startswith("*"):
                temp_pc += 1
                continue
            parts = line.split(" ")
            command = parts[0]
            if command in self.command_handlers:
                temp_pc = self.command_handlers[command](parts, temp_pc, block_lines)
            else:
                raise ValueError(f"Unknown command in block: {command}")


if __name__ == "__main__":
    compiler = InstructionSetCompiler(interpolation_steps=5)
    program = """
    SETE HOME, [0,0,0,0,0,0]
    SETE P1, [10,20,30,40,50,60]
    SETE P2, [20,30,40,50,60,70]
    SETE VIA, [15,25,35,45,55,65]
    SET count, 2
    MOVJ HOME
    MOVL P1
    MOVC VIA P2 20
    FOR i, 1, count
      MOVJ P1
      MOVL HOME
    NEXT i
    """
    try:
        joint_positions = compiler.compile(program)
        print("Compiled joint positions:")
        print(joint_positions)
        print(f"Total points: {len(joint_positions)}")
    except Exception as e:
        print(f"Compilation error: {e}")
