import numpy as np
import pytest
from src.rokisim_revival.instruction_set import InstructionSetCompiler


def test_compile_simple_program():
    compiler = InstructionSetCompiler(interpolation_steps=2)
    program = """
    SETE HOME, [0,0,0,0,0,0]
    SETE P1, [10,20,30,40,50,60]
    MOVJ HOME
    MOVL P1
    """
    joint_positions = compiler.compile(program)
    assert isinstance(joint_positions, np.ndarray)
    assert joint_positions.shape[0] > 0
    np.testing.assert_allclose(joint_positions[0], [0] * 6)
    # Check interpolation points


def test_compile_with_loop():
    compiler = InstructionSetCompiler(interpolation_steps=1)
    program = """
    SETE HOME, [0,0,0,0,0,0]
    SETE P1, [10,10,10,10,10,10]
    FOR i, 1, 2
      MOVJ P1
      MOVJ HOME
    NEXT i
    """
    joint_positions = compiler.compile(program)
    assert (
        len(joint_positions) == 4
    )  # 2 loops * 2 moves, but with interp=1, one per move


def test_compile_with_movc():
    compiler = InstructionSetCompiler(interpolation_steps=2)
    program = """
    SETE HOME, [0,0,0,0,0,0]
    SETE VIA, [5,5,5,5,5,5]
    SETE P2, [10,10,10,10,10,10]
    MOVJ HOME
    MOVC VIA P2 4
    """
    joint_positions = compiler.compile(program)
    assert len(joint_positions) > 1
    # Check for approximated arc points


def test_invalid_command():
    compiler = InstructionSetCompiler()
    program = "INVALID CMD"
    with pytest.raises(ValueError, match="Unknown command"):
        compiler.compile(program)


def test_unmatched_loop():
    compiler = InstructionSetCompiler()
    program = """
    FOR i, 1, 3
      MOVJ HOME
    """
    with pytest.raises(ValueError, match="Unmatched FOR loop"):
        compiler.compile(program)
