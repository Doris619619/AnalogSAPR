#你的代码

from pathlib import Path

from sapr.constraints import validate_circuit
from sapr.io import load_circuit


def test_load_sample_input() -> None:
    circuit = load_circuit(Path("input"))
    assert "M1" in circuit.modules
    assert "OUT" in circuit.nets


def test_sample_validation_is_clean() -> None:
    circuit = load_circuit(Path("input"))
    errors = validate_circuit(circuit)
    assert errors == []
