from __future__ import annotations

import argparse
import json
import sys

from .constraints import validate_circuit, validate_solution
from .io import load_circuit, write_solution
from .optimizer import BaselineSolver, SolverConfig
from .router import measure


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="sapr", description="Analog simultaneous placement/routing reproduction scaffold")
    subparsers = parser.add_subparsers(dest="cmd", required=True)

    validate_parser = subparsers.add_parser("validate", help="validate input files")
    validate_parser.add_argument("--input", default="input")

    run_parser = subparsers.add_parser("run", help="run the baseline solver")
    run_parser.add_argument("--input", default="input")
    run_parser.add_argument("--output", default="output")
    run_parser.add_argument("--spacing", type=float, default=5.0)
    run_parser.add_argument("--row-width", type=float, default=40.0)

    args = parser.parse_args(argv)
    if args.cmd == "validate":
        circuit = load_circuit(args.input)
        errors = validate_circuit(circuit)
        if errors:
            print("INVALID")
            for error in errors:
                print(f"- {error}")
            return 1
        print("OK")
        return 0

    if args.cmd == "run":
        circuit = load_circuit(args.input)
        solver = BaselineSolver(SolverConfig(spacing=args.spacing, row_width=args.row_width))
        try:
            solution = solver.solve(circuit)
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 1
        errors = validate_solution(circuit, solution)
        if errors:
            print("invalid solution:\n" + "\n".join(f"- {error}" for error in errors), file=sys.stderr)
            return 1
        write_solution(solution, args.output)
        metrics = measure(circuit, solution)
        print(json.dumps(metrics.__dict__, indent=2, sort_keys=True))
        return 0

    raise AssertionError(args.cmd)


if __name__ == "__main__":
    raise SystemExit(main())
