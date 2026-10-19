#!/usr/bin/env python3
import argparse
import subprocess
import sys
from pathlib import Path


DEFAULT_INPUT_DIR = Path(
    "/home/hunav_gz_classic_ws/SocNav3_trajectories/src0"
)
DEFAULT_OUTPUT_DIR = Path(
    "/home/hunav_gz_classic_ws/src/socnav_gazebo_reproducer/worlds/"
    "survey_worlds/polito_worlds"
)


def main():
    parser = argparse.ArgumentParser(
        description="Generate Gazebo worlds from trajectory JSON files in a directory."
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=DEFAULT_INPUT_DIR,
        help=f"Directory containing trajectory JSON files (default: {DEFAULT_INPUT_DIR})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Directory for generated world files (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report planned outputs without generating worlds.",
    )
    args = parser.parse_args()

    if not args.input_dir.is_dir():
        parser.error(f"Input directory not found: {args.input_dir}")

    json_files = sorted(args.input_dir.glob("*.json"))
    if not json_files:
        parser.error(f"No JSON files found in {args.input_dir}")

    builder = Path(__file__).resolve().parent / "gazebo_world_builder.py"
    print(
        f"{'Would generate' if args.dry_run else 'Generating'} "
        f"{len(json_files)} worlds in {args.output_dir}."
    )

    failures = []
    if not args.dry_run:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        for json_path in json_files:
            world_path = args.output_dir / f"{json_path.stem}.world"
            try:
                subprocess.run(
                    [sys.executable, str(builder), str(json_path), str(world_path)],
                    check=True,
                )
            except subprocess.CalledProcessError as error:
                failures.append((json_path.name, error.returncode))

    if failures:
        print(f"Failed to generate {len(failures)} worlds:", file=sys.stderr)
        for json_name, return_code in failures:
            print(f"  {json_name}: builder exited with status {return_code}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
