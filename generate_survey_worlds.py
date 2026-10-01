#!/usr/bin/env python3
import argparse
import csv
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path


VIDEO_PATTERN = re.compile(r"(?P<number>\d+)\.mp4$", re.IGNORECASE)
JSON_PATTERN = re.compile(r"(?P<number>\d+)\.json$", re.IGNORECASE)

DEFAULT_CSV_DIR = Path(
    "/home/hunav_gz_classic_ws/SocNav3_trajectories/index_trajectories_survey"
)
DEFAULT_JSON_DIR = Path("/home/hunav_gz_classic_ws/SocNav3_trajectories")
DEFAULT_OUTPUT_DIR = Path(
    "/home/hunav_gz_classic_ws/src/socnav_gazebo_reproducer/worlds/survey_worlds"
)


def read_video_numbers(survey_dir):
    csv_files = sorted(survey_dir.glob("*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {survey_dir}")

    video_numbers = set()
    malformed_rows = []
    for csv_path in csv_files:
        with csv_path.open(newline="", encoding="utf-8-sig") as csv_file:
            reader = csv.DictReader(csv_file)
            if not reader.fieldnames or "file" not in reader.fieldnames:
                raise ValueError(f"CSV does not contain a 'file' column: {csv_path}")

            for row_number, row in enumerate(reader, start=2):
                video_file = (row.get("file") or "").strip().replace("\\", "/")
                match = VIDEO_PATTERN.fullmatch(Path(video_file).name)
                if match:
                    video_numbers.add(match.group("number"))
                else:
                    malformed_rows.append((csv_path.name, row_number, video_file))

    return csv_files, video_numbers, malformed_rows


def index_json_files(dataset_dir):
    json_by_number = defaultdict(list)
    for source_index in range(1, 7):
        source_dir = dataset_dir / f"src{source_index}"
        if not source_dir.is_dir():
            continue
        for json_path in source_dir.glob("*.json"):
            match = JSON_PATTERN.fullmatch(json_path.name)
            if match:
                json_by_number[match.group("number")].append(json_path)

    for paths in json_by_number.values():
        paths.sort()
    return json_by_number


def main():
    parser = argparse.ArgumentParser(
        description="Generate Gazebo worlds for videos listed in the survey CSVs."
    )
    parser.add_argument(
        "--csv-dir",
        type=Path,
        default=DEFAULT_CSV_DIR,
        help=f"Directory containing survey CSV files (default: {DEFAULT_CSV_DIR})",
    )
    parser.add_argument(
        "--json-dir",
        type=Path,
        default=DEFAULT_JSON_DIR,
        help=f"Dataset directory containing src1 through src6 (default: {DEFAULT_JSON_DIR})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Directory for generated world files (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--first-match",
        action="store_true",
        help="For duplicate JSON prefixes, use the first path in sorted order",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report planned outputs without generating worlds",
    )
    args = parser.parse_args()

    try:
        dataset_dir = args.json_dir
        builder = Path(__file__).resolve().parent / "gazebo_world_builder.py"
        output_dir = args.output_dir

        csv_files, video_numbers, malformed_rows = read_video_numbers(args.csv_dir)
        json_by_number = index_json_files(dataset_dir)
    except (FileNotFoundError, ValueError) as error:
        parser.error(str(error))

    selected = []
    missing = []
    ambiguous = []
    for number in sorted(video_numbers):
        matches = json_by_number.get(number, [])
        if not matches:
            missing.append(number)
            continue
        elif len(matches) > 1:
            ambiguous.append((number, matches))
            if not args.first_match:
                continue
        selected.append((number, matches[0]))

    print(f"Read {len(video_numbers)} unique video IDs from {len(csv_files)} CSV files.")
    print(f"{'Would generate' if args.dry_run else 'Generating'} {len(selected)} worlds in {output_dir}.")
    if ambiguous and args.first_match:
        print(f"Using the first sorted JSON path for {len(ambiguous)} duplicate prefixes.")

    failures = []
    if not args.dry_run:
        output_dir.mkdir(parents=True, exist_ok=True)
        for number, json_path in selected:
            world_path = output_dir / f"{number}.world"
            try:
                subprocess.run(
                    [sys.executable, str(builder), str(json_path), str(world_path)],
                    check=True,
                )
            except subprocess.CalledProcessError as error:
                failures.append((number, error.returncode))

    if malformed_rows:
        print(f"Skipped {len(malformed_rows)} rows with invalid video filenames:")
        for csv_name, row_number, video_file in malformed_rows[:20]:
            print(f"  {csv_name}:{row_number}: {video_file!r}")
    if missing:
        print(f"Skipped {len(missing)} IDs with no JSON match; first IDs: {', '.join(missing[:20])}")
    if ambiguous and not args.first_match:
        print(f"Skipped {len(ambiguous)} IDs with multiple JSON matches; first matches:")
        for number, matches in ambiguous[:10]:
            print(f"  {number}: {', '.join(str(path.relative_to(dataset_dir)) for path in matches)}")
    if failures:
        print(f"Failed to generate {len(failures)} worlds:")
        for number, return_code in failures:
            print(f"  {number}: builder exited with status {return_code}")

    return 1 if failures or not selected else 0


if __name__ == "__main__":
    sys.exit(main())