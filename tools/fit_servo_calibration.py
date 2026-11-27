#!/usr/bin/env python3
"""Fit servo mappings from measured arm poses and raw servo commands.

Required CSV columns:
  E1_deg,E2_deg,P_deg,shoulder_raw_deg,elbow_raw_deg,wrist_raw_deg

Optional gripper columns:
  gripper_joint_deg,gripper_raw_deg

E1/E2/P are measured absolute elevations in degrees. Raw columns are the
physical servo angles commanded with the calibration firmware. Add one row per
measured pose. Gripper measurements are independent and may be left blank.
"""

import argparse
import csv
import math
import sys


JOINT_COLUMNS = {
    "shoulder": "shoulder_raw_deg",
    "elbow": "elbow_raw_deg",
    "wrist": "wrist_raw_deg",
    "gripper": "gripper_raw_deg",
}
SERVO_RANGES = {
    "shoulder": 180.0,
    "elbow": 180.0,
    "wrist": 180.0,
    "gripper": 90.0,
}
JOINT_RANGES = {
    "shoulder": (0.0, 180.0),
    "elbow": (0.0, 180.0),
    "wrist": (0.0, 180.0),
    "gripper": (0.0, 90.0),
}
GAIN_TOLERANCE = 0.2


def parse_float(row, column, line_number):
    value = row.get(column, "").strip()
    if not value:
        raise ValueError(f"line {line_number}: missing {column}")
    try:
        number = float(value)
    except ValueError as exc:
        raise ValueError(f"line {line_number}: invalid {column}={value!r}") from exc
    if not math.isfinite(number):
        raise ValueError(f"line {line_number}: {column} must be finite")
    return number


def read_samples(path):
    samples = {name: [] for name in JOINT_COLUMNS}
    with open(path, newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames is None:
            raise ValueError("CSV is empty or has no header")
        required = {
            "E1_deg", "E2_deg", "P_deg",
            "shoulder_raw_deg", "elbow_raw_deg", "wrist_raw_deg",
        }
        missing = required.difference(reader.fieldnames)
        if missing:
            raise ValueError("missing CSV columns: " + ", ".join(sorted(missing)))

        for line_number, row in enumerate(reader, start=2):
            e1 = parse_float(row, "E1_deg", line_number)
            e2 = parse_float(row, "E2_deg", line_number)
            pointing = parse_float(row, "P_deg", line_number)
            logical = {
                "shoulder": e1,
                "elbow": e1 - e2,
                "wrist": e2 - pointing,
            }
            for name, joint_angle in logical.items():
                raw_angle = parse_float(row, JOINT_COLUMNS[name], line_number)
                samples[name].append((joint_angle, raw_angle))

            gripper_joint = row.get("gripper_joint_deg", "").strip()
            gripper_raw = row.get("gripper_raw_deg", "").strip()
            if bool(gripper_joint) != bool(gripper_raw):
                raise ValueError(
                    f"line {line_number}: provide both optional gripper columns or neither"
                )
            if gripper_joint:
                samples["gripper"].append((
                    parse_float(row, "gripper_joint_deg", line_number),
                    parse_float(row, "gripper_raw_deg", line_number),
                ))
    return samples


def fit_mapping(samples, name):
    if len(samples) < 2:
        raise ValueError(f"{name}: need at least two measured samples")
    joint = [point[0] for point in samples]
    raw = [point[1] for point in samples]
    x = [angle - 90.0 for angle in joint]
    x_mean = sum(x) / len(x)
    y_mean = sum(raw) / len(raw)
    denominator = sum((value - x_mean) ** 2 for value in x)
    if denominator == 0.0:
        raise ValueError(f"{name}: joint samples do not span a range")
    slope = sum(
        (x_value - x_mean) * (y_value - y_mean)
        for x_value, y_value in zip(x, raw)
    ) / denominator
    if abs(slope) < 1e-6:
        raise ValueError(f"{name}: fitted direction/gain is effectively zero")
    zero = y_mean - slope * x_mean
    gain = abs(slope)
    direction = 1 if slope > 0.0 else -1
    return zero, direction, gain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", help="CSV file with measured poses and raw commands")
    args = parser.parse_args()

    try:
        samples = read_samples(args.csv_file)
        print("servo,zeroServoDeg,direction,gain")
        for name in JOINT_COLUMNS:
            if not samples[name]:
                if name != "gripper":
                    raise ValueError(f"{name}: no measured arm poses in CSV")
                print(f"{name},90.0000,1,1.00000")
                print(
                    f"INFO: {name} has no samples; retaining the current defaults",
                    file=sys.stderr,
                )
                continue
            zero, direction, gain = fit_mapping(samples[name], name)
            print(f"{name},{zero:.4f},{direction},{gain:.5f}")
            if abs(gain - 1.0) > GAIN_TOLERANCE:
                print(
                    f"WARNING: {name} gain {gain:.4f} is far from 1.0; "
                    "check measurements and whether the servo is a 270-degree model",
                    file=sys.stderr,
                )
            angle_range = SERVO_RANGES[name]
            mapped = [
                zero + direction * gain * (joint - 90.0)
                for joint, _ in samples[name]
            ]
            if not 0.0 <= zero <= angle_range:
                print(
                    f"WARNING: {name} zeroServoDeg {zero:.3f} is outside "
                    f"0..{angle_range:.0f}",
                    file=sys.stderr,
                )
            joint_min, joint_max = JOINT_RANGES[name]
            for joint in (joint_min, joint_max):
                servo_angle = zero + direction * gain * (joint - 90.0)
                if not 0.0 <= servo_angle <= angle_range:
                    print(
                        f"WARNING: {name} soft-limit joint {joint:.3f} "
                        f"maps to {servo_angle:.3f} outside 0..{angle_range:.0f}",
                        file=sys.stderr,
                    )
            for index, (joint, _) in enumerate(samples[name], start=1):
                servo_angle = mapped[index - 1]
                if not 0.0 <= servo_angle <= angle_range:
                    print(
                        f"WARNING: {name} row {index} maps joint {joint:.3f} "
                        f"to {servo_angle:.3f} outside 0..{angle_range:.0f}",
                        file=sys.stderr,
                    )
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
