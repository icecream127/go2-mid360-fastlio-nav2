"""Regression tests for the conservative PCD-to-Nav2 map converter."""

import math
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import unittest


CONVERTER = Path(__file__).resolve().parents[1] / "tools" / "pcd_to_nav2_map.py"
FLOOR_Z = -0.75


def write_binary_pcd(path, points):
    """Write a minimal, valid binary PCD with XYZ float32 points."""
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        for point in points:
            stream.write(struct.pack("<fff", *point))


def read_pgm(path):
    magic, dimensions, maxval, pixels = path.read_bytes().split(b"\n", 3)
    if magic != b"P5" or maxval != b"255":
        raise AssertionError("Expected an 8-bit binary PGM")
    width, height = map(int, dimensions.split())
    if len(pixels) != width * height:
        raise AssertionError("PGM pixel count does not match dimensions")
    return width, height, pixels


def read_map_yaml(path):
    text = path.read_text(encoding="utf-8")
    origin_match = re.search(r"^origin:\s*\[([^]]+)\]", text, re.MULTILINE)
    resolution_match = re.search(r"^resolution:\s*([^\s#]+)", text, re.MULTILINE)
    image_match = re.search(r"^image:\s*(\S+)", text, re.MULTILINE)
    if not (origin_match and resolution_match and image_match):
        raise AssertionError("Missing required Nav2 map YAML entries")
    origin = [float(value) for value in origin_match.group(1).split(",")]
    return text, image_match.group(1), float(resolution_match.group(1)), origin


def pixel_at_world(pgm, yaml_path, x, y):
    width, height, pixels = read_pgm(pgm)
    _, _, resolution, origin = read_map_yaml(yaml_path)
    column = math.floor((x - origin[0]) / resolution + 1e-7)
    bottom_up_row = math.floor((y - origin[1]) / resolution + 1e-7)
    if not (0 <= column < width and 0 <= bottom_up_row < height):
        raise AssertionError(f"World point ({x}, {y}) lies outside the generated map")
    # PGM rows run downward while Nav2's map origin is the lower-left corner.
    return pixels[(height - 1 - bottom_up_row) * width + column]


class PcdToNav2MapTest(unittest.TestCase):
    def setUp(self):
        self.assertTrue(CONVERTER.is_file(), f"Converter is missing: {CONVERTER}")
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.pcd = self.directory / "room.pcd"
        write_binary_pcd(
            self.pcd,
            [
                (0.0, 0.0, FLOOR_Z),
                (2.0, 0.0, FLOOR_Z),
                (0.0, 2.0, FLOOR_Z),
                (2.0, 2.0, FLOOR_Z - 0.03),  # Within floor tolerance.
                (2.0, 0.0, FLOOR_Z + 0.40),  # Obstacle on a floor cell.
                (0.0, 0.0, FLOOR_Z + 0.05),  # Too low to be an obstacle.
                (0.0, 2.0, FLOOR_Z + 2.50),  # Ceiling, outside height band.
            ],
        )

    def convert(self, prefix, free_radius=0.0):
        return subprocess.run(
            [
                sys.executable,
                str(CONVERTER),
                str(self.pcd),
                "--floor-z", str(FLOOR_Z),
                "--floor-tolerance", "0.08",
                "--obstacle-min-height", "0.20",
                "--obstacle-max-height", "1.50",
                "--free-radius", str(free_radius),
                "--resolution", "1.0",
                "--output-prefix", str(prefix),
            ],
            text=True,
            capture_output=True,
            check=False,
        )

    def test_unknown_floor_and_obstacle_cells(self):
        prefix = self.directory / "conservative"
        result = self.convert(prefix)
        self.assertEqual(result.returncode, 0, result.stderr)
        pgm, yaml = prefix.with_suffix(".pgm"), prefix.with_suffix(".yaml")
        self.assertTrue(pgm.is_file())
        self.assertTrue(yaml.is_file())

        yaml_text, image, resolution, origin = read_map_yaml(yaml)
        self.assertEqual(image, pgm.name)
        self.assertIn("mode: trinary", yaml_text)
        self.assertAlmostEqual(resolution, 1.0)
        self.assertEqual(len(origin), 3)
        self.assertEqual(origin[2], 0.0)

        self.assertEqual(pixel_at_world(pgm, yaml, 0.0, 0.0), 254)
        self.assertEqual(pixel_at_world(pgm, yaml, 2.0, 2.0), 254)
        self.assertEqual(pixel_at_world(pgm, yaml, 0.0, 2.0), 254)
        self.assertEqual(pixel_at_world(pgm, yaml, 2.0, 0.0), 0)
        self.assertEqual(pixel_at_world(pgm, yaml, 1.0, 1.0), 127)
        self.assertEqual(pixel_at_world(pgm, yaml, 1.0, 0.0), 127)

    def test_free_radius_expands_only_observed_floor_and_keeps_obstacle(self):
        prefix = self.directory / "expanded"
        result = self.convert(prefix, free_radius=1.1)
        self.assertEqual(result.returncode, 0, result.stderr)
        pgm, yaml = prefix.with_suffix(".pgm"), prefix.with_suffix(".yaml")
        self.assertEqual(pixel_at_world(pgm, yaml, 1.0, 0.0), 254)
        self.assertEqual(pixel_at_world(pgm, yaml, 1.0, 1.0), 127)
        self.assertEqual(pixel_at_world(pgm, yaml, 2.0, 0.0), 0)

    def test_existing_output_file_is_not_overwritten(self):
        for existing_extension in (".pgm", ".yaml"):
            with self.subTest(existing_extension=existing_extension):
                prefix = self.directory / f"collision_{existing_extension[1:]}"
                existing = prefix.with_suffix(existing_extension)
                other = prefix.with_suffix(".yaml" if existing_extension == ".pgm" else ".pgm")
                sentinel = b"existing user map; preserve this content\n"
                existing.write_bytes(sentinel)

                result = self.convert(prefix)

                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(existing.read_bytes(), sentinel)
                self.assertFalse(other.exists(), "A collision must not leave a partial output pair")


if __name__ == "__main__":
    unittest.main()
