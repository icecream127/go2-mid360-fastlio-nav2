#!/usr/bin/env python3
"""Convert a binary PCD into a conservative Nav2 occupancy map.

PCD points are hits, not free-space rays. Only cells supported by observed
floor points become free; all other unobserved cells remain unknown.
"""

import argparse
import math
from pathlib import Path

import numpy as np


UNKNOWN = 127
FREE = 254
OCCUPIED = 0
MAX_GRID_CELLS = 20_000_000


def load_xyz(path):
    header = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header is incomplete")
            parts = line.decode("ascii").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break

        if header["DATA"][0].lower() != "binary":
            raise ValueError("Only uncompressed binary PCD is supported")
        names = header["FIELDS"]
        if not all(name in names for name in ("x", "y", "z")):
            raise ValueError("PCD must contain x, y, z fields")
        sizes = [int(x) for x in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(x) for x in header.get("COUNT", ["1"] * len(names))]
        if not (len(names) == len(sizes) == len(types) == len(counts)):
            raise ValueError("PCD FIELDS, SIZE, TYPE and COUNT lengths differ")
        point_count = int(header["POINTS"][0])
        if point_count <= 0:
            raise ValueError("PCD contains no points")
        formats = {
            ("F", 4): "<f4", ("F", 8): "<f8",
            ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4",
            ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4",
        }
        offsets, fields, point_step = [], [], 0
        for kind, size, count in zip(types, sizes, counts):
            if count <= 0 or (kind, size) not in formats:
                raise ValueError(f"Unsupported PCD field: TYPE={kind}, SIZE={size}, COUNT={count}")
            offsets.append(point_step)
            fields.append(formats[(kind, size)] if count == 1 else (formats[(kind, size)], count))
            point_step += size * count
        dtype = np.dtype({
            "names": names, "formats": fields, "offsets": offsets, "itemsize": point_step,
        })
        cloud = np.fromfile(stream, dtype=dtype, count=point_count)
        if len(cloud) != point_count:
            raise ValueError(f"PCD is truncated: expected {point_count} points, got {len(cloud)}")
    return np.column_stack([cloud[name].astype(np.float64) for name in ("x", "y", "z")])


def dilate_floor(mask, radius_cells):
    """Expand floor evidence locally, without wrapping across map edges."""
    if radius_cells <= 0:
        return mask.copy()
    height, width = mask.shape
    expanded = mask.copy()
    search_radius = math.ceil(radius_cells)
    for dy in range(-search_radius, search_radius + 1):
        for dx in range(-search_radius, search_radius + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            dst_y0, dst_y1 = max(0, dy), min(height, height + dy)
            dst_x0, dst_x1 = max(0, dx), min(width, width + dx)
            src_y0, src_y1 = dst_y0 - dy, dst_y1 - dy
            src_x0, src_x1 = dst_x0 - dx, dst_x1 - dx
            expanded[dst_y0:dst_y1, dst_x0:dst_x1] |= mask[src_y0:src_y1, src_x0:src_x1]
    return expanded


def build_map(xyz, args):
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) == 0:
        raise ValueError("PCD has no finite XYZ points")
    if args.bounds is None:
        low, high = np.quantile(xyz[:, :2], [args.quantile, 1.0 - args.quantile], axis=0)
    else:
        low = np.array(args.bounds[:2], dtype=np.float64)
        high = np.array(args.bounds[2:], dtype=np.float64)
    if np.any(high <= low):
        raise ValueError("XY bounds must have positive width and height")

    pad_cells = math.ceil(args.padding / args.resolution)
    core_width, core_height = np.ceil((high - low) / args.resolution).astype(int) + 1
    width, height = int(core_width + 2 * pad_cells), int(core_height + 2 * pad_cells)
    if width * height > MAX_GRID_CELLS:
        raise ValueError(f"Grid would have {width * height} cells; crop with --bounds")
    origin = low - pad_cells * args.resolution
    within_bounds = (
        (xyz[:, 0] >= low[0]) & (xyz[:, 0] <= high[0]) &
        (xyz[:, 1] >= low[1]) & (xyz[:, 1] <= high[1])
    )
    selected = xyz[within_bounds]
    ix = np.floor((selected[:, 0] - origin[0]) / args.resolution).astype(np.int64)
    iy = np.floor((selected[:, 1] - origin[1]) / args.resolution).astype(np.int64)
    linear = iy * width + ix
    relative_z = selected[:, 2] - args.floor_z

    floor_points = np.abs(relative_z) <= args.floor_tolerance
    obstacle_points = (
        (relative_z >= args.obstacle_min_height) &
        (relative_z <= args.obstacle_max_height)
    )
    floor_hits = np.bincount(linear[floor_points], minlength=width * height).reshape(height, width)
    obstacle_hits = np.bincount(linear[obstacle_points], minlength=width * height).reshape(height, width)
    floor = floor_hits >= args.min_floor_hits
    if not floor.any():
        raise ValueError("No floor-supported cells; check --floor-z and --floor-tolerance")

    radius_cells = args.free_radius / args.resolution
    free = dilate_floor(floor, radius_cells)
    core = np.zeros((height, width), dtype=bool)
    core[pad_cells:pad_cells + core_height, pad_cells:pad_cells + core_width] = True
    # Even one possible obstacle hit must not become free due to floor expansion.
    free &= core & (obstacle_hits == 0)
    occupied = obstacle_hits >= args.min_obstacle_hits
    if not free.any():
        raise ValueError("No free cells remain; check height bands and floor calibration")

    image = np.full((height, width), UNKNOWN, dtype=np.uint8)
    image[free] = FREE
    image[occupied] = OCCUPIED
    stats = {
        "input_points": len(xyz),
        "cropped_points": len(selected),
        "floor_points": int(floor_points.sum()),
        "obstacle_points": int(obstacle_points.sum()),
        "free_cells": int(free.sum()),
        "occupied_cells": int(occupied.sum()),
        "unknown_cells": int(width * height - free.sum() - occupied.sum()),
        "width": width, "height": height,
    }
    return np.flipud(image), origin, stats


def write_map(prefix, image, origin, resolution):
    pgm = Path(str(prefix) + ".pgm")
    yaml = Path(str(prefix) + ".yaml")
    if pgm.exists() or yaml.exists():
        raise FileExistsError(f"Output already exists: {pgm} or {yaml}; choose another --output-prefix")
    if not prefix.parent.is_dir():
        raise FileNotFoundError(f"Output directory does not exist: {prefix.parent}")
    yaml_text = (
        f"image: {pgm.name}\nmode: trinary\nresolution: {resolution:.12g}\n"
        f"origin: [{origin[0]:.10f}, {origin[1]:.10f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    pgm_created = yaml_created = False
    try:
        with pgm.open("xb") as stream:
            pgm_created = True
            stream.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii"))
            stream.write(image.tobytes())
        with yaml.open("x", encoding="ascii") as stream:
            yaml_created = True
            stream.write(yaml_text)
    except Exception:
        if yaml_created:
            yaml.unlink()
        if pgm_created:
            pgm.unlink()
        raise
    return pgm, yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="binary PCD in the same map frame used for localization")
    parser.add_argument("--floor-z", type=float, required=True, help="measured floor z in PCD map frame (m)")
    parser.add_argument("--floor-tolerance", type=float, default=0.10,
                        help="absolute z tolerance for observed floor points (m)")
    parser.add_argument("--obstacle-min-height", type=float, default=0.15,
                        help="minimum obstacle height above floor (m)")
    parser.add_argument("--obstacle-max-height", type=float, default=1.50,
                        help="maximum obstacle height above floor, excluding ceiling (m)")
    parser.add_argument("--resolution", type=float, default=0.05, help="map cell size (m)")
    parser.add_argument("--padding", type=float, default=0.50, help="unknown border (m)")
    parser.add_argument("--quantile", type=float, default=0.001,
                        help="XY tail fraction cropped to reject outliers (0 disables cropping)")
    parser.add_argument("--bounds", type=float, nargs=4, metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
                        help="explicit XY crop in map coordinates; overrides --quantile")
    parser.add_argument("--min-floor-hits", type=int, default=1)
    parser.add_argument("--min-obstacle-hits", type=int, default=1)
    parser.add_argument("--free-radius", type=float, default=0.0,
                        help="optional floor-evidence expansion radius (m); default 0 is most conservative")
    parser.add_argument("--output-prefix", type=Path, help="output filename without .pgm/.yaml")
    args = parser.parse_args()
    numeric = (
        args.floor_z, args.floor_tolerance, args.obstacle_min_height,
        args.obstacle_max_height, args.resolution, args.padding, args.quantile,
        args.free_radius,
    )
    if not all(math.isfinite(value) for value in numeric):
        parser.error("all numeric arguments must be finite")
    if (args.floor_tolerance <= 0 or args.obstacle_min_height <= args.floor_tolerance or
            args.obstacle_max_height <= args.obstacle_min_height or args.resolution <= 0 or
            args.padding < 0 or not 0 <= args.quantile < 0.1 or args.free_radius < 0 or
            args.min_floor_hits < 1 or args.min_obstacle_hits < 1):
        parser.error("invalid height bands, grid, crop, radius or hit count")
    if args.bounds is not None and not all(math.isfinite(value) for value in args.bounds):
        parser.error("--bounds values must be finite")

    prefix = args.output_prefix or args.input.with_name(
        f"{args.input.stem}_nav2_{args.resolution:g}m"
    )
    try:
        xyz = load_xyz(args.input)
        image, origin, stats = build_map(xyz, args)
        pgm, yaml = write_map(prefix, image, origin, args.resolution)
    except (OSError, KeyError, ValueError) as error:
        parser.exit(1, f"error: {error}\n")
    for key in ("input_points", "cropped_points", "floor_points", "obstacle_points",
                "free_cells", "occupied_cells", "unknown_cells"):
        print(f"{key}={stats[key]}")
    print(f"size={stats['width']}x{stats['height']} resolution={args.resolution:g}m")
    print(f"floor_z={args.floor_z:g}m floor_tolerance={args.floor_tolerance:g}m")
    print(f"saved_map_yaml={yaml}\nsaved_map_image={pgm}")
    print("Review the map before driving: floor returns do not prove full robot clearance,")
    print("and the live obstacle layer must still detect people and moved objects.")


if __name__ == "__main__":
    main()
