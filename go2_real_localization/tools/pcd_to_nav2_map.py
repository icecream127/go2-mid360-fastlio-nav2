#!/usr/bin/env python3
"""Create a Nav2 trinary occupancy map by height-projecting a binary PCD."""
import argparse
from pathlib import Path
import numpy as np


def load_xyz(path):
    header = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("PCD header is incomplete")
            parts = line.decode("ascii").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
        if header["DATA"][0].lower() != "binary":
            raise RuntimeError("Only binary PCD is supported")
        names = header["FIELDS"]
        sizes = [int(x) for x in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(x) for x in header.get("COUNT", ["1"] * len(names))]
        n = int(header["POINTS"][0])
        fm = {("F", 4): "<f4", ("F", 8): "<f8", ("I", 1): "<i1", ("I", 2): "<i2",
              ("I", 4): "<i4", ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4"}
        offsets, formats, step = [], [], 0
        for typ, size, count in zip(types, sizes, counts):
            offsets.append(step)
            fmt = fm[(typ, size)]
            formats.append(fmt if count == 1 else (fmt, count))
            step += size * count
        dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": step})
        cloud = np.fromfile(stream, dtype=dtype, count=n)
    return np.column_stack([cloud[k].astype(np.float64) for k in ("x", "y", "z")])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--z-min", type=float, default=0.20, help="lowest obstacle height in map frame")
    parser.add_argument("--z-max", type=float, default=1.50, help="highest obstacle height in map frame")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--padding", type=float, default=0.50, help="unknown border around cropped scan bounds")
    parser.add_argument("--quantile", type=float, default=0.001, help="trim coordinate outliers on each tail")
    parser.add_argument("--min-hits", type=int, default=2, help="projected hits required to mark an obstacle cell")
    args = parser.parse_args()
    if args.z_min >= args.z_max or args.resolution <= 0 or args.padding < 0 or not 0 <= args.quantile < 0.1:
        parser.error("invalid z range, resolution, padding, or quantile")
    xyz = load_xyz(args.input)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    limits = np.quantile(xyz[:, :2], [args.quantile, 1.0 - args.quantile], axis=0)
    lo, hi = limits[0], limits[1]
    pad_cells = int(np.ceil(args.padding / args.resolution))
    core_w, core_h = np.ceil((hi - lo) / args.resolution).astype(int) + 1
    width, height = int(core_w + 2 * pad_cells), int(core_h + 2 * pad_cells)
    origin = lo - pad_cells * args.resolution
    image = np.full((height, width), 127, dtype=np.uint8)  # unknown border
    image[pad_cells:pad_cells + core_h, pad_cells:pad_cells + core_w] = 254  # unoccupied = assumed free
    keep = ((xyz[:, 2] >= args.z_min) & (xyz[:, 2] <= args.z_max) &
            (xyz[:, 0] >= lo[0]) & (xyz[:, 0] <= hi[0]) &
            (xyz[:, 1] >= lo[1]) & (xyz[:, 1] <= hi[1]))
    hits = xyz[keep]
    ix = np.floor((hits[:, 0] - origin[0]) / args.resolution).astype(np.int64)
    iy = np.floor((hits[:, 1] - origin[1]) / args.resolution).astype(np.int64)
    linear = iy * width + ix
    unique, counts = np.unique(linear, return_counts=True)
    occupied = unique[counts >= args.min_hits]
    oy, ox = np.divmod(occupied, width)
    image[height - 1 - oy, ox] = 0
    stem = args.input.with_suffix("")
    pgm = stem.with_name(stem.name + f"_nav2_{args.resolution:.2f}m.pgm")
    yaml = pgm.with_suffix(".yaml")
    with pgm.open("xb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(image.tobytes())
    yaml_text = (f"image: {pgm.name}\nmode: trinary\nresolution: {args.resolution:.3f}\n"
                 f"origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\nnegate: 0\n"
                 "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    with yaml.open("x", encoding="ascii") as stream:
        stream.write(yaml_text)
    print(f"input_points={len(xyz)} projected_points={len(hits)} occupied_cells={len(occupied)}")
    print(f"cropped_xy_bounds=({lo[0]:.3f},{lo[1]:.3f})..({hi[0]:.3f},{hi[1]:.3f})")
    print(f"height_band_m=[{args.z_min:.2f},{args.z_max:.2f}] resolution_m={args.resolution:.2f} size={width}x{height}")
    print(f"saved_map_yaml={yaml}\nsaved_map_image={pgm}")
    print("NOTE: unoccupied cells inside the cropped bounds are assumed free; inspect the PGM before robot use.")


if __name__ == "__main__":
    main()
