#!/usr/bin/env python3
"""Project a binary XYZI PCD height band into a planar XY point cloud."""
import argparse
from pathlib import Path
import numpy as np


def load_pcd(path):
    header = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("PCD header ended before DATA")
            parts = line.decode("ascii").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
        if header["DATA"][0].lower() != "binary":
            raise RuntimeError("Only binary PCD is supported")
        fields = header["FIELDS"]
        sizes = [int(x) for x in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(x) for x in header.get("COUNT", ["1"] * len(fields))]
        n = int(header["POINTS"][0])
        fm = {("F", 4): "<f4", ("F", 8): "<f8", ("I", 1): "<i1", ("I", 2): "<i2",
              ("I", 4): "<i4", ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4"}
        offsets, formats, step = [], [], 0
        for typ, size, count in zip(types, sizes, counts):
            offsets.append(step)
            fmt = fm[(typ, size)]
            formats.append(fmt if count == 1 else (fmt, count))
            step += size * count
        dtype = np.dtype({"names": fields, "formats": formats, "offsets": offsets, "itemsize": step})
        points = np.fromfile(stream, dtype=dtype, count=n)
    if not all(name in fields for name in ("x", "y", "z")):
        raise RuntimeError("PCD must contain x, y, z fields")
    xyz = np.column_stack([points[k].astype(np.float64) for k in ("x", "y", "z")])
    intensity = points["intensity"].astype(np.float64) if "intensity" in fields else None
    return xyz, intensity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--z-min", type=float, default=0.15)
    parser.add_argument("--z-max", type=float, default=1.5)
    parser.add_argument("--resolution", type=float, default=0.05)
    args = parser.parse_args()
    if args.z_min >= args.z_max or args.resolution <= 0:
        parser.error("require z-min < z-max and positive resolution")
    xyz, intensity = load_pcd(args.input)
    valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] >= args.z_min) & (xyz[:, 2] <= args.z_max)
    selected = xyz[valid]
    if selected.size == 0:
        raise RuntimeError("No points in the requested height band")
    ix = np.floor((selected[:, 0] - selected[:, 0].min()) / args.resolution).astype(np.int64)
    iy = np.floor((selected[:, 1] - selected[:, 1].min()) / args.resolution).astype(np.int64)
    _, inverse = np.unique(np.column_stack((ix, iy)), axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    out = np.column_stack((np.bincount(inverse, weights=selected[:, 0]) / counts,
                           np.bincount(inverse, weights=selected[:, 1]) / counts,
                           np.zeros(len(counts))))
    names = "x y z" + (" intensity" if intensity is not None else "")
    if intensity is not None:
        out = np.column_stack((out, np.bincount(inverse, weights=intensity[valid]) / counts))
    out = out.astype("<f4")
    target = args.input.with_name(f"{args.input.stem}_2d_z{args.z_min:.2f}_{args.z_max:.2f}_res{args.resolution:.2f}.pcd")
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS " + names + "\n" +
              "SIZE " + " ".join(["4"] * out.shape[1]) + "\n" +
              "TYPE " + " ".join(["F"] * out.shape[1]) + "\n" +
              "COUNT " + " ".join(["1"] * out.shape[1]) + "\n" +
              f"WIDTH {len(out)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(out)}\nDATA binary\n")
    with target.open("xb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(out.tobytes())
    print(f"input_points={len(xyz)} height_filtered_points={len(selected)} output_points={len(out)}")
    print(f"height_band_m=[{args.z_min:.2f}, {args.z_max:.2f}] xy_cell_m={args.resolution:.2f}")
    print(f"saved={target}")


if __name__ == "__main__":
    main()
