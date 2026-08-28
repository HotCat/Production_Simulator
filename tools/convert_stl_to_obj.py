#!/usr/bin/env python3
"""Convert the MG400 binary STL link meshes to Godot-importable OBJ files.

No third-party packages are required. The original STL coordinates are already
in metres, so this intentionally performs no scaling or axis conversion.
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path


def convert(source: Path, destination: Path) -> None:
    with source.open("rb") as stream:
        stream.read(80)
        triangle_count_raw = stream.read(4)
        if len(triangle_count_raw) != 4:
            raise ValueError(f"{source} is not a binary STL")
        triangle_count = struct.unpack("<I", triangle_count_raw)[0]

        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("w", encoding="ascii", newline="\n") as output:
            output.write(f"# Generated from {source.name}\n")
            output.write(f"o {source.stem}\n")
            vertex_index = 1
            for triangle_index in range(triangle_count):
                record = stream.read(50)
                if len(record) != 50:
                    raise ValueError(
                        f"{source} ended at triangle {triangle_index}/{triangle_count}"
                    )
                values = struct.unpack("<12fH", record)
                normal = values[0:3]
                vertices = (values[3:6], values[6:9], values[9:12])
                for vertex in vertices:
                    output.write("v {:.9g} {:.9g} {:.9g}\n".format(*vertex))
                output.write("vn {:.9g} {:.9g} {:.9g}\n".format(*normal))
                output.write(
                    "f {0}//{3} {1}//{3} {2}//{3}\n".format(
                        vertex_index,
                        vertex_index + 1,
                        vertex_index + 2,
                        triangle_index + 1,
                    )
                )
                vertex_index += 3


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_dir", type=Path)
    parser.add_argument("destination_dir", type=Path)
    args = parser.parse_args()

    sources = sorted(args.source_dir.glob("*.STL"))
    if not sources:
        raise SystemExit(f"No STL files found in {args.source_dir}")
    for source in sources:
        destination = args.destination_dir / f"{source.stem}.obj"
        print(f"{source} -> {destination}")
        convert(source, destination)


if __name__ == "__main__":
    main()
