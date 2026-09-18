#!/usr/bin/env python3

"""Export COLLADA visual triangles as metre-scaled binary STL collisions.

The Pro450 source meshes store coordinates in millimetres and declare that
scale in COLLADA metadata.  Some collision loaders ignore the metadata, so the
unit conversion is baked into STL coordinates here.  Unlike a single convex
hull, the exported surface preserves bends, recesses and clearance openings.
"""

import argparse
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def _source_positions(mesh, source_id, scale):
    source = mesh.find(f"c:source[@id='{source_id}']", NS)
    if source is None:
        raise ValueError(f"missing position source {source_id}")
    values_tag = source.find("c:float_array", NS)
    accessor = source.find("c:technique_common/c:accessor", NS)
    if values_tag is None:
        raise ValueError(f"position source {source_id} has no float_array")
    stride = int(accessor.attrib.get("stride", "3")) if accessor is not None else 3
    values = [float(value) * scale for value in values_tag.text.split()]
    return [tuple(values[i : i + 3]) for i in range(0, len(values), stride)]


def collada_triangles(path):
    root = ET.parse(path).getroot()
    unit = root.find("c:asset/c:unit", NS)
    scale = float(unit.attrib.get("meter", "1")) if unit is not None else 1.0
    result = []

    for geometry in root.findall("c:library_geometries/c:geometry", NS):
        mesh = geometry.find("c:mesh", NS)
        if mesh is None:
            continue
        vertices_sources = {}
        for vertices in mesh.findall("c:vertices", NS):
            position = vertices.find("c:input[@semantic='POSITION']", NS)
            if position is not None:
                vertices_sources[vertices.attrib["id"]] = position.attrib[
                    "source"
                ].lstrip("#")

        position_cache = {}
        for triangles in mesh.findall("c:triangles", NS):
            inputs = triangles.findall("c:input", NS)
            if not inputs:
                continue
            stride = max(int(item.attrib.get("offset", "0")) for item in inputs) + 1
            vertex_input = next(
                (item for item in inputs if item.attrib.get("semantic") == "VERTEX"),
                None,
            )
            if vertex_input is None:
                raise ValueError(f"{path}: triangle set has no VERTEX input")
            vertex_offset = int(vertex_input.attrib.get("offset", "0"))
            vertices_id = vertex_input.attrib["source"].lstrip("#")
            source_id = vertices_sources[vertices_id]
            if source_id not in position_cache:
                position_cache[source_id] = _source_positions(mesh, source_id, scale)
            positions = position_cache[source_id]
            indices = [int(value) for value in triangles.find("c:p", NS).text.split()]
            vertex_indices = indices[vertex_offset::stride]
            if len(vertex_indices) % 3:
                raise ValueError(f"{path}: triangle index count is not divisible by 3")
            for index in range(0, len(vertex_indices), 3):
                result.append(
                    tuple(positions[value] for value in vertex_indices[index : index + 3])
                )
    if not result:
        raise ValueError(f"{path}: no triangles found")
    return result


def _normal(triangle):
    a, b, c = triangle
    ab = tuple(b[i] - a[i] for i in range(3))
    ac = tuple(c[i] - a[i] for i in range(3))
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    length = math.sqrt(sum(value * value for value in cross))
    if length <= 1e-15:
        return (0.0, 0.0, 0.0)
    return tuple(value / length for value in cross)


def write_binary_stl(path, triangles, source_name):
    header = f"Pro450 exact collision from {source_name}".encode("ascii")[:80]
    with path.open("wb") as stream:
        stream.write(header.ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            stream.write(struct.pack("<3f", *_normal(triangle)))
            for vertex in triangle:
                stream.write(struct.pack("<3f", *vertex))
            stream.write(struct.pack("<H", 0))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("meshes", nargs="+", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for source in args.meshes:
        triangles = collada_triangles(source)
        destination = args.output_dir / f"{source.stem}_exact.stl"
        write_binary_stl(destination, triangles, source.name)
        print(f"{source.name}: {len(triangles)} triangles -> {destination}")


if __name__ == "__main__":
    main()
