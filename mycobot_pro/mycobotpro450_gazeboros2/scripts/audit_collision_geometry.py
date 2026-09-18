#!/usr/bin/env python3

"""Audit how well URDF collision primitives cover visual COLLADA meshes."""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def parse_vector(text, default):
    return tuple(float(value) for value in (text or default).split())


def matmul(left, right):
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def transpose(matrix):
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1, 0, 0), (0, cr, -sr), (0, sr, cr))
    ry = ((cp, 0, sp), (0, 1, 0), (-sp, 0, cp))
    rz = ((cy, -sy, 0), (sy, cy, 0), (0, 0, 1))
    return matmul(rz, matmul(ry, rx))


def transform_point(point, translation, rotation):
    return tuple(
        translation[row] + sum(rotation[row][column] * point[column] for column in range(3))
        for row in range(3)
    )


def inverse_transform_point(point, translation, rotation):
    inverse = transpose(rotation)
    shifted = tuple(point[index] - translation[index] for index in range(3))
    return tuple(
        sum(inverse[row][column] * shifted[column] for column in range(3))
        for row in range(3)
    )


def collada_points(path):
    root = ET.parse(path).getroot()
    unit = root.find("c:asset/c:unit", COLLADA_NS)
    scale = float(unit.attrib.get("meter", "1")) if unit is not None else 1.0
    sources = {source.attrib["id"]: source for source in root.findall(".//c:source", COLLADA_NS)}
    position_source_ids = set()
    for vertices in root.findall(".//c:vertices", COLLADA_NS):
        input_tag = vertices.find("c:input[@semantic='POSITION']", COLLADA_NS)
        if input_tag is not None:
            position_source_ids.add(input_tag.attrib["source"].lstrip("#"))

    points = []
    for source_id in position_source_ids:
        source = sources[source_id]
        float_array = source.find("c:float_array", COLLADA_NS)
        accessor = source.find("c:technique_common/c:accessor", COLLADA_NS)
        if float_array is None:
            continue
        stride = int(accessor.attrib.get("stride", "3")) if accessor is not None else 3
        values = [float(value) for value in float_array.text.split()]
        for offset in range(0, len(values), stride):
            point = values[offset : offset + 3]
            if len(point) == 3:
                points.append(tuple(scale * value for value in point))
    return points


def resolve_mesh(filename, description_root):
    prefix = "package://mycobot_description/"
    if not filename.startswith(prefix):
        raise ValueError(f"Unsupported mesh URI: {filename}")
    return description_root / filename[len(prefix) :]


def origin(element):
    origin_tag = element.find("origin")
    if origin_tag is None:
        return (0.0, 0.0, 0.0), rpy_matrix(0.0, 0.0, 0.0)
    xyz = parse_vector(origin_tag.attrib.get("xyz"), "0 0 0")
    rpy = parse_vector(origin_tag.attrib.get("rpy"), "0 0 0")
    return xyz, rpy_matrix(*rpy)


def primitive_distance(point, collision):
    translation, rotation = origin(collision)
    local = inverse_transform_point(point, translation, rotation)
    geometry = collision.find("geometry")
    box = geometry.find("box")
    cylinder = geometry.find("cylinder")
    sphere = geometry.find("sphere")
    if box is not None:
        half = tuple(value / 2.0 for value in parse_vector(box.attrib["size"], "0 0 0"))
        outside = tuple(max(0.0, abs(local[index]) - half[index]) for index in range(3))
        return math.sqrt(sum(value * value for value in outside))
    if cylinder is not None:
        radius = float(cylinder.attrib["radius"])
        half_length = float(cylinder.attrib["length"]) / 2.0
        radial = max(0.0, math.hypot(local[0], local[1]) - radius)
        axial = max(0.0, abs(local[2]) - half_length)
        return math.hypot(radial, axial)
    if sphere is not None:
        return max(0.0, math.sqrt(sum(value * value for value in local)) - float(sphere.attrib["radius"]))
    return math.inf


def format_bounds(points):
    return " ".join(
        f"{axis}=[{min(point[index] for point in points):+.3f},{max(point[index] for point in points):+.3f}]"
        for index, axis in enumerate("xyz")
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    parser.add_argument("description_root", type=Path)
    parser.add_argument("--tolerance-mm", type=float, default=2.0)
    args = parser.parse_args()

    root = ET.parse(args.urdf).getroot()
    tolerance = args.tolerance_mm / 1000.0
    print("Link                     verts  inside  <=tol  max_out(mm)  visual bounds in link frame")
    print("-" * 112)
    for link in root.findall("link"):
        visuals = link.findall("visual")
        collisions = link.findall("collision")
        visual_points = []
        for visual in visuals:
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            mesh_points = collada_points(resolve_mesh(mesh.attrib["filename"], args.description_root))
            mesh_scale = parse_vector(mesh.attrib.get("scale"), "1 1 1")
            translation, rotation = origin(visual)
            for point in mesh_points:
                scaled = tuple(point[index] * mesh_scale[index] for index in range(3))
                visual_points.append(transform_point(scaled, translation, rotation))

        if not visual_points:
            continue
        distances = [
            min((primitive_distance(point, collision) for collision in collisions), default=math.inf)
            for point in visual_points
        ]
        inside = sum(distance <= 1e-9 for distance in distances) / len(distances)
        near = sum(distance <= tolerance for distance in distances) / len(distances)
        maximum = max(distances)
        maximum_text = "mesh/none" if math.isinf(maximum) else f"{maximum * 1000.0:10.1f}"
        print(
            f"{link.attrib['name']:<24} {len(visual_points):>6} "
            f"{inside:>7.1%} {near:>6.1%} {maximum_text:>12}  {format_bounds(visual_points)}"
        )


if __name__ == "__main__":
    main()
