#!/usr/bin/env python3

"""Convert the existing link-frame convex hulls to mesh-local Gazebo hulls.

The collision/*.stl files were generated after applying each visual origin, so
their vertices are already expressed in the URDF link frame.  The split
MoveIt/Gazebo URDF deliberately applies the visual origin to both mesh sets.
Gazebo therefore needs these hulls transformed back into the visual mesh frame
or the origin would be applied twice.

For the arm, this script preserves the existing convex hull triangle count.
The gripper COLLADA files are assemblies whose scene-node matrices were not
used by the legacy hull generator.  Their Gazebo meshes are rebuilt as tight
12-triangle PCA-oriented boxes around the correctly expanded exact scene.
"""

import argparse
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


OUTPUT_NAMES = {
    "base": "PRO450_J1_exact.stl",
    "link1": "PRO450_J2_exact.stl",
    "link2": "PRO450_J3_exact.stl",
    "link3": "PRO450_J4_exact.stl",
    "link4": "PRO450_J5_exact.stl",
    "link5": "PRO450_J6_exact.stl",
    "link6": "PRO450_J6_end_exact.stl",
    "gripper_connection": "mygripper_f100_connection_exact.stl",
    "gripper_base": "pro_gripper_base_exact.stl",
    "gripper_left1": "pro_gripper_left1_exact.stl",
    "gripper_left2": "pro_gripper_left2_exact.stl",
    "gripper_left3": "pro_gripper_left3_exact.stl",
    "gripper_right1": "pro_gripper_right1_exact.stl",
    "gripper_right2": "pro_gripper_right2_exact.stl",
    "gripper_right3": "pro_gripper_right3_exact.stl",
}


def vector(text, default="0 0 0"):
    return tuple(float(value) for value in (text or default).split())


def matrix_multiply(left, right):
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    return matrix_multiply(rz, matrix_multiply(ry, rx))


def inverse_transform(point, translation, rotation):
    shifted = tuple(point[index] - translation[index] for index in range(3))
    return tuple(
        sum(rotation[index][row] * shifted[index] for index in range(3))
        for row in range(3)
    )


def rotate_inverse(direction, rotation):
    return tuple(
        sum(rotation[index][row] * direction[index] for index in range(3))
        for row in range(3)
    )


def transform_forward(point, translation, rotation):
    return tuple(
        translation[row]
        + sum(rotation[row][index] * point[index] for index in range(3))
        for row in range(3)
    )


def read_binary_stl(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL is too short: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError(f"Expected binary STL: {path}")
    triangles = []
    offset = 84
    for _ in range(count):
        values = struct.unpack_from("<12fH", data, offset)
        triangles.append((values[0:3], values[3:6], values[6:9], values[9:12]))
        offset += 50
    return data[:80], triangles


def write_binary_stl(path, header, triangles):
    output = bytearray(header[:80].ljust(80, b"\0"))
    output.extend(struct.pack("<I", len(triangles)))
    for normal, first, second, third in triangles:
        output.extend(struct.pack("<12fH", *(normal + first + second + third), 0))
    path.write_bytes(output)


def triangle_normal(first, second, third):
    ab = np.asarray(second) - np.asarray(first)
    ac = np.asarray(third) - np.asarray(first)
    normal = np.cross(ab, ac)
    length = float(np.linalg.norm(normal))
    if length <= 1e-15:
        return (0.0, 0.0, 0.0)
    return tuple(float(value) for value in normal / length)


def oriented_box_from_stl(path, padding_m=0.0005):
    _header, triangles = read_binary_stl(path)
    points = np.asarray(
        [vertex for _normal, first, second, third in triangles for vertex in (first, second, third)],
        dtype=np.float64,
    )
    center = points.mean(axis=0)
    covariance = np.cov(points - center, rowvar=False)
    _values, axes = np.linalg.eigh(covariance)
    # Make the basis right-handed so face winding remains predictable.
    if np.linalg.det(axes) < 0.0:
        axes[:, 0] *= -1.0
    projected = (points - center) @ axes
    lower = projected.min(axis=0) - padding_m
    upper = projected.max(axis=0) + padding_m
    corners_local = np.asarray(
        [
            (x, y, z)
            for x in (lower[0], upper[0])
            for y in (lower[1], upper[1])
            for z in (lower[2], upper[2])
        ]
    )
    corners = center + corners_local @ axes.T
    # Corner index bits are x/y/z. Each triangle is wound outward.
    faces = (
        (0, 2, 3), (0, 3, 1),  # -x
        (4, 5, 7), (4, 7, 6),  # +x
        (0, 1, 5), (0, 5, 4),  # -y
        (2, 6, 7), (2, 7, 3),  # +y
        (0, 4, 6), (0, 6, 2),  # -z
        (1, 3, 7), (1, 7, 5),  # +z
    )
    result = []
    box_center = corners.mean(axis=0)
    for face in faces:
        first, second, third = (corners[index] for index in face)
        normal = np.cross(second - first, third - first)
        face_center = (first + second + third) / 3.0
        if float(np.dot(normal, face_center - box_center)) < 0.0:
            second, third = third, second
        vertices = tuple(tuple(float(value) for value in point) for point in (first, second, third))
        result.append((triangle_normal(*vertices), *vertices))
    return result, tuple(float(value) for value in upper - lower)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    parser.add_argument("source_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("exact_dir", type=Path)
    args = parser.parse_args()

    root = ET.parse(args.urdf).getroot()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    for link in root.findall("link"):
        name = link.attrib["name"]
        if name not in OUTPUT_NAMES:
            continue
        visual = link.find("visual")
        origin = visual.find("origin") if visual is not None else None
        translation = vector(origin.attrib.get("xyz") if origin is not None else None)
        rotation = rpy_matrix(
            *vector(origin.attrib.get("rpy") if origin is not None else None)
        )

        source = args.source_dir / f"{name}_collision.stl"
        destination = args.output_dir / OUTPUT_NAMES[name]
        if name.startswith("gripper"):
            exact = args.exact_dir / OUTPUT_NAMES[name]
            converted, dimensions = oriented_box_from_stl(exact)
            write_binary_stl(
                destination,
                f"Gazebo PCA box for {name}".encode("ascii"),
                converted,
            )
            print(
                f"{name:<24} triangles={len(converted):>6} "
                f"obb={dimensions[0]:.4f}x{dimensions[1]:.4f}x{dimensions[2]:.4f} "
                f"-> {destination}"
            )
            continue
        header, triangles = read_binary_stl(source)
        converted = []
        maximum_round_trip_error = 0.0
        for normal, first, second, third in triangles:
            local_vertices = tuple(
                inverse_transform(vertex, translation, rotation)
                for vertex in (first, second, third)
            )
            for original, local in zip((first, second, third), local_vertices):
                restored = transform_forward(local, translation, rotation)
                maximum_round_trip_error = max(
                    maximum_round_trip_error,
                    max(abs(restored[index] - original[index]) for index in range(3)),
                )
            converted.append(
                (rotate_inverse(normal, rotation), *local_vertices)
            )

        write_binary_stl(destination, header, converted)
        print(
            f"{name:<24} triangles={len(converted):>6} "
            f"round_trip_error={maximum_round_trip_error:.3e} -> {destination}"
        )


if __name__ == "__main__":
    main()
