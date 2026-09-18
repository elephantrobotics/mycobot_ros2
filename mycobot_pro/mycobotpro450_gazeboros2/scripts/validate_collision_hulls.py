#!/usr/bin/env python3

"""Validate generated Pro450 collision hulls against every visual vertex."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
import trimesh

from audit_collision_geometry import (
    collada_points,
    origin,
    parse_vector,
    resolve_mesh,
    transform_point,
)


def resolve_collision_mesh(filename, description_root):
    return resolve_mesh(filename, description_root)


def bounds(points):
    return np.min(points, axis=0), np.max(points, axis=0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    parser.add_argument("description_root", type=Path)
    args = parser.parse_args()

    root = ET.parse(args.urdf).getroot()
    failures = []
    print("Link                     faces  max outside(mm)  visual size(mm)        hull size(mm)")
    print("-" * 102)
    for link in root.findall("link"):
        visual = link.find("visual")
        visual_mesh = visual.find("geometry/mesh") if visual is not None else None
        collision = link.find("collision")
        collision_mesh = collision.find("geometry/mesh") if collision is not None else None
        if visual_mesh is None:
            continue
        if collision_mesh is None:
            failures.append(f"{link.attrib['name']}: missing mesh collision")
            continue

        translation, rotation = origin(visual)
        mesh_scale = parse_vector(visual_mesh.attrib.get("scale"), "1 1 1")
        visual_points = []
        for point in collada_points(
            resolve_mesh(visual_mesh.attrib["filename"], args.description_root)
        ):
            scaled = tuple(point[index] * mesh_scale[index] for index in range(3))
            visual_points.append(transform_point(scaled, translation, rotation))
        visual_points = np.asarray(visual_points)

        hull_mesh = trimesh.load_mesh(
            resolve_collision_mesh(collision_mesh.attrib["filename"], args.description_root),
            process=True,
        )
        collision_translation, collision_rotation = origin(collision)
        hull_points = np.asarray(
            [
                transform_point(point, collision_translation, collision_rotation)
                for point in hull_mesh.vertices
            ]
        )
        convex = ConvexHull(hull_points)
        violations = visual_points @ convex.equations[:, :3].T + convex.equations[:, 3]
        maximum = float(np.max(violations))
        visual_min, visual_max = bounds(visual_points)
        hull_min, hull_max = bounds(hull_points)
        visual_size = (visual_max - visual_min) * 1000.0
        hull_size = (hull_max - hull_min) * 1000.0
        print(
            f"{link.attrib['name']:<24} {len(hull_mesh.faces):>5} "
            f"{maximum * 1000.0:>16.3f}  "
            f"{visual_size[0]:>5.0f}x{visual_size[1]:>5.0f}x{visual_size[2]:>5.0f}  "
            f"{hull_size[0]:>5.0f}x{hull_size[1]:>5.0f}x{hull_size[2]:>5.0f}"
        )
        if maximum > 1e-6:
            failures.append(
                f"{link.attrib['name']}: visual exceeds collision hull by "
                f"{maximum * 1000.0:.3f} mm"
            )
        if not hull_mesh.is_watertight:
            failures.append(f"{link.attrib['name']}: collision hull is not watertight")

    if failures:
        raise SystemExit("Collision validation failed:\n- " + "\n- ".join(failures))
    print("All visual vertices are enclosed by watertight collision hulls.")


if __name__ == "__main__":
    main()
