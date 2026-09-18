#!/usr/bin/env python3

"""Generate low-complexity convex collision STL files from URDF visuals."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial import ConvexHull, HalfspaceIntersection

from audit_collision_geometry import (
    collada_points,
    origin,
    parse_vector,
    resolve_mesh,
    transform_point,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    parser.add_argument("description_root", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--padding-mm", type=float, default=0.5)
    # 1024 direction samples preserve the long, thin Pro450 links without the
    # excessive enlargement seen with a very coarse convex approximation.
    parser.add_argument("--target-faces", type=int, default=1024)
    args = parser.parse_args()

    root = ET.parse(args.urdf).getroot()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    for link in root.findall("link"):
        points = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            mesh_points = collada_points(
                resolve_mesh(mesh.attrib["filename"], args.description_root)
            )
            mesh_scale = parse_vector(mesh.attrib.get("scale"), "1 1 1")
            translation, rotation = origin(visual)
            for point in mesh_points:
                scaled = tuple(point[index] * mesh_scale[index] for index in range(3))
                points.append(transform_point(scaled, translation, rotation))

        if len(points) < 4:
            continue

        cloud = np.asarray(points, dtype=np.float64)
        hull = trimesh.convex.convex_hull(cloud, qhull_options="QbB Pp Qt")
        if len(hull.faces) > args.target_faces:
            reduced = hull.simplify_quadric_decimation(
                face_count=args.target_faces, aggression=8
            )
            hull = reduced.convex_hull

        # Use the reduced hull only for its facet directions. Move each plane
        # independently to the support plane of the original visual cloud.
        # Their intersection is a tight outer approximation; unlike uniform
        # scaling this does not grossly enlarge long or thin links.
        padding = args.padding_mm / 1000.0
        equations = ConvexHull(hull.vertices).equations
        normals = equations[:, :3]
        supports = np.max(cloud @ normals.T, axis=0) + padding
        outer_halfspaces = np.column_stack((normals, -supports))
        interior = np.mean(cloud, axis=0)
        intersections = HalfspaceIntersection(
            outer_halfspaces, interior, qhull_options="QJ"
        ).intersections
        # STL stores float32 coordinates; quantize to 0.1 micrometre before
        # triangulation so shared vertices remain identical after export.
        intersections = np.unique(np.round(intersections, decimals=7), axis=0)
        outer = ConvexHull(intersections)
        faces = outer.simplices.copy()
        # scipy returns one outward plane normal for each triangular facet,
        # but does not guarantee triangle winding. Orient every triangle
        # directly from that normal so STL consumers see a closed solid.
        for index, face in enumerate(faces):
            p0, p1, p2 = intersections[face]
            triangle_normal = np.cross(p1 - p0, p2 - p0)
            if np.dot(triangle_normal, outer.equations[index, :3]) < 0.0:
                faces[index, 1], faces[index, 2] = faces[index, 2], faces[index, 1]
        hull = trimesh.Trimesh(
            vertices=intersections, faces=faces, process=False
        )
        hull.remove_unreferenced_vertices()
        hull.fix_normals()
        output = args.output_dir / f"{link.attrib['name']}_collision.stl"
        hull.export(output)
        print(
            f"{link.attrib['name']:<24} vertices={len(hull.vertices):>5} "
            f"faces={len(hull.faces):>5} watertight={hull.is_watertight} -> {output}"
        )


if __name__ == "__main__":
    main()
