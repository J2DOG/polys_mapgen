#!/usr/bin/env python3
"""
Generate convex polyhedron obstacles in half-space form (A x + d <= 0) for robot
obstacle avoidance testing. Supports: box, tetrahedron, pyramid, triangular prism.
Output format is compatible with polys_mapgen (cfg YAML with 'hpolys').
"""

import os
import yaml
import random
import math
from typing import List, Tuple, Optional

# Default output path relative to this script's directory
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_OUT = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "cfg", "convex_obstacles.yaml"))

# Half-space format: [a, b, c, d] means a*x + b*y + c*z + d <= 0 (interior)


def vec3(x: float, y: float, z: float) -> List[float]:
    return [x, y, z]


def sub(a: List[float], b: List[float]) -> List[float]:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def cross(a: List[float], b: List[float]) -> List[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def dot(a: List[float], b: List[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def centroid(vertices: List[List[float]]) -> List[float]:
    n = len(vertices)
    cx = sum(v[0] for v in vertices) / n
    cy = sum(v[1] for v in vertices) / n
    cz = sum(v[2] for v in vertices) / n
    return [cx, cy, cz]


def plane_from_face(
    vertices: List[List[float]], face: List[int], center: List[float]
) -> List[float]:
    """One half-space [a,b,c,d] for face (list of vertex indices). Interior: a*x+b*y+c*z+d <= 0."""
    p0 = vertices[face[0]]
    p1 = vertices[face[1]]
    p2 = vertices[face[2]]
    e1 = sub(p1, p0)
    e2 = sub(p2, p0)
    n = cross(e1, e2)
    nlen = math.sqrt(dot(n, n))
    if nlen < 1e-10:
        return None
    n = [n[0] / nlen, n[1] / nlen, n[2] / nlen]
    # Interior: center on negative side of plane (n·(center - p0) <= 0)
    to_center = sub(center, p0)
    if dot(n, to_center) > 0:
        n = [-n[0], -n[1], -n[2]]
    # n·x + d <= 0 with d = -n·p0
    d = -dot(n, p0)
    return [n[0], n[1], n[2], d]


def vertices_to_halfspaces(
    vertices: List[List[float]], faces: List[List[int]]
) -> List[List[float]]:
    """Convert convex polyhedron (vertices + faces) to list of half-space constraints."""
    center = centroid(vertices)
    constraints = []
    for face in faces:
        h = plane_from_face(vertices, face, center)
        if h is not None:
            constraints.append(h)
    return constraints


def random_rotation_matrix() -> List[List[float]]:
    """Uniform random 3x3 rotation matrix (SO(3)) via quaternion."""
    # Uniform quaternion: normalize 4 Gaussians
    u = [random.gauss(0, 1) for _ in range(4)]
    n = math.sqrt(sum(x * x for x in u))
    if n < 1e-10:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    w, x, y, z = u[0] / n, u[1] / n, u[2] / n, u[3] / n
    # Quaternion to rotation matrix (w,x,y,z)
    return [
        [
            w * w + x * x - y * y - z * z,
            2 * (x * y - w * z),
            2 * (x * z + w * y),
        ],
        [
            2 * (x * y + w * z),
            w * w - x * x + y * y - z * z,
            2 * (y * z - w * x),
        ],
        [
            2 * (x * z - w * y),
            2 * (y * z + w * x),
            w * w - x * x - y * y + z * z,
        ],
    ]


def rotate_halfspaces(halfspaces: List[List[float]], R: List[List[float]]) -> List[List[float]]:
    """Apply rotation R to each half-space [a,b,c,d]: (a',b',c') = R @ (a,b,c), d unchanged."""
    out = []
    for (a, b, c, d) in halfspaces:
        na = R[0][0] * a + R[0][1] * b + R[0][2] * c
        nb = R[1][0] * a + R[1][1] * b + R[1][2] * c
        nc = R[2][0] * a + R[2][1] * b + R[2][2] * c
        out.append([na, nb, nc, d])
    return out


def random_in_map(
    map_origin: List[float],
    map_size: List[float],
    margin_xy: float,
    margin_z_low: float,
    margin_z_high: float,
) -> Tuple[float, float, float]:
    """Random center point so an obstacle stays inside [origin, origin+size]. margin_* = max extent from center."""
    x_lo = map_origin[0] + margin_xy
    x_hi = map_origin[0] + map_size[0] - margin_xy
    y_lo = map_origin[1] + margin_xy
    y_hi = map_origin[1] + map_size[1] - margin_xy
    z_lo = map_origin[2] + margin_z_low
    z_hi = map_origin[2] + map_size[2] - margin_z_high
    if x_lo >= x_hi or y_lo >= y_hi or z_lo >= z_hi:
        raise ValueError("Map too small for obstacle size (empty placement range)")
    x = random.uniform(x_lo, x_hi)
    y = random.uniform(y_lo, y_hi)
    z = random.uniform(z_lo, z_hi)
    return x, y, z


# --------------- Shape generators (vertices in local frame, then scale & translate) ---------------


def make_box(
    cx: float, cy: float, cz: float,
    hx: float, hy: float, hz: float,
) -> List[List[float]]:
    """Axis-aligned box: half-spaces [a,b,c,d] for a*x+b*y+c*z+d <= 0."""
    return [
        [1, 0, 0, -(cx + hx)],
        [-1, 0, 0, cx - hx],
        [0, 1, 0, -(cy + hy)],
        [0, -1, 0, cy - hy],
        [0, 0, 1, -(cz + hz)],
        [0, 0, -1, cz - hz],
    ]


def make_tetrahedron(
    cx: float, cy: float, cz: float,
    scale: float,
) -> List[List[float]]:
    """Regular tetrahedron centered at (cx,cy,cz) with edge scale."""
    # Unit tetrahedron vertices (centered at origin), then scale and translate
    s = scale / math.sqrt(2)
    v0 = [0, 0, s * math.sqrt(2/3)]
    v1 = [s, 0, -s / math.sqrt(6)]
    v2 = [-s/2, s * math.sqrt(3)/2, -s / math.sqrt(6)]
    v3 = [-s/2, -s * math.sqrt(3)/2, -s / math.sqrt(6)]
    verts = [
        [v0[0] + cx, v0[1] + cy, v0[2] + cz],
        [v1[0] + cx, v1[1] + cy, v1[2] + cz],
        [v2[0] + cx, v2[1] + cy, v2[2] + cz],
        [v3[0] + cx, v3[1] + cy, v3[2] + cz],
    ]
    faces = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]]
    return vertices_to_halfspaces(verts, faces)


def make_pyramid(
    cx: float, cy: float, cz: float,
    base_half: float,
    height: float,
) -> List[List[float]]:
    """Square-base pyramid: base at z=cz, apex at z=cz+height."""
    b = base_half
    verts = [
        [cx - b, cy - b, cz],
        [cx + b, cy - b, cz],
        [cx + b, cy + b, cz],
        [cx - b, cy + b, cz],
        [cx, cy, cz + height],
    ]
    faces = [
        [0, 1, 2], [0, 2, 3],  # base (two triangles)
        [4, 0, 1], [4, 1, 2], [4, 2, 3], [4, 3, 0],
    ]
    return vertices_to_halfspaces(verts, faces)


def make_triangular_prism(
    cx: float, cy: float, cz: float,
    base_radius: float,
    height: float,
    rotation_z: float = 0.0,
) -> List[List[float]]:
    """Triangular prism: triangular base in xy, extent in z. rotation_z in radians."""
    r = base_radius
    c, s = math.cos(rotation_z), math.sin(rotation_z)
    # Bottom triangle (z=0)
    p0 = [r, 0, 0]
    p1 = [r * math.cos(2*math.pi/3), r * math.sin(2*math.pi/3), 0]
    p2 = [r * math.cos(4*math.pi/3), r * math.sin(4*math.pi/3), 0]
    rot = lambda u: [c*u[0] - s*u[1], s*u[0] + c*u[1], u[2]]
    b0 = [cx + rot(p0)[0], cy + rot(p0)[1], cz]
    b1 = [cx + rot(p1)[0], cy + rot(p1)[1], cz]
    b2 = [cx + rot(p2)[0], cy + rot(p2)[1], cz]
    t0 = [b0[0], b0[1], cz + height]
    t1 = [b1[0], b1[1], cz + height]
    t2 = [b2[0], b2[1], cz + height]
    verts = [b0, b1, b2, t0, t1, t2]
    faces = [
        [0, 1, 2], [3, 4, 5],
        [0, 1, 4], [0, 4, 3],
        [1, 2, 5], [1, 5, 4],
        [2, 0, 3], [2, 3, 5],
    ]
    return vertices_to_halfspaces(verts, faces)


def make_wedge(
    cx: float, cy: float, cz: float,
    lx: float, ly: float, hz: float,
) -> List[List[float]]:
    """Wedge: rectangular base, top is a triangle (one edge of the box is at full height)."""
    verts = [
        [cx - lx/2, cy - ly/2, cz],
        [cx + lx/2, cy - ly/2, cz],
        [cx + lx/2, cy + ly/2, cz],
        [cx - lx/2, cy + ly/2, cz],
        [cx - lx/2, cy - ly/2, cz + hz],
        [cx + lx/2, cy - ly/2, cz + hz],
        [cx + lx/2, cy + ly/2, cz + hz],
    ]
    faces = [
        [0, 1, 2], [0, 2, 3],
        [0, 1, 5], [0, 5, 4],
        [1, 2, 6], [1, 6, 5],
        [2, 3, 6],
        [3, 0, 4], [3, 4, 6],
        [4, 5, 6],
    ]
    return vertices_to_halfspaces(verts, faces)


# --------------- Random scene generation ---------------


def generate_mixed_obstacles(
    num_obstacles: int,
    map_origin_x: float, map_origin_y: float, map_origin_z: float,
    map_size_x: float, map_size_y: float, map_size_z: float,
    size_min: float, size_max: float,
    shape_weights: Optional[dict] = None,
) -> List[List[List[float]]]:
    """
    Generate a list of convex polytopes (each is list of [a,b,c,d] half-spaces).
    Obstacle centers are placed so the full obstacle stays inside the map region
    [origin, origin+size]. shape_weights: e.g. {'box': 3, 'tetrahedron': 1}. Default: equal.
    """
    map_origin = [map_origin_x, map_origin_y, map_origin_z]
    map_size = [map_size_x, map_size_y, map_size_z]
    # Per-axis margins: max extent from obstacle center so it stays inside map
    margin_xy = min(size_max, map_size_x / 2 - 0.01, map_size_y / 2 - 0.01)
    margin_z_low = size_max / 2  # box/prism extend below center
    # Cap margin_z_high so z placement range is never empty (map_size_z can be < 1.5*size_max)
    margin_z_high = min(size_max, map_size_z - margin_z_low - 0.01)
    if margin_xy <= 0:
        raise ValueError(
            "Map XY too small for obstacle size: need map_size_x and map_size_y >= 2*size_max (see cfg/mapgen.yaml)"
        )
    if margin_z_high <= 0:
        raise ValueError(
            "Map Z too small: need map_size_z > size_max/2 (see cfg/mapgen.yaml)"
        )

    if shape_weights is None:
        shape_weights = {"box": 1, "tetrahedron": 1, "pyramid": 1, "prism": 1, "wedge": 1}

    choices = []
    for name, w in shape_weights.items():
        choices.extend([name] * max(1, int(w)))
    if not choices:
        choices = ["box"]

    hpolys = []
    for _ in range(num_obstacles):
        try:
            cx, cy, cz = random_in_map(
                map_origin, map_size, margin_xy, margin_z_low, margin_z_high
            )
        except ValueError:
            continue
        shape = random.choice(choices)
        s = random.uniform(size_min, size_max)
        # Clamp dimensions so obstacle stays inside map (z height especially)
        max_hz = max(0.1, map_size_z - (cz - map_origin_z))
        max_hz_center = min(size_max, cz - map_origin_z, map_origin_z + map_size_z - cz)

        if shape == "box":
            hx = random.uniform(size_min / 2, min(size_max / 2, margin_xy))
            hy = random.uniform(size_min / 2, min(size_max / 2, margin_xy))
            hz = random.uniform(size_min / 2, min(size_max / 2, margin_z_high, max_hz_center))
            obs = make_box(cx, cy, cz, hx, hy, hz)
        elif shape == "tetrahedron":
            # tetrahedron circumradius ~ 0.61*scale; keep inside map
            s_tet = min(s, margin_xy / 0.62, margin_z_high / 0.62)
            obs = make_tetrahedron(cx, cy, cz, s_tet)
        elif shape == "pyramid":
            base_half = random.uniform(size_min / 2, min(size_max / 2, margin_xy))
            height = random.uniform(size_min, min(size_max, max_hz))
            obs = make_pyramid(cx, cy, cz, base_half, height)
        elif shape == "prism":
            base_r = random.uniform(size_min / 2, min(size_max / 2, margin_xy))
            height = random.uniform(size_min, min(size_max, max_hz))
            rot = random.uniform(0, 2 * math.pi)
            obs = make_triangular_prism(cx, cy, cz, base_r, height, rot)
        elif shape == "wedge":
            lx = random.uniform(size_min, min(size_max, 2 * margin_xy))
            ly = random.uniform(size_min, min(size_max, 2 * margin_xy))
            hz = random.uniform(size_min, min(size_max, max_hz))
            obs = make_wedge(cx, cy, cz, lx, ly, hz)
        else:
            obs = make_box(cx, cy, cz, s/2, s/2, s/2)

        if obs:
            # R = random_rotation_matrix()
            # obs = rotate_halfspaces(obs, R)
            hpolys.append(obs)
    return hpolys


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Generate convex polyhedron obstacles for polys_mapgen")
    parser.add_argument("--num", type=int, default=30, help="Number of obstacles")
    # Defaults must match cfg/mapgen.yaml so obstacles are inside the map region
    parser.add_argument("--map-size", type=float, nargs=3, default=[20.0, 20.0, 5.0],
                        metavar=("X", "Y", "Z"), help="Map size (same as mapgen.yaml)")
    parser.add_argument("--map-origin", type=float, nargs=3, default=[-10.0, -10.0, 0.0],
                        metavar=("X", "Y", "Z"), help="Map origin (same as mapgen.yaml)")
    parser.add_argument("--size-min", type=float, default=1.0, help="Min obstacle scale (m)")
    parser.add_argument("--size-max", type=float, default=5.0, help="Max obstacle scale (m)")
    parser.add_argument("--out", type=str, default=_DEFAULT_OUT,
                        help="Output YAML path")
    parser.add_argument("--seed", type=int, default=None, help="Random seed")
    parser.add_argument("--box", type=int, default=1, help="Weight for box shape")
    parser.add_argument("--tetrahedron", type=int, default=1, help="Weight for tetrahedron")
    parser.add_argument("--pyramid", type=int, default=1, help="Weight for pyramid")
    parser.add_argument("--prism", type=int, default=1, help="Weight for triangular prism")
    parser.add_argument("--wedge", type=int, default=1, help="Weight for wedge")
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    shape_weights = {
        "box": args.box,
        "tetrahedron": args.tetrahedron,
        "pyramid": args.pyramid,
        "prism": args.prism,
        "wedge": args.wedge,
    }
    hpolys = generate_mixed_obstacles(
        num_obstacles=args.num,
        map_origin_x=args.map_origin[0], map_origin_y=args.map_origin[1], map_origin_z=args.map_origin[2],
        map_size_x=args.map_size[0], map_size_y=args.map_size[1], map_size_z=args.map_size[2],
        size_min=args.size_min, size_max=args.size_max,
        shape_weights=shape_weights,
    )
    config = {"hpolys": [[list(c) for c in poly] for poly in hpolys]}
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w") as f:
        yaml.dump(config, f, default_flow_style=None)
    print(f"Generated {len(hpolys)} convex obstacles -> {args.out}")


if __name__ == "__main__":
    main()
