#!/usr/bin/env python3
"""
批量转换 Matterport3D OBJ → 确定性 occupancy map (.npy + meta.json)

核心改动：固定 np.random.seed(42)，消除 voxelization 的非确定性。
输出目录结构：
    {output_dir}/{scan_id}/{scan_id}_res{resolution}.npy    — voxel world coords (N,3) float32
    {output_dir}/{scan_id}/{scan_id}_res{resolution}.json   — metadata

用法：
    python3 convert_all_meshes.py \
        --mesh-root /home/steam/project/dataset/matterport3d/v1/scans \
        --output-dir /home/steam/project/dataset/matterport3d_occmaps \
        --resolution 0.1
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Set, Tuple

import numpy as np


def load_obj_mesh(obj_path: str):
    """Load OBJ file → (vertices, faces)."""
    vertices, faces = [], []
    with open(obj_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'v':
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif parts[0] == 'f':
                face_verts = []
                for i in range(1, len(parts)):
                    face_verts.append(int(parts[i].split('/')[0]) - 1)
                for i in range(1, len(face_verts) - 1):
                    faces.append([face_verts[0], face_verts[i], face_verts[i + 1]])
    return np.array(vertices, dtype=np.float32), np.array(faces, dtype=np.int32)


def voxelize_triangle(v0, v1, v2, resolution, origin):
    """Voxelize a single triangle (deterministic: uses pre-seeded RNG)."""
    voxels = set()
    ox, oy, oz = origin

    num_samples = max(10, int(np.linalg.norm(v1 - v0) / resolution * 2),
                          int(np.linalg.norm(v2 - v0) / resolution * 2))

    for _ in range(num_samples):
        r1 = np.random.random()
        r2 = np.random.random()
        if r1 + r2 > 1:
            r1, r2 = 1 - r1, 1 - r2
        point = v0 + r1 * (v1 - v0) + r2 * (v2 - v0)
        ix = int(np.floor((point[0] - ox) / resolution))
        iy = int(np.floor((point[1] - oy) / resolution))
        iz = int(np.floor((point[2] - oz) / resolution))
        voxels.add((ix, iy, iz))

    for e0, e1 in [(v0, v1), (v1, v2), (v2, v0)]:
        edge_len = np.linalg.norm(e1 - e0)
        n_samples = max(2, int(edge_len / resolution * 2))
        for i in range(n_samples):
            t = i / (n_samples - 1) if n_samples > 1 else 0.5
            point = e0 + t * (e1 - e0)
            ix = int(np.floor((point[0] - ox) / resolution))
            iy = int(np.floor((point[1] - oy) / resolution))
            iz = int(np.floor((point[2] - oz) / resolution))
            voxels.add((ix, iy, iz))

    return voxels


def voxelize_mesh(vertices, faces, resolution, origin=(0, 0, 0)):
    """Voxelize full mesh → (N,3) float32 world coords."""
    all_voxels = set()
    n_faces = len(faces)
    for i, face in enumerate(faces):
        if i > 0 and i % 50000 == 0:
            print(f'    {i}/{n_faces} faces ({100*i/n_faces:.0f}%)', flush=True)
        v0, v1, v2 = vertices[face[0]], vertices[face[1]], vertices[face[2]]
        all_voxels.update(voxelize_triangle(v0, v1, v2, resolution, origin))

    ox, oy, oz = origin
    occupied = np.array([
        [ox + (ix + 0.5) * resolution,
         oy + (iy + 0.5) * resolution,
         oz + (iz + 0.5) * resolution]
        for ix, iy, iz in all_voxels
    ], dtype=np.float32)
    return occupied


def find_all_obj_meshes(root: str):
    """Find all Matterport3D OBJ files, return [(scan_id, obj_path), ...]."""
    results = []
    scans_dir = Path(root)
    for scan_dir in sorted(scans_dir.iterdir()):
        if not scan_dir.is_dir():
            continue
        scan_id = scan_dir.name
        mesh_dir = scan_dir / scan_id / 'matterport_mesh'
        if not mesh_dir.exists():
            continue
        for sub in mesh_dir.iterdir():
            obj = sub / f'{sub.name}.obj'
            if obj.exists():
                results.append((scan_id, str(obj)))
                break  # one OBJ per scan
    return results


def main():
    parser = argparse.ArgumentParser(description='Batch convert Matterport3D meshes to occmaps')
    parser.add_argument('--mesh-root', required=True,
                        help='Root dir containing scan_id/ folders')
    parser.add_argument('--output-dir', required=True,
                        help='Output directory for .npy + .json files')
    parser.add_argument('--resolution', type=float, default=0.1,
                        help='Voxel resolution in meters (default: 0.1)')
    parser.add_argument('--seed', type=int, default=42,
                        help='Random seed for deterministic voxelization')
    parser.add_argument('--skip-existing', action='store_true',
                        help='Skip scans that already have output files')
    args = parser.parse_args()

    meshes = find_all_obj_meshes(args.mesh_root)
    print(f'Found {len(meshes)} OBJ meshes in {args.mesh_root}')

    os.makedirs(args.output_dir, exist_ok=True)
    res_str = f'{args.resolution}'

    total_t0 = time.time()
    for idx, (scan_id, obj_path) in enumerate(meshes):
        out_dir = Path(args.output_dir) / scan_id
        npy_path = out_dir / f'{scan_id}_res{res_str}.npy'
        meta_path = out_dir / f'{scan_id}_res{res_str}.json'

        if args.skip_existing and npy_path.exists():
            print(f'[{idx+1}/{len(meshes)}] {scan_id}: skipping (exists)')
            continue

        print(f'[{idx+1}/{len(meshes)}] {scan_id}: loading {obj_path}')

        # Fixed seed per scan for reproducibility
        np.random.seed(args.seed)

        t0 = time.time()
        vertices, faces = load_obj_mesh(obj_path)
        print(f'  {len(vertices)} vertices, {len(faces)} faces')

        occupied = voxelize_mesh(vertices, faces, args.resolution)
        t1 = time.time()
        elapsed = t1 - t0

        os.makedirs(out_dir, exist_ok=True)
        np.save(npy_path, occupied)

        meta = {
            'scan_id': scan_id,
            'obj_path': obj_path,
            'resolution': args.resolution,
            'seed': args.seed,
            'n_voxels': int(occupied.shape[0]),
            'n_vertices': int(vertices.shape[0]),
            'n_faces': int(faces.shape[0]),
            'voxelization_time_s': round(elapsed, 2),
            'bbox_min': occupied.min(axis=0).tolist() if len(occupied) > 0 else [],
            'bbox_max': occupied.max(axis=0).tolist() if len(occupied) > 0 else [],
        }
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)

        print(f'  → {occupied.shape[0]} voxels, {elapsed:.1f}s → {npy_path}')

    total_elapsed = time.time() - total_t0
    print(f'\nDone: {len(meshes)} scans in {total_elapsed/60:.1f} min')


if __name__ == '__main__':
    main()
