"""Unified voxel loading — .npy / .obj / format detection.

No ROS dependency; usable from benchmark scripts and ROS nodes alike.
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np


def load_npy(path: str) -> np.ndarray:
    """Load pre-built (N,3) float32 voxel centres from .npy."""
    arr = np.load(path)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f'Expected shape (N,3), got {arr.shape}: {path}')
    if arr.dtype != np.float32:
        arr = arr.astype(np.float32)
    return arr


def load_obj(path: str, resolution: float,
             origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
             auto_center: bool = False,
             logger=None) -> np.ndarray:
    """Load OBJ mesh, voxelize, return (N,3) float32 voxel centres."""
    from occmap_tools.obj_mesh_loader import load_obj_mesh, voxelize_mesh

    vertices, faces = load_obj_mesh(path)
    if vertices.size == 0 or faces.size == 0:
        raise ValueError(f'OBJ contains no geometry: {path}')

    if auto_center:
        origin = tuple(-vertices.mean(axis=0))

    pts = voxelize_mesh(vertices, faces, resolution, origin, logger=logger)
    if pts.size == 0:
        raise ValueError(f'Voxelization produced 0 voxels: {path}')
    return pts


def load_voxels(path: str, resolution: float = 0.1,
                origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                auto_center: bool = False,
                logger=None) -> Tuple[np.ndarray, float]:
    """Auto-detect format by suffix and load voxel centres.

    Returns (voxel_centres_float32, elapsed_ms).
    """
    suffix = Path(path).suffix.lower()
    t0 = time.time()
    if suffix == '.npy':
        pts = load_npy(path)
    elif suffix == '.obj':
        pts = load_obj(path, resolution, origin, auto_center, logger)
    else:
        raise ValueError(f'Unsupported format {suffix!r}: {path}')
    elapsed_ms = (time.time() - t0) * 1000.0
    return pts, elapsed_ms


def find_npy_for_scan(obj_path: str, resolution: float,
                      occmap_dir: Optional[str]) -> Optional[str]:
    """Look up a pre-built .npy occmap for a Matterport3D OBJ path.

    Convention: {occmap_dir}/{scan_id}/{scan_id}_res{resolution}.npy
    """
    if not occmap_dir:
        return None
    scan_id = _scan_id_from_obj_path(obj_path)
    if not scan_id:
        return None
    npy = Path(occmap_dir) / scan_id / f'{scan_id}_res{resolution}.npy'
    return str(npy) if npy.exists() else None


def _scan_id_from_obj_path(obj_path: str) -> Optional[str]:
    """Extract scan_id from OBJ path.

    Supports:
      - MP3D: .../scans/{scan_id}/...
      - HM3D: .../obj_scenes/{scan_id}.obj  (or any .obj where stem is scan_id)
    """
    p = Path(obj_path)
    parts = p.parts
    # MP3D convention: .../scans/{scan_id}/...
    for i, part in enumerate(parts):
        if part == 'scans' and i + 1 < len(parts):
            return parts[i + 1]
    # HM3D / generic: use filename stem
    if p.suffix.lower() == '.obj':
        return p.stem
    return None
