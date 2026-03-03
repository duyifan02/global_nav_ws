"""
Metrics calculation utilities for planner benchmarking.

Implements evaluation metrics from PCT Paper Section IV-C:
- Planning time (ms)
- Path length (m)
- Success rate (%)
- Jerk integral (trajectory smoothness)
- Search efficiency (nodes explored)
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

@dataclass
class PlanningResult:
    """Result from a single planning trial"""
    success: bool
    planning_time_ms: float
    path_length_m: Optional[float] = None
    jerk_integral: Optional[float] = None
    nodes_explored: Optional[int] = None
    trajectory_duration_s: Optional[float] = None
    waypoints: Optional[np.ndarray] = None  # Nx3 array of (x,y,z)
    trajectory: Optional[np.ndarray] = None  # Mx3 array of dense trajectory points
    error_message: Optional[str] = None


def compute_path_length(waypoints: np.ndarray) -> float:
    """
    Compute Euclidean path length from waypoints.
    
    Args:
        waypoints: Nx3 array of (x, y, z) positions
        
    Returns:
        Total path length in meters
    """
    if waypoints is None or len(waypoints) < 2:
        return 0.0
    
    diffs = np.diff(waypoints, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    return np.sum(segment_lengths)


def compute_jerk_integral(trajectory: np.ndarray, timestamps: np.ndarray) -> float:
    """
    Compute jerk integral: ∫||jerk(t)||² dt
    
    This measures trajectory smoothness. Lower is smoother.
    
    Args:
        trajectory: Mx3 array of (x, y, z) positions
        timestamps: M-length array of timestamps
        
    Returns:
        Jerk integral value
    """
    if trajectory is None or len(trajectory) < 4:
        return float('inf')
    
    # Numerical differentiation to get velocity, acceleration, jerk
    dt = np.diff(timestamps)
    
    # Velocity: v = dp/dt
    velocity = np.diff(trajectory, axis=0) / dt[:, np.newaxis]
    
    # Acceleration: a = dv/dt
    accel = np.diff(velocity, axis=0) / dt[1:, np.newaxis]
    
    # Jerk: j = da/dt
    jerk = np.diff(accel, axis=0) / dt[2:, np.newaxis]
    
    # Jerk magnitude squared
    jerk_squared = np.sum(jerk**2, axis=1)
    
    # Integrate over time
    jerk_integral = np.trapz(jerk_squared, timestamps[:-3])
    
    return jerk_integral


def compute_trajectory_duration(trajectory: np.ndarray, 
                                timestamps: np.ndarray,
                                max_velocity: float = 3.0) -> float:
    """
    Estimate trajectory execution duration.
    
    Args:
        trajectory: Mx3 array of (x, y, z) positions
        timestamps: M-length array of timestamps
        max_velocity: Maximum allowed velocity (m/s)
        
    Returns:
        Trajectory duration in seconds
    """
    if timestamps is not None and len(timestamps) > 0:
        return timestamps[-1] - timestamps[0]
    
    # Fallback: estimate from path length and max velocity
    path_length = compute_path_length(trajectory)
    return path_length / max_velocity


def aggregate_results(results: List[PlanningResult]) -> dict:
    """
    Aggregate results from multiple trials.
    
    Args:
        results: List of PlanningResult from multiple trials
        
    Returns:
        Dictionary with aggregated statistics
    """
    successful = [r for r in results if r.success]
    
    stats = {
        "total_trials": len(results),
        "successful_trials": len(successful),
        "success_rate": len(successful) / len(results) if results else 0.0,
    }
    
    if not successful:
        return stats
    
    # Planning time statistics
    planning_times = [r.planning_time_ms for r in successful]
    stats["planning_time_mean_ms"] = np.mean(planning_times)
    stats["planning_time_std_ms"] = np.std(planning_times)
    stats["planning_time_median_ms"] = np.median(planning_times)
    stats["planning_time_min_ms"] = np.min(planning_times)
    stats["planning_time_max_ms"] = np.max(planning_times)
    
    # Path length statistics
    path_lengths = [r.path_length_m for r in successful if r.path_length_m is not None]
    if path_lengths:
        stats["path_length_mean_m"] = np.mean(path_lengths)
        stats["path_length_std_m"] = np.std(path_lengths)
        stats["path_length_median_m"] = np.median(path_lengths)
    
    # Jerk integral statistics
    jerk_integrals = [r.jerk_integral for r in successful if r.jerk_integral is not None]
    if jerk_integrals:
        stats["jerk_integral_mean"] = np.mean(jerk_integrals)
        stats["jerk_integral_std"] = np.std(jerk_integrals)
        stats["jerk_integral_median"] = np.median(jerk_integrals)
    
    # Nodes explored statistics
    nodes_explored = [r.nodes_explored for r in successful if r.nodes_explored is not None]
    if nodes_explored:
        stats["nodes_explored_mean"] = np.mean(nodes_explored)
        stats["nodes_explored_std"] = np.std(nodes_explored)
        stats["nodes_explored_median"] = np.median(nodes_explored)
    
    # Trajectory duration statistics
    durations = [r.trajectory_duration_s for r in successful if r.trajectory_duration_s is not None]
    if durations:
        stats["trajectory_duration_mean_s"] = np.mean(durations)
        stats["trajectory_duration_std_s"] = np.std(durations)
    
    return stats


def save_results_csv(results: List[PlanningResult], 
                     planner_name: str,
                     scene_name: str,
                     output_path: str):
    """
    Save trial results to CSV file.
    
    Args:
        results: List of PlanningResult
        planner_name: Name of the planner (PCT, 3D2M, JPS)
        scene_name: Name of the scene (Spiral, Building, Plaza)
        output_path: Path to output CSV file
    """
    import csv
    
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Header
        writer.writerow([
            "planner", "scene", "trial", "success",
            "planning_time_ms", "path_length_m", "jerk_integral",
            "nodes_explored", "trajectory_duration_s", "error"
        ])
        
        # Data rows
        for i, result in enumerate(results):
            writer.writerow([
                planner_name,
                scene_name,
                i,
                result.success,
                result.planning_time_ms if result.success else None,
                result.path_length_m if result.success else None,
                result.jerk_integral if result.success else None,
                result.nodes_explored if result.success else None,
                result.trajectory_duration_s if result.success else None,
                result.error_message if not result.success else ""
            ])


def compare_planners(results_dict: dict) -> dict:
    """
    Compare multiple planners on same scenario.
    
    Args:
        results_dict: Dict mapping planner_name -> List[PlanningResult]
        
    Returns:
        Comparison statistics dictionary
    """
    comparison = {}
    
    for planner_name, results in results_dict.items():
        comparison[planner_name] = aggregate_results(results)
    
    return comparison
