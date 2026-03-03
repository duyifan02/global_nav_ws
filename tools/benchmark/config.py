"""
Benchmark Configuration for 3D Path Planner Comparison

This configuration follows PCT Planner's experimental methodology for fair comparison
between PCT, 3D2M, and JPS planners.
"""

from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class SceneConfig:
    """Configuration for a test scenario"""
    name: str
    pcd_file: str  # Point cloud file path
    resolution: float  # Voxel resolution in meters
    start: Tuple[float, float, float]  # Start position (x, y, z)
    goal: Tuple[float, float, float]   # Goal position (x, y, z)
    z_ground: float = 0.0  # Ground level z-coordinate
    
    # Optional: multiple start/goal pairs for robustness testing
    additional_pairs: List[Tuple[Tuple[float, float, float], 
                                  Tuple[float, float, float]]] = None

# Test scenarios from PCT Paper
SCENES = {
    "spiral": SceneConfig(
        name="Spiral",
        pcd_file="spiral0.3_2.pcd",
        resolution=0.20,
        start=(-16.0, -6.0, 0.0),
        goal=(-26.0, -5.0, 0.0),
        z_ground=0.0
    ),
    
    "building": SceneConfig(
        name="Building",
        pcd_file="building2_9.pcd",
        resolution=0.10,
        start=(5.0, 5.0, 0.0),
        goal=(-6.0, -1.0, 0.0),
        z_ground=0.0
    ),
    
    "plaza": SceneConfig(
        name="Plaza",
        pcd_file="plaza3_10.pcd",
        resolution=0.10,
        start=(0.0, 0.0, 0.0),
        goal=(23.0, 10.0, 0.0),
        z_ground=0.0
    )
}

# Evaluation metrics (from PCT Paper Section IV-C)
METRICS = [
    "planning_time_ms",      # Total planning time in milliseconds
    "path_length_m",         # Euclidean path length in meters
    "success_rate",          # Success rate (0-1)
    "jerk_integral",         # Trajectory smoothness (∫||jerk(t)||² dt)
    "nodes_explored",        # Search efficiency metric
    "trajectory_duration_s", # Time to execute trajectory
]

# Experimental parameters
NUM_TRIALS = 50  # Number of trials per scenario (PCT paper uses 50)
TIMEOUT_SECONDS = 30.0  # Planning timeout

# Planner-specific configurations
PLANNER_CONFIGS = {
    "PCT": {
        "tomogram_dir": "../PCT_planner/rsc/tomogram/",
        "planning_script": "../PCT_planner/planner/scripts/plan.py",
        "ros_version": "1",
    },
    
    "3D2M": {
        "pcd_dir": "../3D2M-planner/planner/src/read_pcd/PCDFiles/",
        "launch_file": "../3D2M-planner/planner/launch/exp1.launch",
        "ros_version": "1",
    },
    
    "JPS": {
        "mesh_dir": "../global_nav_ws/maps/",
        "node_executable": "../global_nav_ws/install/jps_planner_3d/lib/jps_planner_3d/jps_planner_node",
        "ros_version": "2",
    }
}

# Output configuration
OUTPUT_DIR = "./results"
CSV_OUTPUT = True
PLOT_OUTPUT = True
SAVE_TRAJECTORIES = True

# Parameter sensitivity analysis (optional)
SENSITIVITY_ANALYSIS = {
    "resolution_sweep": [0.05, 0.10, 0.15, 0.20, 0.25],
    "heuristic_weight_sweep": [1.0, 1.2, 1.5, 2.0, 3.0],
}
