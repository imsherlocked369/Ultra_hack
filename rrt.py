"""Main entrypoint: load config.yaml and run the area-exploration mission.

Usage:
    python rrt.py [config.yaml]
"""
import os
import sys
import time
import csv
from dataclasses import dataclass
from typing import Optional

import yaml
import mav


@dataclass
class Metrics:
    start_time: float
    total_distance: float = 0.0
    viewpoints_visited: int = 0
    false_candidates_inspected: int = 0
    repeated_visits_to_rejected_candidates: int = 0
    rejected_objects: int = 0
    target_building_found: bool = False
    confirmed_target_id: Optional[str] = None


def load_config(path):
    if not os.path.exists(path):
        sys.exit(f"Config file not found: {path}")
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if not cfg:
        sys.exit(f"Config file is empty: {path}")
    return cfg


def write_metrics(metrics, output_file):
    mission_time = time.time() - metrics.start_time
    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value"])
        writer.writerow(["mission_time_sec", round(mission_time, 2)])
        writer.writerow(["total_distance_m", round(metrics.total_distance, 2)])
        writer.writerow(["viewpoints_visited", metrics.viewpoints_visited])
        writer.writerow(["false_candidates_inspected", metrics.false_candidates_inspected])
        writer.writerow(["repeated_visits_to_rejected_candidates", metrics.repeated_visits_to_rejected_candidates])
        writer.writerow(["rejected_objects", metrics.rejected_objects])
        writer.writerow(["target_building_found", metrics.target_building_found])
        writer.writerow(["confirmed_target_id", metrics.confirmed_target_id])
    print(f"[METRICS] Written to {output_file}")


def run_mission(cfg):
    grid = mav.ExplorationGrid(
        search_radius_m=cfg["SEARCH_RADIUS_M"],
        grid_cell_size_m=cfg["GRID_CELL_SIZE_M"],
        camera_range_m=cfg["CAMERA_RANGE_M"],
        camera_fov_deg=cfg["CAMERA_FOV_DEG"],
        rejection_radius_m=cfg["REJECTION_RADIUS_M"],
    )
    planner = mav.NBVPlanner(
        grid=grid,
        search_radius_m=cfg["SEARCH_RADIUS_M"],
        viewpoint_step_m=cfg["VIEWPOINT_STEP_M"],
        gain_weight=cfg["PLANNER_GAIN_WEIGHT"],
        distance_weight=cfg["PLANNER_DISTANCE_WEIGHT"],
        rejection_penalty=cfg["PLANNER_REJECTION_PENALTY"],
    )
    perception = mav.SimulatedPerception(
        objects=cfg["SIM_OBJECTS"],
        camera_range_m=cfg["CAMERA_RANGE_M"],
        camera_fov_deg=cfg["CAMERA_FOV_DEG"],
    )
    flight = mav.FlightClient(
        connection_string=cfg["MAVLINK_CONNECTION"],
        altitude_m=cfg["ALTITUDE_M"],
        arrival_radius_m=cfg["ARRIVAL_RADIUS_M"],
        max_move_wait_sec=cfg["MAX_MOVE_WAIT_SEC"],
    )

    metrics = Metrics(start_time=time.time())
    rejected_ids: set = set()
    last_xy = (0.0, 0.0)
    inspect_standoff_m = cfg["INSPECT_STANDOFF_M"]
    mission_time_limit_sec = cfg["MISSION_TIME_LIMIT_SEC"]

    flight.set_mode("GUIDED")
    flight.arm()
    flight.takeoff(cfg["TAKEOFF_ALTITUDE"])

    while time.time() - metrics.start_time < mission_time_limit_sec:
        px, py, _ = flight.get_local_position()
        current_xy = (px, py)

        vp = planner.choose_next_viewpoint(current_xy)
        if vp is None:
            print("[MISSION] No more viewpoints")
            break

        print(f"\n[NBV] Selected x={vp.x:.1f}, y={vp.y:.1f}, yaw={vp.yaw_deg:.1f}, score={vp.score:.1f}")

        if grid.is_in_rejected_zone(vp.x, vp.y):
            metrics.repeated_visits_to_rejected_candidates += 1

        flight.goto_local(vp.x, vp.y, vp.yaw_deg)
        metrics.total_distance += mav.distance(last_xy, (vp.x, vp.y))
        metrics.viewpoints_visited += 1
        last_xy = (vp.x, vp.y)

        grid.mark_visited(vp.x, vp.y)
        grid.mark_visible_cone(vp.x, vp.y, vp.yaw_deg)

        candidate = perception.detect_candidate((vp.x, vp.y), vp.yaw_deg, rejected_ids)
        if candidate is None:
            print("[PERCEPTION] No candidate visible")
            continue

        print(f"[PERCEPTION] Candidate detected: {candidate['id']} ({candidate['type']})")

        obj_xy = candidate["xy"]
        yaw_to_obj = mav.angle_deg((vp.x, vp.y), obj_xy)
        d_to_obj = mav.distance((vp.x, vp.y), obj_xy)

        if d_to_obj > inspect_standoff_m:
            ratio = max((d_to_obj - inspect_standoff_m) / d_to_obj, 0)
            inspect_x = vp.x + (obj_xy[0] - vp.x) * ratio
            inspect_y = vp.y + (obj_xy[1] - vp.y) * ratio
            print(f"[INSPECT] Approaching candidate {candidate['id']}")
            flight.goto_local(inspect_x, inspect_y, yaw_to_obj)
            metrics.total_distance += mav.distance(last_xy, (inspect_x, inspect_y))
            last_xy = (inspect_x, inspect_y)

        confirmed = perception.confirm_candidate(candidate)

        if confirmed:
            print(f"[MISSION] TARGET BUILDING CONFIRMED: {candidate['id']}")
            metrics.target_building_found = True
            metrics.confirmed_target_id = candidate["id"]
            break

        print(f"[MISSION] False candidate rejected: {candidate['id']}")
        rejected_ids.add(candidate["id"])
        grid.reject_object(candidate["id"], candidate["xy"])
        metrics.false_candidates_inspected += 1
        metrics.rejected_objects += 1

    if metrics.target_building_found:
        flight.land()
    else:
        flight.rtl()

    write_metrics(metrics, cfg["METRICS_OUTPUT_FILE"])


if __name__ == "__main__":
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
    cfg = load_config(config_path)
    print(f"[CONFIG] Loaded {config_path}")
    run_mission(cfg)
