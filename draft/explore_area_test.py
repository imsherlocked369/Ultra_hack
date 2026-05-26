#!/usr/bin/env python3

### DEPRICATED - This file is no longer maintained and may not work with current versions of the simulator. It is kept for historical reference only. IGNORE this file.


import math
import time
import csv
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

from pymavlink import mavutil


TAKEOFF_ALTITUDE = 5.0
SEARCH_RADIUS_M = 60
GRID_CELL_SIZE_M = 5
CAMERA_RANGE_M = 25
CAMERA_FOV_DEG = 70
REJECTION_RADIUS_M = 8
MISSION_TIME_LIMIT_SEC = 15 * 60
VIEWPOINT_STEP_M = 8
ALTITUDE_M = 5.0
MAVLINK_CONNECTION = "udpin:0.0.0.0:14500"
ARRIVAL_RADIUS_M = 2.5
MAX_MOVE_WAIT_SEC = 60


SIM_OBJECTS = [
    {"id": "F1", "xy": (15, 10), "type": "false_cuboid"},
    {"id": "F2", "xy": (-20, 15), "type": "false_cuboid"},
    {"id": "B1", "xy": (35, -20), "type": "target_building"},
]


@dataclass
class Viewpoint:
    x: float
    y: float
    yaw_deg: float
    score: float


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


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def angle_deg(from_xy, to_xy):
    dx = to_xy[0] - from_xy[0]
    dy = to_xy[1] - from_xy[1]
    return math.degrees(math.atan2(dy, dx))


def angle_diff_deg(a, b):
    return abs((a - b + 180) % 360 - 180)


def inside_radius(x, y, r):
    return math.hypot(x, y) <= r


class FlightClient:
    def __init__(self, connection_string):
        print(f"[MAVLINK] Connecting on {connection_string}")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print("[MAVLINK] Heartbeat received")

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print(f"[FLIGHT] Mode: {mode_name}")
        time.sleep(2)

    def arm(self):
        print("[FLIGHT] Arming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        self.master.motors_armed_wait()
        print("[FLIGHT] Armed")

    def takeoff(self, altitude_m):
        print(f"[FLIGHT] Takeoff to {altitude_m} m")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            altitude_m,
        )
        time.sleep(8)

    def get_local_position(self):
        msg = self.master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg is None:
            return 0.0, 0.0, -ALTITUDE_M
        return float(msg.x), float(msg.y), float(msg.z)

    def goto_local(self, x, y, altitude_m, yaw_deg):
        print(f"[FLIGHT] Goto x={x:.1f}, y={y:.1f}, yaw={yaw_deg:.1f}")

        z = -altitude_m
        yaw_rad = math.radians(yaw_deg)

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        start = time.time()
        while time.time() - start < MAX_MOVE_WAIT_SEC:
            self.master.mav.set_position_target_local_ned_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                x, y, z,
                0, 0, 0,
                0, 0, 0,
                yaw_rad,
                0,
            )

            px, py, _ = self.get_local_position()
            d = distance((px, py), (x, y))
            print(f"  current=({px:.1f}, {py:.1f}), dist={d:.1f}")

            if d <= ARRIVAL_RADIUS_M:
                print("[FLIGHT] Reached viewpoint")
                return

            time.sleep(1)

        print("[WARN] Move timeout, continuing")

    def land(self):
        print("[FLIGHT] Landing")
        self.set_mode("LAND")

    def rtl(self):
        print("[FLIGHT] RTL")
        self.set_mode("RTL")


class ExplorationGrid:
    def __init__(self):
        self.visited = set()
        self.visible = set()
        self.rejected_zones: Dict[str, Tuple[float, float]] = {}

    def cell(self, x, y):
        return int(round(x / GRID_CELL_SIZE_M)), int(round(y / GRID_CELL_SIZE_M))

    def mark_visited(self, x, y):
        self.visited.add(self.cell(x, y))

    def mark_visible_cone(self, x, y, yaw_deg):
        for gx in range(-SEARCH_RADIUS_M, SEARCH_RADIUS_M + 1, GRID_CELL_SIZE_M):
            for gy in range(-SEARCH_RADIUS_M, SEARCH_RADIUS_M + 1, GRID_CELL_SIZE_M):
                if not inside_radius(gx, gy, SEARCH_RADIUS_M):
                    continue
                if distance((x, y), (gx, gy)) > CAMERA_RANGE_M:
                    continue
                bearing = angle_deg((x, y), (gx, gy))
                if angle_diff_deg(bearing, yaw_deg) <= CAMERA_FOV_DEG / 2:
                    self.visible.add(self.cell(gx, gy))

    def visible_unknown_count(self, x, y, yaw_deg):
        count = 0
        for gx in range(-SEARCH_RADIUS_M, SEARCH_RADIUS_M + 1, GRID_CELL_SIZE_M):
            for gy in range(-SEARCH_RADIUS_M, SEARCH_RADIUS_M + 1, GRID_CELL_SIZE_M):
                if not inside_radius(gx, gy, SEARCH_RADIUS_M):
                    continue
                c = self.cell(gx, gy)
                if c in self.visible or c in self.visited:
                    continue
                if distance((x, y), (gx, gy)) > CAMERA_RANGE_M:
                    continue
                bearing = angle_deg((x, y), (gx, gy))
                if angle_diff_deg(bearing, yaw_deg) <= CAMERA_FOV_DEG / 2:
                    count += 1
        return count

    def is_in_rejected_zone(self, x, y):
        for zone_xy in self.rejected_zones.values():
            if distance((x, y), zone_xy) <= REJECTION_RADIUS_M:
                return True
        return False

    def reject_object(self, object_id, xy):
        self.rejected_zones[object_id] = xy


class SimulatedPerception:
    def __init__(self, objects):
        self.objects = objects

    def detect_candidate(self, drone_xy, yaw_deg, rejected_ids):
        detections = []

        for obj in self.objects:
            if obj["id"] in rejected_ids:
                continue

            obj_xy = obj["xy"]
            d = distance(drone_xy, obj_xy)

            if d > CAMERA_RANGE_M:
                continue

            bearing = angle_deg(drone_xy, obj_xy)

            if angle_diff_deg(bearing, yaw_deg) <= CAMERA_FOV_DEG / 2:
                detections.append((d, obj))

        if not detections:
            return None

        detections.sort(key=lambda item: item[0])
        return detections[0][1]

    def confirm_candidate(self, obj):
        print(f"[PERCEPTION] Stabilizing detection for {obj['id']}")
        for i in range(3):
            print(f"  stable frame {i + 1}/3")
            time.sleep(0.5)

        return obj["type"] == "target_building"


class NBVPlanner:
    def __init__(self, grid):
        self.grid = grid

    def sample_viewpoints(self):
        points = []
        r = SEARCH_RADIUS_M
        step = VIEWPOINT_STEP_M

        x = -r
        while x <= r:
            y = -r
            while y <= r:
                if inside_radius(x, y, r):
                    points.append((x, y))
                y += step
            x += step

        return points

    def best_yaw_for_viewpoint(self, x, y):
        yaw_options = [0, 45, 90, 135, 180, -135, -90, -45]
        best_yaw = 0
        best_gain = -1

        for yaw in yaw_options:
            gain = self.grid.visible_unknown_count(x, y, yaw)
            if gain > best_gain:
                best_gain = gain
                best_yaw = yaw

        return best_yaw

    def choose_next_viewpoint(self, current_xy):
        best = None

        for x, y in self.sample_viewpoints():
            if self.grid.cell(x, y) in self.grid.visited:
                continue

            yaw = self.best_yaw_for_viewpoint(x, y)
            visible_unknown = self.grid.visible_unknown_count(x, y, yaw)
            travel_distance = distance(current_xy, (x, y))
            rejected_penalty = 1 if self.grid.is_in_rejected_zone(x, y) else 0

            score = (
                3.0 * visible_unknown
                - 2.0 * travel_distance
                - 5.0 * rejected_penalty
            )

            if best is None or score > best.score:
                best = Viewpoint(x, y, yaw, score)

        return best


def write_metrics(metrics):
    mission_time = time.time() - metrics.start_time

    with open("explore_area_metrics.csv", "w", newline="") as f:
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

    print("[METRICS] Written to explore_area_metrics.csv")


def main():
    metrics = Metrics(start_time=time.time())

    grid = ExplorationGrid()
    planner = NBVPlanner(grid)
    perception = SimulatedPerception(SIM_OBJECTS)
    rejected_ids = set()

    flight = FlightClient(MAVLINK_CONNECTION)

    flight.set_mode("GUIDED")
    flight.arm()
    flight.takeoff(TAKEOFF_ALTITUDE)

    last_xy = (0.0, 0.0)

    while time.time() - metrics.start_time < MISSION_TIME_LIMIT_SEC:
        px, py, _ = flight.get_local_position()
        current_xy = (px, py)

        vp = planner.choose_next_viewpoint(current_xy)

        if vp is None:
            print("[MISSION] No more viewpoints")
            break

        print(
            f"\n[NBV] Selected x={vp.x:.1f}, y={vp.y:.1f}, "
            f"yaw={vp.yaw_deg:.1f}, score={vp.score:.1f}"
        )

        if grid.is_in_rejected_zone(vp.x, vp.y):
            metrics.repeated_visits_to_rejected_candidates += 1

        flight.goto_local(vp.x, vp.y, ALTITUDE_M, vp.yaw_deg)

        metrics.total_distance += distance(last_xy, (vp.x, vp.y))
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
        yaw_to_obj = angle_deg((vp.x, vp.y), obj_xy)
        d_to_obj = distance((vp.x, vp.y), obj_xy)

        if d_to_obj > 8:
            ratio = max((d_to_obj - 8) / d_to_obj, 0)
            inspect_x = vp.x + (obj_xy[0] - vp.x) * ratio
            inspect_y = vp.y + (obj_xy[1] - vp.y) * ratio

            print(f"[INSPECT] Approaching candidate {candidate['id']}")
            flight.goto_local(inspect_x, inspect_y, ALTITUDE_M, yaw_to_obj)

            metrics.total_distance += distance(last_xy, (inspect_x, inspect_y))
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

    write_metrics(metrics)


if __name__ == "__main__":
    main()