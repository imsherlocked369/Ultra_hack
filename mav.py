import math
import time
from dataclasses import dataclass
from typing import Dict, Tuple

from pymavlink import mavutil


@dataclass
class Viewpoint:
    x: float
    y: float
    yaw_deg: float
    score: float


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
    def __init__(self, connection_string, altitude_m, arrival_radius_m, max_move_wait_sec):
        self.altitude_m = altitude_m
        self.arrival_radius_m = arrival_radius_m
        self.max_move_wait_sec = max_move_wait_sec
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
            return 0.0, 0.0, -self.altitude_m
        return float(msg.x), float(msg.y), float(msg.z)

    def goto_local(self, x, y, yaw_deg):
        print(f"[FLIGHT] Goto x={x:.1f}, y={y:.1f}, yaw={yaw_deg:.1f}")

        z = -self.altitude_m
        yaw_rad = math.radians(yaw_deg)

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        start = time.time()
        while time.time() - start < self.max_move_wait_sec:
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

            if d <= self.arrival_radius_m:
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
    def __init__(self, search_radius_m, grid_cell_size_m, camera_range_m, camera_fov_deg, rejection_radius_m):
        self.search_radius_m = search_radius_m
        self.grid_cell_size_m = grid_cell_size_m
        self.camera_range_m = camera_range_m
        self.camera_fov_deg = camera_fov_deg
        self.rejection_radius_m = rejection_radius_m
        self.visited: set = set()
        self.visible: set = set()
        self.rejected_zones: Dict[str, Tuple[float, float]] = {}

    def cell(self, x, y):
        return int(round(x / self.grid_cell_size_m)), int(round(y / self.grid_cell_size_m))

    def mark_visited(self, x, y):
        self.visited.add(self.cell(x, y))

    def mark_visible_cone(self, x, y, yaw_deg):
        r = self.search_radius_m
        step = self.grid_cell_size_m
        for gx in range(-r, r + 1, step):
            for gy in range(-r, r + 1, step):
                if not inside_radius(gx, gy, r):
                    continue
                if distance((x, y), (gx, gy)) > self.camera_range_m:
                    continue
                bearing = angle_deg((x, y), (gx, gy))
                if angle_diff_deg(bearing, yaw_deg) <= self.camera_fov_deg / 2:
                    self.visible.add(self.cell(gx, gy))

    def visible_unknown_count(self, x, y, yaw_deg):
        count = 0
        r = self.search_radius_m
        step = self.grid_cell_size_m
        for gx in range(-r, r + 1, step):
            for gy in range(-r, r + 1, step):
                if not inside_radius(gx, gy, r):
                    continue
                c = self.cell(gx, gy)
                if c in self.visible or c in self.visited:
                    continue
                if distance((x, y), (gx, gy)) > self.camera_range_m:
                    continue
                bearing = angle_deg((x, y), (gx, gy))
                if angle_diff_deg(bearing, yaw_deg) <= self.camera_fov_deg / 2:
                    count += 1
        return count

    def is_in_rejected_zone(self, x, y):
        for zone_xy in self.rejected_zones.values():
            if distance((x, y), zone_xy) <= self.rejection_radius_m:
                return True
        return False

    def reject_object(self, object_id, xy):
        self.rejected_zones[object_id] = xy


class SimulatedPerception:
    def __init__(self, objects, camera_range_m, camera_fov_deg):
        self.objects = objects
        self.camera_range_m = camera_range_m
        self.camera_fov_deg = camera_fov_deg

    def detect_candidate(self, drone_xy, yaw_deg, rejected_ids):
        detections = []
        for obj in self.objects:
            if obj["id"] in rejected_ids:
                continue
            obj_xy = obj["xy"]
            d = distance(drone_xy, obj_xy)
            if d > self.camera_range_m:
                continue
            bearing = angle_deg(drone_xy, obj_xy)
            if angle_diff_deg(bearing, yaw_deg) <= self.camera_fov_deg / 2:
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
    def __init__(self, grid, search_radius_m, viewpoint_step_m, gain_weight, distance_weight, rejection_penalty):
        self.grid = grid
        self.search_radius_m = search_radius_m
        self.viewpoint_step_m = viewpoint_step_m
        self.gain_weight = gain_weight
        self.distance_weight = distance_weight
        self.rejection_penalty = rejection_penalty

    def sample_viewpoints(self):
        points = []
        r = self.search_radius_m
        step = self.viewpoint_step_m
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
                self.gain_weight * visible_unknown
                - self.distance_weight * travel_distance
                - self.rejection_penalty * rejected_penalty
            )
            if best is None or score > best.score:
                best = Viewpoint(x, y, yaw, score)
        return best


