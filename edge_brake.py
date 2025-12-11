#!/usr/bin/env python3
"""
edge_brake_multi.py — leader stops mid-track then resumes; every other vehicle follows the vehicle directly ahead.
Writes per-step logs to CSV.
Based on your original edge_brake.py logic (kept behavior names & tuning).
"""

import traci
import sys
import math
import csv
import time

# ===== Configuration (tune these values) =====
SUMO_BINARY = "sumo-gui"
SUMO_CONFIG = "simple.sumocfg"

LEADER_ID = "leader"

# Behavior positions (meters along lane)
SLOW_ZONE_START = 200.0
STOP_POSITION = 200.0
STOP_DURATION = 80          # seconds the leader stays stopped
FAST_ZONE_START = 700.0

# Speeds (m/s)
NORMAL_SPEED = 8.0
SLOW_SPEED = 2.0
FAST_SPEED = 20.0

BRAKE_TIME = 2.0

# Safety model for followers
BASE_SAFE_DIST = 5.0
REACTION_TIME = 1.0
DECEL = 4.5

# Simulation control
MAX_STEPS = 20000
LOG_CSV = "edge_log_multi.csv"
STEP_WARMUP = 5
# ==============================================


def calc_safe_dist(v):
    """Simple safe distance: base + reaction + braking distance."""
    return BASE_SAFE_DIST + v * REACTION_TIME + (v ** 2) / (2 * DECEL)


def get_lane_pos(vehicle_id):
    try:
        return traci.vehicle.getLanePosition(vehicle_id)
    except traci.exceptions.TraCIException:
        return None


def get_lane_length_for_vehicle(vehicle_id):
    try:
        lane_id = traci.vehicle.getLaneID(vehicle_id)
        if lane_id:
            return traci.lane.getLength(lane_id)
    except Exception:
        pass
    return None


def get_distance_same_lane(a_id, b_id):
    """
    distance from a to b when on same lane: pos_a - pos_b (a ahead)
    returns positive if a ahead of b.
    """
    try:
        lane_a = traci.vehicle.getLaneID(a_id)
        lane_b = traci.vehicle.getLaneID(b_id)
        if lane_a and lane_b and lane_a == lane_b:
            pos_a = traci.vehicle.getLanePosition(a_id)
            pos_b = traci.vehicle.getLanePosition(b_id)
            return max(0.0, pos_a - pos_b)
    except Exception:
        pass
    return float("inf")


def short_log(step, t, vid, pos, speed, leader_id, gap, safe, action):
    pos_str = f"{pos:6.1f}" if pos is not None else "   N/A"
    print(f"Step {step:04d} t={t:6.2f}s | {vid:12s} pos:{pos_str} m | spd:{speed:4.1f} m/s | lead:{leader_id:12s} gap:{gap:6.2f} safe:{safe:5.2f} | {action}")


def run():
    step = 0
    leader_state = "normal"   # normal -> slowing -> stopped -> resuming -> fast
    stop_timer = 0

    # Open CSV
    with open(LOG_CSV, mode="w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        # header
        writer.writerow([
            "step", "time_s", "vehicle_id", "lane_pos_m", "speed_mps",
            "leader_of_this_vehicle", "gap_to_leader_m", "safe_dist_m", "action"
        ])

        # warm-up few steps
        for _ in range(STEP_WARMUP):
            traci.simulationStep()
            step += 1

        while step < MAX_STEPS:
            traci.simulationStep()
            t = traci.simulation.getTime()
            ids = traci.vehicle.getIDList()

            leader_present = LEADER_ID in ids

            # If nothing expected & no vehicles present, finish
            if (not ids) and traci.simulation.getMinExpectedNumber() == 0:
                print("No vehicles present and none expected. Ending.")
                break

            # Leader state machine
            if leader_present:
                try:
                    posL = get_lane_pos(LEADER_ID)
                    vL = traci.vehicle.getSpeed(LEADER_ID)
                except Exception:
                    posL = None
                    vL = 0.0

                if posL is not None:
                    if leader_state == "normal" and posL >= SLOW_ZONE_START and posL < STOP_POSITION:
                        traci.vehicle.slowDown(LEADER_ID, SLOW_SPEED, BRAKE_TIME)
                        leader_state = "slowing"
                    elif leader_state in ("normal", "slowing") and posL >= STOP_POSITION:
                        traci.vehicle.slowDown(LEADER_ID, 0.0, BRAKE_TIME)
                        leader_state = "stopped"
                        stop_timer = 0
                    elif leader_state == "stopped":
                        if stop_timer < STOP_DURATION:
                            traci.vehicle.setSpeed(LEADER_ID, 0.0)
                            stop_timer += 1
                        else:
                            traci.vehicle.slowDown(LEADER_ID, NORMAL_SPEED, BRAKE_TIME)
                            leader_state = "resuming"
                    elif leader_state == "resuming" and posL >= FAST_ZONE_START:
                        try:
                            traci.vehicle.setMaxSpeed(LEADER_ID, FAST_SPEED)
                        except Exception:
                            pass
                        traci.vehicle.slowDown(LEADER_ID, FAST_SPEED, BRAKE_TIME)
                        leader_state = "fast"
                # else leader not yet in lane or lost

            # For every vehicle currently present, determine the vehicle ahead on same lane
            # Build a list of (vehicle_id, lanePos) for vehicles that have lane position
            veh_positions = []
            for vid in ids:
                pos = get_lane_pos(vid)
                if pos is not None:
                    # lane position is distance from lane start; larger = further along
                    veh_positions.append((vid, pos))
            # sort descending so first is the lead-most vehicle
            veh_positions.sort(key=lambda x: x[1], reverse=True)

            # Build a mapping: for each vehicle, who is directly ahead (None if none)
            ahead_map = {}
            for i, (vid, pos) in enumerate(veh_positions):
                if i == 0:
                    ahead_map[vid] = None
                else:
                    ahead_map[vid] = veh_positions[i - 1][0]  # vehicle ahead

            # Now apply follower control for every vehicle except the true leader (LEADER_ID handles its own FSM)
            for vid, pos in veh_positions:
                try:
                    v = traci.vehicle.getSpeed(vid)
                except Exception:
                    v = 0.0

                leader_of_vid = ahead_map.get(vid)
                action = "cruise"

                if vid == LEADER_ID:
                    # log leader state for visibility
                    action = f"leader_state:{leader_state}"
                    gap = ""
                    safe = calc_safe_dist(v)
                    writer.writerow([step, f"{t:.3f}", vid, f"{pos:.3f}", f"{v:.3f}", "", "", f"{safe:.3f}", action])
                    short_log(step, t, vid, pos, v, "", 0.0, safe, action)
                    continue

                # If there is a vehicle ahead, compute gap and control relative to that vehicle
                if leader_of_vid is not None:
                    gap = get_distance_between(leader_of_vid, vid)
                    try:
                        v_lead = traci.vehicle.getSpeed(leader_of_vid)
                    except Exception:
                        v_lead = 0.0

                    safe = calc_safe_dist(v)

                    # Simple reactive controller:
                    # - if leader ahead is much slower than us -> brake to leader speed
                    # - if gap < safe -> slow proportionally
                    if v_lead + 2.0 < v and v_lead < 5.0:
                        traci.vehicle.slowDown(vid, 0.0, BRAKE_TIME)
                        action = f"brake (lead slow:{leader_of_vid})"
                    elif gap < safe:
                        # reduce speed to a fraction of current speed (tunable)
                        new_speed = max(0.0, v * 0.35)
                        traci.vehicle.slowDown(vid, new_speed, BRAKE_TIME)
                        action = f"brake (too close to {leader_of_vid})"
                    else:
                        traci.vehicle.setSpeed(vid, -1)  # revert to vehicle's desired speed
                        action = "follow"

                    # log
                    writer.writerow([step, f"{t:.3f}", vid, f"{pos:.3f}", f"{v:.3f}", leader_of_vid, f"{gap:.3f}", f"{safe:.3f}", action])
                    short_log(step, t, vid, pos, v, leader_of_vid, gap, safe, action)
                else:
                    # no vehicle ahead -> free cruise
                    traci.vehicle.setSpeed(vid, -1)
                    gap = ""
                    safe = calc_safe_dist(v)
                    writer.writerow([step, f"{t:.3f}", vid, f"{pos:.3f}", f"{v:.3f}", "", "", f"{safe:.3f}", "free"])
                    short_log(step, t, vid, pos, v, "None", 0.0, safe, "free")

            # check leader reached end-of-lane condition (stop simulation after brief wait)
            if leader_present:
                lane_len = get_lane_length_for_vehicle(LEADER_ID)
                posL = get_lane_pos(LEADER_ID)
                if lane_len is not None and posL is not None and posL >= max(0.0, lane_len - 0.5):
                    print(f"Leader reached end of lane at t={t:.2f}s (pos {posL:.2f}/{lane_len:.2f}). Waiting a few steps then ending.")
                    # run a few more steps to let followers finish
                    for _ in range(20):
                        traci.simulationStep()
                        step += 1
                    break

            step += 1

    traci.close()
    print(f"\nSimulation finished. Log saved to {LOG_CSV}")


def get_distance_between(a_id, b_id):
    """
    Return distance along lane from a (ahead) to b (behind) when both on same lane.
    Falls back to euclidean distance if lane info missing.
    """
    try:
        lane_a = traci.vehicle.getLaneID(a_id)
        lane_b = traci.vehicle.getLaneID(b_id)
        if lane_a and lane_b and lane_a == lane_b:
            pos_a = traci.vehicle.getLanePosition(a_id)
            pos_b = traci.vehicle.getLanePosition(b_id)
            return max(0.0, pos_a - pos_b)
    except Exception:
        pass
    # fallback euclidean
    try:
        pA = traci.vehicle.getPosition(a_id)
        pB = traci.vehicle.getPosition(b_id)
        return math.hypot(pA[0] - pB[0], pA[1] - pB[1])
    except Exception:
        return float("inf")


if __name__ == "__main__":
    try:
        traci.start([SUMO_BINARY, "-c", SUMO_CONFIG, "--tripinfo-output", "tripinfo.xml"])
        run()
    except traci.exceptions.TraCIException as e:
        print("Error starting TraCI:", e)
        sys.exit(1)
