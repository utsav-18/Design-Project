#!/usr/bin/env python3
"""
edge_brake.py — leader stops mid-track for a while then resumes, follower follows safely.
Ends when leader reaches end of final edge (or when both vehicles are gone).
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
FOLLOWER_ID = "follower"

# Behavior positions (meters along lane)
SLOW_ZONE_START = 200.0     # where leader begins to slow
STOP_POSITION = 200.0       # where leader will stop (mid-track)
STOP_DURATION = 80         # how long leader stays stopped (seconds)
FAST_ZONE_START = 700.0     # if road long enough, where leader speeds up

# Speeds (m/s)
NORMAL_SPEED = 8.0
SLOW_SPEED = 2.0
FAST_SPEED = 20.0

BRAKE_TIME = 2.0

# Safety model for follower
BASE_SAFE_DIST = 5.0
REACTION_TIME = 1.0
DECEL = 4.5

# Simulation control
MAX_STEPS = 20000
LOG_CSV = "edge_log.csv"
# ==============================================


def calc_safe_dist(v_f):
    """Simple safe distance: base + reaction + braking distance."""
    return BASE_SAFE_DIST + v_f * REACTION_TIME + (v_f ** 2) / (2 * DECEL)


def get_lane_pos(vehicle_id):
    try:
        return traci.vehicle.getLanePosition(vehicle_id)
    except traci.TraCIException:
        return None


def get_lane_length_for_vehicle(vehicle_id):
    """Return length of the lane the vehicle is currently on (or None)."""
    try:
        lane_id = traci.vehicle.getLaneID(vehicle_id)  # like "A0B0_0"
        if lane_id:
            return traci.lane.getLength(lane_id)
    except Exception:
        pass
    return None


def get_distance(leader_id, follower_id):
    """Prefer same-lane longitudinal distance; otherwise euclidean distance."""
    try:
        lid = traci.vehicle.getLaneID(leader_id)
        fid = traci.vehicle.getLaneID(follower_id)
        if lid and fid and lid == fid:
            # leader ahead lanepos - follower lanepos
            return max(0.0, traci.vehicle.getLanePosition(leader_id) - traci.vehicle.getLanePosition(follower_id))
    except traci.TraCIException:
        pass
    # fallback: euclidean distance
    try:
        pL = traci.vehicle.getPosition(leader_id)
        pF = traci.vehicle.getPosition(follower_id)
        return math.hypot(pL[0] - pF[0], pL[1] - pF[1])
    except Exception:
        return float("inf")


def short_log(step, t, posL, dist, vL, vF, safe, action):
    posL_str = f"{posL:6.1f}" if posL is not None else "   N/A"
    print(f"Step {step:04d} t={t:6.2f}s | Lpos:{posL_str} m | Dist:{dist:6.2f} m | L:{vL:4.1f} m/s | F:{vF:4.1f} m/s | Safe:{safe:5.2f} m | {action}")


def run():
    step = 0
    leader_state = "normal"   # normal -> slowing -> stopped -> resuming -> fast
    stop_timer = 0

    # Open CSV
    with open(LOG_CSV, mode="w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "step", "time_s", "vehicle_id_leader", "leader_lane_pos_m",
            "vehicle_id_follower", "follower_lane_pos_m",
            "leader_speed_mps", "follower_speed_mps", "gap_m", "safe_dist_m", "action"
        ])

        # warm-up few steps
        for _ in range(5):
            traci.simulationStep()
            step += 1

        while step < MAX_STEPS:
            traci.simulationStep()
            t = traci.simulation.getTime()
            ids = traci.vehicle.getIDList()

            leader_present = LEADER_ID in ids
            follower_present = FOLLOWER_ID in ids

            # If both vehicles gone and SUMO expects none, finish
            if (not leader_present and not follower_present) and traci.simulation.getMinExpectedNumber() == 0:
                print("No vehicles present and none expected. Ending.")
                break

            # If leader finished the track (reached end of its lane), and follower is gone or behind, finish.
            if not leader_present:
                # leader already finished
                if not follower_present and traci.simulation.getMinExpectedNumber() == 0:
                    break
            else:
                # check if leader reached end of its current lane
                lane_len = get_lane_length_for_vehicle(LEADER_ID)
                posL = get_lane_pos(LEADER_ID)
                if lane_len is not None and posL is not None and posL >= max(0.0, lane_len - 0.5):
                    # leader reached end — wait until follower leaves (or short timeout) then stop
                    print(f"Leader reached end of lane at t={t:.2f}s (pos {posL:.2f}/{lane_len:.2f}). Waiting for follower to clear.")
                    # run a few more steps to let follower finish
                    extra_wait = 0
                    while extra_wait < 20:
                        traci.simulationStep()
                        extra_wait += 1
                    break

            action = "None"

            if leader_present and follower_present:
                vL = traci.vehicle.getSpeed(LEADER_ID)
                vF = traci.vehicle.getSpeed(FOLLOWER_ID)
                posL = get_lane_pos(LEADER_ID)
                posF = get_lane_pos(FOLLOWER_ID)
                gap = get_distance(LEADER_ID, FOLLOWER_ID)
                safe = calc_safe_dist(vF)

                # Leader state machine (stop mid-track then resume)
                if posL is not None:
                    if leader_state == "normal" and posL >= SLOW_ZONE_START and posL < STOP_POSITION:
                        traci.vehicle.slowDown(LEADER_ID, SLOW_SPEED, BRAKE_TIME)
                        leader_state = "slowing"
                        action = "Leader: slowing"
                    elif leader_state in ("normal", "slowing") and posL >= STOP_POSITION:
                        # stop at STOP_POSITION
                        traci.vehicle.slowDown(LEADER_ID, 0.0, BRAKE_TIME)
                        leader_state = "stopped"
                        stop_timer = 0
                        action = "Leader: stopped"
                    elif leader_state == "stopped":
                        if stop_timer < STOP_DURATION:
                            traci.vehicle.setSpeed(LEADER_ID, 0.0)
                            stop_timer += 1
                            action = f"Leader: stopped ({stop_timer}/{STOP_DURATION})"
                        else:
                            traci.vehicle.slowDown(LEADER_ID, NORMAL_SPEED, BRAKE_TIME)
                            leader_state = "resuming"
                            action = "Leader: resuming"
                    elif leader_state == "resuming" and posL >= FAST_ZONE_START:
                        try:
                            traci.vehicle.setMaxSpeed(LEADER_ID, FAST_SPEED)
                        except Exception:
                            pass
                        traci.vehicle.slowDown(LEADER_ID, FAST_SPEED, BRAKE_TIME)
                        leader_state = "fast"
                        action = "Leader: fast"
                    elif leader_state == "fast":
                        action = "Leader: fast"
                    else:
                        # no state change, cruising
                        if action == "None":
                            action = "Leader: cruise"

                else:
                    posL = float("nan")
                    posF = float("nan")
                    vL = 0.0
                    vF = 0.0
                    gap = float("inf")
                    safe = calc_safe_dist(0.0)

                # Follower logic (reactive)
                if vL + 2.0 < vF and vL < 5.0:
                    traci.vehicle.slowDown(FOLLOWER_ID, 0.0, BRAKE_TIME)
                    action = action + " | Follower: brake (leader slower)"
                elif gap < safe:
                    traci.vehicle.slowDown(FOLLOWER_ID, max(0.0, vF * 0.35), BRAKE_TIME)
                    action = action + " | Follower: brake (too close)"
                else:
                    traci.vehicle.setSpeed(FOLLOWER_ID, -1)
                    action = action + " | Follower: follow"

                # log
                writer.writerow([
                    step, f"{t:.3f}", LEADER_ID, f"{posL:.3f}",
                    FOLLOWER_ID, f"{posF:.3f}",
                    f"{vL:.3f}", f"{vF:.3f}", "" if gap == float("inf") else f"{gap:.3f}",
                    f"{safe:.3f}", action
                ])

                short_log(step, t, posL, gap, vL, vF, safe, action)

            else:
                # if one is missing, log who is present
                present = ids if ids else []
                # write a minimal row for visibility
                writer.writerow([step, f"{t:.3f}", LEADER_ID if leader_present else "", f"{get_lane_pos(LEADER_ID) if leader_present else ''}",
                                 FOLLOWER_ID if follower_present else "", f"{get_lane_pos(FOLLOWER_ID) if follower_present else ''}",
                                 "", "", "", "", f"Vehicles present: {present}"])
                if ids:
                    print(f"Step {step:04d} t={t:.2f}s | Vehicles present: {', '.join(ids)}")
                else:
                    print(f"Step {step:04d} t={t:.2f}s | No vehicles on network")

            step += 1

    traci.close()
    print(f"\nSimulation finished. Log saved to {LOG_CSV}")


if __name__ == "__main__":
    try:
        traci.start([SUMO_BINARY, "-c", SUMO_CONFIG, "--tripinfo-output", "tripinfo.xml"])
        run()
    except traci.exceptions.TraCIException as e:
        print("Error starting TraCI:", e)
        sys.exit(1)
