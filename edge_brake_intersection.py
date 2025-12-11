#!/usr/bin/env python3
import traci
import sys
import csv
import math
import time

# ===== CONFIG =====
SUMO_BINARY = "sumo-gui"            # or "sumo"
SUMO_CONFIG = "simple_intersection.sumocfg"

LEADER_ID = "leader"

# leader stop location (along the main edge W_C)
SLOW_ZONE_START = 120.0
STOP_POSITION = 140.0
STOP_DURATION = 40

NORMAL_SPEED = 8.0
SLOW_SPEED = 2.0
FAST_SPEED = 18.0
BRAKE_TIME = 2.0

BASE_SAFE_DIST = 4.0
REACTION_TIME = 1.0
DECEL = 4.5

LOGFILE = "edge_log_intersection.csv"
MAX_STEPS = 20000
# ===================

def calc_safe_dist(v):
    return BASE_SAFE_DIST + v * REACTION_TIME + (v**2) / (2*DECEL)

def get_lane_pos(veh):
    try:
        return traci.vehicle.getLanePosition(veh)
    except Exception:
        return None

def get_distance_between(a, b):
    """distance along same lane from a (ahead) to b (behind)"""
    try:
        la = traci.vehicle.getLaneID(a)
        lb = traci.vehicle.getLaneID(b)
        if la and lb and la == lb:
            pa = traci.vehicle.getLanePosition(a)
            pb = traci.vehicle.getLanePosition(b)
            return max(0.0, pa - pb)
    except Exception:
        pass
    # fallback euclidean
    try:
        pa = traci.vehicle.getPosition(a)
        pb = traci.vehicle.getPosition(b)
        return math.hypot(pa[0]-pb[0], pa[1]-pb[1])
    except Exception:
        return float("inf")

def vehicle_stopped_for_tls(veh, tls_threshold=10.0):
    """
    Heuristic: if vehicle speed ~0 and there is a traffic light ahead within tls_threshold meters
    and that traffic light's lane state for the lane is red, assume vehicle stopped for TLS.
    This avoids the controller fighting the TLS.
    """
    try:
        spd = traci.vehicle.getSpeed(veh)
        if spd > 0.1:
            return False
        # get next TLS info (list of triples) - if none, bail
        next_tls = traci.vehicle.getNextTLS(veh)  # returns list of (tlsID, dist, index) or empty
        if not next_tls:
            return False
        # pick closest
        tls_id, dist, idx = next_tls[0]
        if dist > tls_threshold:
            return False
        # get TLS state string for the whole light
        state = traci.trafficlight.getRedYellowGreenState(tls_id)
        # idx maps to the position in the TLS state string; idx could be -1 sometimes
        if idx >= 0 and idx < len(state):
            # 'r' or 'R' indicates red
            return state[idx] == 'r' or state[idx] == 'R'
    except Exception:
        return False
    return False

def main():
    try:
        traci.start([SUMO_BINARY, "-c", SUMO_CONFIG])
    except Exception as e:
        print("Error starting SUMO/TraCI:", e)
        sys.exit(1)

    step = 0
    leader_state = "normal"
    stop_timer = 0

    csvf = open(LOGFILE, "w", newline="")
    writer = csv.writer(csvf)
    writer.writerow(["step","time_s","veh","lane_pos_m","speed_mps","lead_of_this","gap_m","safe_m","action"])

    try:
        while step < MAX_STEPS:
            traci.simulationStep()
            t = traci.simulation.getTime()
            ids = traci.vehicle.getIDList()

            # leader machine (works only if leader exists)
            if LEADER_ID in ids:
                posL = get_lane_pos(LEADER_ID)
                vL = traci.vehicle.getSpeed(LEADER_ID)
                if posL is not None:
                    if leader_state == "normal" and posL >= SLOW_ZONE_START and posL < STOP_POSITION:
                        traci.vehicle.slowDown(LEADER_ID, SLOW_SPEED, BRAKE_TIME)
                        leader_state = "slowing"
                    elif leader_state in ("normal","slowing") and posL >= STOP_POSITION:
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
                    elif leader_state == "resuming":
                        traci.vehicle.setMaxSpeed(LEADER_ID, FAST_SPEED)
                        leader_state = "fast"

            # gather vehicles and positions
            veh_pos = []
            for vid in ids:
                pos = get_lane_pos(vid)
                if pos is not None:
                    veh_pos.append((vid,pos))
            veh_pos.sort(key=lambda x: x[1], reverse=True)

            # map each vehicle to the vehicle ahead (on same lane)
            ahead = {}
            for i, (vid,pos) in enumerate(veh_pos):
                if i == 0:
                    ahead[vid] = None
                else:
                    ahead[vid] = veh_pos[i-1][0]

            # control each non-leader vehicle relative to its ahead vehicle
            for vid,pos in veh_pos:
                v = traci.vehicle.getSpeed(vid)
                lead_of_vid = ahead.get(vid)
                action = "cruise"
                safe = calc_safe_dist(v)
                gap = ""
                if vid == LEADER_ID:
                    writer.writerow([step,f"{t:.3f}",vid,f"{pos:.3f}",f"{v:.3f}","", "", f"{safe:.3f}", f"leader_state:{leader_state}"])
                    continue

                # if vehicle appears stopped for TLS, do not fight it
                if vehicle_stopped_for_tls(vid):
                    action = "stopped_for_TLS"
                    try:
                        writer.writerow([step,f"{t:.3f}",vid,f"{pos:.3f}",f"{v:.3f}",lead_of_vid,"NA",f"{safe:.3f}",action])
                    except Exception:
                        pass
                    continue

                if lead_of_vid is not None:
                    gap = get_distance_between(lead_of_vid, vid)
                    try:
                        v_lead = traci.vehicle.getSpeed(lead_of_vid)
                    except Exception:
                        v_lead = 0.0
                    # simple rules
                    if v_lead + 2.0 < v and v_lead < 6.0:
                        traci.vehicle.slowDown(vid, max(0.0, v_lead), BRAKE_TIME)
                        action = f"brake_to_lead:{lead_of_vid}"
                    elif gap < safe:
                        new_speed = max(0.0, v * 0.4)
                        traci.vehicle.slowDown(vid, new_speed, BRAKE_TIME)
                        action = f"brake_close_to:{lead_of_vid}"
                    else:
                        traci.vehicle.setSpeed(vid, -1)
                        action = "follow_free"
                    writer.writerow([step,f"{t:.3f}",vid,f"{pos:.3f}",f"{v:.3f}",lead_of_vid,f"{gap:.3f}",f"{safe:.3f}",action])
                else:
                    # no vehicle ahead
                    traci.vehicle.setSpeed(vid, -1)
                    writer.writerow([step,f"{t:.3f}",vid,f"{pos:.3f}",f"{v:.3f}","", "",f"{safe:.3f}","free"])

            step += 1
            # optional: exit when leader finished route
            if LEADER_ID in ids:
                # if leader near net end -> stop
                try:
                    lane_len = traci.lane.getLength(traci.vehicle.getLaneID(LEADER_ID))
                    if get_lane_pos(LEADER_ID) >= lane_len - 1.0:
                        print("Leader finished. Ending soon.")
                        break
                except Exception:
                    pass

    finally:
        csvf.close()
        traci.close()
        print("Sim finished; log saved to", LOGFILE)

if __name__ == "__main__":
    main()
