"""
Autonomous Orbital Debris Assessment and Collision Avoidance System
Version 2.0 (High-Speed Bulk TLE Catalog Loader)
"""

import os
import sys
import math
import requests
import numpy as np
import pandas as pd
from datetime import datetime, timedelta, timezone
from sgp4.api import Satrec, jday
import logging

# ==============================================================================
# CONFIGURATION THRESHOLDS
# ==============================================================================
PROTECTED_OBJECT     = 25544   # NORAD ID for ISS
ALTITUDE_MARGIN_KM   = 100.0   # Shell overlap filter margin
PREDICTION_HOURS     = 24      # Propagation window length
TIMESTEP_SECONDS     = 60      # Time resolution
R_THRESHOLD_KM       = 50.0    # Smart Sieve bounding box distance (dx, dy, dz)
SAFE_DISTANCE_KM     = 5.0     # Decision threshold for maneuvers
MANEUVER_SET         = [-0.20, -0.10, -0.05, 0.05, 0.10, 0.20] # delta-v in m/s

# Set to None to run the entire catalog, or keep at 50 for rapid confirmation tests
MAX_TEST_OBJECTS     = None      
FORCE_CONJUNCTION_TEST = False  # Set to True if you want to force our synthetic hazard back in

# ==============================================================================
# DIRECTORY SETUP (Safe for macOS / Linux)
# ==============================================================================
BASE_DIR             = os.path.expanduser("~/DebrisAvoidanceSystem") 
CACHE_DIR            = os.path.join(BASE_DIR, "tle_cache")
OUTPUT_DIR           = os.path.join(BASE_DIR, "outputs")
SATCAT_FILE          = os.path.join(os.path.dirname(os.path.abspath(__file__)), "satcat.csv") 

os.makedirs(CACHE_DIR, exist_ok=True)
os.makedirs(OUTPUT_DIR, exist_ok=True)

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

# ==============================================================================
# NEW BULK DOWNLOADING ENGINE
# ==============================================================================
def download_and_cache_bulk_catalog():
    """Downloads the entire active orbital catalog in a single 5-second stream."""
    logging.info("Downloading bulk active catalog from CelesTrak...")
    url = "https://celestrak.org/NORAD/elements/gp.php?GROUP=active&FORMAT=TLE"
    
    try:
        response = requests.get(url, timeout=30)
        if response.status_code != 200:
            logging.error(f"Bulk download failed with status code: {response.status_code}")
            return False
            
        lines = [line.strip() for line in response.text.splitlines() if line.strip()]
        
        # Parse 3-line format chunks: Line 0 (Name), Line 1 (L1), Line 2 (L2)
        count = 0
        for i in range(0, len(lines) - 2, 3):
            line0 = lines[i]
            line1 = lines[i+1]
            line2 = lines[i+2]
            
            # Extract NORAD ID from Chars 3-7 of Line 1
            try:
                norad_id = int(line1[2:7].strip())
                cache_path = os.path.join(CACHE_DIR, f"{norad_id}.txt")
                with open(cache_path, 'w') as f:
                    f.write(f"{line0}\n{line1}\n{line2}")
                count += 1
            except ValueError:
                continue
                
        logging.info(f"Bulk engine successfully saved {count} active TLE files to disk.")
        return True
    except Exception as e:
        logging.error(f"Failed bulk catalog download: {e}")
        return False

def get_cached_tle(norad_id):
    """Reads TLE strictly from your local hard drive cache."""
    cache_path = os.path.join(CACHE_DIR, f"{norad_id}.txt")
    if os.path.exists(cache_path):
        with open(cache_path, 'r') as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]
            if len(lines) >= 2:
                return lines[-2], lines[-1]
    return None, None

def modify_tle_mean_motion(line1, line2, dv_m_s, v_0_km_s):
    """Simulates a tangential burn by adjusting the mean motion in the TLE."""
    dv_km_s = dv_m_s / 1000.0
    n_str = line2[52:63]
    n_old = float(n_str)
    n_new = n_old * (1 - 3 * (dv_km_s / v_0_km_s))
    n_new_str = f"{n_new:11.8f}"[:11]
    line2_new = line2[:52] + n_new_str + line2[63:68]
    checksum = sum(int(c) if c.isdigit() else (1 if c == '-' else 0) for c in line2_new) % 10
    line2_new += str(checksum)
    return line1, line2_new

# ==============================================================================
# PIPELINE EXECUTION
# ==============================================================================
def main():
    # 1. Run the high-speed bulk downloader first
    bulk_success = download_and_cache_bulk_catalog()
    if not bulk_success:
        logging.warning("Proceeding with existing cached data if available.")

    # 2. Parse Catalog Filter
    if not os.path.exists(SATCAT_FILE):
        logging.error(f"Cannot find catalog file: {SATCAT_FILE}. Move it to the same folder as this script.")
        sys.exit(1)
        
    satcat = pd.read_csv(SATCAT_FILE)
    satcat['APOGEE'] = pd.to_numeric(satcat['APOGEE'], errors='coerce')
    satcat['PERIGEE'] = pd.to_numeric(satcat['PERIGEE'], errors='coerce')
    
    iss_row = satcat[satcat['NORAD_CAT_ID'] == PROTECTED_OBJECT]
    if iss_row.empty:
        logging.error(f"Protected Object {PROTECTED_OBJECT} not found.")
        sys.exit(1)
        
    iss_apogee = iss_row.iloc[0]['APOGEE']
    iss_perigee = iss_row.iloc[0]['PERIGEE']
    
    candidates_df = satcat[
        (satcat['APOGEE'] >= iss_perigee - ALTITUDE_MARGIN_KM) & 
        (satcat['PERIGEE'] <= iss_apogee + ALTITUDE_MARGIN_KM) &
        (satcat['NORAD_CAT_ID'] != PROTECTED_OBJECT)
    ].copy()
    
    if MAX_TEST_OBJECTS is not None and len(candidates_df) > MAX_TEST_OBJECTS:
        candidates_df = candidates_df.head(MAX_TEST_OBJECTS)
        
    # 3. Process TLE states directly out of the local cache
    iss_l1, iss_l2 = get_cached_tle(PROTECTED_OBJECT)
    if not iss_l1:
        # Emergency backup hardcoded ISS state if cache missing
        iss_l1 = "1 25544U 98067A   26162.50000000  .00016717  00000+0  30129-3 0  9990"
        iss_l2 = "2 25544  51.6416 322.9554 0005545  56.0965  67.6627 15.49887413422615"
        
    iss_sat = Satrec.twoline2rv(iss_l1, iss_l2)
    
    candidate_sats = {}
    for _, row in candidates_df.iterrows():
        cid = int(row['NORAD_CAT_ID'])
        l1, l2 = get_cached_tle(cid)
        if l1 and l2:
            candidate_sats[cid] = Satrec.twoline2rv(l1, l2)

    logging.info(f"Loaded {len(candidate_sats)} valid target objects for evaluation.")

    # 4. Math Matrix Setup
    num_steps = int((PREDICTION_HOURS * 3600) / TIMESTEP_SECONDS)
    jd_array, fr_array = np.zeros(num_steps), np.zeros(num_steps)
    time_array = []
    
    start_time = datetime.now(timezone.utc)
    for i in range(num_steps):
        t = start_time + timedelta(seconds=i * TIMESTEP_SECONDS)
        time_array.append(t)
        jd, fr = jday(t.year, t.month, t.day, t.hour, t.minute, t.second + t.microsecond/1e6)
        jd_array[i], fr_array[i] = jd, fr
        
    e_iss, r_iss, v_iss = iss_sat.sgp4_array(jd_array, fr_array)
    valid_iss = (e_iss == 0)

    # 5. Smart Sieve Execution
    logging.info("Applying CLAPS Smart Sieve Filtering processing...")
    threats = []
    
    for cid, sat in candidate_sats.items():
        e_obj, r_obj, v_obj = sat.sgp4_array(jd_array, fr_array)
        valid = valid_iss & (e_obj == 0)
        if not np.any(valid): continue
            
        dx = np.abs(r_obj[valid, 0] - r_iss[valid, 0])
        dy = np.abs(r_obj[valid, 1] - r_iss[valid, 1])
        dz = np.abs(r_obj[valid, 2] - r_iss[valid, 2])
        
        sieve_mask = (dx <= R_THRESHOLD_KM) & (dy <= R_THRESHOLD_KM) & (dz <= R_THRESHOLD_KM)
        if not np.any(sieve_mask): continue 
            
        d_precise = np.sqrt(dx[sieve_mask]**2 + dy[sieve_mask]**2 + dz[sieve_mask]**2)
        min_idx_in_sieve = np.argmin(d_precise)
        min_d = d_precise[min_idx_in_sieve]
        
        if min_d <= R_THRESHOLD_KM:
            sieve_indices = np.where(sieve_mask)[0]
            absolute_min_idx = np.where(valid)[0][sieve_indices[min_idx_in_sieve]]
            tca = time_array[absolute_min_idx]
            v_rel = np.linalg.norm(v_obj[valid][sieve_indices[min_idx_in_sieve]] - v_iss[valid][sieve_indices[min_idx_in_sieve]])
            name = candidates_df[candidates_df['NORAD_CAT_ID'] == cid]['OBJECT_NAME'].iloc[0]
            
            threats.append({
                "NORAD": cid, "Object Name": name, "TCA": tca,
                "Miss Distance": min_d, "Relative Velocity": v_rel, "satrec": sat 
            })

    # Optional synthetic test hazard injection logic
    if FORCE_CONJUNCTION_TEST:
        target_conjunction_idx = int(num_steps / 2)
        r_hazard = np.copy(r_iss)
        drift_rate_km_per_step = 0.44
        for idx in range(num_steps):
            time_offset_steps = idx - target_conjunction_idx
            r_hazard[idx, 0] += 0.4 
            r_hazard[idx, 2] -= 0.2 
            r_hazard[idx, 1] += (time_offset_steps * drift_rate_km_per_step)
        threats.append({
            "NORAD": 99999, "Object Name": "TEST_HAZARD_99999", "TCA": time_array[target_conjunction_idx],
            "Miss Distance": 0.447, "Relative Velocity": 7.42, "is_synthetic": True, "r_traj": r_hazard 
        })

    # 6. Sorting & Recommendations
    if not threats:
        logging.info("Execution Finished. No tracked items violating envelope safety parameters.")
        return

    threats_df = pd.DataFrame(threats).sort_values(by="Miss Distance", ascending=True)
    export_df = threats_df.drop(columns=["satrec", "is_synthetic", "r_traj"], errors='ignore')
    export_df.to_csv(os.path.join(OUTPUT_DIR, "threat_ranking.csv"), index=False)
    
    print("\n--- LIVE SYSTEM THREAT MATRIX ---")
    print(export_df.head(10).to_string(index=False))
    print("---------------------------------\n")
    
    primary_threat = threats_df.iloc[0]
    min_miss_distance = primary_threat["Miss Distance"]

    if min_miss_distance > SAFE_DISTANCE_KM:
        logging.info(f"System Check: Operational Environment SAFE. Minimum Proximity: {min_miss_distance:.2f} km.")
        return
        
    logging.warning(f"CRITICAL WARNING: Avoidance action triggered. Distance drops to {min_miss_distance:.2f} km.")
    
    # Run avoidance sets
    v0_iss_mag = np.linalg.norm(v_iss[0]) 
    maneuver_results = []
    
    for dv in MANEUVER_SET:
        new_l1, new_l2 = modify_tle_mean_motion(iss_l1, iss_l2, dv, v0_iss_mag)
        mod_iss = Satrec.twoline2rv(new_l1, new_l2)
        e_mod, r_mod, _ = mod_iss.sgp4_array(jd_array, fr_array)
        
        global_min_distance = float('inf')
        for idx, threat_data in threats_df.iterrows():
            if "is_synthetic" in threat_data and threat_data["is_synthetic"]:
                rt = threat_data["r_traj"]
                v_mask = (e_mod == 0)
            else:
                tsat = threat_data["satrec"]
                et, rt, _ = tsat.sgp4_array(jd_array, fr_array)
                v_mask = (e_mod == 0) & (et == 0)
            
            d_new = np.linalg.norm(rt[v_mask] - r_mod[v_mask], axis=1)
            local_min_d = np.min(d_new) if len(d_new) > 0 else 0
            if local_min_d < global_min_distance:
                global_min_distance = local_min_d
                
        maneuver_results.append({
            "Burn (m/s)": dv, "Expected Miss Distance": global_min_distance,
            "Direction": "Prograde" if dv > 0 else "Retrograde"
        })
        
    maneuvers_df = pd.DataFrame(maneuver_results)
    maneuvers_df.to_csv(os.path.join(OUTPUT_DIR, "maneuver_recommendations.csv"), index=False)
    
    safe_maneuvers = maneuvers_df[maneuvers_df["Expected Miss Distance"] >= SAFE_DISTANCE_KM].copy()
    if not safe_maneuvers.empty:
        safe_maneuvers["Fuel Cost"] = safe_maneuvers["Burn (m/s)"].abs()
        best_maneuver = safe_maneuvers.sort_values(by="Fuel Cost").iloc[0]
    else:
        best_maneuver = maneuvers_df.sort_values(by="Expected Miss Distance", ascending=False).iloc[0]

    print("--- CHOSEN FUEL-OPTIMAL MANEUVER ---")
    print(f"Burn:                 {best_maneuver['Burn (m/s)']:+.2f} m/s ({best_maneuver['Direction']})")
    print(f"Projected Clear Dist: {best_maneuver['Expected Miss Distance']:.2f} km")
    print(f"Status Post-Maneuver: {'RESOLVED' if best_maneuver['Expected Miss Distance'] >= SAFE_DISTANCE_KM else 'CRITICAL FAILURE'}")
    print("------------------------------------\n")

if __name__ == "__main__":
    main()