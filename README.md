# Autonomous Orbital Debris Assessment and Collision Avoidance System
---
A Python-based autonomous collision avoidance system for Earth-orbiting spacecraft that continuously evaluates conjunction risks using live orbital data and recommends fuel-efficient avoidance maneuvers.
---

## Overview

The increasing density of satellites and orbital debris in Low Earth Orbit (LEO) has made collision avoidance a critical challenge for spacecraft operations. This project implements an end-to-end collision assessment pipeline capable of:

- Downloading and processing live NORAD orbital data
- Propagating spacecraft trajectories using the SGP4 model
- Detecting potential conjunction events
- Estimating miss distance and relative velocity
- Ranking collision threats
- Generating avoidance maneuvers
- Selecting the most fuel-efficient safe maneuver

The current implementation uses the International Space Station (ISS) as the protected spacecraft but can be adapted for any cataloged space object.

---

## Methodology

### 1. Catalog Acquisition

The system downloads active satellite Two-Line Element (TLE) data from the CelesTrak catalog and stores it locally for repeated analysis.

### 2. Candidate Filtering

Potential threats are filtered using orbital shell overlap criteria based on apogee and perigee altitude limits.

### 3. Orbit Propagation

The trajectories of the protected spacecraft and all candidate objects are propagated over a configurable prediction horizon using the SGP4 orbital propagator.

### 4. Conjunction Detection

A Smart Sieve screening stage rapidly eliminates distant objects before performing precise distance calculations on remaining candidates.

### 5. Threat Assessment

For each close approach event, the system computes:

- Time of Closest Approach (TCA)
- Miss Distance
- Relative Velocity
- Threat Ranking

### 6. Maneuver Planning

A set of candidate prograde and retrograde burns is simulated by modifying orbital mean motion.

Each maneuver is evaluated based on:

- Minimum achieved separation distance
- Fuel cost (ΔV magnitude)
- Post-maneuver safety status

The lowest-cost safe maneuver is selected as the recommended action.

---

## References

1. Vallado, D. A., Crawford, P., Hujsak, R., & Kelso, T. S. (2006). *Revisiting Spacetrack Report #3*. AIAA/AAS Astrodynamics Specialist Conference.

2. Hoots, F. R., & Roehrich, R. L. (1980). *Spacetrack Report No. 3: Models for Propation of NORAD Element Sets.*

3. Kelso, T. S. *CelesTrak Satellite Catalog.*  
   https://celestrak.org/

4. Levit, C., & Marshall, W. (2011). *Improved Orbit Predictions Using Two-Line Elements.*

5. Hall, D., Hejduk, M., & Johnson, N. (2019). *Orbital Debris Collision Avoidance and Conjunction Assessment Methodologies*. Proceedings of the Orbital Debris Conference.  
   https://www.hou.usra.edu/meetings/orbitaldebris2019/orbital2019paper/pdf/6158.pdf
