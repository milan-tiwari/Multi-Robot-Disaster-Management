# ReLIEF‑VOR (MATLAB): Resilient Multi‑Robot Relief Delivery via Weighted Voronoi Coverage & Capacitated Routing

**What it does**  
Simulates a team of robots that spread out over a disaster area using **weighted Lloyd coverage** on a **priority heat map** and then deliver relief at **hotspots** inside their Voronoi regions. A **safety filter** keeps robots separated. The visualization overlays the heat map, robots, (discrete) Voronoi assignment, paths, and live metrics.

## Quick start
1. Open MATLAB.
2. `cd` into this folder.
3. Run:
```matlab
main
```
This runs a default scenario (≈180 s sim) and produces figures in the `out/` folder.

## Files
- `main.m` — entry point.  
- `defaultParams.m` — all tunable parameters in one struct.  
- `initScenario.m` — grid, priority field, demand, robots, bases.  
- `generateField.m` — synthesizes a priority heat map (sum of Gaussians).  
- `runSimulation.m` — core loop and visualization.  
- `discreteVoronoi.m` — grid-based Voronoi assignment (no toolboxes).  
- `weightedCentroids.m` — priority-weighted centroids for Lloyd descent.  
- `taskSelection.m` — pick hotspot targets per robot within its cell.  
- `cbfFilter.m` — control-barrier safety (uses `quadprog` if available, else a smooth repulsion fallback).  
- `demandUpdate.m` — relief delivery dynamics and capacity/refill.  
- `measureMetrics.m` — locational cost, unmet demand, safety margins.  
- `visualizeState.m` — heat map + robots + paths + metrics overlays.  
- `smoothField.m` — lightweight Gaussian smoothing (no toolboxes).  

## Notes
- The Voronoi used in control is **discrete** (grid-based). For plotting, MATLAB’s `voronoi` is used for lines only (not for control).  
- If Optimization Toolbox is unavailable, the CBF falls back to a differentiable repulsion that still preserves separation in practice.
- Parameters such as number of robots, capacity, service rate, safety distance, etc., are in `defaultParams.m`.

## Outputs
- `out/sim_summary.mat` — metrics and final state.  
- `out/frame_####.png` — optional saved frames.  

## Citation/style inspiration
Residual plots & figure organization follow the A+ examples provided in class.
