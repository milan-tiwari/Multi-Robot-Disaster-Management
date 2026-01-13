# Multi-Robot Disaster Response with Voronoi Coverage (MAE 598 Final Project)

Authors: **Milan Tiwari, Kareena Salim Lakhani**  
---

## Overview

This repository contains the code and report for our 
**multi-robot disaster response using Voronoi-based coverage control**.

A team of ground robots must leave a single base, travel through a
disaster-affected field, and deliver relief supplies to several spatially
distributed demand hotspots. The controller combines:

- **Voronoi coverage control (Lloyd / centroid algorithm)** over a priority
  field and a time-varying demand field.
- A **greedy hotspot task-allocation layer**, which selects and uniquely
  assigns high-demand cells to individual robots.
- A **capacity and refilling model**, so robots must periodically return to
  base when they run out of supplies.
- A **repulsive safety controller**, which maintains a minimum pairwise
  distance between robots.

All simulations are implemented in **MATLAB**.

The repository reproduces:

- The **main simulation video / snapshot** with three hotspots (H1–H3),
  robot trajectories, and demand contours.
- The four metric plots:
  - Locational cost vs. time.
  - Total unmet demand vs. time.
  - Coverage served vs. time.
  - Minimum pairwise distance (safety) vs. time.
- The final LaTeX **report PDF** submitted for the course.

---

## Repository structure


```text
.
├── code/
│   ├── main.m
│   ├── initParams.m
│   ├── initState.m
│   ├── runSimulation.m
│   ├── discreteVoronoi.m
│   ├── weightedCentroids.m
│   ├── taskSelection.m
│   ├── safetyController.m
│   ├── visualizeState.m
│   ├── plotMetrics.m
│   └── utils/           
├── report/
│   ├── final_report.pdf
└── README.md
