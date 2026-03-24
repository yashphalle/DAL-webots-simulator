# Branching for a Study

DAL Webots Simulator can be used as a **branch-per-study** model. The `main` branch is the stable, cross-platform base. Each research study or experiment lives in its own branch and extends `main` with study-specific planners, controllers, and configs.

---

## Branch Naming Convention

```
study/<short-name>
```

Examples:
- `study/ltl-exploration` — LTL-based multi-robot exploration
- `study/frontier-mapping` — Frontier-based occupancy grid mapping
- `study/arm-manipulation` — YouBot arm grasping experiments
- `study/multi-agent-sync` — Synchronization protocol experiments

Names work best when kept lowercase, hyphenated, and descriptive.

---

## Creating a Study Branch

```bash
git checkout main
git pull
git checkout -b study/my-experiment
```

All study-specific work can live on this branch — keeping `main` free of study code helps everyone build on a clean base.

---

## What Belongs in main vs. a Branch

| `main` branch | `study/*` branch |
|---------------|-----------------|
| Core controllers (`waypoint_controller`, `dal_controller`) | Study-specific controllers |
| `utils/protocol.py`, `utils/occupancy_grid.py` | Extended protocol files (e.g., AprilTag messages) |
| `tools/slam_viz.py`, `tools/robot_pos_viz.py`, `tools/camera_viz.py` | Custom visualizers (e.g., Tkinter semantic grid) |
| `planners/simple_planner.py` | Study planner (`Helper.py`, `classes.py`, etc.) |
| `worlds/DAL2.wbt` | Study-specific worlds |
| Cross-platform dependencies only | Linux-only or study-specific deps (e.g., `spot`, `pupil-apriltags`) |
| `requirements.txt` | `requirements_<study>.txt` |
| `docs/` | `docs/<study>/` or study-specific README |


---

## Keeping a Branch Up to Date with main

When `main` gets improvements (bug fixes, new docs, visualizer updates), you can merge them into your study branch:

```bash
git checkout study/my-experiment
git merge main
```

After resolving any conflicts, you can continue working on your study as usual.

---

## Sharing a Study Branch

When your study is done or ready for others to use:

1. Having a `docs/` folder or README explaining how to run the study is helpful for others getting started
2. It's a good idea to make sure `requirements_<study>.txt` is up to date
3. Push the branch:
   ```bash
   git push origin study/my-experiment
   ```
4. Others can clone and checkout:
   ```bash
   git checkout study/my-experiment
   pip install -r requirements_<study>.txt
   ```

---

## Summary of File Locations Per Branch

```
DAL-webots-simulator/         (main branch)
├── controllers/              base controllers only
├── planners/                 simple_planner.py only
├── tools/                    slam_viz.py, robot_pos_viz.py, camera_viz.py
├── utils/                    protocol.py, occupancy_grid.py
├── worlds/                   DAL2.wbt only
├── docs/                     this documentation
└── requirements.txt

DAL-webots-simulator/         (study/ltl-exploration branch — adds:)
├── controllers/
│   ├── waypoint_controller_tcp/   extended navigation with AprilTag
│   └── teleop_vision/             vision streaming controller
├── planners/
│   ├── Helper.py                  DFA synthesis, grid utilities
│   ├── classes.py                 Agent and Environment classes
│   ├── final_server.py            multi-agent sync server
│   └── (client scripts)
├── tools/
│   ├── grid_visualizer.py         Tkinter semantic grid
│   ├── dual_grid_visualizer.py    2-robot grid view
│   └── triple_grid_visualizer.py  3-robot grid view
├── configs/
│   ├── config.json
│   ├── agents_config.json
│   └── vision_config.json
├── worlds/
│   ├── factory.wbt
│   ├── factory2.wbt
│   ├── factory3.wbt
│   └── DAL-Factory.wbt
├── textures/
│   └── apriltag_*.png
├── docs/ltl_study/
└── requirements_ltl.txt
```
