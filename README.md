# Formation-Based UAV Path Planning in Simulation — Implementation Report

## 1. What the project does

This is a self-contained Python simulation of a **9-drone swarm** that flies in a fixed rigid formation (shaped like the letter **"G"**) from one side of a 100×100 map to the other, around a single circular obstacle. It has five modules, each with one job:

| Module | Responsibility |
|---|---|
| `map_setup.py` | Defines the world (size, obstacle, start/goal) and rasterizes it into a grid |
| `path_planner.py` | Finds a safe route through the grid with A*, then simplifies it |
| `trajectory.py` | Converts the waypoint route into two smooth, time-parameterized flight profiles (fast vs. efficient) |
| `formation.py` | Defines the 9-drone "G" shape and re-centers it along the planned route |
| `simulate.py` | Orchestrates all of the above and renders the plots/animation in `results/` |

A note on scope: this is a **2D, kinematic, centroid-follower simulation** — there's no dynamics model (mass, thrust, drag), no inter-drone collision avoidance, and no controller. The "formation" is a rigid geometric offset applied to a single planned centroid trajectory. That's a reasonable and common simplification for a first-pass planning demo, and the sections below explain where that choice shows up and what it costs.

**Note on `formation.py`:** this file was missing from the uploaded archive (only its compiled `__pycache__/formation.cpython-39.pyc` was present). I recovered the exact source by disassembling the bytecode and reconstructing it statically — then ran the full pipeline end-to-end to confirm the reconstruction is behaviorally identical to the original. That recovered file is included alongside this report (`formation_reconstructed.py`).

---

## 2. `map_setup.py` — the world model

```python
MAP_WIDTH, MAP_HEIGHT = 100, 100
START, GOAL = (5, 50), (95, 50)
OBSTACLE_CENTER, OBSTACLE_RADIUS = (50, 50), 10
SAFETY_MARGIN = 3
GRID_RESOLUTION = 1
```

**Design decisions:**

- **Continuous world, discretized only for planning.** The map's true coordinates are floating-point ("units"), but `build_obstacle_grid()` bakes the obstacle into a boolean occupancy grid at 1-unit resolution for A* to search over. This is the standard way to make a continuous obstacle usable by a discrete grid-search algorithm — cheap to build, cheap to query, and easy to reason about. The trade-off is that a single circular obstacle (a few lines of geometry) gets turned into a 100×100 = 10,000-cell boolean array purely so a generic grid search can be reused; it's more machinery than the specific problem needs, but it makes the planner obstacle-agnostic (any occupancy pattern would work, not just circles).
- **Symmetric start/goal on the obstacle's centerline** (`(5,50)` → `(95,50)`, obstacle centered at `(50,50)`) — this guarantees the obstacle actually blocks the straight-line path, so the demo has something for A* to route around instead of a trivial straight shot.
- **`SAFETY_MARGIN = 3` inflates the obstacle before the grid is built**, rather than being handled later as a per-node buffer check. Baking the margin directly into the occupied-cell test (`OBSTACLE_RADIUS + safety_margin`) means every downstream consumer (A*, the smoother, the plots) automatically respects clearance without needing to know it exists — the grid *is* the source of truth for "safe." The cost is that the margin is now a static circle grown at build time; it can't be tightened or relaxed per-drone or per-flight-phase without regenerating the whole grid.
- **`world_to_grid` / `grid_to_world` use cell-center convention** (`col*res + res/2`), so a coordinate is always mapped to the middle of its containing cell rather than a corner. This avoids a whole class of off-by-one/edge-ambiguity bugs where a point sitting exactly on a cell boundary could be attributed to the wrong cell.
- **`visualise_map()` is a separate, reusable plotting function** rather than being duplicated in `simulate.py`'s main script — though in practice `simulate.py` re-implements very similar plotting logic itself (see §6) rather than calling this function, which is a minor duplication.

---

## 3. `path_planner.py` — A* search + path smoothing

### 3.1 A* over an 8-connected grid

```python
_MOVES = [(1,0,1.0), (-1,0,1.0), (0,1,1.0), (0,-1,1.0),
          (1,1,1.4142), (-1,1,1.4142), (1,-1,1.4142), (-1,-1,1.4142)]
```

- **8-connectivity (not 4)** lets the path move diagonally, with diagonal moves costed at `√2` instead of `1`. This is what makes A* paths look like straight/diagonal lines instead of a staircase — with only 4-connectivity, every diagonal-ish route would zig-zag in unit steps and be needlessly long and jagged.
- **Cost function:** `f = g + h`, with `g` = accumulated path cost and `h` = **Euclidean distance to goal** (`_euclidean`). Euclidean distance is an *admissible* heuristic here because it never overestimates the true remaining cost on this grid (the cheapest possible move sequence between two cells can't beat a straight line), which is what guarantees A* returns the optimal (grid-shortest) path rather than just *a* path.
- **Implementation uses a binary heap (`heapq`)** with a `closed` set and a `came_from` dict for standard heap-based A* — this is the textbook-efficient version (O(log n) push/pop) rather than a linear scan of an "open list," which matters once the grid has thousands of cells (here, up to 10,000).
- **Lazy deletion pattern:** nodes can be pushed to the heap multiple times with different `g` values; instead of decrease-key (which `heapq` doesn't support natively), the code just re-pushes and checks `if node in closed: continue` when popped. This is a well-known, correct way to work around Python's lack of a heap decrease-key operation, at the cost of a marginally larger heap.

### 3.2 String-pulling / line-of-sight smoothing

Raw A* on a grid returns one waypoint per grid cell it touches — for this map that's **91 cells**, i.e., a path with dozens of near-collinear points. `smooth_path()` implements the classic **"string pulling"** greedy algorithm:

1. Start at the path's first point.
2. Look as far ahead as possible (from the *last* point backward) for the farthest point with a clear Bresenham line-of-sight (`_line_of_sight`, a standard Bresenham line rasterization) that doesn't cross any occupied cell.
3. Jump directly there, repeat.

Running the pipeline end-to-end, this collapses the **91-cell raw A* path down to just 4 waypoints** (start → two points flanking the obstacle → goal) — because most of the raw path was already straight lines that the grid search had artificially chopped into 1-unit steps.

**Why this matters:** A* on a grid is *optimal for the grid*, but a grid-optimal path is a poor proxy for a physically realistic flight path — it's needlessly kinked and grid-aligned. Smoothing is what turns "the shortest path on this artificial 1-unit lattice" into "a small number of straight segments that a real vehicle would actually want to fly," and critically, it does this *without* touching the obstacle grid or re-running any search — it's a cheap O(n²) post-process on an already-small path.

### 3.3 Endpoint snapping

```python
waypoints[0] = start
waypoints[-1] = goal
```

Grid cells map back to their *centers* (§2), so the first/last waypoints coming out of `grid_to_world` would be off by up to half a cell from the true requested start/goal. Forcibly overwriting the endpoints guarantees the planned path starts and ends exactly where the user asked, rather than at "the center of the cell nearest to where the user asked."

---

## 4. `trajectory.py` — turning waypoints into flyable, timed profiles

This is the most physics-flavored module, and it deliberately implements **two different trajectory philosophies** side by side.

### 4.1 Geometry: cubic spline over arc length

```python
cs_x = CubicSpline(cumlen, wps[:, 0])
cs_y = CubicSpline(cumlen, wps[:, 1])
```

- Both `x(s)` and `y(s)` are fit as **cubic splines parameterized by cumulative arc length `s`**, not by waypoint index or time. Parameterizing by arc length (rather than, say, `x` as a function of `y`, or index `0,1,2,...`) means the spline is well-defined even if the path doubles back on itself, and it directly supports "how far do I need to have traveled by time t" queries, which is exactly what a speed profile needs.
- Cubic splines were chosen over straight-line interpolation because they guarantee continuous first and second derivatives — i.e., the resulting path has continuous **velocity and acceleration**, not sudden direction changes at every waypoint. That matters for anything meant to look like a real vehicle path rather than a polyline.

### 4.2 Min-Time profile: constant cruise speed

```python
MIN_TIME_SPEED = 18.0
```
`_sample_trajectory()` just walks the spline at a **constant speed of 18 units/s**, sampling every `DT=0.05 s`. Velocity/acceleration are derived by differentiating position with respect to arc length and multiplying by speed (chain rule), then acceleration via `np.gradient` on the resulting velocity. This is the "get there as fast as possible" profile: no ramp-up, no ramp-down — full speed the entire way. It's a simplification, but it establishes an upper-bound-on-speed baseline to compare against.

### 4.3 Min-Energy profile: trapezoidal speed profile

```python
MIN_ENERGY_SPEED = 6.0
ramp_frac = 0.25
```
`_min_energy_profile()` instead builds a **trapezoidal velocity profile**: ramp up linearly from 0 to 6 units/s over the first 25% of the path, cruise at 6 units/s across the middle 50%, then ramp back down over the final 25%. This is a deliberate, classic choice for "energy efficient" motion:

- **Lower peak speed** directly reduces induced drag/power for realistic rotorcraft (energy roughly scales with speed², so cutting cruise speed from 18 to 6 units/s is a large energy saving).
- **Smooth ramps instead of instant starts/stops** avoid the large accelerations that dominate the energy-proxy metric (`accel²`, see below) — a sudden 0→18 speed jump would need enormous acceleration at t=0, which is exactly what this profile avoids by spreading the speed change over a quarter of the path.
- The profile is built on a **fine, uniform arc-length grid (`s_fine`)** first (so the piecewise-linear speed law is easy to define), converted to a time axis by integrating `ds/v(s)`, and only *then* resampled onto a uniform time grid via `np.interp` — because downstream consumers (animation, plotting) want uniform time steps, not uniform arc-length steps.

### 4.4 The energy "proxy" metric

```python
energy = np.trapezoid(traj['accel'] ** 2, traj['t'])
```

There's no real vehicle energy model here (no mass, no motor efficiency curve, no aerodynamic drag coefficient) — instead, **integrated squared acceleration over time** is used as a stand-in. This is a well-established simplification in trajectory-optimization literature (it's the same objective minimized by "minimum-jerk/minimum-acceleration" trajectory generators), because in a simple point-mass model, force (and hence power) scales with acceleration, so penalizing `∫a²dt` is a reasonable proxy for penalizing energy without needing a full dynamics model.

Running the simulation end-to-end on this map (94.84-unit arc-length path) gives:

| | Duration | Max speed | Energy proxy |
|---|---|---|---|
| **Min-Time** | 5.30 s | 18.66 units/s | 95.71 |
| **Min-Energy** | 45.50 s | 6.11 units/s | 11.69 |

i.e. Min-Time is **8.6× faster**, but Min-Energy uses **~88% less energy** by this proxy — which is exactly the trade-off the two profiles were designed to expose side by side.

---

## 5. `formation.py` — rigid "G"-shaped 9-drone formation

*(Reconstructed from bytecode — see note in §1.)*

```python
FORMATION_NAME = 'G'
FORMATION_OFFSETS = np.array([
    (-3.0, 4.0), (0.0, 4.0), (3.0, 4.0),
    (-4.0, 1.5), (-4.0, -1.5),
    (-3.0, -4.0), (0.0, -4.0), (3.0, -4.0), (3.0, 0.0)
])
N_DRONES = len(FORMATION_OFFSETS)   # 9
```

**Design decisions:**

- **The formation is a set of fixed (dx, dy) offsets from a moving centroid, not 9 independently planned paths.** `get_drone_positions(cx, cy)` just adds the offset array to a single centroid point, and `expand_to_drones(centroid_traj)` broadcasts an *entire trajectory dict* (x, y, t, vx, vy, ax, ay, speed, accel) across all 9 drones by adding `dx`/`dy` to the x/y channels and passing every other channel through untouched, tagging each with a `drone_id`. This is the single biggest architectural simplification in the project: **plan once, replicate nine times.** It's efficient (one A* search + one spline instead of nine) and guarantees the formation never distorts, but it also means:
  - No drone-specific obstacle avoidance — if the obstacle were wide enough, an outer drone in the "G" could clip it even though the centroid clears it. The safety margin (§2) has to be generous enough to cover the formation's radius (~4 units here) for this to be safe in practice, which is likely *why* `SAFETY_MARGIN=3` combined with `OBSTACLE_RADIUS=10` was chosen — worth flagging as an implicit, undocumented coupling between the two modules.
  - No inter-drone collision checking is needed *by construction*, since all drones move with an identical shared velocity vector — the formation's shape literally cannot change, so pairwise distances between drones stay constant automatically.
- **The 9 offsets are hand-authored coordinates that trace a "G" glyph** when plotted and connected by the `skeleton` edge list `[(2,0),(1,0),(0,3),(3,4),(4,5),(5,6),(6,7),(7,8)]` — i.e., top-right → top-left → down the left spine → around the bottom → back up to the inner bar, which is exactly the stroke order you'd draw a capital G in. This is simply an artistic/demonstrative choice (letter-shaped swarms are a common, visually compelling way to show off formation flight) rather than a functional one.
- **`visualise_formation()` renders the shape on a dark, "sci-fi HUD"-styled standalone plot** (`#1a1a2e` background, plasma colormap per drone, white scatter edges) — purely cosmetic, but consistent with the equally dark-themed animation panels in `simulate.py`, suggesting a deliberate visual identity for the whole project rather than default matplotlib styling.

---

## 6. `simulate.py` — orchestration, plotting, and animation

This is the entry point; it does not contain new algorithms, only **glue and visualization**, in 5 explicit stages (`[1/5]` … `[5/5]`):

1. **`plan_path()`** — get the smoothed A* waypoints.
2. **`save_path_plot()`** — a static overview PNG: obstacle + safety margin (as translucent circles), the blocked straight-line reference, the actual A* path, and start/goal markers. Building this as its own nested function (rather than importing `visualise_map`) means it can layer the *specific planned path* onto the map with its own annotation (an arrow labeling the obstacle's center/radius) — a reasonable one-off customization, though it does duplicate logic already in `map_setup.visualise_map`.
3. **`generate_trajectories()` + `plot_trajectory_comparison()`** — the min-time vs. min-energy speed/acceleration-vs-time comparison chart described in §4.
4. **`expand_to_drones()` called twice** — once for the min-time trajectory, once for min-energy — producing two independent 9-drone datasets so the animation can show both flight styles simultaneously, side by side.
5. **Animation build-out**, which is the most involved part of this file:
   - **`subsample(..., max_frames=200)`** — both drone trajectories get downsampled to at most 200 frames before animating. The min-time trajectory runs for 5.3 s at `DT=0.05` (≈106 samples) while the min-energy trajectory runs for 45.5 s at the same `DT` (≈910 samples) — a >8× difference in raw sample count. Capping both at ~200 frames keeps the GIF file size and render time bounded regardless of how long the min-energy trajectory is, at the cost of some temporal fidelity in the slow, long-duration run.
   - **`pad(...)`** — after subsampling, the two panels can end up with a different number of frames (since one trajectory is 8.6× longer in wall-clock time). The animation needs both panels driven by the *same* frame index for a synchronized `FuncAnimation`, so the shorter one is padded by repeating its final state — visually, the faster drone formation simply "arrives and holds position" while the slower one is still catching up, which correctly communicates the time-mismatch to the viewer rather than silently looping or desyncing.
   - **Dual-panel dark-themed figure** (`#0d0d1a` background), each drone colored via a `plasma` colormap sampled across `N_DRONES`, with fading position **trails** (`trail_len=25` frames) so drone motion history is visible without needing motion blur.
   - **`blit=True`** in `FuncAnimation` — only redraws the artists that actually changed (drone scatter points, trails, time-readout text) each frame instead of the whole figure, which is standard practice to keep matplotlib animation rendering fast, especially important here since there are 9 drones × 2 panels × ~200 frames of geometry being pushed per render.
   - Saved via the **Pillow writer at 20 fps / 80 dpi** — Pillow is the lightweight, dependency-free choice for GIF output (vs. `ffmpeg`, which would need to be separately installed), and 80 dpi keeps file size reasonable for a GIF meant for quick viewing rather than publication-quality export.
6. **Final console summary** — restates formation, path length, waypoint count, and both trajectories' duration/max-speed/energy, then computes and prints the **speedup ratio** and **energy-saving percentage** directly from the two metrics dicts — turning the whole run into one legible, human-readable takeaway rather than requiring the user to dig through the plots.

**`matplotlib.use('Agg')` at the top of the file** forces the non-interactive Agg backend before any other matplotlib import — this is what allows the whole script to run headlessly (no display/X server needed) purely to produce PNG/GIF files, which is exactly the right call for a script meant to be run from the command line or a CI environment rather than an interactive session.

---

## 7. End-to-end pipeline summary

```
map_setup.py           →  100×100 grid, obstacle @ (50,50) r=10, +3 safety margin
        │
        ▼
path_planner.py         →  A* on 8-connected grid  →  91-cell raw path
                            →  line-of-sight smoothing  →  4 waypoints
        │
        ▼
trajectory.py            →  cubic spline over arc length (94.84 units)
                            →  Min-Time profile   (18 u/s constant)   → 5.30 s
                            →  Min-Energy profile  (6 u/s trapezoid)  → 45.50 s
        │
        ▼
formation.py              →  9 fixed offsets tracing "G", applied to centroid
        │
        ▼
simulate.py                →  static plots + dual-panel animated GIF + console summary
```

Running the project (`python simulate.py`) reproduces exactly this, and I verified it by executing the full pipeline: it produces `results/path_plot.png`, `results/trajectory_comparison.png`, and `results/formation_animation.gif`, matching what's already checked into the repo's `results/` folder.

## 8. Design themes across the whole codebase

A few decisions recur across modules and are worth naming as project-wide choices rather than per-file ones:

- **Plan once at the centroid, replicate for the formation.** Consistently cheaper and simpler than per-drone planning, at the cost of formation-aware safety margins and no ability for individual drones to react to local obstacles.
- **Two named cross-cutting trajectory "modes" (time vs. energy) threaded through every downstream module** (trajectory generation, formation expansion, animation panels, summary stats) rather than one default speed — this is the project's core "finding," and the architecture is clearly built to make that comparison, not just to fly from A to B.
- **A consistent dark, colored, "mission-control" visual style** across the formation, animation, and (partially) path plots, suggesting the deliverable is meant to be shown/demoed, not just a debugging script.
- **Headless-first design** (`Agg` backend, everything saved to `results/` rather than shown interactively) — the project is built to run as a script and produce artifacts, not as an interactive notebook.

## 9. Notable limitations (given the above design choices)

- **2D only** — no altitude/z-axis, so vertical separation as a collision-avoidance strategy isn't modeled.
- **No real dynamics or control model** — speeds and accelerations are kinematic outputs of a spline, not the result of a vehicle model or controller being commanded to a reference trajectory.
- **No formation-aware obstacle clearance** — as noted in §5, the fixed safety margin has to implicitly be large enough to cover the whole formation's radius; the planner has no notion of formation width.
- **Single static obstacle** — the grid/A* machinery would generalize to multiple or moving obstacles, but nothing in this codebase populates the grid with more than the one circle.
- **The energy metric is a proxy, not a physical energy quantity** — useful for *relative* comparison between the two profiles, not for absolute energy budgeting (e.g., battery sizing).
