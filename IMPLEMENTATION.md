# Comparative Study of NLP Solvers on Multi-Agent Safe Navigation (CLF-CBF-QP)
## Complete Implementation Guide & Development Roadmap

**Project Category:** III – Safe Control and Optimization (CLF-CBF-QP)  
**Course:** CS424 / CS524 – Robotics and Control  
**Team:** Assadullah Farrukh, Talha Qureshi, Mihab Khan  
**Target System:** Multi-Agent Differential-Drive (TurtleBot3) in ROS/Gazebo  

---

## 1. Executive Summary & Course Compliance Checklist

This guide translates your one-page proposal into an actionable, week-by-week engineering plan that satisfies **every** requirement in the CS424/524 capstone rubric.

| Course Requirement | How This Guide Addresses It |
|---|---|
| **Functional ROS/Gazebo demo** | Phases 2–5 provide launch files, world files, multi-agent spawners, and a real-time safety-filter node. |
| **Mathematical proofs (Lyapunov, CBF, QP)** | Phase 1 derives the full state-space model, CLF stability proof, CBF forward-invariance proof, and the final QP/NLP standard form. |
| **State-space formulation & linearization** | Unicycle model + look-ahead point feedback transformation (exact I/O linearization). |
| **Optimization-based control** | Core CLF-CBF-QP formulation; solver abstraction layer for OSQP, IPOPT, and SNOPT (or SLSQP fallback). |
| **Quantitative evaluation** | Phase 5 defines scenarios, metrics (solve time, feasibility residual, control effort), and plotting scripts. |
| **IEEE-style final report (10–15 pg)** | Phase 6 maps each code module to a report section and lists the exact plots/proofs required. |
| **Screen recording & presentation** | Phase 6 includes recording commands and a recommended slide structure. |

---

## 2. High-Level System Architecture

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ROS / GAZEBO SIMULATION                           │
│  ┌──────────┐   /tf, /scan, /odom   ┌──────────────┐   cmd_vel   ┌────────┐│
│  │  Agent i │ ─────────────────────>│ Safety Filter│────────────>│Gazebo  ││
│  │ (Turtle3)│                       │ (CLF-CBF-QP) │             │ Plugin ││
│  └──────────┘                       └──────┬───────┘             └────────┘│
│       ▲                                    │                                │
│       │ state                              │ u_virt (safe)                  │
│       │                                    ▼                                │
│       │                         ┌─────────────────────┐                     │
│       └─────────────────────────│  Solver Benchmark   │                     │
│                                 │  - OSQP (sparse QP) │                     │
│                                 │  - IPOPT (NLP, IP)  │                     │
│                                 │  - SNOPT/SLSQP (SQP)│                     │
│                                 └─────────────────────┘                     │
│       ▲                                                                     │
│       │ /agent_states (all j ≠ i)                                           │
│       └─────────────────────────────────────────────────────────────────────┘
│                                    Multi-Agent Comms
└─────────────────────────────────────────────────────────────────────────────┘
```

### Key Design Decisions
1. **Robot Platform:** TurtleBot3 Burger (differential drive). Lightweight, well-supported in ROS, and its unicycle dynamics are the canonical model for CLF-CBF literature.
2. **Feedback Linearization:** We use a **look-ahead point** (a point offset from the robot center) to convert the nonholonomic unicycle into a *single-integrator* in the virtual point space. This yields affine CLF/CBF constraints and a clean convex QP—ideal for a fair solver benchmark.
3. **Decentralized Safety:** Each agent runs its own safety-filter node. Inter-agent safety is achieved via a **pairwise CBF** that uses communicated virtual controls from neighbors (or assumes constant velocity if comms drop).
4. **Solver Abstraction:** All solvers implement a common `solve(cost, constraints, bounds)` interface so the controller logic is solver-agnostic.

---

## 3. Phase 0 – Prerequisites & Workspace Setup

> **Platform Note:** Native ROS/Gazebo on Windows is **not recommended** for this project. Use **WSL2 (Ubuntu 20.04/22.04)** or a native Linux dual-boot. All commands below assume Ubuntu.

### 3.1 Install ROS & Gazebo
```bash
# Ubuntu 20.04 → ROS Noetic
sudo apt update && sudo apt install -y ros-noetic-desktop-full
# Ubuntu 22.04 → ROS2 Humble (if course allows; adjust syntax)
```

### 3.2 Install TurtleBot3 Packages
```bash
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
```

### 3.3 Python Optimization Stack
```bash
pip install numpy scipy matplotlib pyyaml rospkg
pip install osqp          # Sparse QP solver (primary)
pip install casadi        # Modeling language + IPOPT interface
pip install cyipopt       # Native IPOPT Python wrapper
# SNOPT is commercial. If unavailable, we use SciPy SLSQP as fallback:
pip install scipy         # For SLSQP / trust-constr fallback
```

### 3.4 Create Catkin Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

### 3.5 Project Directory (Recommended)
Create the following packages inside `~/catkin_ws/src/`:
```text
safe_nav_benchmark/
├── config/
│   ├── solver_params.yaml
│   └── experiment_scenarios.yaml
├── launch/
│   ├── single_agent_world.launch
│   ├── multi_agent_benchmark.launch
│   └── rviz.launch
├── scripts/
│   ├── safety_filter.py          # Main CLF-CBF-QP node
│   ├── nominal_controller.py     # Simple goal-to-virtual-input controller
│   ├── agent_state_broadcaster.py
│   ├── obstacle_publisher.py
│   ├── solver_wrappers/
│   │   ├── base_solver.py
│   │   ├── osqp_solver.py
│   │   ├── ipopt_solver.py
│   │   └── snopt_fallback.py     # SciPy SLSQP if SNOPT unavailable
│   └── utils/
│       └── math_utils.py
├── worlds/
│   └── cluttered.world
├── models/
│   └── cylinder_obstacle/
└── CMakeLists.txt & package.xml
```

---

## 4. Phase 1 – Mathematical Foundations (The "Why")

This section is **both** your codebase reference *and* the core of your IEEE report (Methodology & Proofs).

### 4.1 Robot Model: Unicycle → Look-Ahead Point

The TurtleBot3 obeys unicycle kinematics:
```
ẋ = v cos(θ)
ẏ = v sin(θ)
θ̇ = ω
```
State `q = [x, y, θ]ᵀ`, control `u = [v, ω]ᵀ`.

**Problem:** The position `(x, y)` has relative degree **2** with respect to `ω`, making standard CBFs fail directly.

**Solution:** Define a look-ahead point at distance `L > 0` ahead of the robot:
```
p = [ p_x ] = [ x + L cos(θ) ]
    [ p_y ]   [ y + L sin(θ) ]
```
Its dynamics are affine in `u`:
```
ṗ = R(θ) u,  where  R(θ) = [ cos(θ)  -L sin(θ) ]
                             [ sin(θ)   L cos(θ) ]
```
Because `det(R) = L ≠ 0`, we define a **virtual control**:
```
u_virt = [ u_x ] = R(θ) [ v  ]
         [ u_y ]         [ ω  ]
```
This renders the look-ahead point a **single integrator**: `ṗ = u_virt`.  
After solving the QP for `u_virt`, map back:
```
[ v ] = R(θ)⁻¹ u_virt
[ ω ]
```

> **Report Requirement:** Show the full Jacobian of the transformation, prove `R` is invertible, and state the domain `θ ∈ ℝ` where it holds.

### 4.2 Control Lyapunov Function (CLF) – Goal Convergence

Let `p_goal` be the target position. Define:
```
V(p) = ½ ||p - p_goal||²
```
Taking the time derivative along `ṗ = u_virt`:
```
V̇ = (p - p_goal)ᵀ u_virt
```
To drive `V → 0`, we require exponential stability:
```
V̇ ≤ -γ V     (γ > 0)
```
In a QP we relax this with a **slack variable** `δ ≥ 0` (penalized heavily in the cost):
```
L_g V u_virt ≤ -γ V + δ
```
where `L_g V = (p - p_goal)ᵀ`.

> **Lyapunov Proof for Report:**  
> If `δ = 0` (no relaxation active), `V̇ ≤ -γV < 0` for `p ≠ p_goal`. By Lyapunov’s direct method, `p_goal` is globally asymptotically stable. The slack `δ` is driven to zero by the cost penalty `q·δ²` with large `q`.

### 4.3 Control Barrier Function (CBF) – Static Obstacles

For an obstacle at `p_obs` with safety radius `r`, define:
```
h_s(p) = ||p - p_obs||² - r²
```
The safe set is `C = { p | h_s(p) ≥ 0 }`.  
To keep `p` inside `C` (forward invariance), enforce:
```
ḣ_s = L_g h_s u_virt ≥ -α(h_s)
```
where `L_g h_s = 2(p - p_obs)ᵀ` and `α` is a class-`K` function (e.g., `α(h) = k_α h` with `k_α > 0`).

> **CBF Proof for Report:**  
> Show `h_s` is a valid CBF by verifying that `L_g h_s ≠ 0` when `h_s = 0` (or use a higher-order CBF if needed). Apply Nagumo's theorem: if the CBF inequality holds, `C` is forward invariant; the robot never enters the danger disc.

### 4.4 Inter-Agent CBF – Collision Avoidance

For agent `i` and neighbor `j`, let `p_i`, `p_j` be their look-ahead points. Define:
```
h_ij(p_i, p_j) = ||p_i - p_j||² - D²
```
where `D` is the minimum inter-agent distance.  
Differentiating:
```
ḣ_ij = 2(p_i - p_j)ᵀ (u_virt_i - u_virt_j)
```
For a **decentralized QP**, agent `i` treats `u_virt_j` as a known constant (received via `/agent_states` or predicted). The constraint becomes affine in `u_virt_i`:
```
2(p_i - p_j)ᵀ u_virt_i ≥ -α(h_ij) + 2(p_i - p_j)ᵀ u_virt_j
```

> **Communication Note:** Each agent broadcasts its current `u_virt` (computed at the previous timestep) on a latched topic. This is a zero-order hold approximation; in practice it is stable for small `dt`.

### 4.5 Nominal Controller

A simple proportional controller drives the virtual point toward the goal:
```
u_nom = -k_p (p - p_goal)
```
This is the reference input fed into the QP cost.

### 4.6 Final QP Formulation (Standard Form)

**Decision variables:** `z = [u_virt_x, u_virt_y, δ]ᵀ ∈ ℝ³`  
**Cost:** Minimize deviation from nominal control + slack penalty
```
min_z   ½ zᵀ P z + qᵀ z
P = diag(1, 1, q_slack)       # q_slack = 1e4 (very large)
q = [-u_nom_x, -u_nom_y, 0]ᵀ
```
**Subject to:**
1. **CLF:** `(p - p_goal)ᵀ [u_virt_x; u_virt_y] + γ V - δ ≤ 0`
2. **Static CBF:** `-2(p - p_obs)ᵀ [u_virt_x; u_virt_y] ≤ α(h_s)`   *(one per obstacle)*
3. **Inter-Agent CBF:** `-2(p_i - p_j)ᵀ [u_virt_x; u_virt_y] ≤ α(h_ij) - 2(p_i - p_j)ᵀ u_virt_j`   *(one per neighbor)*
4. **Input Bounds (mapped):** `u_min ≤ R⁻¹ u_virt ≤ u_max`
5. **Slack positivity:** `-δ ≤ 0`

All constraints are affine in `z`; therefore this is a **convex QP**.

### 4.7 Solver Comparison Strategy

Your proposal lists *NLP* solvers (IPOPT, SNOPT) alongside OSQP (a QP solver). To make the comparison fair and academically honest:

| Solver | Type | Interface | Why Include It |
|---|---|---|---|
| **OSQP** | Sparse QP | `osqp` Python | Industry standard for real-time MPC/CBF; uses ADMM; supports warm-starting. |
| **IPOPT** | General NLP (Interior Point) | `cyipopt` or CasADi | Solves the same QP, but via a general NLP path. Benchmark highlights overhead of full NLP machinery vs. dedicated QP. |
| **SNOPT** | Sparse NLP (SQP) | `casadi` (if license available) | Gold-standard SQP for robotics. If unavailable, use **SciPy SLSQP** as a freely available SQP substitute and mention this in the report. |

**Recommendation:** Keep the core demo as the convex QP above. Then add **one** nonlinear extension (Phase 4) to create a true NLP so IPOPT/SNOPT demonstrate their strengths.

---

## 5. Phase 2 – ROS/Gazebo Simulation Environment

### 5.1 Robot Model & Namespace Convention

Use TurtleBot3 Burger URDF but spawn multiple instances under namespaces:
```xml
<!-- In launch file -->
<group ns="agent_0">
  <param name="tf_prefix" value="agent_0_tf" />
  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch" />
  <!-- Remap cmd_vel, odom, scan locally -->
</group>
```

### 5.2 World File (`cluttered.world`)

Create a Gazebo world with:
- **Floor:** 20 m × 20 m plane.
- **Static Obstacles:** 5–10 cylinders (`model://cylinder_obstacle`) at random `(x, y)`.
- **Dynamic Obstacle (optional):** One cylinder with a velocity plugin moving in a circle.

### 5.3 Sensor Configuration
- **Odometry:** Use Gazebo differential-drive plugin (`/agent_i/odom`).
- **LiDAR:** TurtleBot3 LDS-01 (`/agent_i/scan`). Use this for *obstacle detection* if you want to avoid hard-coding obstacle positions. For the benchmark, hard-coded positions ensure repeatability; LiDAR can be used in a "sensor-based" variant.

### 5.4 Multi-Agent Launch Sequence
```bash
roslaunch safe_nav_benchmark multi_agent_benchmark.launch num_agents:=3 scenario:=crossing
```
This launch file should:
1. Start Gazebo with `cluttered.world`.
2. Spawn `agent_0`, `agent_1`, `agent_2` at predefined poses.
3. Start an RViz config showing all agent paths and obstacle markers.
4. Start a `benchmark_logger` node.

---

## 6. Phase 3 – Core Software Implementation

### 6.1 Solver Abstraction Layer

Create `scripts/solver_wrappers/base_solver.py`:
```python
from abc import ABC, abstractmethod
import numpy as np

class BaseSolver(ABC):
    @abstractmethod
    def solve(self, P: np.ndarray, q: np.ndarray,
              G: np.ndarray, h: np.ndarray,
              A: np.ndarray = None, b: np.ndarray = None) -> dict:
        """
        Solve: min 0.5 x' P x + q' x
              s.t. G x <= h,  A x == b (optional)
        Returns dict with {'x': array, 'solve_time': float, 'status': str, 'iter': int}
        """
        pass
```

Implement three wrappers:
- **OSQP:** Convert `P, q, G, h` to OSQP's `P, q, A, l, u` format. Note that OSQP handles bounds as `l ≤ A x ≤ u`.
- **IPOPT:** Use `cyipopt` with a `Problem` class defining `objective`, `gradient`, `constraints`, `jacobian`. Since your problem is small (`n=3`), dense matrices are fine.
- **SLSQP (SNOPT fallback):** Use `scipy.optimize.minimize(method='SLSQP', bounds=..., constraints=...)`.

### 6.2 Safety Filter Node (`safety_filter.py`)

**Subscriptions:**
- `/agent_i/odom` → current pose `q = [x, y, θ]`
- `/agent_i/nominal_cmd` → `u_nom` from the nominal controller
- `/obstacles` → list of `(x, y, r)` static obstacles
- `/agent_states` → array of neighbor states + last `u_virt`

**Publish:** `/agent_i/cmd_vel` → `[v, ω]`

**Timer Callback (at 50–100 Hz):**
```python
def control_loop(self, event):
    # 1. State extraction
    p = compute_lookahead_point(x, y, theta, L)

    # 2. Build QP matrices P, q, G, h
    P, q = build_cost(u_nom, q_slack=1e4)
    G, h = build_constraints(p, p_goal, obstacles, neighbors, v_max, omega_max)

    # 3. Solve with selected solver
    result = self.solver.solve(P, q, G, h)

    # 4. Log metrics
    self.logger.log(
        solve_time=result['solve_time'],
        solver_status=result['status'],
        clf_violation=...,      # compute post-hoc
        cbf_violation=...,
        control_effort=np.linalg.norm(result['x'][:2])
    )

    # 5. Map back to unicycle and publish
    u_virt = result['x'][:2]
    v, omega = map_back(u_virt, theta, L)
    publish_cmd_vel(v, omega)
```

### 6.3 Nominal Controller (`nominal_controller.py`)
Simple proportional controller that publishes `u_nom` to `/agent_i/nominal_cmd`:
```python
u_nom = -k_p * (p - p_goal)
# Limit magnitude to avoid saturating the safety filter
u_nom = np.clip(u_nom, -u_nom_max, u_nom_max)
```

### 6.4 Agent State Broadcaster (`agent_state_broadcaster.py`)
Publishes a custom message (or `geometry_msgs/PoseArray` + `std_msgs/Float64MultiArray`) on `/agent_states` at the same rate as the safety filter, containing:
- `agent_id`
- `p_x`, `p_y`
- `u_virt_x`, `u_virt_y` (the *applied* control from previous cycle)

### 6.5 Obstacle Publisher (`obstacle_publisher.py`)
Publishes a `visualization_msgs/MarkerArray` of cylinders on `/obstacles` so RViz renders them and the safety filter can subscribe.

---

## 7. Phase 4 – Advanced NLP Extension (For a True Solver Comparison)

The standard QP above exercises OSQP perfectly but is "overkill" for IPOPT/SNOPT. To justify calling this an "NLP solver comparison," add **one** of the following nonlinear extensions to a second benchmark branch.

### Option A: Field-of-View (FOV) Visibility Cone (Recommended)
Robots have a forward-facing camera with a limited FOV half-angle `β`. To guarantee the goal remains visible while approaching:
```
h_fov(p, θ) = cos(β) - ( (p_goal - p) · [cos θ, sin θ] ) / ||p_goal - p||
```
`h_fov ≥ 0` means the goal is inside the FOV.  
`h_fov` is **nonlinear** in both `p` and `θ`. Its derivative with respect to `u_virt` and `v, ω` involves trigonometric terms. This creates a **non-convex NLP constraint**.

**Implementation:** Add `h_fov` and its Jacobian (computed via CasADi or manual differentiation) to the problem. Now the cost remains quadratic but constraints are nonlinear → true NLP.

### Option B: Minimum Curvature Cost
Replace the quadratic cost with a penalty on path curvature `κ = ω / v`:
```
min  ||u_virt - u_nom||² + λ (ω / v)²
```
This is nonlinear (rational) in `v`, making the objective non-quadratic.

### Option C: Velocity-Dependent Safety Margin
Make the safety radius shrink with speed: `r(v) = r₀ - k_v |v|`. The CBF constraint becomes bilinear in `v` and position.

> **Report Tip:** Run the core convex QP with all three solvers for baseline timing. Then run the NLP extension with IPOPT and SLSQP (and SNOPT if available). Compare: (a) solve time, (b) iteration count, (c) constraint violation, (d) control smoothness.

---

## 8. Phase 5 – Evaluation, Benchmarking & Scenarios

### 8.1 Defined Scenarios (Reproducible)

Store scenarios in `config/experiment_scenarios.yaml`:

| ID | Agents | Static Obs | Dynamic Obs | Goal Arrangement | Purpose |
|---|---|---|---|---|---|
| `S1_single_static` | 1 | 3 | 0 | Single goal | Sanity check; verify CLF convergence & CBF invariance. |
| `S2_crossing` | 2 | 0 | 0 | Opposite corners, crossing paths | Stress-test inter-agent CBF; agents must slow down and swerve. |
| `S3_dense` | 5 | 8 | 1 | Random goals | Scalability; measure solver time vs. agent count. |
| `S4_narrow_passage` | 2 | 2 (wall gap) | 0 | Goals through gap | Test tight CBF constraints and potential deadlock. |

### 8.2 Metrics to Log (at 50–100 Hz)

For each solver run, log a CSV with columns:
1. `t` – simulation time
2. `solve_time_ms` – solver wall-clock time
3. `solver_status` – `optimal`, `max_iter`, `infeasible`, etc.
4. `clf_violation` – `max(0, V̇ + γV - δ)` (should be ≈ 0)
5. `cbf_min_margin` – `min_j h_j(p)` (must stay ≥ 0)
6. `control_effort` – `||u_virt||`
7. `path_length` – cumulative `∫ ||ṗ|| dt`
8. `goal_error` – `||p - p_goal||`

### 8.3 Plotting Scripts (`scripts/analysis/`)
Create `generate_plots.py` to produce:
- **Trajectory overlay:** X-Y plot of all agent paths + obstacles (one per scenario).
- **Solve-time CDF:** Cumulative distribution of `solve_time_ms` per solver.
- **Barrier margin over time:** `min(h_ij)` to prove no collisions occurred.
- **Control effort comparison:** Time series of `||u||` per solver (smoothness).
- **Phase portrait (optional):** `V` vs. `V̇` to illustrate Lyapunov decrease.

> **Report Requirement:** Include all plots above in the Results section.

### 8.4 Quantitative Table Template (for Report)

| Metric | OSQP | IPOPT | SLSQP (SNOPT fallback) |
|---|---|---|---|
| Mean solve time (ms) | | | |
| Max solve time (ms) | | | |
| Std dev (ms) | | | |
| Collisions | 0 | 0 | 0 |
| Goal reached? | Yes | Yes | Yes |
| Avg. control effort | | | |
| Iterations (avg) | | | |

---

## 9. Phase 6 – Deliverables: Report, Video & Presentation

### 9.1 IEEE Report Structure (Map to Your Code)
Your final report must be 10–15 pages. Here is the exact mapping from this guide to each section.

1. **Abstract & Introduction**
   - Motivation: Real-time safety is hard; solver choice affects latency and robustness.
   - Context: CLF-CBF-QP framework for multi-agent systems.
   - Goal: Benchmark OSQP, IPOPT, and SQP-based solvers in identical Gazebo scenarios.

2. **Background & Related Work**
   - CLF/CBF theory (Ames et al.).
   - Existing solver comparisons in MPC (e.g., OSQP paper).

3. **Methodology & Mathematical Proofs** ⭐ *Most Important Section*
   - **3.1 Modeling:** Unicycle → look-ahead point (Section 4.1). Include `R(θ)` and its inverse.
   - **3.2 CLF Design:** Define `V`, derive `V̇`, state the exponential stability theorem with proof.
   - **3.3 CBF Design:** Define `h_s` and `h_ij`, derive constraints, prove forward invariance using Nagumo's theorem.
   - **3.4 QP Formulation:** Write the full standard-form QP with all matrices explicitly shown (Section 4.6).
   - **3.5 NLP Extension:** Present the FOV constraint (Section 7, Option A) and its Jacobian.

4. **Simulation Architecture**
   - ROS node graph (use `rqt_graph` screenshot).
   - Description of TurtleBot3, sensors, and namespace strategy.
   - Solver abstraction layer UML/class diagram.

5. **Results & Analysis**
   - Scenario descriptions (`S1`–`S4`).
   - Trajectory plots (Section 8.3).
   - Timing tables (Section 8.4).
   - Interpretation: *Why is OSQP faster?* (ADMM, warm-start). *When does IPOPT struggle?* (small dense problems have overhead). *What is the trade-off between speed and accuracy?*

6. **Discussion & Conclusion**
   - Limitations: Decentralized CBF without reciprocal guarantees; zero-order hold on neighbor controls.
   - Future work: Distributed optimization, hardware deployment.

### 9.2 Code Submission Checklist
```text
📁 bscs23213_bscs23122_bscs23168/
├── 📄 README.md                  # How to build & run
├── 📄 requirements.txt           # Python deps
├── 📁 report/
│   └── main.pdf                  # IEEE formatted
├── 📁 src/
│   └── safe_nav_benchmark/       # ROS package
│       ├── scripts/
│       ├── launch/
│       ├── config/
│       ├── worlds/
│       └── models/
├── 📁 data/
│   ├── logs/                     # Raw CSV logs
│   └── plots/                    # Generated figures
├── 📁 video/
│   └── demo.mp4                  # Screen recording
└── 📁 presentation/
    └── slides.pdf
```

### 9.3 Video Recording Guide
```bash
# 1. Launch a scenario
roslaunch safe_nav_benchmark multi_agent_benchmark.launch num_agents:=3 scenario:=dense

# 2. Record with ffmpeg (Linux)
ffmpeg -f x11grab -r 30 -s 1920x1080 -i :1.0 -pix_fmt yuv420p demo.mp4

# Or use OBS Studio.
```
**Video should show:**
- Gazebo view (top-down) of agents navigating without collisions.
- RViz view of CLF/CBF constraints (optional MarkerArray).
- Terminal showing real-time solver solve-times.
- At least one scenario with a "near-miss" where the CBF actively overrides the nominal controller.

### 9.4 Presentation Outline (10–12 min)
1. **Slide 1:** Title, team, category.
2. **Slide 2:** Problem statement & motivation.
3. **Slide 3:** Math (unicycle → look-ahead point) – keep concise, show `R(θ)`.
4. **Slide 4:** CLF proof (show `V` and `V̇ ≤ -γV`).
5. **Slide 5:** CBF proof (show `h` and forward invariance sketch).
6. **Slide 6:** QP formulation screenshot.
7. **Slide 7:** Gazebo demo video (embedded or linked).
8. **Slide 8:** Solver comparison table + plots.
9. **Slide 9:** Discussion / limitations.
10. **Slide 10:** Conclusion.

> **Presentation Defense:** Be ready to answer: *"Why did you choose the look-ahead point instead of a higher-order CBF?"* and *"What happens if the QP becomes infeasible?"* (Answer: add slack variables on CBFs too, or use a relaxed barrier function.)

---

## 10. Appendices

### Appendix A: Quick Launch Reference
```bash
# Terminal 1: Core simulation
roslaunch safe_nav_benchmark multi_agent_benchmark.launch num_agents:=3 solver:=osqp

# Terminal 2: Dynamic reconfigure (if needed)
rosrun rqt_reconfigure rqt_reconfigure

# Terminal 3: Logger / plotter
rosrun safe_nav_benchmark generate_plots.py --bag data/run_01.bag
```

### Appendix B: Common Pitfalls & Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| `OSQP: Max iter reached` | CBF constraints conflict (deadlock) | Reduce `k_α`, increase safety margin `D`, or add CBF slack. |
| Robot spins in place | `L` too small or `v` bounds too tight | Increase look-ahead `L` (try 0.1–0.2 m). |
| Agents collide in Gazebo | Inter-agent CBF uses outdated `u_virt_j` | Increase publish rate to 100 Hz; ensure `/agent_states` is latched. |
| IPOPT reports "Infeasible" | Jacobian mismatch in wrapper | Double-check `cyipopt` constraint Jacobian against analytical derivative. |
| Gazebo crashes on spawn | Namespace collision in URDF | Ensure `<group ns="...">` wraps all params and plugins. |

### Appendix C: Expected Git History (Recommended Milestones)
```
commit 1: [INIT] ROS package skeleton + world files
commit 2: [MATH] Add unicycle math utils + look-ahead transform
commit 3: [CLF] Nominal controller + goal convergence demo
commit 4: [CBF] Static obstacle avoidance (single agent)
commit 5: [MULTI] Inter-agent CBF + state broadcaster
commit 6: [SOLVERS] OSQP wrapper integration
commit 7: [SOLVERS] IPOPT wrapper integration
commit 8: [SOLVERS] SLSQP fallback + benchmark logger
commit 9: [NLP] FOV nonlinear constraint extension
commit 10: [EVAL] Scenarios, plotting scripts, dense world
commit 11: [DOC] Final report LaTeX + video
```

---

## 11. Final Checklist Before Submission

- [ ] All agents reach goals in `S1`–`S4` without collisions.
- [ ] OSQP, IPOPT, and SLSQP have all been tested in at least one shared scenario.
- [ ] `solve_time` CSV logs exist for every solver × scenario pair.
- [ ] At least one figure in the report contains a **Lyapunov function plot** showing `V(t)` decreasing.
- [ ] At least one figure shows **CBF margin** `h(t) ≥ 0` for the entire trajectory.
- [ ] The report contains the **full QP matrix formulation** (Section 4.6).
- [ ] The report contains a **proof sketch** of CBF forward invariance.
- [ ] Video is under 3 minutes and shows Gazebo + terminal.
- [ ] Code is pushed to a repository (GitHub/GitLab) with a clean `README.md`.

---

**Good luck! This roadmap is designed to keep you on track from proposal to final demo. If a specific solver install fails or a Gazebo plugin misbehaves, debug that sub-system in isolation before integrating it back into the full pipeline.**
