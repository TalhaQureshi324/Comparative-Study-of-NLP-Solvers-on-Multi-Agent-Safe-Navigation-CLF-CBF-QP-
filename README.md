# Comparative Study of NLP Solvers on Multi-Agent Safe Navigation

**Course:** CS424 / CS524 – Robotics and Control  
**Category:** III – Safe Control and Optimization (CLF-CBF-QP)  
**Team:** Assadullah Farrukh (bscs23213), Talha Qureshi (bscs23122), Mihab Khan (bscs23168)

---

## Project Summary

This project benchmarks different optimization backends—**OSQP**, **IPOPT**, and an **SQP fallback (SciPy SLSQP / SNOPT)**—on a real-time multi-agent safe navigation problem. Each differential-drive robot (TurtleBot3) must reach its goal while avoiding collisions with static obstacles and other agents. Safety and convergence are mathematically guaranteed via **Control Barrier Functions (CBF)** and **Control Lyapunov Functions (CLF)**, unified in a Quadratic Program (QP) solved at every control timestep.

### Key Features
- **Mathematically Rigorous:** Full Lyapunov and CBF proofs provided in the report; constraints guarantee forward invariance of safe sets.
- **Solver-Agnostic Architecture:** A unified `BaseSolver` interface allows swapping OSQP, IPOPT, or SLSQP without changing the controller logic.
- **Reproducible Benchmarks:** Pre-defined scenarios (`single_static`, `crossing`, `dense`, `narrow_passage`) with automated logging and plotting.
- **ROS / Gazebo Simulation:** Multi-agent TurtleBot3 simulation with obstacle visualization and real-time metrics.

---

## Repository Structure

```text
.
├── IMPLEMENTATION.md          # Detailed step-by-step build guide (START HERE)
├── README.md                  # This file
├── robotics project proposal.pdf
│
└── src/
    └── safe_nav_benchmark/    # ROS package (see IMPLEMENTATION.md Phase 3)
        ├── config/            # Solver params & scenario definitions
        ├── launch/            # Gazebo + multi-agent spawners
        ├── scripts/           # Python nodes & solver wrappers
        ├── worlds/            # Gazebo world files
        └── models/            # Obstacle models
```

---

## Quick Start

> **Prerequisite:** Ubuntu 20.04 (or WSL2) with ROS Noetic installed. See `IMPLEMENTATION.md` Phase 0 for full setup.

```bash
# 1. Build the workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 2. Launch a single-agent sanity check
roslaunch safe_nav_benchmark single_agent_world.launch solver:=osqp

# 3. Run a 3-agent dense benchmark
roslaunch safe_nav_benchmark multi_agent_benchmark.launch num_agents:=3 scenario:=dense solver:=ipopt

# 4. Generate plots from logs
rosrun safe_nav_benchmark generate_plots.py --log_dir logs/run_01/
```

---

## Mathematical Core (Summary)

1. **Unicycle → Look-Ahead Point:** We transform the nonholonomic differential-drive model into a single-integrator via an invertible feedback law on a point `L` meters ahead of the robot.
2. **CLF:** `V = ½‖p − p_goal‖²` guarantees exponential convergence: `V̇ ≤ −γV`.
3. **CBF:** `h = ‖p − p_obs‖² − r²` guarantees collision avoidance via forward invariance of the safe set `C = {h ≥ 0}`.
4. **QP:** At each timestep we solve:
   ```
   min   ‖u_virt − u_nom‖² + q·δ²
   s.t.  CLF(p) + δ ≥ 0
         CBF_static(p) ≥ 0
         CBF_inter_agent(p_i, p_j) ≥ 0
         u_min ≤ R(θ)⁻¹ u_virt ≤ u_max
   ```

For the full derivation, standard-form matrices, and proof sketches, see **Section 4** of `IMPLEMENTATION.md`.

---

## Development Roadmap

| Phase | Goal | Approx. Time |
|---|---|---|
| **0** | Environment setup (ROS, solvers, workspace) | 1–2 days |
| **1** | Derive math & proofs (report backbone) | 3–4 days |
| **2** | Build Gazebo world & multi-agent spawner | 2 days |
| **3** | Implement safety filter + solver wrappers | 5–7 days |
| **4** | Add NLP extension (FOV constraint) | 2–3 days |
| **5** | Run benchmarks, generate plots | 2–3 days |
| **6** | Write IEEE report, record video, prepare slides | 4–5 days |

> **Total estimated effort:** 3–4 weeks of focused work.

See `IMPLEMENTATION.md` for the exhaustive version of each phase.

---

## Deliverables Checklist

- [ ] ROS/Gazebo simulation demo (screen recording)
- [ ] Source code & launch files
- [ ] Comparative analysis report (IEEE format, 10–15 pages)
- [ ] Performance plots (solve time, barrier margins, trajectories)
- [ ] In-class presentation with mathematical defense

---

## Citation & References

- Ames, A. D. et al. *Control Barrier Functions: Theory and Applications*. ECC 2019.
- Stellato, B. et al. *OSQP: An Operator Splitting Solver for Quadratic Programs*. Mathematical Programming Computation, 2020.
- Wächter, A. & Biegler, L. T. *On the Implementation of an Interior-Point Filter Line-Search Algorithm for Large-Scale Nonlinear Programming*. Mathematical Programming, 2006.

---

**For questions about solver installation, Gazebo crashes, or mathematical derivations, consult the Troubleshooting section in `IMPLEMENTATION.md` (Appendix B).**
