# 🌌 Unity XPBD Cubic Barrier Physics Engine (PPF Implementation)

This project is a high-performance, Data-Oriented physics engine implemented in Unity using **DOTS (Data-Oriented Technology Stack)** and the **Burst Compiler**.

Its primary goal is to replicate the fidelity and robustness of **Ryoichi Ando's (2024)** *"Cubic Barrier with Elasticity-Inclusive Dynamic Stiffness"* (PPF Solver) within a real-time game engine context.

Instead of using traditional Impulse-based methods (like PhysX) or global energy minimization (like the original PPF), this engine adapts the **Cubic Barrier** theory to **Extended Position Based Dynamics (XPBD)**. This allows for "offline-quality" contact resolution—where soft objects stay soft and rigid objects stay rigid—without the computational cost of implicit Newton solvers.

---

## 🎯 Core Philosophy: "Elasticity-Inclusive Contact"

In traditional game physics, collision stiffness is a magic number. If set too high, soft objects explode; if too low, rigid objects sink into each other.

**This engine solves this by deriving contact stiffness from the object's own material properties:**
* **Soft Body (Jelly):** The barrier automatically becomes compliant, absorbing impact.
* **Rigid Body (Steel):** The barrier becomes infinitely stiff, preventing penetration.
* **Result:** A parameter-free contact system where you define *Material*, not *Collision*.

---

## 🚀 Key Features

* **Unified Solver:** Handles both **Single Particles** (Soft Bodies/Ropes) and **Rigid Bodies** (Solids) in the same simulation loop.
* **Cubic Barrier Potential:** Implements Ando's non-linear barrier force ($f \propto d^3$). This guarantees non-penetration with a force that ramps up smoothly, eliminating jitter/popping.
* **Hybrid Collision Detection:**
    * **Particle-Particle:** Fast, radius-based checks for ropes and granular materials.
    * **Point-Triangle:** Exact surface collision for rigid bodies against the world.
* **XPBD Architecture:** Uses *Compliance* ($\alpha$) instead of Stiffness ($k$), ensuring stability independent of time-step size.
* **High Performance:**
    * **Body-Centric Slicing:** Eliminates race conditions in parallel jobs.
    * **Spatial Hashing:** $O(N)$ broadphase using `NativeParallelMultiHashMap`.
    * **Burst Compiled:** All math runs on optimized native machine code.

---

## 📂 Architecture Overview

The codebase is strictly separated into **Core** (Pure C# Math) and **UnityIntegration** (Visuals/MonoBehaviours).

### 1. 🧠 Core (Pure Physics)
*Located in `Scripts/Physics/Core`. No Unity GameObjects allowed here.*

| Class / File | Role | Implementation Detail |
| :--- | :--- | :--- |
| **`PhysicsContext`** | **The Memory (SoA)** | Holds raw arrays for Particles (`Position`, `Velocity`) and Rigid Bodies (`Inertia`, `Rotation`). Implements the **Slicing** mechanism to map Bodies to their constituent particles. |
| **`PhysicsEngine`** | **The API** | Manages the simulation lifecycle. It handles the **"Elasticity-Inclusive"** logic by accumulating stiffness from constraints and feeding it to the solver jobs. |
| **`PhysicsJobs`** | **The Workers (Burst)** | Contains the parallel logic: <br>• **`RigidBodyPrediction`**: Integrates velocities.<br>• **`BuildSpatialMap`**: Hashes particles to grid cells.<br>• **`RigidBodyBarrierJob`**: The core **Cubic Barrier** solver.<br>• **`ParticleBarrierJob`**: Handles soft-body self-collisions. |
| **`PhysicsConfig`** | **The Rules** | Defines global settings like `Gravity`, `SolverIterations`, and the **`BarrierStiffnessRatio`** (Ando's global tuning parameter). |
| **`SpatialGrid`** | **The Broadphase** | Maps 3D coordinates to 1D hash keys for fast neighbor lookup. |

### 2. 🔌 Unity Integration
*Located in `Scripts/Physics/UnityIntegration`. Connects the Core to the Unity Scene.*

| Class / File | Role |
| :--- | :--- |
| **`PhysicsSceneHook`** | The "Manager". Initializes the engine, syncs time (`FixedUpdate`), and disposes memory. |
| **`XPBDRigidBody`** | Converts a Unity Mesh into a Physical Body. It voxelizes the mesh or uses vertices to create **Collision Proxies**, calculates **Inertia Tensors**, and syncs the Transform. |
| **`XPBDBody`** | A lightweight component for single particles (e.g., falling spheres, debris). |
| **`XPBDParticleRenderer`** | Debug visualizer using **GPU Instancing** to render thousands of collision proxies efficiently. |

---

## 🧪 The Simulation Loop

Every `FixedUpdate`, the engine executes the following **XPBD Sub-Steps**:

1.  **Prediction:** * Velocities are updated with gravity.
    * Candidate positions ($x_{pred}$) are calculated.
2.  **Broadphase:**
    * Particles and Triangles are mapped into the `SpatialGrid`.
3.  **Solver Phase (Iterative):**
    * **Barrier Solver:** Checks for collisions using the **Cubic Barrier**.
        * *If penetrating:* Calculates a corrective impulse based on the **Combined Stiffness** of the colliding bodies.
        * *Two-Way Coupling:* Impulse is applied to both the Particle (Push) and the Rigid Body (Torque/Force).
    * **Topology Solver:** Resolves internal constraints (if any soft body links exist).
4.  **Integration:**
    * Final velocities are updated based on the corrected positions.
5.  **Visual Sync:**
    * Data is copied to visualization buffers for rendering.

---

## 🛠️ Usage Guide

### 1. Setup the World
1.  Create an empty GameObject.
2.  Add the **`PhysicsSceneHook`** component.
3.  Set `Grid Cell Size` to approx **1.0** (or slightly larger than your average object).

### 2. Create a Rigid Body (Solid)
1.  Add a **Cube** or **Mesh**.
2.  Add **`XPBDRigidBody`**.
3.  Set **Generation Mode**:
    * `BoxCornersOnly`: For simple boxes (Fast).
    * `MeshVertices`: For complex shapes (High Fidelity).
4.  Set **Deformation Compliance**:
    * `0.0`: Hard Body (Steel).
    * `0.1`: Soft Body (Rubber).

### 3. Create a Particle (Soft/Simple)
1.  Add a **Sphere**.
2.  Add **`XPBDBody`**.
3.  Adjust `Radius` and `Mass`.

---

## 📚 References

* **Ryoichi Ando (2024):** *A Cubic Barrier with Elasticity-Inclusive Dynamic Stiffness.* (The foundation of this engine).
* **Macklin et al. (2016):** *XPBD: Position-Based Simulation of Compliant Constrained Dynamics.*
* **Teschner et al. (2003):** *Optimized Spatial Hashing for Collision Detection of Deformable Objects.*