# 🌌 Unity XPBD Cubic Barrier Physics Engine

This project is a high-performance, Data-Oriented physics engine implemented in Unity using **DOTS (Data-Oriented Technology Stack)** and the **Burst Compiler**.

[cite_start]It implements the **"Cubic Barrier with Elasticity-Inclusive Dynamic Stiffness"** method proposed by **Ryoichi Ando (2024)** [cite: 4, 40][cite_start], adapted for real-time simulation using **Extended Position Based Dynamics (XPBD)**[cite: 8, 83].

[cite_start]Unlike standard Unity physics (PhysX), this engine is particle-based and resolves collisions using energy-based **Cubic Barriers** rather than impulses or penalty forces[cite: 3, 41]. [cite_start]This allows for "offline-quality" stability where soft objects behave softly and rigid objects behave rigidly during collisions, without manual parameter tuning[cite: 8, 42].

---

## 🚀 Key Features

* [cite_start]**XPBD (Extended Position Based Dynamics):** Solves constraints using compliance ($\alpha$) rather than stiffness ($k$), ensuring the simulation behaves consistently regardless of frame rate or iteration count[cite: 84, 89].
* **Cubic Barrier Collision:** Replaces unstable logarithmic barriers with a $C^2$ continuous cubic polynomial. [cite_start]This guarantees non-penetration with a bounded force that doesn't "explode" the solver[cite: 7, 42, 44].
* **Elasticity-Inclusive (Parameter-Free):** The engine automatically calculates the barrier stiffness based on the topology of the object. [cite_start]A soft jelly will have a soft collision response; a steel chain will have a hard response[cite: 45, 49].
* **High Performance:** Written entirely in C# using `NativeArray` and `Burst`. [cite_start]Capable of simulating 100,000+ particles with self-collision[cite: 112, 142].
* [cite_start]**Spatial Hashing:** Uses a `NativeParallelMultiHashMap` to perform broadphase collision detection in $O(N)$ time[cite: 127, 157].

---

## 📂 Architecture Overview

The project is strictly separated into **Core** (Pure Math/Data) and **UnityIntegration** (Visuals/Components).

### 1. 🧠 Core (Pure Physics)
*Located in `Scripts/Physics/Core`. These classes have no dependency on `UnityEngine.GameObject` or `MonoBehaviour`.*

| Class / File | Role | Underlying Logic |
| :--- | :--- | :--- |
| **`PhysicsContext.cs`** | **Memory (RAM)** | [cite_start]Holds the raw simulation state (Position, Velocity, InverseMass) and Topology (Constraints) in **Structure of Arrays (SoA)** format for cache efficiency[cite: 134, 136]. Also stores the **Automatically Calculated Stiffness** for every particle. |
| **`PhysicsEngine.cs`** | **The Brain** | The high-level API. It manages memory, configuration, and schedules the **Jobs** to run on worker threads. It handles the "Parameter-Free" logic by accumulating stiffness from constraints when `AddConstraint` is called. |
| **`PhysicsJobs.cs`** | **The Workers** | Contains the **Burst-Compiled** logic. [cite_start]This includes the *Prediction* [cite: 145][cite_start], *Broadphase* [cite: 146][cite_start], *Cubic Barrier Solver* [cite: 151][cite_start], and *XPBD Constraint Solver*[cite: 150]. This is where the math happens. |
| **`SpatialGrid.cs`** | **The Map** | A mathematical helper that maps 3D world positions to 1D hash integers. [cite_start]It allows the solver to find neighboring particles instantly without checking every other particle[cite: 157, 160]. |
| **`PhysicsConfig.cs`** | **Settings** | Defines global simulation parameters like Gravity, Solver Iterations, and the `BarrierStiffnessRatio`. |
| **`PhysicsConstants.cs`** | **Constants** | Stores immutable values like Earth Gravity ($9.81$) and Epsilon tolerances to avoid magic numbers in the code. |

### 2. 🔌 Unity Integration
*Located in `Scripts/Physics/UnityIntegration`. These bridge the Core engine to the Unity Scene.*

| Class / File | Role |
| :--- | :--- |
| **`PhysicsSceneHook.cs`** | Connects the Unity `FixedUpdate` loop to the `PhysicsEngine.Step()` method. It acts as the "Manager" for the scene. |
| **`XPBDBody.cs`** | The component you add to GameObjects (like a Rigidbody). It registers the object with the engine at `Start` and syncs the transform position every frame. |
| **`XPBDParticleRenderer.cs`** | [cite_start]A high-performance renderer that uses **GPU Instancing** (`DrawMeshInstancedIndirect`) to draw thousands of particles directly from the physics data buffers, bypassing the overhead of GameObjects[cite: 10, 187]. |

---

## 🧪 How It Works (The Simulation Loop)

Every `FixedUpdate`, the engine performs the following steps `SolverIterations` times:

1.  [cite_start]**Prediction:** Velocity and Gravity are applied to predict where particles *want* to be ($x_{pred} = x + v\Delta t$)[cite: 145].
2.  **Broadphase (Spatial Hashing):** Particles are mapped into a hash grid. [cite_start]We clear and rebuild this map every sub-step to handle fast-moving objects[cite: 157].
3.  **Cubic Barrier Solver (The "Ando" Step):** The solver checks neighbors. [cite_start]If distance $d < \hat{g}$ (sum of radii)[cite: 43]:
    * [cite_start]It reads the **Local Stiffness** of the particle (automatically derived from its constraints)[cite: 49].
    * [cite_start]It calculates a **Cubic Factor** ($t^3$) based on penetration depth[cite: 44].
    * [cite_start]It applies a correction impulse that is soft at the surface but becomes rigid at the center[cite: 42].
4.  [cite_start]**Topology Solver (Constraints):** Distance constraints (the structure of the cloth/rope) are resolved using XPBD to maintain the shape[cite: 85, 86].
5.  [cite_start]**Integration:** Velocities are updated based on the valid non-penetrating positions ($v = (x_{new} - x_{old}) / \Delta t$)[cite: 153].

---

## 🛠️ Usage Guide

### 1. Setup the Scene
1.  Create an empty GameObject and add the **`PhysicsSceneHook`** component.
2.  (Optional) Add the **`XPBDParticleRenderer`** to visualize particles. Assign a Mesh (Sphere) and Material.

### 2. Creating Objects
Add the **`XPBDBody`** component to any Cube or Sphere.

* **Static Floor:**
    * Set `IsKinematic = True`.
    * Set `Material Compliance = 0` (Hard).
* **Falling Ball:**
    * Set `IsKinematic = False`.
    * Set `Material Compliance = 0.1` (Soft) or `0.0` (Hard).

### 3. Understanding Parameters

* **Material Compliance:**
    * [cite_start]This is the inverse of stiffness ($\alpha = 1/k$)[cite: 84].
    * `0.0`: **Concrete / Steel**. The barrier will be extremely hard.
    * `0.01`: **Hard Plastic / Wood**.
    * `1.0`: **Rubber / Jelly**. The barrier will yield slightly to absorb energy.
    * *Note: You do not set the Barrier Stiffness directly. [cite_start]The engine calculates it from this value*[cite: 49].

* **Barrier Stiffness Ratio (Global Config):**
    * Found in `PhysicsConfig`.
    * Defines how much stiffer the barrier is compared to the object.
    * Default `1.0` means the barrier is as stiff as the object. `10.0` means it is 10x stiffer (safer for preventing tunneling).

---

## 📚 References

* [cite_start]**Ryoichi Ando (2024):** *A Cubic Barrier with Elasticity-Inclusive Dynamic Stiffness*[cite: 4].
* **Macklin et al. (2016)[cite_start]:** *XPBD: Position-Based Simulation of Compliant Constrained Dynamics*[cite: 83, 84].
* **Teschner et al. (2003)[cite_start]:** *Optimized Spatial Hashing for Collision Detection of Deformable Objects*[cite: 160].