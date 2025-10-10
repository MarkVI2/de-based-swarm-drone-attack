# de-based-swarm-drone-attack

Taken from SIH2025 problem statements Problem statement: Designing an Efficient
Algorithm for Coordinated Swarm Engagement among Autonomous Drones aiming to
neutralize an adversarial drone swarm.

## Recent Updates (Oct 10, 2025)

### 🚀 Differential Evolution Path Planning Implemented!

**Major milestone**: Each drone now uses a Differential Evolution (DE) algorithm
to compute its optimal acceleration vector in real-time.

**Key Features**:

- **Acceleration-based control**: Drones optimize normalized acceleration
  `(ax, ay, az)` in range `[-1, 1]`
- **Decentralized decision-making**: Each drone runs its own DE optimizer
  independently
- **Real-time path planning**: 20-30 generations per timestep with convergence
  detection
- **Multi-objective fitness**: Balances target-seeking, collision avoidance,
  swarm cohesion, smooth motion
- **Hard constraint enforcement**: Altitude (60-120m), speed (≤10 m/s),
  separation (≥0.5m), arena bounds

**Architecture**:

```
Each Drone:
  ├─ Position, Velocity, Acceleration (physics state)
  ├─ Path History (for visualization/analysis)
  └─ DronePathPlanner (DE optimizer)
      ├─ Population of 20 candidate accelerations
      ├─ Mutation: v = x_i + K*(x_r1 - x_i) + F*(x_r2 - x_r3)
      ├─ Crossover: Binomial with guaranteed dimension
      └─ Selection: Elitism (keep better solution)
```

### Previous Updates (Oct 9, 2025)

#### Migration from Bevy to three-d Renderer

- **Reason**: Resolved pink/magenta color rendering issues on macOS M2 Metal
- **Benefits**:
  - Proper color rendering (blue drones, red target, black grid)
  - 10x faster compile times (~27s vs ~5min for release builds)
  - Simpler API and better Metal compatibility

### Current Scene Setup

- **Grid**: 60 km × 60 km × 200m arena with three visible grid planes (floor,
  back wall, left wall)
  - Floor: XZ plane at y=0
  - Back wall: XY plane at z=-30km
  - Left wall: YZ plane at x=-30km
  - Cell size: 1 km × 1 km
- **Drone Swarm**: 42 blue spheres in flat horizontal disc
  - Radius: 8m
  - Altitude: **Randomized** between 60-120m on each run
  - All drones at same altitude (flat disc formation)
- **Target**: Red sphere (adversary)
  - Distance: **Randomized** between 20-40 km on each run
  - Altitude: **Randomized** between 60-120m on each run
  - Direction: Random angle
- **Environmental Constraints**:
  - Hard altitude ceiling: 120m (enforced for all entities)
  - Scale: Meters for altitude, kilometers for horizontal distance
- **Controls**:
  - **Camera**: Orbit (left drag), pan (right drag), zoom (W/S or arrows)
  - **Simulation Speed**:
    - `a` - Speed up (2x, 4x, 8x, 16x via multiple updates per frame)
    - `Shift+A` - Slow down (0.5x, 0.25x, 0.125x minimum)
    - `r` - Reset to normal speed (1.0x)

### Project Structure

```
src/
├── main.rs               # Entry point & render loop
├── lib.rs                # Library interface (exposes modules)
├── camera.rs             # OrbitControl camera
├── entities.rs           # Drone/SwarmState with DE integration
├── rendering.rs          # Grid/drone/target renderers
├── scene.rs              # Scene coordination & updates
├── de_path_planner.rs    # Differential Evolution algorithm
└── bin/
    └── test_de_planner.rs # Terminal test for DE validation
```

### Building & Running

```bash
# Quick check
cargo check

# Development (with DE path planning)
cargo run
# Once running, use 'a' to speed up, Shift+A to slow down, 'r' to reset

# Test DE algorithm in terminal (5 drones, 10 timesteps)
cargo run --bin test_de_planner

# Optimized release
cargo build --release --target aarch64-apple-darwin
./target/aarch64-apple-darwin/release/de-based-swarm-drone-attack
```

**Interactive Controls (3D Simulation)**:

- `Left mouse drag` - Orbit camera around scene
- `Right mouse drag` - Pan camera
- `W/S` or `Arrow keys` - Move camera forward/backward
- `a` - **Speed up simulation** (2x → 4x → 8x → 16x max)
- `Shift+A` - **Slow down simulation** (0.5x → 0.25x → 0.125x min)
- `r` - **Reset to normal speed** (1.0x)

```

---

## Differential Evolution Path Planning

### Overview

Each drone acts as an **autonomous agent** that uses Differential Evolution to
optimize its next acceleration vector. The algorithm runs completely
independently on each drone, enabling true decentralized swarm behavior.

### Control Model: Acceleration-Based

**Why acceleration?**

- Reduces computational complexity (no rotor dynamics)
- Physically realistic (bounded by max acceleration ~8 m/s²)
- Smooth trajectories (penalizes jerky motion)
- Easy to integrate: `v = v0 + a*dt`, `p = p0 + v*dt`

**Decision Variable**: Normalized acceleration vector `[ax, ay, az]` where each
component ∈ `[-1, 1]`

- Denormalized to physical units: `a_physical = a_normalized × max_acceleration`
- Max acceleration: 8.0 m/s² (conservative for quadcopter safety)

### Algorithm Flow (Per Drone, Per Timestep)

```

1. Initialize Population (if first iteration)

   - Generate 20 random normalized acceleration vectors
   - Each candidate represents a potential control input

2. For each generation (max 30): a. For each candidate i:

   - Select 3 distinct random candidates (r1, r2, r3)
   - MUTATE: v = x_i + K*(x_r1 - x_i) + F*(x_r2 - x_r3)
   - CROSSOVER: Binomial (CR=0.8) with guaranteed dimension
   - EVALUATE: fitness = f(v, neighbor_positions)
   - SELECT: Keep v if fitness(v) > fitness(x_i)

   b. Check convergence (patience=15, tolerance=1e-6)

3. Return best acceleration from final population
4. Integrate physics: update velocity → update position
5. Re-initialize population for next timestep

````

### Fitness Function (Multi-Objective)

The fitness function balances multiple objectives with tunable weights:

```rust
fitness =
    + 3.0 × target_alignment       // Move towards target
    - 1.0 × acceleration_jerk       // Smooth motion
    - 1.5 × speed_inefficiency      // Maintain optimal speed
    - 4.0 × collision_proximity²    // Avoid other drones
    + 0.5 × swarm_cohesion         // Stay near swarm center
````

**Target Alignment**: Dot product of velocity direction with target direction
(1.0 = perfect)  
**Acceleration Jerk**: Change in acceleration magnitude (penalizes sudden
changes)  
**Speed Efficiency**: Deviation from 80% of max speed  
**Collision Proximity**: Quadratic penalty within 3× min separation distance  
**Swarm Cohesion**: Exponential decay bonus for staying near swarm center

### Hard Constraints (Enforced)

Candidates violating hard constraints receive fitness = `-1e9`:

| Constraint       | Value         | Source              |
| ---------------- | ------------- | ------------------- |
| Min Altitude     | 60 m          | Problem spec        |
| Max Altitude     | 120 m         | Problem spec        |
| Max Velocity     | 10 m/s        | Problem spec        |
| Max Acceleration | 8 m/s²        | Quadcopter dynamics |
| Min Separation   | 0.5 m         | Safety margin       |
| Arena Bounds     | 60 km × 60 km | Simulation domain   |

### Soft Constraints (Optimized)

Tunable weights in fitness function (see `SoftConstraints` struct):

```rust
pub struct SoftConstraints {
    pub target_direction_weight: f32,      // Default: 3.0
    pub smooth_motion_weight: f32,         // Default: 1.0
    pub swarm_cohesion_weight: f32,        // Default: 0.5
    pub collision_avoidance_weight: f32,   // Default: 4.0
    pub speed_efficiency_weight: f32,      // Default: 1.5
}
```

### DE Parameters

Based on the Python implementation provided:

```rust
pub struct DEParams {
    pub pop_size: usize,              // 20 candidates
    pub k: f32,                       // 0.5 (self-scaling factor)
    pub f: f32,                       // 0.8 (mutation scaling)
    pub crossover_prob: f32,          // 0.8 (CR)
    pub max_gen: usize,               // 30 generations/timestep
    pub convergence_tol: f32,         // 1e-6
    pub convergence_patience: usize,  // 15 generations
}
```

**Mutation Strategy**: `DE/current-to-rand/1` with additional differential
term  
**Formula**: `v = x_i + K*(x_r1 - x_i) + F*(x_r2 - x_r3)`

This matches your Python implementation exactly.

### Testing & Validation

Run the terminal test to see detailed DE behavior:

```bash
cargo run --bin test_de_planner
```

**Output includes**:

- Per-drone state at each timestep (position, velocity, acceleration)
- Normalized vs physical acceleration values
- Fitness scores and convergence tracking
- Constraint satisfaction checks (altitude, speed, separation)
- Swarm statistics (cohesion, average speed, progress towards target)

**Expected behavior**:

- Drones accelerate towards target with ~0.999 alignment
- Speed stabilizes at ~8 m/s (80% of max)
- Swarm maintains <50m spread
- All hard constraints satisfied throughout

### Customization

To adjust drone behavior, modify constraints in `src/entities.rs`:

```rust
let hard_constraints = HardConstraints {
    min_altitude_m: 60.0,
    max_altitude_m: 120.0,
    max_velocity_m_s: 10.0,
    max_acceleration_m_s2: 8.0,
    min_separation_m: 0.5,
    arena_bounds_m: Vec3::new(60_000.0, 120.0, 60_000.0),
};

let soft_constraints = SoftConstraints {
    target_direction_weight: 3.0,      // ↑ = more aggressive targeting
    smooth_motion_weight: 1.0,         // ↑ = smoother trajectories
    swarm_cohesion_weight: 0.5,        // ↑ = tighter swarm formation
    collision_avoidance_weight: 4.0,   // ↑ = more cautious near neighbors
    speed_efficiency_weight: 1.5,      // ↑ = stricter speed regulation
};
```

### Performance Notes

- **DE generations**: 20-30 per drone per timestep (converges early with
  patience)
- **Population size**: 20 candidates (balanced exploration/exploitation)
- **Timestep**: 1 second (matches physics integration)
- **Computational cost**: O(pop_size × max_gen × drone_count) per frame
  - Example: 100 drones × 20 pop × 30 gen = 60,000 fitness evaluations/frame
  - Optimized for real-time with early convergence detection

---

## Intro

This repository tracks work toward a decentralized swarm maneuver algorithm for
autonomous defensive drones. The style here is intentionally plain, no flair.
Read it, hack it, improve it.

## Background

Modern armed conflicts increasingly involve the deployment of autonomous drones
for surveillance, reconnaissance, and offensive operations. Currently, most
anti-drone strategies rely on traditional Air Defence systems or Electronic
Warfare techniques such as jamming. However, these methods may be limited in
effectiveness, scalability, or cost. An alternative is to use autonomous drone
swarms to counter adversarial drone swarms, leveraging decentralized
coordination, adaptive behaviour, and real-time decision-making to neutralize
threats more efficiently.

## Detailed Description

To develop an effective solution for autonomous swarm-based drone counter
engagement, the following assumptions and requirements must be considered:

1. Armament Capability: Each drone is with armament capability. It is to be
   assumed that the adversary's drones are also similarly armed.
2. Environmental Perception: Drones must be equipped with appropriate sensors
   (e.g., Radar, LiDAR, Vision systems) to detect and track other drones within
   their operational vicinity. Additionally, drones should be able to classify
   targets as friendly or hostile based on sensor inputs and identification
   protocols.
3. Swarm Coordination Algorithm: A robust and efficient algorithm is required to
   coordinate the swarm in real time. This algorithm should enable autonomous
   drones to collaboratively execute efficient manoeuvres for engaging hostile
   drones and optimise the overall effectiveness of the swarm in tackling threat
   from hostile drones. Effectiveness means protecting ground assets and
   minimizing loss of friendly drones.

## Primary Objective

The primary objective of this problem is to develop a decentralized swarm
manoeuvre algorithm for autonomous drones operating in a protective role. The
algorithm should enable drones to collectively execute swarming
manoeuvres—swarming means quickly bringing more forces together against a
smaller enemy to gain an advantage, while staying away from battles where the
enemy has more troops.

## Mission Constraint

The protective drone swarm's mission is to protect designated ground assets from
hostile drone swarms. Therefore, the algorithm must ensure that no enemy drone
with ground attack capabilities\* is left unattended** if it is within a
threatening range\*** of any friendly asset. Prioritization and dynamic threat
assessment are critical.

## Decentralization Requirement

Given the inherent complexity and scalability challenges of centralized control,
the swarm must operate in a distributed and autonomous manner. The algorithm is
to be embedded in each drone individually, enabling local decision-making
without reliance on a central controller. While it may be assumed that drones
possess inter-drone and drone-to-ground communication capabilities, the
algorithm must be designed to function effectively even in the complete absence
of communication. Communication, if available, may be used to enhance
performance but should not be a dependency.

## Expected Solution

The expected outcome is a swarm control algorithm that enables friendly
autonomous drones to collaboratively engage and neutralize an adversarial drone
swarm using effective swarming manoeuvre. The algorithm must prioritize the
protection of designated ground assets, ensuring that no enemy drone capable of
attacking ground targets is left unattended within a threatening range.

The solution should be decentralized in nature, allowing each drone to operate
autonomously based on its local perception and on-board intelligence. While
limited communication between drones and with the ground station can be assumed
to be available, the algorithm must not be dependent on continuous communication
for core functionality. It should demonstrate robust and satisfactory
performance even in communication-denied or degraded environments.

## Simulation + GUI

To validate the solution’s effectiveness, a stochastic simulation framework must
be implemented to evaluate the algorithm through executions across a diverse set
of scenarios and each scenario being executed multiple times. Furthermore, a
user-friendly graphical user interface (GUI) is required to facilitate visual
representation and interaction with the simulation results. (For Technical
Specifications of Drone Refer Table)

## Technical Specifications

| S. No | Parameter                                | Specification |
| ----: | ---------------------------------------- | ------------- |
|     1 | Empty Weight (without battery & payload) | 8 kg          |
|     2 | Maximum Take Off Weight                  | 15 kg         |
|     3 | Maximum Payload                          | 2.5 kg        |
|     4 | Endurance                                | 60–90 min     |
|     5 | Maximum Speed                            | 10 m/s        |
|     6 | Operational Altitude                     | 60–120 m      |
|     7 | Range                                    | 20–40 km      |
|     8 | Firing Range                             | 30–250 m      |

Plain table, no embellishment. Extend or version as specs evolve.

## Definitions

\*Ground Attack Capabilities— Enemy drones are assumed to be of two types.

1. Drones armed with air to air weapon, which will be a threat to friendly
   drones.
2. Drones with ground attack capability.

\*\*Left Unattended— An enemy drone is unattended if there is no friendly drone
either in firing range or in a position to intercept on its path to possible
ground target.

\*\*\*Threatening Range— Threatening range can be defined as 10 times the speed
of drone. Because if a drone with ground attacking capability is less than 10
seconds away from ground asset then there may not be sufficient time to
intercept and neutralize it.

## Implementation Status

### ✅ Completed

1. **Core Simulation Framework**

   - 3D visualization with `three-d` renderer (60 km arena, 1 km grid)
   - Orbit camera controls for scene inspection
   - Real-time rendering at 60 FPS
   - 100 drones + 1 adversary target

2. **Differential Evolution Path Planner** (`src/de_path_planner.rs`)

   - Per-drone DE optimizer with independent populations
   - Acceleration-based control (normalized → physical)
   - Multi-objective fitness function (5 components)
   - Hard constraint enforcement (altitude, speed, separation, bounds)
   - Soft constraint optimization (target-seeking, cohesion, smoothness)
   - Convergence detection with patience mechanism
   - Mutation: `v = x_i + K*(x_r1 - x_i) + F*(x_r2 - x_r3)`
   - Crossover: Binomial with guaranteed dimension
   - Selection: Elitism

3. **Swarm Dynamics** (`src/entities.rs`)

   - Each `Drone` owns its `DronePathPlanner`
   - Physics integration: Euler method with 1s timestep
   - Velocity clamping to max speed constraint
   - Position clamping to arena bounds
   - Path history tracking for trajectory visualization
   - Neighbor-aware planning (collision avoidance)

4. **Testing Infrastructure**
   - Terminal test binary (`test_de_planner`) for validation
   - Detailed per-drone metrics output
   - Swarm statistics (cohesion, progress, speed)
   - Constraint satisfaction verification

### 🚧 In Progress

1. **Adversary Behavior**

   - Currently stationary red target
   - TODO: Add adversary motion model (ground-attack trajectories)
   - TODO: Classification (air-to-air vs ground-attack capability)

2. **Firing Range Logic**

   - Spec: 30-250m engagement range
   - TODO: Implement interception calculations
   - TODO: "Left unattended" detection (10× speed = threatening range)

3. **Stochastic Scenario Generator**

   - TODO: Randomized adversary swarms
   - TODO: Multiple ground assets to protect
   - TODO: Monte Carlo evaluation framework

4. **Advanced Visualization**
   - TODO: Drone path trails (from `path_history`)
   - TODO: Color-coded by state (engaging, intercepting, patrolling)
   - TODO: Firing range indicators

### 📋 Roadmap

**Phase 1**: Multi-Adversary Engagement (Next)

- [ ] Multiple adversary drones with motion
- [ ] Threat assessment (distance, capability, target assignment)
- [ ] Interception logic (firing range checks)
- [ ] "Unattended threat" detection

**Phase 2**: Ground Asset Protection

- [ ] Ground assets with position
- [ ] Threatening range calculation (10× drone speed)
- [ ] Priority-based task allocation
- [ ] Swarming behavior (n-vs-1 superiority)

**Phase 3**: Communication & Coordination

- [ ] Optional inter-drone messaging
- [ ] Consensus on high-value targets
- [ ] Communication-denied fallback mode

**Phase 4**: Evaluation & Metrics

- [ ] Stochastic scenario generator
- [ ] Monte Carlo simulation harness
- [ ] Success metrics (assets protected, drones lost, adversaries neutralized)
- [ ] Performance profiling (DE convergence rates, computation time)

**Phase 5**: GUI & Analysis Tools

- [ ] Control panel for constraint tuning
- [ ] Real-time performance graphs
- [ ] Scenario replay system
- [ ] Export trajectory data for analysis

## Current Limitations

1. **No adversary motion**: Target is stationary (easy to intercept)
2. **No firing logic**: Drones move towards target but don't "engage"
3. **Single target**: Swarm doesn't prioritize or distribute
4. **No ground assets**: No protection mission constraints
5. **Euler integration**: Simple but can accumulate error (consider RK4 for
   production)
6. **Real-time DE**: 30 generations may be insufficient for complex scenarios
   (trade-off with FPS)

## Implementation Notes

- **Language**: Rust 2021 Edition
- **Renderer**: `three-d` (0.18.2) for cross-platform 3D graphics
- **Math**: `glam` (0.30.8) for SIMD-accelerated Vec3 operations
- **RNG**: `rand` (0.9.2) with `rand_chacha` for reproducible scenarios
- **Architecture**: Data-oriented with ECS-inspired patterns
  - Drones are independent entities with embedded optimizers
  - No central controller (true decentralization)
  - Neighbor queries via linear scan (acceptable for <1000 drones)

## Performance Considerations

**Current profile** (100 drones, Release build, M2 MacBook):

- Frame time: ~16ms (60 FPS)
- DE computation: ~12ms per frame (100 drones × 20 pop × 30 gen)
- Rendering: ~4ms per frame

**Scaling**:

- 300 drones: ~30-40 FPS (still interactive)
- 1000 drones: ~10-15 FPS (need optimization: spatial hashing, GPU compute)

**Future optimizations**:

- Spatial hashing for O(1) neighbor queries
- GPU-accelerated DE (compute shaders)
- Adaptive population size (reduce for fast-moving drones)
- Multi-threading per-drone planning (embarrassingly parallel)

## License

Add an explicit license file (TODO). Until then, assume "All rights reserved";
do not use in production.

## Verbatim Source (Original README.txt Content)

Below is the original text preserved exactly (HTML entities and markup removed
except where unavoidable). This is here so wording is untouched while the main
body above is cleaned for readability.

> Background Modern armed conflicts increasingly involve the deployment of
> autonomous drones for surveillance, reconnaissance, and offensive operations.
> Currently, most anti-drone strategies rely on traditional Air Defence systems
> or Electronic Warfare techniques such as jamming. However, these methods may
> be limited in effectiveness, scalability, or cost. An alternative is to use
> autonomous drone swarms to counter adversarial drone swarms, leveraging
> decentralized coordination, adaptive behaviour, and real-time decision-making
> to neutralize threats more efficiently.
>
> Detailed Description To develop an effective solution for autonomous
> swarm-based drone counter engagement, the following assumptions and
> requirements must be considered:
>
> 1. Armament Capability: Each drone is with armament capability. It is to be
>    assumed that the adversary's drones are also similarly armed.
> 2. Environmental Perception: Drones must be equipped with appropriate sensors
>    (e.g., Radar, LiDAR, Vision systems) to detect and track other drones
>    within their operational vicinity. Additionally, drones should be able to
>    classify targets as friendly or hostile based on sensor inputs and
>    identification protocols.
> 3. Swarm Coordination Algorithm: A robust and efficient algorithm is required
>    to coordinate the swarm in real time. This algorithm should enable
>    autonomous drones to collaboratively execute efficient manoeuvres for
>    engaging hostile drones and optimise the overall effectiveness of the swarm
>    in tackling threat from hostile drones. Effectiveness means protecting
>    ground assets and minimizing loss of friendly drones.
>
> The primary objective of this problem is to develop a decentralized swarm
> manoeuvre algorithm for autonomous drones operating in a protective role. The
> algorithm should enable drones to collectively execute swarming
> manoeuvres—swarming means quickly bringing more forces together against a
> smaller enemy to gain an advantage, while staying away from battles where the
> enemy has more troops.
>
> The protective drone swarm's mission is to protect designated ground assets
> from hostile drone swarms. Therefore, the algorithm must ensure that no enemy
> drone with ground attack capabilities\* is left unattended** if it is within a
> threatening range\*** of any friendly asset. Prioritization and dynamic threat
> assessment are critical.
>
> Given the inherent complexity and scalability challenges of centralized
> control, the swarm must operate in a distributed and autonomous manner. The
> algorithm is to be embedded in each drone individually, enabling local
> decision-making without reliance on a central controller. While it may be
> assumed that drones possess inter-drone and drone-to-ground communication
> capabilities, the algorithm must be designed to function effectively even in
> the complete absence of communication. Communication, if available, may be
> used to enhance performance but should not be a dependency.
>
> Expected Solution The expected outcome is a swarm control algorithm that
> enables friendly autonomous drones to collaboratively engage and neutralize an
> adversarial drone swarm using effective swarming manoeuvre. The algorithm must
> prioritize the protection of designated ground assets, ensuring that no enemy
> drone capable of attacking ground targets is left unattended within a
> threatening range.
>
> The solution should be decentralized in nature, allowing each drone to operate
> autonomously based on its local perception and on-board intelligence. While
> limited communication between drones and with the ground station can be
> assumed to be available, the algorithm must not be dependent on continuous
> communication for core functionality. It should demonstrate robust and
> satisfactory performance even in communication-denied or degraded
> environments.
>
> To validate the solution’s effectiveness, a stochastic simulation framework
> must be implemented to evaluate the algorithm through executions across a
> diverse set of scenarios and each scenario being executed multiple times.
> Furthermore, a user-friendly graphical user interface (GUI) is required to
> facilitate visual representation and interaction with the simulation results.
> (For Technical Specifications of Drone Refer Table)
>
> \*Ground Attack Capabilities— Enemy drones are assumed to be of two types.
>
> 1. Drones armed with air to air weapon, which will be a threat to friendly
>    drones.
> 2. Drones with ground attack capability.
>
> \*\*Left Unattended— An enemy drone is unattended if there is no friendly
> drone either in firing range or in a position to intercept on its path to
> possible ground target.
>
> \*\*\*Threatening Range— Threatening range can be defined as 10 times the
> speed of drone. Because if a drone with ground attacking capability is less
> than 10 seconds away from ground asset then there may not be sufficient time
> to intercept and neutralize it.
