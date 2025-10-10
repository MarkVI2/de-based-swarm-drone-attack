/// Differential Evolution Path Planner for Drone Swarm
/// 
/// This module implements a DE-based path planning algorithm where each drone
/// is a candidate solution that optimizes its normalized acceleration vector (ax, ay, az).
/// 
/// Key Design Principles:
/// - Each drone maintains its own population of candidate accelerations
/// - Control is via normalized acceleration [-1.0, 1.0] in each axis
/// - The drone is always aware of the target direction
/// - Hard constraints are enforced (altitude, speed, separation)
/// - Soft constraints are optimized via fitness function
/// - No central selection needed - each drone independently optimizes

use glam::Vec3;
use rand::Rng;

/// A candidate solution representing a normalized acceleration vector
#[derive(Debug, Clone)]
pub struct AccelerationCandidate {
    /// Normalized acceleration in x, y, z directions [-1.0, 1.0]
    pub acceleration: Vec3,
    /// Fitness score for this candidate
    pub fitness: f32,
}

impl AccelerationCandidate {
    pub fn new(acceleration: Vec3) -> Self {
        Self {
            acceleration,
            fitness: 0.0,
        }
    }
}

/// Hard constraints that MUST be satisfied
#[derive(Debug, Clone)]
pub struct HardConstraints {
    /// Minimum altitude in meters (from problem spec: 60m)
    pub min_altitude_m: f32,
    /// Maximum altitude in meters (from problem spec: 120m)
    pub max_altitude_m: f32,
    /// Maximum velocity in m/s (from problem spec: 10 m/s)
    pub max_velocity_m_s: f32,
    /// Maximum acceleration in m/s² (typical quadcopter: ~5-10 m/s²)
    pub max_acceleration_m_s2: f32,
    /// Minimum separation distance from other drones (meters)
    pub min_separation_m: f32,
    /// Arena bounds (x_max, y_max, z_max) in meters
    pub arena_bounds_m: Vec3,
}

impl Default for HardConstraints {
    fn default() -> Self {
        Self {
            min_altitude_m: 60.0,
            max_altitude_m: 120.0,
            max_velocity_m_s: 10.0,
            max_acceleration_m_s2: 8.0, // Conservative for safety
            min_separation_m: 0.5, // Half-meter minimum
            arena_bounds_m: Vec3::new(60_000.0, 120.0, 60_000.0),
        }
    }
}

/// Soft constraints that are optimized (weights for fitness function)
#[derive(Debug, Clone)]
pub struct SoftConstraints {
    /// Weight for moving towards target
    pub target_direction_weight: f32,
    /// Weight for maintaining smooth trajectory (avoiding jerky movements)
    pub smooth_motion_weight: f32,
    /// Weight for staying cohesive with swarm
    pub swarm_cohesion_weight: f32,
    /// Weight for avoiding collisions with other drones
    pub collision_avoidance_weight: f32,
    /// Weight for maintaining efficient speed towards target
    pub speed_efficiency_weight: f32,
}

impl Default for SoftConstraints {
    fn default() -> Self {
        Self {
            target_direction_weight: 3.0,
            smooth_motion_weight: 1.0,
            swarm_cohesion_weight: 0.5,
            collision_avoidance_weight: 4.0,
            speed_efficiency_weight: 1.5,
        }
    }
}

/// Differential Evolution parameters (matching your Python implementation)
#[derive(Debug, Clone)]
pub struct DEParams {
    /// Population size (number of candidate solutions per drone)
    pub pop_size: usize,
    /// K: scaling factor for difference from current (x_i)
    pub k: f32,
    /// F: scaling factor for differential mutation (x_r2 - x_r3)
    pub f: f32,
    /// Crossover probability (CR in DE literature)
    pub crossover_prob: f32,
    /// Maximum number of generations per planning step
    pub max_gen: usize,
    /// Convergence tolerance
    pub convergence_tol: f32,
    /// Patience for convergence checking
    pub convergence_patience: usize,
}

impl Default for DEParams {
    fn default() -> Self {
        Self {
            pop_size: 20,
            k: 0.5,
            f: 0.8,
            crossover_prob: 0.8,
            max_gen: 50,
            convergence_tol: 1e-6,
            convergence_patience: 20,
        }
    }
}

/// Per-drone Differential Evolution path planner
/// Each drone maintains its own population and optimizes independently
#[derive(Debug, Clone)]
pub struct DronePathPlanner {
    /// Current 3D position of the drone (meters)
    pub current_position: Vec3,
    /// Current velocity vector (m/s)
    pub current_velocity: Vec3,
    /// Current acceleration (m/s²) - for smooth motion constraint
    pub current_acceleration: Vec3,
    /// Unit vector pointing towards the target
    pub target_direction: Vec3,
    /// Distance to target (meters)
    pub distance_to_target: f32,
    
    /// Population of candidate acceleration vectors
    pub population: Vec<AccelerationCandidate>,
    /// Best acceleration found so far
    pub best_acceleration: Vec3,
    /// Best fitness achieved
    pub best_fitness: f32,
    
    /// Hard constraints
    pub hard_constraints: HardConstraints,
    /// Soft constraints (weights)
    pub soft_constraints: SoftConstraints,
    /// DE algorithm parameters
    pub de_params: DEParams,
    
    /// Fitness history for convergence checking
    best_fitness_history: Vec<f32>,
    /// Time step for integration (seconds)
    dt: f32,
}

impl DronePathPlanner {
    /// Create a new path planner for a drone
    pub fn new(
        current_position: Vec3,
        current_velocity: Vec3,
        current_acceleration: Vec3,
        target_position: Vec3,
        hard_constraints: HardConstraints,
        soft_constraints: SoftConstraints,
        de_params: DEParams,
    ) -> Self {
        let target_vector = target_position - current_position;
        let distance_to_target = target_vector.length();
        let target_direction = if distance_to_target > 0.01 {
            target_vector.normalize()
        } else {
            Vec3::X // Default direction if at target
        };
        
        let mut planner = Self {
            current_position,
            current_velocity,
            current_acceleration,
            target_direction,
            distance_to_target,
            population: Vec::with_capacity(de_params.pop_size),
            best_acceleration: Vec3::ZERO,
            best_fitness: f32::NEG_INFINITY,
            hard_constraints,
            soft_constraints,
            de_params,
            best_fitness_history: Vec::new(),
            dt: 1.0, // 1 second time step
        };
        
        planner.initialize_population();
        planner
    }

    /// Initialize population with random normalized accelerations
    fn initialize_population(&mut self) {
        let mut rng = rand::rng();
        self.population.clear();
        self.best_fitness_history.clear();
        
        for _ in 0..self.de_params.pop_size {
            // Generate random normalized acceleration [-1, 1] for each axis
            let ax = rng.random_range(-1.0..=1.0);
            let ay = rng.random_range(-1.0..=1.0);
            let az = rng.random_range(-1.0..=1.0);
            
            let acceleration = Vec3::new(ax, ay, az);
            self.population.push(AccelerationCandidate::new(acceleration));
        }
        
        // Initialize best
        self.best_acceleration = Vec3::ZERO;
        self.best_fitness = f32::NEG_INFINITY;
    }

    /// Check if the algorithm has converged
    fn check_convergence(&self) -> bool {
        let patience = self.de_params.convergence_patience;
        if self.best_fitness_history.len() < patience + 1 {
            return false;
        }
        
        let recent = &self.best_fitness_history[self.best_fitness_history.len() - patience - 1..];
        let max_val = recent.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        let min_val = recent.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        
        (max_val - min_val) < self.de_params.convergence_tol
    }

    /// Denormalize acceleration: convert [-1, 1] to actual m/s²
    fn denormalize_acceleration(&self, normalized: Vec3) -> Vec3 {
        normalized * self.hard_constraints.max_acceleration_m_s2
    }

    /// Predict next state given a normalized acceleration
    fn predict_next_state(&self, normalized_accel: Vec3) -> (Vec3, Vec3) {
        let accel = self.denormalize_acceleration(normalized_accel);
        let next_velocity = self.current_velocity + accel * self.dt;
        let next_position = self.current_position + next_velocity * self.dt;
        (next_position, next_velocity)
    }

    /// Check if a predicted state violates hard constraints
    fn violates_hard_constraints(
        &self,
        position: Vec3,
        velocity: Vec3,
        neighbor_positions: &[Vec3],
    ) -> bool {
        // Altitude constraints
        if position.y < self.hard_constraints.min_altitude_m 
            || position.y > self.hard_constraints.max_altitude_m {
            return true;
        }
        
        // Arena bounds
        if position.x < 0.0 || position.x > self.hard_constraints.arena_bounds_m.x
            || position.z < 0.0 || position.z > self.hard_constraints.arena_bounds_m.z {
            return true;
        }
        
        // Velocity constraint
        let speed = velocity.length();
        if speed > self.hard_constraints.max_velocity_m_s {
            return true;
        }
        
        // Separation constraint (check predicted position against current neighbor positions)
        for neighbor_pos in neighbor_positions {
            let distance = (position - *neighbor_pos).length();
            if distance < self.hard_constraints.min_separation_m {
                return true;
            }
        }
        
        false
    }

    /// Calculate fitness for a candidate acceleration
    /// Returns large negative value if hard constraints violated
    fn calculate_fitness(
        &self,
        normalized_accel: Vec3,
        neighbor_positions: &[Vec3],
    ) -> f32 {
        let (next_pos, next_vel) = self.predict_next_state(normalized_accel);
        
        // Hard constraint violation = very poor fitness
        if self.violates_hard_constraints(next_pos, next_vel, neighbor_positions) {
            return -1e9;
        }
        
        let mut fitness = 0.0;
        let accel = self.denormalize_acceleration(normalized_accel);
        
        // 1. Target direction: reward acceleration towards target
        if next_vel.length() > 0.1 {
            let velocity_direction = next_vel.normalize();
            let target_alignment = velocity_direction.dot(self.target_direction);
            fitness += target_alignment * self.soft_constraints.target_direction_weight;
        }
        
        // 2. Smooth motion: penalize jerky changes in acceleration
        let accel_change = (accel - self.current_acceleration).length();
        let smoothness_penalty = accel_change / self.hard_constraints.max_acceleration_m_s2;
        fitness -= smoothness_penalty * self.soft_constraints.smooth_motion_weight;
        
        // 3. Speed efficiency: reward moving at optimal speed towards target
        let speed = next_vel.length();
        let optimal_speed = self.hard_constraints.max_velocity_m_s * 0.8; // 80% of max
        let speed_error = (speed - optimal_speed).abs() / self.hard_constraints.max_velocity_m_s;
        fitness -= speed_error * self.soft_constraints.speed_efficiency_weight;
        
        // 4. Collision avoidance: penalize being close to other drones
        for neighbor_pos in neighbor_positions {
            let distance = (next_pos - *neighbor_pos).length();
            let safety_distance = self.hard_constraints.min_separation_m * 3.0;
            
            if distance < safety_distance {
                let proximity = 1.0 - (distance / safety_distance);
                fitness -= proximity.powi(2) * self.soft_constraints.collision_avoidance_weight;
            }
        }
        
        // 5. Swarm cohesion: reward staying near swarm center
        if !neighbor_positions.is_empty() {
            let swarm_center: Vec3 = neighbor_positions.iter().sum::<Vec3>() 
                / neighbor_positions.len() as f32;
            let distance_to_center = (next_pos - swarm_center).length();
            let cohesion_bonus = (-distance_to_center / 1000.0).exp(); // Decay over 1km
            fitness += cohesion_bonus * self.soft_constraints.swarm_cohesion_weight;
        }
        
        fitness
    }

    /// DE Mutation: v = x_i + K*(x_r1 - x_i) + F*(x_r2 - x_r3)
    /// This matches your Python implementation exactly
    fn mutate(&self, i: usize, r1: usize, r2: usize, r3: usize) -> Vec3 {
        let x_i = self.population[i].acceleration;
        let x_r1 = self.population[r1].acceleration;
        let x_r2 = self.population[r2].acceleration;
        let x_r3 = self.population[r3].acceleration;
        
        x_i + self.de_params.k * (x_r1 - x_i) + self.de_params.f * (x_r2 - x_r3)
    }

    /// DE Crossover: binomial crossover with guaranteed dimension
    fn crossover(&self, target: Vec3, mutant: Vec3) -> Vec3 {
        let mut rng = rand::rng();
        let mut trial = target;
        let j_rand = rng.random_range(0..3); // Ensure at least one dimension from mutant
        
        for j in 0..3 {
            let rand_chance = rng.random::<f32>();
            if rand_chance <= self.de_params.crossover_prob || j == j_rand {
                match j {
                    0 => trial.x = mutant.x,
                    1 => trial.y = mutant.y,
                    2 => trial.z = mutant.z,
                    _ => unreachable!(),
                }
            }
        }
        
        // Clamp to normalized range [-1, 1]
        trial.x = trial.x.clamp(-1.0, 1.0);
        trial.y = trial.y.clamp(-1.0, 1.0);
        trial.z = trial.z.clamp(-1.0, 1.0);
        
        trial
    }

    /// Run one generation of DE algorithm
    fn evolve_generation(&mut self, neighbor_positions: &[Vec3]) {
        let mut rng = rand::rng();
        let pop_size = self.population.len();
        let mut new_population = Vec::with_capacity(pop_size);
        
        for i in 0..pop_size {
            // Select three distinct random individuals (r1, r2, r3) different from i
            let mut candidates: Vec<usize> = (0..pop_size).filter(|&x| x != i).collect();
            candidates.sort_by_key(|_| rng.random::<u32>());
            let (r1, r2, r3) = (candidates[0], candidates[1], candidates[2]);
            
            // Mutation
            let mutant = self.mutate(i, r1, r2, r3);
            
            // Crossover
            let trial = self.crossover(self.population[i].acceleration, mutant);
            
            // Evaluate fitness
            let trial_fitness = self.calculate_fitness(trial, neighbor_positions);
            let current_fitness = self.population[i].fitness;
            
            // Selection: keep better solution (elitism)
            if trial_fitness > current_fitness {
                let mut candidate = AccelerationCandidate::new(trial);
                candidate.fitness = trial_fitness;
                new_population.push(candidate);
                
                // Update global best
                if trial_fitness > self.best_fitness {
                    self.best_fitness = trial_fitness;
                    self.best_acceleration = trial;
                }
            } else {
                new_population.push(self.population[i].clone());
            }
        }
        
        self.population = new_population;
    }

    /// Run complete DE optimization and return best normalized acceleration
    /// This is the main entry point called each timestep
    pub fn compute_next_acceleration(&mut self, neighbor_positions: &[Vec3]) -> Vec3 {
        // Evaluate initial population
        for i in 0..self.population.len() {
            let fitness = self.calculate_fitness(
                self.population[i].acceleration,
                neighbor_positions,
            );
            self.population[i].fitness = fitness;
            
            if fitness > self.best_fitness {
                self.best_fitness = fitness;
                self.best_acceleration = self.population[i].acceleration;
            }
        }
        
        let mut converged_at = self.de_params.max_gen;
        
        // Run DE for max_gen generations or until convergence
        for generation in 0..self.de_params.max_gen {
            self.evolve_generation(neighbor_positions);
            
            // Track best fitness
            self.best_fitness_history.push(self.best_fitness);
            
            // Check convergence
            if self.check_convergence() {
                converged_at = generation + 1;
                break;
            }
        }
        
        // Debug: track convergence
        if converged_at < self.de_params.max_gen {
            // println!("DE converged at generation {}", converged_at);
        }
        
        self.best_acceleration
    }

    /// Update planner state after drone moves
    /// This should be called after applying the acceleration and updating position/velocity
    pub fn update_state(
        &mut self,
        new_position: Vec3,
        new_velocity: Vec3,
        new_acceleration: Vec3,
        target_position: Vec3,
    ) {
        self.current_position = new_position;
        self.current_velocity = new_velocity;
        self.current_acceleration = new_acceleration;
        
        let target_vector = target_position - new_position;
        self.distance_to_target = target_vector.length();
        self.target_direction = if self.distance_to_target > 0.01 {
            target_vector.normalize()
        } else {
            Vec3::X
        };
        
        // Reinitialize population for next planning step
        self.initialize_population();
    }

    /// Get the denormalized acceleration (actual m/s²) for physics integration
    pub fn get_physical_acceleration(&self) -> Vec3 {
        self.denormalize_acceleration(self.best_acceleration)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_acceleration_normalization() {
        let constraints = HardConstraints::default();
        let planner = DronePathPlanner::new(
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::new(1000.0, 100.0, 1000.0),
            constraints.clone(),
            SoftConstraints::default(),
            DEParams::default(),
        );
        
        let normalized = Vec3::new(1.0, 0.5, -0.5);
        let denormalized = planner.denormalize_acceleration(normalized);
        
        assert_eq!(denormalized.x, constraints.max_acceleration_m_s2);
        assert_eq!(denormalized.y, constraints.max_acceleration_m_s2 * 0.5);
        assert_eq!(denormalized.z, -constraints.max_acceleration_m_s2 * 0.5);
    }

    #[test]
    fn test_hard_constraint_violation() {
        let planner = DronePathPlanner::new(
            Vec3::new(1000.0, 80.0, 1000.0),
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::new(2000.0, 100.0, 2000.0),
            HardConstraints::default(),
            SoftConstraints::default(),
            DEParams::default(),
        );
        
        // Test altitude violation (too low)
        let low_pos = Vec3::new(1000.0, 50.0, 1000.0);
        assert!(planner.violates_hard_constraints(low_pos, Vec3::ZERO, &[]));
        
        // Test altitude violation (too high)
        let high_pos = Vec3::new(1000.0, 130.0, 1000.0);
        assert!(planner.violates_hard_constraints(high_pos, Vec3::ZERO, &[]));
        
        // Test valid altitude
        let valid_pos = Vec3::new(1000.0, 90.0, 1000.0);
        assert!(!planner.violates_hard_constraints(valid_pos, Vec3::ZERO, &[]));
    }
}
