use glam::Vec3;
use crate::de_path_planner::{DronePathPlanner, HardConstraints, SoftConstraints, DEParams};

#[derive(Debug, Clone)]
pub struct Drone {
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    pub path_history: Vec<Vec3>,
    pub planner: Option<DronePathPlanner>,
}

#[allow(dead_code)]
impl Drone {
    pub fn update_velocity(&mut self, new_velocity: Vec3) {
        self.velocity = new_velocity;
    }
    
    pub fn update_acceleration(&mut self, new_acceleration: Vec3) {
        self.acceleration = new_acceleration;
    }
    
    pub fn update_position(&mut self, new_position: Vec3) {
        self.position = new_position;
        self.path_history.push(new_position);
    }
}

impl Drone {
    pub fn new(position: Vec3) -> Self {
        Self {
            position,
            velocity: Vec3::ZERO,
            acceleration: Vec3::ZERO,
            path_history: vec![position],
            planner: None,
        }
    }
    
    pub fn with_planner(
        position: Vec3, 
        target_position: Vec3,
        hard_constraints: HardConstraints,
        soft_constraints: SoftConstraints,
        de_params: DEParams,
    ) -> Self {
        let planner = DronePathPlanner::new(
            position,
            Vec3::ZERO,
            Vec3::ZERO,
            target_position,
            hard_constraints,
            soft_constraints,
            de_params,
        );
        
        Self {
            position,
            velocity: Vec3::ZERO,
            acceleration: Vec3::ZERO,
            path_history: vec![position],
            planner: Some(planner),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Target {
    pub position: Vec3,
}

impl Target {
    pub fn new(position: Vec3) -> Self {
        Self { position }
    }
}

pub struct SwarmState {
    pub drones: Vec<Drone>,
    pub target: Target,
    pub hard_constraints: HardConstraints,
    pub soft_constraints: SoftConstraints,
    pub de_params: DEParams,
    pub dt: f32, // Time step for physics integration
}

impl SwarmState {
    pub fn new(drone_count: usize, locality_radius_m: f32, _target_distance_km: f32) -> Self {
        use rand::Rng;
        let mut rng = rand::rng();
        
        // Randomly pick a locality center within the arena (avoid edges)
        let arena_size_m = 60_000.0; // 60 km
        let margin_m = 5_000.0; // Stay 5 km away from edges
        
        let locality_center_x = rng.random_range(margin_m..(arena_size_m - margin_m));
        let locality_center_z = rng.random_range(margin_m..(arena_size_m - margin_m));
        let locality_altitude = rng.random_range(60.0..=120.0);
        
        // Target at random distance between 20-40 km from origin
        let target_distance_km: f32 = rng.random_range(20.0..=40.0);
        let target_distance_m: f32 = target_distance_km * 1000.0;
        let target_altitude: f32 = rng.random_range(60.0..=120.0);
        
        let horizontal_distance = (target_distance_m.powi(2) - target_altitude.powi(2))
            .max(0.0)
            .sqrt();
        
        // Random angle, but biased towards positive quadrant (into the arena)
        let angle: f32 = rng.random_range(0.0..std::f32::consts::PI / 2.0); // 0-90 degrees
        let direction = Vec3::new(angle.cos(), 0.0, angle.sin()).normalize();
        
        let target_position = Vec3::new(
            direction.x * horizontal_distance,
            target_altitude,
            direction.z * horizontal_distance,
        );
        
        // Setup DE parameters for all drones
        let hard_constraints = HardConstraints::default();
        let soft_constraints = SoftConstraints::default();
        let de_params = DEParams {
            pop_size: 20,
            k: 0.5,
            f: 0.8,
            crossover_prob: 0.8,
            max_gen: 30, // Balanced for real-time performance
            convergence_tol: 1e-6,
            convergence_patience: 15,
        };
        
        // Each drone gets a random position within the locality
        // Ensure minimum 0.5m spacing between drones
        let min_distance = 0.5;
        let mut drones: Vec<Drone> = Vec::with_capacity(drone_count);
        
        for _ in 0..drone_count {
            let mut attempts = 0;
            let max_attempts = 100;
            
            loop {
                // Random offset from locality center
                let offset_x: f32 = rng.random_range(-locality_radius_m..=locality_radius_m);
                let offset_z: f32 = rng.random_range(-locality_radius_m..=locality_radius_m);
                let offset_y: f32 = rng.random_range(-20.0..=20.0); // Small vertical variance
                
                let position = Vec3::new(
                    locality_center_x + offset_x,
                    (locality_altitude + offset_y).clamp(60.0, 120.0),
                    locality_center_z + offset_z,
                );
                
                // Check if position is far enough from all existing drones
                let mut valid = true;
                for existing_drone in &drones {
                    let distance = (position - existing_drone.position).length();
                    if distance < min_distance {
                        valid = false;
                        break;
                    }
                }
                
                if valid || attempts >= max_attempts {
                    drones.push(Drone::with_planner(
                        position,
                        target_position,
                        hard_constraints.clone(),
                        soft_constraints.clone(),
                        de_params.clone(),
                    ));
                    break;
                }
                
                attempts += 1;
            }
        }

        Self {
            drones,
            target: Target::new(target_position),
            hard_constraints,
            soft_constraints,
            de_params,
            dt: 1.0, // 1 second time step
        }
    }
    
    /// Update all drones using DE path planning
    pub fn update_drones(&mut self) {
        let target_pos = self.target.position;
        
        // For each drone, compute next acceleration and update physics
        for i in 0..self.drones.len() {
            // Get neighbor positions for this drone
            let neighbor_positions: Vec<Vec3> = self.drones
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, d)| d.position)
                .collect();
            
            let drone = &mut self.drones[i];
            
            if let Some(planner) = &mut drone.planner {
                // Compute optimal acceleration using DE
                let _normalized_accel = planner.compute_next_acceleration(&neighbor_positions);
                let physical_accel = planner.get_physical_acceleration();
                
                // Update acceleration
                drone.acceleration = physical_accel;
                
                // Integrate velocity: v = v0 + a*dt
                drone.velocity = drone.velocity + drone.acceleration * self.dt;
                
                // Clamp velocity to max speed constraint
                let speed = drone.velocity.length();
                if speed > self.hard_constraints.max_velocity_m_s {
                    drone.velocity = drone.velocity.normalize() * self.hard_constraints.max_velocity_m_s;
                }
                
                // Integrate position: p = p0 + v*dt
                let new_position = drone.position + drone.velocity * self.dt;
                
                // Enforce hard constraints on position
                let clamped_position = Vec3::new(
                    new_position.x.clamp(0.0, self.hard_constraints.arena_bounds_m.x),
                    new_position.y.clamp(
                        self.hard_constraints.min_altitude_m,
                        self.hard_constraints.max_altitude_m,
                    ),
                    new_position.z.clamp(0.0, self.hard_constraints.arena_bounds_m.z),
                );
                
                // Manually update position and path history
                drone.position = clamped_position;
                drone.path_history.push(clamped_position);
                
                // Update planner state for next iteration
                planner.update_state(
                    clamped_position,
                    drone.velocity,
                    drone.acceleration,
                    target_pos,
                );
            }
        }
    }
}

#[allow(dead_code)]
fn fibonacci_disc_points(count: usize, radius_m: f32) -> Vec<Vec3> {
    if count == 0 {
        return Vec::new();
    }

    let mut points = Vec::with_capacity(count);
    let golden_angle = std::f32::consts::PI * (3.0 - 5.0_f32.sqrt());

    for i in 0..count {
        let i_f = i as f32;
        let r = radius_m * (i_f / count as f32).sqrt();
        let theta = golden_angle * i_f;
        let x = r * theta.cos();
        let z = r * theta.sin();
        points.push(Vec3::new(x, 0.0, z));
    }

    points
}
