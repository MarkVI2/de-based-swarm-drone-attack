use glam::Vec3;

#[derive(Debug, Clone)]
pub struct Drone {
    pub position: Vec3,
    pub velocity: Vec3,
}

#[allow(dead_code)]
impl Drone {
    pub fn update_velocity(&mut self, new_velocity: Vec3) {
        self.velocity = new_velocity;
    }
}

impl Drone {
    pub fn new(position: Vec3) -> Self {
        Self {
            position,
            velocity: Vec3::ZERO,
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
                    drones.push(Drone::new(position));
                    break;
                }
                
                attempts += 1;
            }
        }

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

        Self {
            drones,
            target: Target::new(target_position),
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
