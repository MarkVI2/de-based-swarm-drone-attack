use three_d::*;

use crate::entities::SwarmState;
use crate::rendering::{DroneRenderer, GridPlanes, TargetRenderer};

pub struct Scene {
    pub swarm_state: SwarmState,
    grids: GridPlanes,
    drone_renderer: DroneRenderer,
    target_renderer: TargetRenderer,
    ambient_light: AmbientLight,
    directional_light: DirectionalLight,
    updates_per_frame: u32, // Number of simulation updates per render frame (speed multiplier)
}

impl Scene {
    pub fn new(context: &Context) -> Self {
        const DRONE_COUNT: usize = 100; // Increased to 300 drones for more visibility
        const LOCALITY_RADIUS_M: f32 = 200.0; // Each drone spawns within 200m of locality center
        const TARGET_DISTANCE_KM: f32 = 25.0; // Will be randomized internally
        const GRID_SIZE_KM: f32 = 60.0; // 60 km
        const GRID_CELL_SIZE_KM: f32 = 1.0; // 1 km cells
        const DRONE_RADIUS_M: f32 = 50.0; // 50m radius for visibility at km scale
        const TARGET_RADIUS_M: f32 = 100.0; // 100m radius for target

        let swarm_state = SwarmState::new(DRONE_COUNT, LOCALITY_RADIUS_M, TARGET_DISTANCE_KM);

        let grids = GridPlanes::new(context, GRID_SIZE_KM, GRID_CELL_SIZE_KM);

        let drone_positions: Vec<glam::Vec3> =
            swarm_state.drones.iter().map(|d| d.position).collect();
        let drone_renderer = DroneRenderer::new(context, &drone_positions, DRONE_RADIUS_M);

        let target_renderer =
            TargetRenderer::new(context, swarm_state.target.position, TARGET_RADIUS_M);

        let ambient_light = AmbientLight::new(context, 0.4, Srgba::WHITE);
        let directional_light =
            DirectionalLight::new(context, 2.0, Srgba::WHITE, vec3(-0.5, -1.0, -0.5));

        Self {
            swarm_state,
            grids,
            drone_renderer,
            target_renderer,
            ambient_light,
            directional_light,
            updates_per_frame: 1, // One update per frame by default (1x speed)
        }
    }

    pub fn update(&mut self, _context: &Context, events: &[Event]) {
        // Handle keyboard events for time speed control
        for event in events {
            if let Event::KeyPress { kind, modifiers, .. } = event {
                match kind {
                    Key::A => {
                        if modifiers.shift {
                            // Shift+A (capital A) - Slow down (divide by 2)
                            self.updates_per_frame = (self.updates_per_frame / 2).max(1); // Min 1 update (can't go slower than real-time)
                            println!("⏱️  Simulation speed: {}x (SLOWER - {} update{}/frame)", 
                                self.updates_per_frame, 
                                self.updates_per_frame,
                                if self.updates_per_frame == 1 { "" } else { "s" });
                        } else {
                            // 'a' - Speed up (multiply by 2)
                            self.updates_per_frame = (self.updates_per_frame * 2).min(16); // Max 16 updates/frame
                            println!("⏱️  Simulation speed: {}x (FASTER - {} update{}/frame)", 
                                self.updates_per_frame, 
                                self.updates_per_frame,
                                if self.updates_per_frame == 1 { "" } else { "s" });
                        }
                    }
                    Key::R => {
                        // 'r' - Reset to normal speed
                        self.updates_per_frame = 1;
                        println!("⏱️  Simulation speed: RESET to 1x");
                    }
                    _ => {}
                }
            }
        }

        // Run multiple updates per frame if speed is increased
        for _ in 0..self.updates_per_frame {
            // Update drone positions using DE path planning
            self.swarm_state.update_drones();
        }
        
        // Update renderer with new positions (only once per render frame)
        let drone_positions: Vec<glam::Vec3> =
            self.swarm_state.drones.iter().map(|d| d.position).collect();
        self.drone_renderer.update_positions(&drone_positions);
        self.target_renderer
            .update_position(self.swarm_state.target.position);
    }

    pub fn render_objects(&self) -> Vec<&dyn Object> {
        let mut objects: Vec<&dyn Object> = Vec::new();
        
        // Add all grid line models
        for model in &self.grids.models {
            objects.push(model as &dyn Object);
        }
        
        // Add all grid labels
        for label in &self.grids.labels {
            objects.push(label as &dyn Object);
        }
        
        // Add drones and target
        objects.push(&self.drone_renderer.instances as &dyn Object);
        objects.push(&self.target_renderer.model as &dyn Object);
        
        objects
    }

    pub fn lights(&self) -> Vec<&dyn Light> {
        vec![
            &self.ambient_light as &dyn Light,
            &self.directional_light as &dyn Light,
        ]
    }
}
