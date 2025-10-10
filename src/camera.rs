use three_d::*;

pub struct SceneCamera {
    camera: Camera,
    control: OrbitControl,
}

impl SceneCamera {
    pub fn new(_context: &Context, viewport: Viewport) -> Self {
        // Origin is now at corner (0,0,0), look towards center of the arena
        let scene_size_m = 60_000.0; // 60 km in meters
        let target = vec3(scene_size_m * 0.3, 100.0, scene_size_m * 0.3); // Look at 30% into arena
        let distance = scene_size_m * 0.6;

        let camera = Camera::new_perspective(
            viewport,
            target + vec3(distance * 0.5, distance * 0.3, distance * 0.5),
            target,
            vec3(0.0, 1.0, 0.0),
            degrees(50.0),
            1.0, // Near plane at 1m
            200_000.0, // Far plane at 200km
        );

        let control = OrbitControl::new(target, 10.0, 200_000.0);

        Self { camera, control }
    }

    pub fn update(&mut self, events: &mut [Event]) {
        self.control.handle_events(&mut self.camera, events);
        
        // Handle arrow key camera movement
        let pan_speed = 500.0; // 500m per frame
        for event in events.iter() {
            if let Event::KeyPress { kind, .. } = event {
                let right = self.camera.right_direction();
                let forward = vec3(
                    -self.camera.view_direction().x,
                    0.0,
                    -self.camera.view_direction().z,
                )
                .normalize();
                
                match kind {
                    Key::ArrowLeft => {
                        self.camera.translate(-right * pan_speed);
                    }
                    Key::ArrowRight => {
                        self.camera.translate(right * pan_speed);
                    }
                    Key::ArrowUp => {
                        self.camera.translate(forward * pan_speed);
                    }
                    Key::ArrowDown => {
                        self.camera.translate(-forward * pan_speed);
                    }
                    _ => {}
                }
            }
        }
    }

    pub fn camera(&self) -> &Camera {
        &self.camera
    }

    pub fn set_viewport(&mut self, viewport: Viewport) {
        self.camera.set_viewport(viewport);
    }
}
