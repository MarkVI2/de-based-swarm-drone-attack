use three_d::*;

pub struct GridPlanes {
    pub models: Vec<Gm<Mesh, ColorMaterial>>,
    pub labels: Vec<Gm<Mesh, ColorMaterial>>,
}

impl GridPlanes {
    pub fn new(context: &Context, size_km: f32, cell_size_km: f32) -> Self {
        let size_m = size_km * 1000.0;
        let cell_size_m = cell_size_km * 1000.0;
        
        let mut models = Vec::new();
        
        // Origin (0,0,0) is now at the corner where three planes meet
        // Floor grid (XZ plane at y=0, extends in +X and +Z directions)
        models.extend(Self::create_grid_lines(
            context,
            size_m,
            cell_size_m,
            vec3(size_m / 2.0, 0.0, size_m / 2.0),
            GridOrientation::Floor,
        ));

        // Back wall (XY plane at z=0, extends in +X and +Y directions)
        models.extend(Self::create_grid_lines(
            context,
            size_m,
            cell_size_m,
            vec3(size_m / 2.0, size_m / 2.0, 0.0),
            GridOrientation::WallBack,
        ));

        // Left wall (YZ plane at x=0, extends in +Y and +Z directions)
        models.extend(Self::create_grid_lines(
            context,
            size_m,
            cell_size_m,
            vec3(0.0, size_m / 2.0, size_m / 2.0),
            GridOrientation::WallLeft,
        ));

        // Create grid labels showing km coordinates
        let labels = Self::create_grid_labels(context, size_km, cell_size_km);

        Self { models, labels }
    }

    fn create_grid_lines(
        context: &Context,
        size: f32,
        cell_size: f32,
        position: Vec3,
        orientation: GridOrientation,
    ) -> Vec<Gm<Mesh, ColorMaterial>> {
        let cells = (size / cell_size) as usize;
        let mut positions = Vec::new();
        let mut colors = Vec::new();

        let half_size = size / 2.0;
        let black = Srgba::new(18, 18, 18, 255);

        // Generate grid lines based on orientation
        for i in 0..=cells {
            let offset = -half_size + (i as f32 * cell_size);

            match orientation {
                GridOrientation::Floor => {
                    // Lines parallel to X axis
                    positions.push(vec3(-half_size, 0.0, offset));
                    positions.push(vec3(half_size, 0.0, offset));
                    colors.push(black);
                    colors.push(black);

                    // Lines parallel to Z axis
                    positions.push(vec3(offset, 0.0, -half_size));
                    positions.push(vec3(offset, 0.0, half_size));
                    colors.push(black);
                    colors.push(black);
                }
                GridOrientation::WallBack => {
                    // Lines parallel to X axis
                    positions.push(vec3(-half_size, offset, 0.0));
                    positions.push(vec3(half_size, offset, 0.0));
                    colors.push(black);
                    colors.push(black);

                    // Lines parallel to Y axis
                    positions.push(vec3(offset, -half_size, 0.0));
                    positions.push(vec3(offset, half_size, 0.0));
                    colors.push(black);
                    colors.push(black);
                }
                GridOrientation::WallLeft => {
                    // Lines parallel to Z axis
                    positions.push(vec3(0.0, offset, -half_size));
                    positions.push(vec3(0.0, offset, half_size));
                    colors.push(black);
                    colors.push(black);

                    // Lines parallel to Y axis
                    positions.push(vec3(0.0, -half_size, offset));
                    positions.push(vec3(0.0, half_size, offset));
                    colors.push(black);
                    colors.push(black);
                }
            }
        }

        let mut line_models = Vec::new();
        let line_thickness = 20.0; // 20m thick lines for visibility at km scale
        
        // Create thin cylinder for each line segment
        for i in (0..positions.len()).step_by(2) {
            let start = positions[i];
            let end = positions[i + 1];
            
            let direction = end - start;
            let length = direction.magnitude();
            let center = (start + end) * 0.5;
            
            let mut cylinder = CpuMesh::cylinder(8);
            
            // Apply transformations: scale, rotate, translate
            let scale_mat = Mat4::from_nonuniform_scale(line_thickness, length / 2.0, line_thickness);
            cylinder.transform(scale_mat).unwrap();
            
            // Rotate to align with direction
            let up = vec3(0.0, 1.0, 0.0);
            let dir_norm = direction.normalize();
            let rotation = if dir_norm.dot(up).abs() < 0.999 {
                Quat::from_arc(up, dir_norm, None)
            } else {
                Quat::new(1.0, 0.0, 0.0, 0.0) // Identity quaternion
            };
            let rotation_mat = Mat4::from(rotation);
            cylinder.transform(rotation_mat).unwrap();
            
            // Translate to position
            let translation = Mat4::from_translation(center + position);
            cylinder.transform(translation).unwrap();
            
            let line_mesh = Gm::new(
                Mesh::new(context, &cylinder),
                ColorMaterial {
                    color: Srgba::BLACK,
                    ..Default::default()
                },
            );
            
            line_models.push(line_mesh);
        }
        
        line_models
    }

    fn create_grid_labels(
        context: &Context,
        size_km: f32,
        cell_size_km: f32,
    ) -> Vec<Gm<Mesh, ColorMaterial>> {
        let mut label_models = Vec::new();
        let cells = (size_km / cell_size_km) as usize;
        let label_height = 200.0; // 200m tall text
        
        // Create labels along floor grid at regular intervals
        for i in 0..=cells {
            let km_value = i as f32 * cell_size_km;
            let pos_m = km_value * 1000.0;
            
            // X-axis labels (along Z=0)
            if i % 5 == 0 { // Label every 5km
                let label_mesh = Self::create_text_mesh(
                    context,
                    &format!("{}km", km_value as i32),
                    label_height,
                );
                let mut label = Gm::new(label_mesh, ColorMaterial {
                    color: Srgba::new(50, 50, 50, 255),
                    ..Default::default()
                });
                label.set_transformation(Mat4::from_translation(vec3(pos_m, 50.0, 100.0)));
                label_models.push(label);
            }
            
            // Z-axis labels (along X=0)
            if i % 5 == 0 {
                let label_mesh = Self::create_text_mesh(
                    context,
                    &format!("{}km", km_value as i32),
                    label_height,
                );
                let mut label = Gm::new(label_mesh, ColorMaterial {
                    color: Srgba::new(50, 50, 50, 255),
                    ..Default::default()
                });
                label.set_transformation(Mat4::from_translation(vec3(100.0, 50.0, pos_m)));
                label_models.push(label);
            }
        }
        
        label_models
    }

    fn create_text_mesh(context: &Context, text: &str, height: f32) -> Mesh {
        // Create simple 3D text using boxes for each character
        // This is a simplified version - for production, use a proper text rendering library
        let char_width = height * 0.6;
        let thickness = height * 0.1;
        
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        
        for (i, _ch) in text.chars().enumerate() {
            let offset_x = i as f32 * char_width;
            
            // Create a simple box for the character
            let box_verts = vec![
                // Front face
                vec3(offset_x, 0.0, 0.0),
                vec3(offset_x + char_width * 0.8, 0.0, 0.0),
                vec3(offset_x + char_width * 0.8, height, 0.0),
                vec3(offset_x, height, 0.0),
                // Back face
                vec3(offset_x, 0.0, thickness),
                vec3(offset_x + char_width * 0.8, 0.0, thickness),
                vec3(offset_x + char_width * 0.8, height, thickness),
                vec3(offset_x, height, thickness),
            ];
            
            let base_idx = positions.len() as u32;
            positions.extend(box_verts);
            
            // Add indices for the box
            let box_indices = vec![
                // Front
                base_idx, base_idx + 1, base_idx + 2,
                base_idx, base_idx + 2, base_idx + 3,
                // Back
                base_idx + 4, base_idx + 6, base_idx + 5,
                base_idx + 4, base_idx + 7, base_idx + 6,
            ];
            indices.extend(box_indices);
        }
        
        let cpu_mesh = CpuMesh {
            positions: Positions::F32(positions),
            indices: Indices::U32(indices),
            ..Default::default()
        };
        
        Mesh::new(context, &cpu_mesh)
    }
}

enum GridOrientation {
    Floor,
    WallBack,
    WallLeft,
}

pub struct DroneRenderer {
    pub instances: Gm<InstancedMesh, PhysicalMaterial>,
}

impl DroneRenderer {
    pub fn new(context: &Context, positions: &[glam::Vec3], radius: f32) -> Self {
        let mut cpu_mesh = CpuMesh::sphere(32);
        cpu_mesh.transform(Mat4::from_scale(radius)).unwrap();

        let transformations: Vec<Mat4> = positions
            .iter()
            .map(|pos| Mat4::from_translation(vec3(pos.x, pos.y, pos.z)))
            .collect();

        let instances = Gm::new(
            InstancedMesh::new(
                context,
                &Instances {
                    transformations,
                    colors: None,
                    texture_transformations: None,
                },
                &cpu_mesh,
            ),
            PhysicalMaterial::new_opaque(
                context,
                &CpuMaterial {
                    albedo: Srgba::new(25, 76, 230, 255),
                    roughness: 0.5,
                    metallic: 0.1,
                    ..Default::default()
                },
            ),
        );

        Self { instances }
    }

    pub fn update_positions(&mut self, positions: &[glam::Vec3]) {
        let transformations: Vec<Mat4> = positions
            .iter()
            .map(|pos| Mat4::from_translation(vec3(pos.x, pos.y, pos.z)))
            .collect();
        
        self.instances.set_instances(&Instances {
            transformations,
            colors: None,
            texture_transformations: None,
        });
    }
}

pub struct TargetRenderer {
    pub model: Gm<Mesh, PhysicalMaterial>,
}

impl TargetRenderer {
    pub fn new(context: &Context, position: glam::Vec3, radius: f32) -> Self {
        let mut cpu_mesh = CpuMesh::sphere(32);
        cpu_mesh.transform(Mat4::from_scale(radius)).unwrap();

        let mut model = Gm::new(
            Mesh::new(context, &cpu_mesh),
            PhysicalMaterial::new_opaque(
                context,
                &CpuMaterial {
                    albedo: Srgba::new(230, 13, 26, 255),
                    roughness: 0.4,
                    metallic: 0.2,
                    ..Default::default()
                },
            ),
        );
        model.set_transformation(Mat4::from_translation(vec3(
            position.x, position.y, position.z,
        )));

        Self { model }
    }

    pub fn update_position(&mut self, position: glam::Vec3) {
        self.model.set_transformation(Mat4::from_translation(vec3(
            position.x, position.y, position.z,
        )));
    }
}
