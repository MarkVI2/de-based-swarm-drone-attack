use bevy::asset::RenderAssetUsages;
use bevy::input::mouse::MouseMotion;
use bevy::math::{
    Affine2,
    primitives::{Plane3d, Sphere},
};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};

#[derive(Component)]
pub struct Drone {
    pub velocity: Vec3,
}

#[derive(Component)]
pub struct Adversary;

const GRID_SIZE: f32 = 50.0;
const METERS_PER_UNIT: f32 = 1.0;
const METERS_PER_KILOMETER: f32 = 1_000.0;
const DRONE_COUNT: usize = 42;
const DRONE_SWARM_RADIUS_METERS: f32 = 8.0;
const DRONE_BASE_ALTITUDE_METERS: f32 = 60.0;
const DRONE_MAX_ALTITUDE_METERS: f32 = 120.0;
const DRONE_RADIUS_METERS: f32 = 0.6;
const TARGET_DISTANCE_KM: f32 = 25.0;
const GRID_CELL_METERS: f32 = 5.0;

#[derive(Resource, Default)]
struct SwarmEntities {
    drones: Vec<Entity>,
    target: Option<Entity>,
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgba(0.97, 0.97, 0.94, 1.0)))
        .insert_resource(SwarmEntities::default())
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Drone Swarm".into(),
                ..default()
            }),
            ..default()
        }))
        .add_systems(Startup, setup_scene)
        .add_systems(Update, orbit_camera_controls)
        .run();
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut swarm_entities: ResMut<SwarmEntities>,
) {
    let fov = std::f32::consts::PI / 3.0; // ~60 deg
    let d = (GRID_SIZE / 2.0) / (fov / 2.0).tan();
    let camera_pos = Vec3::new(d, d * 0.9, d * 1.1);

    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(camera_pos).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let grid_texture = generate_grid_texture(&mut images);
    let repeats = GRID_SIZE / GRID_CELL_METERS;
    let floor_material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        base_color_texture: Some(grid_texture),
        unlit: true,
        uv_transform: Affine2::from_scale(Vec2::splat(repeats)),
        ..default()
    });
    let wall_material = materials.add(StandardMaterial {
        base_color: Color::srgba(0.95, 0.93, 0.88, 0.65),
        unlit: true,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    let floor_mesh = meshes.add(Plane3d {
        normal: Dir3::Y,
        half_size: Vec2::splat(GRID_SIZE / 2.0),
    });
    commands.spawn((
        Mesh3d(floor_mesh),
        MeshMaterial3d(floor_material),
        Transform::from_translation(Vec3::ZERO),
    ));

    let back_mesh = meshes.add(Plane3d {
        normal: Dir3::Z,
        half_size: Vec2::splat(GRID_SIZE / 2.0),
    });
    commands.spawn((
        Mesh3d(back_mesh),
        MeshMaterial3d(wall_material.clone()),
        Transform::from_translation(Vec3::new(0.0, GRID_SIZE / 2.0, -GRID_SIZE / 2.0)),
    ));

    let left_mesh = meshes.add(Plane3d {
        normal: Dir3::X,
        half_size: Vec2::splat(GRID_SIZE / 2.0),
    });
    commands.spawn((
        Mesh3d(left_mesh),
        MeshMaterial3d(wall_material),
        Transform::from_translation(Vec3::new(-GRID_SIZE / 2.0, GRID_SIZE / 2.0, 0.0)),
    ));

    let drone_mesh = meshes.add(Sphere {
        radius: meters_to_units(DRONE_RADIUS_METERS),
        ..default()
    });
    let drone_material = materials.add(StandardMaterial {
        base_color: Color::srgba(0.1, 0.3, 0.9, 1.0),
        unlit: true,
        ..default()
    });

    let drone_offsets = fibonacci_sphere_points(DRONE_COUNT, DRONE_SWARM_RADIUS_METERS);
    for offset in drone_offsets {
        let altitude =
            (DRONE_BASE_ALTITUDE_METERS + offset.y).clamp(0.0, DRONE_MAX_ALTITUDE_METERS);
        let world_position = Vec3::new(
            meters_to_units(offset.x),
            meters_to_units(altitude),
            meters_to_units(offset.z),
        );

        let entity = commands
            .spawn((
                Drone {
                    velocity: Vec3::ZERO,
                },
                Mesh3d(drone_mesh.clone()),
                MeshMaterial3d(drone_material.clone()),
                Transform::from_translation(world_position),
            ))
            .id();

        swarm_entities.drones.push(entity);
    }

    let target_mesh = meshes.add(Sphere {
        radius: meters_to_units(DRONE_RADIUS_METERS * 1.5),
        ..default()
    });
    let target_material = materials.add(StandardMaterial {
        base_color: Color::srgba(0.8, 0.05, 0.1, 1.0),
        unlit: true,
        ..default()
    });

    let target_distance_units = kilometers_to_units(TARGET_DISTANCE_KM);
    let target_altitude_units =
        meters_to_units((DRONE_BASE_ALTITUDE_METERS + 20.0).min(DRONE_MAX_ALTITUDE_METERS));
    let horizontal_distance = (target_distance_units.powi(2) - target_altitude_units.powi(2))
        .max(0.0)
        .sqrt();
    let direction = Vec2::new(0.85, 0.52).normalize_or_zero();
    let target_position = Vec3::new(
        direction.x * horizontal_distance,
        target_altitude_units,
        direction.y * horizontal_distance,
    );

    let target_entity = commands
        .spawn((
            Adversary,
            Mesh3d(target_mesh),
            MeshMaterial3d(target_material),
            Transform::from_translation(target_position),
        ))
        .id();

    swarm_entities.target = Some(target_entity);
}

fn generate_grid_texture(images: &mut Assets<Image>) -> Handle<Image> {
    let cells = (GRID_SIZE / GRID_CELL_METERS).round().max(1.0) as u32;
    const CELL_PIXELS: u32 = 64;
    const LINE_THICKNESS: u32 = 4;

    let resolution = cells * CELL_PIXELS;
    let mut data = vec![0u8; (resolution * resolution * 4) as usize];
    let off_white = [248u8, 248, 242, 255];
    let black = [18u8, 18, 18, 255];

    for y in 0..resolution {
        for x in 0..resolution {
            let idx = ((y * resolution + x) * 4) as usize;
            let x_mod = x % CELL_PIXELS;
            let y_mod = y % CELL_PIXELS;
            let is_edge = x_mod < LINE_THICKNESS || y_mod < LINE_THICKNESS;
            let color = if is_edge { black } else { off_white };
            data[idx..idx + 4].copy_from_slice(&color);
        }
    }

    let image = Image::new_fill(
        Extent3d {
            width: resolution,
            height: resolution,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    );

    images.add(image)
}

fn fibonacci_sphere_points(count: usize, radius_meters: f32) -> Vec<Vec3> {
    if count == 0 {
        return Vec::new();
    }

    let mut points = Vec::with_capacity(count);
    let golden_angle = std::f32::consts::PI * (3.0 - 5.0_f32.sqrt());
    let count_f = count as f32;

    for i in 0..count {
        let i_f = i as f32;
        let fraction = (i_f + 0.5) / count_f;
        let y = 1.0 - 2.0 * fraction;
        let radius = (1.0 - y * y).max(0.0).sqrt();
        let theta = golden_angle * i_f;
        let x = theta.cos() * radius;
        let z = theta.sin() * radius;
        points.push(Vec3::new(x, y, z) * radius_meters);
    }

    points
}

fn meters_to_units(meters: f32) -> f32 {
    meters / METERS_PER_UNIT
}

fn kilometers_to_units(kilometers: f32) -> f32 {
    meters_to_units(kilometers * METERS_PER_KILOMETER)
}

fn orbit_camera_controls(
    time: Res<Time>,
    mut mouse_motion: MessageReader<MouseMotion>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut Transform, With<Camera3d>>,
) {
    let Ok(mut transform) = query.single_mut() else {
        return;
    };

    let mut rotation_delta = Vec2::ZERO;

    if mouse_buttons.pressed(MouseButton::Left) {
        for ev in mouse_motion.read() {
            rotation_delta += ev.delta;
        }
    } else {
        for _ in mouse_motion.read() {}
    }

    if rotation_delta.length_squared() > 0.0 {
        let yaw = -rotation_delta.x * 0.005;
        let pitch = -rotation_delta.y * 0.005;
        let pivot = Vec3::ZERO;
        let dir = transform.translation - pivot;
        let yaw_rot = Quat::from_axis_angle(Vec3::Y, yaw);
        let pitch_rot = Quat::from_axis_angle((*transform.right()).into(), pitch);
        let new_dir = (yaw_rot * pitch_rot).mul_vec3(dir);
        transform.translation = pivot + new_dir;
        transform.look_at(pivot, Vec3::Y);
    }

    let mut zoom = 0.0;
    if keys.pressed(KeyCode::KeyW) || keys.pressed(KeyCode::ArrowUp) {
        zoom -= 30.0 * time.delta_secs();
    }
    if keys.pressed(KeyCode::KeyS) || keys.pressed(KeyCode::ArrowDown) {
        zoom += 30.0 * time.delta_secs();
    }
    if zoom.abs() > 0.0 {
        let pivot = Vec3::ZERO;
        let dir = (transform.translation - pivot).normalize();
        transform.translation += dir * zoom;
    }

    if mouse_buttons.pressed(MouseButton::Right) {
        for ev in mouse_motion.read() {
            let pan_speed = 0.02 * (transform.translation.length() / GRID_SIZE.max(1.0));
            let right = transform.right();
            let up = Vec3::Y;
            transform.translation -= right * ev.delta.x * pan_speed;
            transform.translation += up * ev.delta.y * pan_speed;
        }
        transform.look_at(Vec3::ZERO, Vec3::Y);
    }
}
