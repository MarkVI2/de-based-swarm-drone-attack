use bevy::prelude::*;
use bevy::input::mouse::MouseMotion;

#[derive(Component)]
pub struct Drone {
    pub velocity: Vec3,
}

#[derive(Component)]
pub struct Adversary;

const GRID_SIZE: f32 = 50.0;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::BLACK))
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
) {
    // Scientific-style framing: compute a distance so the grid volume fits
    let fov = std::f32::consts::PI / 3.0; // ~60 deg
    let d = (GRID_SIZE / 2.0) / (fov / 2.0).tan();
    let camera_pos = Vec3::new(d, d * 0.9, d * 1.1); // slight bias for depth

    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(camera_pos).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Materials
    let floor_material = materials.add(StandardMaterial {
        base_color: Color::srgba(0.0, 0.4, 0.0, 1.0),
        unlit: true,
        ..default()
    });
    let wall_material = materials.add(StandardMaterial {
        base_color: Color::srgba(0.95, 0.9, 0.8, 0.4), // semi-transparent cream
        unlit: true,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    // Floor (XY)
    let floor_mesh = meshes.add(Plane3d { normal: Dir3::Y, half_size: Vec2::splat(GRID_SIZE / 2.0) });
    commands.spawn((
        Mesh3d(floor_mesh),
        MeshMaterial3d(floor_material),
        Transform::from_translation(Vec3::ZERO),
    ));
    // Back wall (XZ)
    let back_mesh = meshes.add(Plane3d { normal: Dir3::Z, half_size: Vec2::splat(GRID_SIZE / 2.0) });
    commands.spawn((
        Mesh3d(back_mesh),
        MeshMaterial3d(wall_material.clone()),
        Transform::from_translation(Vec3::new(0.0, GRID_SIZE / 2.0, -GRID_SIZE / 2.0)),
    ));
    // Left wall (YZ)
    let left_mesh = meshes.add(Plane3d { normal: Dir3::X, half_size: Vec2::splat(GRID_SIZE / 2.0) });
    commands.spawn((
        Mesh3d(left_mesh),
        MeshMaterial3d(wall_material),
        Transform::from_translation(Vec3::new(-GRID_SIZE / 2.0, GRID_SIZE / 2.0, 0.0)),
    ));

    // (Optional placeholder) Adversary spawn at origin (uncomment if needed)
    // let adv_material = materials.add(StandardMaterial { base_color: Color::srgba(0.3,0.3,0.9,1.0), unlit: true, ..default() });
    // let adv_mesh = meshes.add(Sphere { radius: 0.5 });
    // commands.spawn((Adversary, MaterialMeshBundle { mesh: adv_mesh, material: adv_material, transform: Transform::from_translation(Vec3::ZERO), ..default() }));
}

// =============================
// Camera Orbit / Pan / Zoom Controls (Scientific View)
// =============================
fn orbit_camera_controls(
    time: Res<Time>,
    mut mouse_motion: MessageReader<MouseMotion>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut Transform, With<Camera3d>>,
) {
    let Ok(mut transform) = query.single_mut() else { return };
    let mut rotation_delta = Vec2::ZERO;

    if mouse_buttons.pressed(MouseButton::Left) {
        for ev in mouse_motion.read() { rotation_delta += ev.delta; }
    } else {
        // drain events to avoid buildup when not rotating
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

    // Zoom
    let mut zoom = 0.0;
    if keys.pressed(KeyCode::KeyW) || keys.pressed(KeyCode::ArrowUp) { zoom -= 30.0 * time.delta_secs(); }
    if keys.pressed(KeyCode::KeyS) || keys.pressed(KeyCode::ArrowDown) { zoom += 30.0 * time.delta_secs(); }
    if zoom.abs() > 0.0 {
        let pivot = Vec3::ZERO;
        let dir = (transform.translation - pivot).normalize();
        transform.translation += dir * zoom;
    }

    // Pan (Right Mouse)
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