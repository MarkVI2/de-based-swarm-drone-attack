mod camera;
mod entities;
mod rendering;
mod scene;
mod de_path_planner;

use three_d::*;

use camera::SceneCamera;
use scene::Scene;

fn main() {
    let window = Window::new(WindowSettings {
        title: "Drone Swarm - Defensive Simulation".to_string(),
        max_size: Some((1920, 1080)),
        ..Default::default()
    })
    .unwrap();

    let context = window.gl();

    let mut scene_camera = SceneCamera::new(&context, window.viewport());
    let mut scene = Scene::new(&context);

    window.render_loop(move |mut frame_input| {
        scene_camera.set_viewport(frame_input.viewport);
        scene_camera.update(&mut frame_input.events);

        scene.update(&context, &frame_input.events);

        frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.97, 0.97, 0.94, 1.0, 1.0))
            .render(
                scene_camera.camera(),
                &scene.render_objects(),
                &scene.lights(),
            );

        FrameOutput::default()
    });
}
