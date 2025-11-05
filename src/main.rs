use avian3d::prelude::*;
use bevy::{camera::Exposure, prelude::*};
use bevy_third_person_camera_2::*;

use crate::vehicle_plugin::{CharacterControllerPlugin, KartCharacter};

mod vehicle_plugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(ThirdPersonCameraPlugin::default())
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(CharacterControllerPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, rotate_camera)
        .run();
}

#[derive(Component)]
struct Kart;

fn setup(
    mut commands: Commands,
    // mut meshes: ResMut<Assets<Mesh>>,
    // mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    info!("spawning track");
    let track = asset_server.load(GltfAssetLabel::Scene(0).from_asset("Track.glb"));
    commands.spawn((
        SceneRoot(track),
        Transform::from_translation(Vec3::new(0.0, -30.0, 0.0)),
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh),
        // ColliderConstructorHierarchy::new(ColliderConstructor::ConvexHullFromMesh),
        CollisionMargin(0.5),
        Restitution::ZERO.with_combine_rule(CoefficientCombine::Min),
        RigidBody::Static,
        Friction::new(0.4),
    ));

    info!("spawning player kart");
    let player_transform = Transform::from_translation(Vec3::ZERO);
    let player = commands
        .spawn((
            Kart,
            SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("Kart.glb"))),
            Collider::cuboid(1.0, 1.0, 1.0),
            player_transform,
            KartCharacter::default(),
        ))
        .id();

    commands.spawn((
        Camera3d::default(),
        Exposure { ev100: 7.5 },
        CameraOffset(Vec3::new(0.0, -5.0, -15.0)),
        ThirdPersonCamera::aimed_at(player),
    ));
}

fn rotate_camera(
    mut camera: Query<(&Transform, Entity), With<ThirdPersonCamera>>,
    mut character: Query<&Transform, (With<Kart>, Changed<Transform>)>,
    mut commands: Commands,
) {
    for (cam_transform, cam_entity) in &mut camera {
        for char_transform in &mut character {
            // make both transforms y = 0 to ignore vertical difference
            // calculate angle difference
            let cam_forward =
                Vec2::new(cam_transform.forward().x, cam_transform.forward().z).normalize();
            let char_forward =
                Vec2::new(char_transform.forward().x, char_transform.forward().z).normalize();
            let angle_diff = cam_forward.angle_to(char_forward);
            info!("angle diff: {}", angle_diff);
            let rotate_event = RotateAroundTarget {
                camera: cam_entity,
                delta: Vec2::new(angle_diff, 0.0),
            };
            commands.trigger(rotate_event);
        }
    }
}
