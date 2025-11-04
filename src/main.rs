use avian3d::{math::Scalar, prelude::*};
use bevy::{color::palettes::css::RED, prelude::*};
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
        // .add_systems(Update, move_kart)
        .run();
}

#[derive(Component)]
struct Kart;

/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    let track = asset_server.load(GltfAssetLabel::Scene(0).from_asset("karting_track.glb"));
    commands.spawn((
        SceneRoot(track),
        Transform::from_translation(Vec3::new(0.0, -30.0, 0.0)),
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh),
        // ColliderConstructorHierarchy::new(ColliderConstructor::ConvexHullFromMesh),
        CollisionMargin(0.1),
        Restitution::ZERO.with_combine_rule(CoefficientCombine::Min),
        RigidBody::Static,
        Friction::new(0.4),
    ));

    let player_transform = Transform::from_translation(Vec3::ZERO);
    let player_forward = player_transform.forward();

    let player = commands
        .spawn((
            Kart,
            // SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("Kart.glb"))),
            // 1 x 1 x 1 cube for testing
            Mesh3d(meshes.add(Mesh::from(Cuboid {
                half_size: Vec3::new(0.5, 0.5, 0.5),
            }))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: RED.into(),
                ..Default::default()
            })),
            Collider::cuboid(1.0, 1.0, 1.0),
            player_transform,
            KartCharacter::default(),
            // RigidBody::Kinematic,
            // CharacterControllerBundle::new(Collider::sphere(0.4)).with_movement(
            //     // 30.0,
            //     // 0.92,
            //     // 7.0,
            //     (90 as Scalar).to_radians(),
            //     Dir2::new(player_forward.xz().normalize()).expect("TODO"),
            // ),
            // Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
            // Friction::new(0.4),
            // Restitution::ZERO.with_combine_rule(CoefficientCombine::Min),
            // LockedAxes::new().lock_rotation_z(),
            // GravityScale(2.0),
        ))
        .id();

    commands.spawn((
        PointLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));

    commands.spawn((
        Camera3d::default(),
        CameraOffset(Vec3::new(0.0, -5.0, -15.0)),
        ThirdPersonCamera::aimed_at(player),
    ));
}

// fn move_kart(
//     keyboard_input: Res<ButtonInput<KeyCode>>,
//     time: Res<Time>,
//     mut query: Query<(&mut LinearVelocity, &mut AngularVelocity, &Transform), With<Kart>>,
// ) {
//     let mut velocity = Vec3::ZERO;
//     let mut up_angular_velocity = Vec3::ZERO;
//     if keyboard_input.pressed(KeyCode::KeyW) {
//         velocity.z = -1.0;
//     }
//     if keyboard_input.pressed(KeyCode::KeyS) {
//         velocity.z = 1.0;
//     }
//     if keyboard_input.pressed(KeyCode::KeyA) {
//         up_angular_velocity.y = 1.0;
//     }
//     if keyboard_input.pressed(KeyCode::KeyD) {
//         up_angular_velocity.y = -1.0;
//     }

//     let delta_secs = time.delta_secs();

//     for (mut linear_velocity, mut angular_velocity, transform) in &mut query {
//         info!("vroooom");
//         linear_velocity.0 += velocity.normalize_or_zero() * 30.0 * delta_secs;
//         linear_velocity.0 = linear_velocity.0.project_onto(*transform.forward());

//         angular_velocity.0 += up_angular_velocity.normalize_or_zero() * 10.0 * delta_secs;
//         // angular velocity should drop off sharply when no input is given
//         angular_velocity.0 *= 0.8_f32.powf(delta_secs * 60.0);
//     }
// }
