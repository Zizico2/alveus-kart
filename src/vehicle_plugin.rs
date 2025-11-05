use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_tnua::prelude::*;
use bevy_tnua_avian3d::prelude::*;

use crate::kart_basis::TnuaBuiltinKart;

pub struct CharacterControllerPlugin;

impl Plugin for CharacterControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            TnuaControllerPlugin::new(FixedUpdate),
            TnuaAvian3dPlugin::new(FixedUpdate),
        ))
        .add_message::<TurningAction>()
        .add_message::<AccelerationAction>()
        .add_systems(FixedUpdate, (acceleration, direction).in_set(TnuaUserControlsSystems))
        .add_systems(
             Update,
             keyboard_input,
        )
        //     (
        //         keyboard_input,
        //         // TODO
        //         update_grounded,
        //         acceleration,
        //         direction,
        //         update_linear_velocity_from_direction,
        //         linear_velocity_max_speed,
        //         update_transform_from_direction,
        //         apply_horizontal_acceleration,
        //         remove_horizontal_acceleration_when_not_grounded,
        //     )
        //         .chain(),
        // )
        ;
    }
}

#[derive(Message)]
pub enum TurningAction {
    TurnLeft,
    TurnRight,
    None,
    // Jump,
}

#[derive(Message)]
pub enum AccelerationAction {
    Accelerate,
    DecelerateOrReverse,
    NoAcceleration,
}

#[derive(Bundle)]
pub struct KartCharacter {
    rigid_body: RigidBody,
    controller: TnuaController,
    locked_axes: LockedAxes,
    direction: Direction,
}

impl Default for KartCharacter {
    fn default() -> Self {
        Self {
            rigid_body: RigidBody::Dynamic,
            controller: TnuaController::default(),
            locked_axes: LockedAxes::new().lock_rotation_x().lock_rotation_z(),
            direction: Direction(Dir2::NEG_Y),
        }
    }
}

// /// A marker component indicating that an entity is using a character controller.
// #[derive(Component)]
// pub struct CharacterController;

// /// A marker component indicating that an entity is on the ground.
// #[derive(Component)]
// #[component(storage = "SparseSet")]
// pub struct Grounded;

// /// The acceleration used for character movement.
// // #[derive(Component)]
// // pub struct MovementAcceleration(Scalar);

// // /// The damping factor used for slowing down movement.
// // #[derive(Component)]
// // pub struct MovementDampingFactor(Scalar);

// /// The strength of a jump.
// // #[derive(Component)]
// // pub struct JumpImpulse(Scalar);

// // #[derive(Component)]
// // pub struct LinearSpeed(Scalar);

#[derive(Component)]
pub struct Direction(Dir2);

// /// The maximum angle a slope can have for a character controller
// /// to be able to climb and jump. If the slope is steeper than this angle,
// /// the character will slide down.
// #[derive(Component)]
// pub struct MaxSlopeAngle(Scalar);

// #[derive(Component)]
// pub struct HorizontalAcceleration(Vector2);

// /// A bundle that contains the components needed for a basic
// /// kinematic character controller.
// #[derive(Bundle)]
// pub struct CharacterControllerBundle {
//     character_controller: CharacterController,

//     locked_axes: LockedAxes,
// }

// /// A bundle that contains components for character movement.
// #[derive(Bundle)]
// pub struct MovementBundle {
//     // acceleration: MovementAcceleration,
//     // damping: MovementDampingFactor,
//     // jump_impulse: JumpImpulse,
//     // linear_speed: LinearSpeed,
//     direction: Direction,
//     max_slope_angle: MaxSlopeAngle,
//     acceleration: ConstantLinearAcceleration,
//     horizontal_acceleration: HorizontalAcceleration,
// }

// impl MovementBundle {
//     pub const fn new(
//         // acceleration: Scalar,
//         // damping: Scalar,
//         // jump_impulse: Scalar,
//         max_slope_angle: Scalar,
//         direction: Dir2,
//     ) -> Self {
//         Self {
//             // acceleration: MovementAcceleration(acceleration),
//             // damping: MovementDampingFactor(damping),
//             // jump_impulse: JumpImpulse(jump_impulse),
//             // linear_speed: LinearSpeed(0.0),
//             direction: Direction(direction),
//             max_slope_angle: MaxSlopeAngle(max_slope_angle),
//             acceleration: ConstantLinearAcceleration(Vector3::ZERO),
//             horizontal_acceleration: HorizontalAcceleration(Vector2::ZERO),
//         }
//     }
// }

// impl Default for MovementBundle {
//     fn default() -> Self {
//         Self::new(PI * 0.45, Dir2::Y)
//     }
// }

// impl CharacterControllerBundle {
//     pub fn new(collider: Collider) -> Self {
//         // Create shape caster as a slightly smaller version of collider
//         let mut caster_shape = collider.clone();
//         caster_shape.set_scale(Vector::ONE * 0.99, 10);

//         Self {
//             character_controller: CharacterController,
//             body: RigidBody::Dynamic,
//             collider,
//             ground_caster: ShapeCaster::new(
//                 caster_shape,
//                 Vector::ZERO,
//                 Quaternion::default(),
//                 Dir3::NEG_Y,
//             )
//             .with_max_distance(0.2),
//             locked_axes: LockedAxes::ROTATION_LOCKED,
//             movement: MovementBundle::default(),
//         }
//     }

//     pub fn with_movement(
//         mut self,
//         // acceleration: Scalar,
//         // damping: Scalar,
//         // jump_impulse: Scalar,
//         max_slope_angle: Scalar,
//         direction: Dir2,
//     ) -> Self {
//         self.movement = MovementBundle::new(
//             // acceleration,
//             // damping,
//             // jump_impulse,
//             max_slope_angle,
//             direction,
//         );
//         self
//     }
// }

/// Sends [`MovementAction`] events based on keyboard input.
fn keyboard_input(
    mut turning_action_writer: MessageWriter<TurningAction>,
    mut acceleration_action_writer: MessageWriter<AccelerationAction>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
) {
    if keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]) {
        acceleration_action_writer.write(AccelerationAction::Accelerate);
    } else if keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]) {
        acceleration_action_writer.write(AccelerationAction::DecelerateOrReverse);
    } else {
        acceleration_action_writer.write(AccelerationAction::NoAcceleration);
    }

    if keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]) {
        turning_action_writer.write(TurningAction::TurnLeft);
    } else if keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]) {
        turning_action_writer.write(TurningAction::TurnRight);
    } else {
        turning_action_writer.write(TurningAction::None);
    }
}

// // TODO
// /// Sends [`MovementAction`] events based on gamepad input.
// // fn gamepad_input(mut movement_writer: MessageWriter<MovementAction>, gamepads: Query<&Gamepad>) {
// //     for gamepad in gamepads.iter() {
// //         if let (Some(x), Some(y)) = (
// //             gamepad.get(GamepadAxis::LeftStickX),
// //             gamepad.get(GamepadAxis::LeftStickY),
// //         ) {
// //             movement_writer.write(MovementAction::Move(
// //                 Vector2::new(x as Scalar, y as Scalar).clamp_length_max(1.0),
// //             ));
// //         }

// //         if gamepad.just_pressed(GamepadButton::South) {
// //             movement_writer.write(MovementAction::Jump);
// //         }
// //     }
// // }

// /// Updates the [`Grounded`] status for character controllers.
// fn update_grounded(
//     mut commands: Commands,
//     mut query: Query<
//         (
//             Entity,
//             &ShapeHits,
//             &Rotation,
//             &mut LinearVelocity,
//             Option<&MaxSlopeAngle>,
//         ),
//         With<CharacterController>,
//     >,
// ) {
//     for (entity, hits, rotation, mut linear_velocity, max_slope_angle) in &mut query {
//         // The character is grounded if the shape caster has a hit with a normal
//         // that isn't too steep.
//         let is_grounded = hits.iter().any(|hit| {
//             if let Some(max_slope_angle) = max_slope_angle {
//                 let real_angle = (rotation * -hit.normal2).angle_between(Vector::Y).abs();
//                 info!("max_slope_angle: {}", max_slope_angle.0);
//                 info!("real_angle: {}", real_angle);
//                 real_angle <= max_slope_angle.0
//             } else {
//                 true
//             }
//         });

//         if is_grounded {
//             commands.entity(entity).insert(Grounded);
//             commands.entity(entity).insert(GravityScale(100.0));
//             info!("grounded");
//         } else {
//             commands.entity(entity).remove::<Grounded>();
//             commands.entity(entity).remove::<GravityScale>();
//             info!("not grounded");
//         }
//     }
// }

fn acceleration(
    mut acceleration_reader: MessageReader<AccelerationAction>,
    mut query: Query<(&mut TnuaController, &Direction)>,
) {
    for event in acceleration_reader.read() {
        for (mut controller, direction) in &mut query {
            let unsigned_desired_velocity = Vec3::new(direction.0.x, 0.0, direction.0.y) * 100.0;
            match event {
                AccelerationAction::Accelerate => {
                    info!("acceleration!");
                    let desired_velocity = unsigned_desired_velocity;
                    controller.basis(TnuaBuiltinKart {
                        desired_velocity,
                        desired_forward: Some(
                            Dir3::new(unsigned_desired_velocity.normalize()).expect("ERR"),
                        ),
                        acceleration: 40.0,
                        turning_acceleration: 500.0,
                        float_height: 2.0,
                        ..TnuaBuiltinKart::default()
                    });
                }
                AccelerationAction::DecelerateOrReverse => {
                    info!("deceleration!");
                    let desired_velocity = -unsigned_desired_velocity;
                    controller.basis(TnuaBuiltinKart {
                        desired_velocity,
                        desired_forward: Some(
                            Dir3::new(unsigned_desired_velocity.normalize()).expect("ERR"),
                        ),
                        acceleration: 20.0,
                        turning_acceleration: 500.0,
                        float_height: 2.0,
                        ..Default::default()
                    });
                }
                AccelerationAction::NoAcceleration => {
                    info!("no acceleration!");
                    controller.basis(TnuaBuiltinKart {
                        desired_velocity: Vec3::ZERO,
                        desired_forward: None,
                        // TODO: think about acceleration behavior when no input
                        acceleration: 70.0,
                        turning_acceleration: 500.0,
                        float_height: 2.0,
                        ..Default::default()
                    });
                }
            }
        }
    }
}

// fn apply_horizontal_acceleration(
//     mut query: Query<
//         (&HorizontalAcceleration, &mut ConstantLinearAcceleration),
//         (
//             With<CharacterController>,
//             Changed<HorizontalAcceleration>,
//             With<Grounded>,
//         ),
//     >,
// ) {
//     for (horizontal_acceleration, mut constant_linear_acceleration) in &mut query {
//         constant_linear_acceleration.0.x = horizontal_acceleration.0.x;
//         constant_linear_acceleration.0.z = horizontal_acceleration.0.y;
//     }
// }

// fn remove_horizontal_acceleration_when_not_grounded(
//     mut query: Query<
//         &mut ConstantLinearAcceleration,
//         (
//             With<CharacterController>,
//             Changed<Grounded>,
//             Without<Grounded>,
//         ),
//     >,
// ) {
//     for mut constant_linear_acceleration in &mut query {
//         constant_linear_acceleration.0.x = 0.0;
//         constant_linear_acceleration.0.z = 0.0;
//     }
// }

fn direction(mut turning_reader: MessageReader<TurningAction>, mut query: Query<&mut Direction>) {
    for event in turning_reader.read() {
        for mut direction in &mut query {
            match event {
                TurningAction::TurnLeft => {
                    info!("turning left");
                    direction.0 =
                        Dir2::new(Vec2::from_angle(-0.01).rotate(*direction.0)).expect("ERR");
                }
                TurningAction::TurnRight => {
                    info!("turning right");
                    direction.0 =
                        Dir2::new(Vec2::from_angle(0.01).rotate(*direction.0)).expect("ERR");
                }
                TurningAction::None => {
                    info!("not turning");
                    // controller.basis(TnuaBuiltinKart {
                    //     // desired_velocity: basis.desired_velocity.rotate_y(0.0),
                    //     ..basis
                    // });
                }
            }
        }
    }
}

// fn update_linear_velocity_from_direction(
//     mut query: Query<
//         (&Direction, &mut LinearVelocity),
//         (With<CharacterController>, Changed<Direction>),
//     >,
// ) {
//     for (direction, mut linear_velocity) in &mut query {
//         let normalized_velocity = linear_velocity.0.normalize_or_zero();
//         let speed = linear_velocity.0.length();
//         linear_velocity.0 =
//             Vector3::new(direction.0.x, normalized_velocity.y, direction.0.y) * speed;
//     }
// }

// // TODO: this should be at the top level, not in the vehicle plugin
// fn update_transform_from_direction(
//     mut query: Query<(&Direction, &mut Transform), With<CharacterController>>,
// ) {
//     for (direction, mut transform) in &mut query {
//         let forward = Vec3::new(direction.0.x, 0.0, direction.0.y);
//         transform.look_to(forward, Vec3::Y);
//     }
// }

// fn linear_velocity_max_speed(
//     mut query: Query<
//         (&mut LinearVelocity, &mut ConstantLinearAcceleration),
//         With<CharacterController>,
//     >,
// ) {
//     for (mut linear_velocity, mut constant_linear_acceleration) in &mut query {
//         let horizontal_velocity = Vec2::new(linear_velocity.x, linear_velocity.z);
//         let max_speed = 100.0;
//         if horizontal_velocity.length() > max_speed {
//             let clamped_velocity = horizontal_velocity.normalize() * max_speed;
//             linear_velocity.x = clamped_velocity.x;
//             linear_velocity.z = clamped_velocity.y;

//             constant_linear_acceleration.0 = Vec3::ZERO;
//         }
//     }
// }
