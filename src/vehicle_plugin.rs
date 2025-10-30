use avian3d::{math::*, prelude::*};
use bevy::prelude::*;

pub struct CharacterControllerPlugin;

impl Plugin for CharacterControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<TurningAction>()
            .add_message::<AccelerationAction>()
            .add_systems(
                Update,
                (
                    keyboard_input,
                    // TODO
                    update_grounded,
                    acceleration,
                    direction,
                    update_linear_velocity_from_direction,
                    linear_velocity_max_speed,
                    update_transform_from_direction,
                )
                    .chain(),
            );
    }
}

#[derive(Message)]
pub enum TurningAction {
    TurnLeft,
    TurnRight,
    // Jump,
}

#[derive(Message)]
pub enum AccelerationAction {
    Accelerate,
    DecelerateOrReverse,
    NoAcceleration,
}

/// A marker component indicating that an entity is using a character controller.
#[derive(Component)]
pub struct CharacterController;

/// A marker component indicating that an entity is on the ground.
#[derive(Component)]
#[component(storage = "SparseSet")]
pub struct Grounded;

/// The acceleration used for character movement.
// #[derive(Component)]
// pub struct MovementAcceleration(Scalar);

/// The damping factor used for slowing down movement.
#[derive(Component)]
pub struct MovementDampingFactor(Scalar);

/// The strength of a jump.
#[derive(Component)]
pub struct JumpImpulse(Scalar);

// #[derive(Component)]
// pub struct LinearSpeed(Scalar);

#[derive(Component)]
pub struct Direction(Dir2);

/// The maximum angle a slope can have for a character controller
/// to be able to climb and jump. If the slope is steeper than this angle,
/// the character will slide down.
#[derive(Component)]
pub struct MaxSlopeAngle(Scalar);

/// A bundle that contains the components needed for a basic
/// kinematic character controller.
#[derive(Bundle)]
pub struct CharacterControllerBundle {
    character_controller: CharacterController,
    body: RigidBody,
    collider: Collider,
    ground_caster: ShapeCaster,
    locked_axes: LockedAxes,
    movement: MovementBundle,
}

/// A bundle that contains components for character movement.
#[derive(Bundle)]
pub struct MovementBundle {
    // acceleration: MovementAcceleration,
    damping: MovementDampingFactor,
    jump_impulse: JumpImpulse,
    // linear_speed: LinearSpeed,
    direction: Direction,
    max_slope_angle: MaxSlopeAngle,
    acceleration: ConstantLinearAcceleration,
}

impl MovementBundle {
    pub const fn new(
        acceleration: Scalar,
        damping: Scalar,
        jump_impulse: Scalar,
        max_slope_angle: Scalar,
        direction: Dir2,
    ) -> Self {
        Self {
            // acceleration: MovementAcceleration(acceleration),
            damping: MovementDampingFactor(damping),
            jump_impulse: JumpImpulse(jump_impulse),
            // linear_speed: LinearSpeed(0.0),
            direction: Direction(direction),
            max_slope_angle: MaxSlopeAngle(max_slope_angle),
            acceleration: ConstantLinearAcceleration(Vector3::ZERO),
        }
    }
}

impl Default for MovementBundle {
    fn default() -> Self {
        Self::new(30.0, 0.9, 7.0, PI * 0.45, Dir2::Y)
    }
}

impl CharacterControllerBundle {
    pub fn new(collider: Collider) -> Self {
        // Create shape caster as a slightly smaller version of collider
        let mut caster_shape = collider.clone();
        caster_shape.set_scale(Vector::ONE * 0.99, 10);

        Self {
            character_controller: CharacterController,
            body: RigidBody::Dynamic,
            collider,
            ground_caster: ShapeCaster::new(
                caster_shape,
                Vector::ZERO,
                Quaternion::default(),
                Dir3::NEG_Y,
            )
            .with_max_distance(0.2),
            locked_axes: LockedAxes::ROTATION_LOCKED,
            movement: MovementBundle::default(),
        }
    }

    pub fn with_movement(
        mut self,
        acceleration: Scalar,
        damping: Scalar,
        jump_impulse: Scalar,
        max_slope_angle: Scalar,
        direction: Dir2,
    ) -> Self {
        self.movement = MovementBundle::new(
            acceleration,
            damping,
            jump_impulse,
            max_slope_angle,
            direction,
        );
        self
    }
}

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
    }
    if keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]) {
        turning_action_writer.write(TurningAction::TurnRight);
    }
}

// TODO
/// Sends [`MovementAction`] events based on gamepad input.
// fn gamepad_input(mut movement_writer: MessageWriter<MovementAction>, gamepads: Query<&Gamepad>) {
//     for gamepad in gamepads.iter() {
//         if let (Some(x), Some(y)) = (
//             gamepad.get(GamepadAxis::LeftStickX),
//             gamepad.get(GamepadAxis::LeftStickY),
//         ) {
//             movement_writer.write(MovementAction::Move(
//                 Vector2::new(x as Scalar, y as Scalar).clamp_length_max(1.0),
//             ));
//         }

//         if gamepad.just_pressed(GamepadButton::South) {
//             movement_writer.write(MovementAction::Jump);
//         }
//     }
// }

/// Updates the [`Grounded`] status for character controllers.
fn update_grounded(
    mut commands: Commands,
    mut query: Query<
        (Entity, &ShapeHits, &Rotation, Option<&MaxSlopeAngle>),
        With<CharacterController>,
    >,
) {
    for (entity, hits, rotation, max_slope_angle) in &mut query {
        // The character is grounded if the shape caster has a hit with a normal
        // that isn't too steep.
        let is_grounded = hits.iter().any(|hit| {
            if let Some(angle) = max_slope_angle {
                (rotation * -hit.normal2).angle_between(Vector::Y).abs() <= angle.0
            } else {
                true
            }
        });

        if is_grounded {
            commands.entity(entity).insert(Grounded);
        } else {
            commands.entity(entity).remove::<Grounded>();
        }
    }
}

fn acceleration(
    mut acceleration_reader: MessageReader<AccelerationAction>,
    mut controllers: Query<
        (&Direction, &mut ConstantLinearAcceleration),
        With<CharacterController>,
    >,
) {
    for event in acceleration_reader.read() {
        for (direction, mut constant_linear_acceleration) in &mut controllers {
            match event {
                AccelerationAction::Accelerate => {
                    // Apply acceleration in the direction the character is facing
                    constant_linear_acceleration.0 =
                        Vector3::new(direction.0.x, 0.0, direction.0.y) * 100.0;
                }
                AccelerationAction::DecelerateOrReverse => {
                    // Apply reverse acceleration
                    let reverse_dir = direction.0.rotate(Vec2::from_angle(PI));
                    constant_linear_acceleration.0 =
                        Vector3::new(reverse_dir.x, 0.0, reverse_dir.y) * 100.0;
                }
                AccelerationAction::NoAcceleration => {
                    // No acceleration
                    constant_linear_acceleration.0 = Vector3::ZERO;
                }
            }
        }
    }
}

fn direction(
    time: Res<Time>,
    mut turning_reader: MessageReader<TurningAction>,
    mut controllers: Query<(&mut Direction, &mut LinearVelocity), With<CharacterController>>,
) {
    let delta_secs = time.delta_secs_f64().adjust_precision();

    for event in turning_reader.read() {
        for (mut direction, mut linear_velocity) in &mut controllers {
            match event {
                TurningAction::TurnLeft => {
                    direction.0 =
                        Dir2::new(direction.0.rotate(Vec2::from_angle(-2.0 * delta_secs)))
                            .expect("TODO");
                }
                TurningAction::TurnRight => {
                    direction.0 = Dir2::new(direction.0.rotate(Vec2::from_angle(2.0 * delta_secs)))
                        .expect("TODO");
                }
            }
        }
    }
}

fn update_linear_velocity_from_direction(
    mut query: Query<
        (&Direction, &mut LinearVelocity),
        (With<CharacterController>, Changed<Direction>),
    >,
) {
    for (direction, mut linear_velocity) in &mut query {
        linear_velocity.0 =
            linear_velocity
                .0
                .project_onto(Vector3::new(direction.0.x, 0.0, direction.0.y));
    }
}

// TODO: this should be at the top level, not in the vehicle plugin
fn update_transform_from_direction(
    mut query: Query<(&Direction, &mut Transform), With<CharacterController>>,
) {
    for (direction, mut transform) in &mut query {
        let forward = Vec3::new(direction.0.x, 0.0, direction.0.y);
        transform.look_to(forward, Vec3::Y);
    }
}

fn linear_velocity_max_speed(mut query: Query<(&mut LinearVelocity, &CharacterController)>) {
    for (mut linear_velocity, _) in &mut query {
        let horizontal_velocity = Vec2::new(linear_velocity.x, linear_velocity.z);
        let max_speed = 100.0;
        if horizontal_velocity.length() > max_speed {
            let clamped_velocity = horizontal_velocity.normalize() * max_speed;
            linear_velocity.x = clamped_velocity.x;
            linear_velocity.z = clamped_velocity.y;
        }
    }
}
