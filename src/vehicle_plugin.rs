use avian3d::{math::*, prelude::*};
use bevy::{ecs::query::Has, prelude::*};

pub struct CharacterControllerPlugin;

impl Plugin for CharacterControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<MovementAction>().add_systems(
            Update,
            (
                keyboard_input,
                // TODO
                // gamepad_input,
                update_grounded,
                movement,
                // apply_linear_speed,
                // apply_angular_damping,
                // apply_movement_damping,
                linear_velocity_max_speed,
                update_transform_from_direction,
            )
                .chain(),
        );
    }
}


#[derive(Message)]
pub enum MovementAction {
    TurnLeft,
    TurnRight,
    SpeedUp,
    SlowDownOrReverse,
    NoSpeedInput,
    // Move(Vector2),
    // Jump,
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
    mut movement_writer: MessageWriter<MovementAction>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
) {
    if keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]) {
        movement_writer.write(MovementAction::SpeedUp);
    } else if keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]) {
        movement_writer.write(MovementAction::SlowDownOrReverse);
    } else {
        movement_writer.write(MovementAction::NoSpeedInput);
    }

    
    if keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]) {
        movement_writer.write(MovementAction::TurnLeft);
    }
    if keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]) {
        movement_writer.write(MovementAction::TurnRight);
    }

    // let horizontal = right as i8 - left as i8;
    // let vertical = up as i8 - down as i8;
    // let direction = Vector2::new(horizontal as Scalar, vertical as Scalar).clamp_length_max(1.0);

    // if direction != Vector2::ZERO {
    //     movement_writer.write(MovementAction::Move(direction));
    // }

    // if keyboard_input.just_pressed(KeyCode::Space) {
    //     movement_writer.write(MovementAction::Jump);
    // }
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

/// Responds to [`MovementAction`] events and moves character controllers accordingly.
fn movement(
    time: Res<Time>,
    mut movement_reader: MessageReader<MovementAction>,
    mut controllers: Query<(
        // &MovementAcceleration,
        // &JumpImpulse,
        // &mut LinearSpeed,
        // &mut LinearVelocity,
        // &mut AngularVelocity,
        &mut Direction,
        // &Transform,
        &mut ConstantLinearAcceleration,
        Has<Grounded>,
    )>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_secs_f64().adjust_precision();

    for event in movement_reader.read() {
        for (
            // movement_acceleration,
            // jump_impulse,
            // mut linear_speed,
            // mut linear_velocity,
            // mut angular_velocity,
            mut direction,
            mut constant_linear_acceleration,
            // transform,
            _is_grounded,
        ) in &mut controllers
        {
            match event {
                MovementAction::SpeedUp => {
                    // linear_speed.0 += movement_acceleration.0 * delta_time * 10.0;
                    constant_linear_acceleration.0 =
                        Vector3::new(direction.0.x, 0.0, direction.0.y) * 100.0;
                }
                MovementAction::SlowDownOrReverse => {
                    // linear_speed.0 -= movement_acceleration.0 * delta_time * 10.0;

                    let reverse_dir = direction.0.rotate(Vec2::from_angle(PI));
                    constant_linear_acceleration.0 =
                        Vector3::new(reverse_dir.x, 0.0, reverse_dir.y) * 100.0;
                }
                MovementAction::TurnLeft => {
                    // angular_velocity.y += 100.0 * delta_time;
                    direction.0 =
                        Dir2::new(direction.0.rotate(Vec2::from_angle(-2.0 * delta_time)))
                            .expect("TODO");
                }
                MovementAction::TurnRight => {
                    // angular_velocity.y -= 100.0 * delta_time;
                    direction.0 = Dir2::new(direction.0.rotate(Vec2::from_angle(2.0 * delta_time)))
                        .expect("TODO");
                }
                MovementAction::NoSpeedInput => {
                    // linear_speed.0 *= 0.8_f32.powf(delta_time * 60.0);
                    constant_linear_acceleration.0 = Vector3::ZERO;
                } // MovementAction::Move(direction) => {
                  //     linear_velocity.x += direction.x * movement_acceleration.0 * delta_time;
                  //     linear_velocity.z -= direction.y * movement_acceleration.0 * delta_time;
                  // }
                  // MovementAction::Jump => {
                  //     if is_grounded {
                  //         linear_velocity.y = jump_impulse.0;
                  //     }
                  // }
            }

            // info!("linear_velocity: {:?}", linear_velocity);
            // let forward = transform.forward();

            // **linear_velocity = forward * linear_speed.0;

            // linear_velocity.x = forward.x * linear_velocity.0.x;
            // linear_velocity.z = forward.z * linear_velocity.0.z;
            // info!("new_linear_velocity: {:?}", linear_velocity);
        }
    }
}

// fn apply_linear_speed(mut query: Query<(&LinearSpeed, &mut LinearVelocity, &Direction)>) {
//     for (linear_speed, mut linear_velocity, direction) in &mut query {
//         let linear_velocity_vec_2 = direction.0 * linear_speed.0;
//         linear_velocity.x = linear_velocity_vec_2.x;
//         linear_velocity.z = linear_velocity_vec_2.y;
//     }
// }

// TODO: this should be at the top level, not in the vehicle plugin
fn update_transform_from_direction(
    mut query: Query<(&Direction, &mut Transform), With<CharacterController>>,
) {
    for (direction, mut transform) in &mut query {
        let forward = Vec3::new(direction.0.x, 0.0, direction.0.y);
        transform.look_to(forward, Vec3::Y);
    }
}

// fn apply_movement_damping(
//     mut query: Query<(
//         &MovementDampingFactor,
//         &mut LinearVelocity,
//         // &mut LinearSpeed,
//     )>,
// ) {
//     for (damping_factor, mut linear_velocity) in &mut query {
//         // We could use `LinearDamping`, but we don't want to dampen movement along the Y axis
//         linear_velocity.x *= damping_factor.0;
//         linear_velocity.z *= damping_factor.0;
//         // linear_speed.0 *= damping_factor.0;
//     }
// }

/// Applies angular damping to character controllers.
// fn apply_angular_damping(
//     mut query: Query<(&mut AngularVelocity, &mut LinearVelocity, &Transform)>,
// ) {
//     for (mut angular_velocity, mut linear_velocity, transform) in &mut query {
//         angular_velocity.0 *= 0.4;
//         // **linear_velocity = linear_velocity.project_onto(*transform.forward());
//     }
// }

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
