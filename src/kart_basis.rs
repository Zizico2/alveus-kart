use bevy::prelude::*;
use bevy_tnua::math::{AdjustPrecision, AsF32, Float, Quaternion, Vector3, float_consts};
use std::time::Duration;

use bevy_tnua::TnuaBasisContext;
use bevy_tnua::util::rotation_arc_around_axis;
use bevy_tnua::{TnuaBasis, TnuaVelChange};

// CHANGED: Renamed struct to TnuaBuiltinKart
#[derive(Clone, Debug)]
pub struct TnuaBuiltinKart {
    /// The direction (in the world space) and speed to accelerate to.
    ///
    /// Tnua assumes that this vector is orthogonal to the up dierction.
    pub desired_velocity: Vector3,

    /// If non-zero, Tnua will rotate the character so that its negative Z will face in that
    /// direction.
    ///
    /// Tnua assumes that this vector is orthogonal to the up direction.
    pub desired_forward: Option<Dir3>,

    /// The height at which the character will float above ground at rest.
    ///
    /// Note that this is the height of the character's center of mass - not the distance from its
    /// collision mesh.
    ///
    /// To make a character crouch, instead of altering this field, prefer to use the
    /// [`TnuaBuiltinCrouch`](crate::builtins::TnuaBuiltinCrouch) action.
    pub float_height: Float,

    /// Extra distance above the `float_height` where the spring is still in effect.
    ///
    /// When the character is at at most this distance above the
    /// [`float_height`](Self::float_height), the spring force will kick in and move it to the
    /// float height - even if that means pushing it down. If the character is above that distance
    /// above the `float_height`, Tnua will consider it to be in the air.
    pub cling_distance: Float,

    /// The force that pushes the character to the float height.
    ///
    /// The actual force applied is in direct linear relationship to the displacement from the
    /// `float_height`.
    pub spring_strength: Float,

    /// A force that slows down the characters vertical spring motion.
    ///
    /// The actual dampening is in direct linear relationship to the vertical velocity it tries to
    /// dampen.
    ///
    /// Note that as this approaches 2.0, the character starts to shake violently and eventually
    /// get launched upward at great speed.
    pub spring_dampening: Float,

    /// The acceleration for horizontal movement (forward and backward).
    ///
    /// This controls how quickly the kart speeds up or brakes.
    pub acceleration: Float,

    // NEW: Acceleration for turning.
    /// The acceleration for horizontal turning movement.
    ///
    /// This controls how "grippy" the tires are, allowing the kart
    /// to change its velocity direction quickly without sliding.
    pub turning_acceleration: Float,

    /// The acceleration for horizontal movement while in the air.
    ///
    /// Set to 0.0 to completely disable air movement.
    pub air_acceleration: Float,

    /// The time, in seconds, the character can still jump after losing their footing.
    pub coyote_time: Float,

    /// Extra gravity for free fall (fall that's not initiated by a jump or some other action that
    /// provides its own fall gravity)
    ///
    /// **NOTE**: This force will be added to the normal gravity.
    ///
    /// **NOTE**: If the parameter set to this option is too low, the character may be able to run
    /// up a slope and "jump" potentially even higher than a regular jump, even without pressing
    /// the jump button.
    pub free_fall_extra_gravity: Float,

    /// The maximum angular velocity used for keeping the character standing upright.
    ///
    /// NOTE: The character's rotation can also be locked to prevent it from being tilted, in which
    /// case this paramter is redundant and can be set to 0.0.
    pub tilt_offset_angvel: Float,

    /// The maximum angular acceleration used for reaching `tilt_offset_angvel`.
    ///
    /// NOTE: The character's rotation can also be locked to prevent it from being tilted, in which
    /// case this paramter is redundant and can be set to 0.0.
    pub tilt_offset_angacl: Float,

    /// The maximum angular velocity used for turning the character when the direction changes.
    pub turning_angvel: Float,

    /// The maximum slope, in radians, that the character can stand on without slipping.
    pub max_slope: Float,
}

// CHANGED: Renamed to TnuaBuiltinKart
impl Default for TnuaBuiltinKart {
    fn default() -> Self {
        Self {
            desired_velocity: Vector3::ZERO,
            desired_forward: None,
            float_height: 0.0,
            cling_distance: 1.0,
            spring_strength: 400.0,
            spring_dampening: 1.2,
            acceleration: 60.0,
            turning_acceleration: 120.0, // NEW: Added turning_acceleration default
            air_acceleration: 20.0,
            coyote_time: 0.15,
            free_fall_extra_gravity: 60.0,
            tilt_offset_angvel: 5.0,
            tilt_offset_angacl: 500.0,
            turning_angvel: 10.0,
            max_slope: float_consts::FRAC_PI_2,
        }
    }
}

// CHANGED: Renamed to TnuaBuiltinKart
impl TnuaBasis for TnuaBuiltinKart {
    // CHANGED: Renamed
    const NAME: &'static str = "TnuaBuiltinKart";
    // CHANGED: Renamed
    type State = TnuaBuiltinKartState;

    fn apply(
        &self,
        state: &mut Self::State,
        ctx: TnuaBasisContext,
        motor: &mut bevy_tnua::TnuaMotor,
    ) {
        if let Some(stopwatch) = &mut state.airborne_timer {
            #[allow(clippy::unnecessary_cast)]
            stopwatch.tick(Duration::from_secs_f64(ctx.frame_duration as f64));
        }

        let climb_vectors: Option<ClimbVectors>;
        let considered_in_air: bool;
        let impulse_to_offset: Vector3;
        let slipping_vector: Option<Vector3>;

        if let Some(sensor_output) = &ctx.proximity_sensor.output {
            state.effective_velocity = ctx.tracker.velocity - sensor_output.entity_linvel;
            let sideways_unnormalized = sensor_output
                .normal
                .cross(*ctx.up_direction)
                .adjust_precision();
            if sideways_unnormalized == Vector3::ZERO {
                climb_vectors = None;
            } else {
                climb_vectors = Some(ClimbVectors {
                    direction: sideways_unnormalized
                        .cross(sensor_output.normal.adjust_precision())
                        .normalize_or_zero()
                        .adjust_precision(),
                    sideways: sideways_unnormalized.normalize_or_zero().adjust_precision(),
                });
            }

            slipping_vector = {
                let angle_with_floor = sensor_output
                    .normal
                    .angle_between(*ctx.up_direction)
                    .adjust_precision();
                if angle_with_floor <= self.max_slope {
                    None
                } else {
                    Some(
                        sensor_output
                            .normal
                            .reject_from(*ctx.up_direction)
                            .adjust_precision(),
                    )
                }
            };

            if state.airborne_timer.is_some() {
                considered_in_air = true;
                impulse_to_offset = Vector3::ZERO;
                state.standing_on = None;
            } else {
                if let Some(standing_on_state) = &state.standing_on {
                    if standing_on_state.entity != sensor_output.entity {
                        impulse_to_offset = Vector3::ZERO;
                    } else {
                        impulse_to_offset =
                            sensor_output.entity_linvel - standing_on_state.entity_linvel;
                    }
                } else {
                    impulse_to_offset = Vector3::ZERO;
                }

                if slipping_vector.is_none() {
                    considered_in_air = false;
                    state.standing_on = Some(StandingOnState {
                        entity: sensor_output.entity,
                        entity_linvel: sensor_output.entity_linvel,
                    });
                } else {
                    considered_in_air = true;
                    state.standing_on = None;
                }
            }
        } else {
            state.effective_velocity = ctx.tracker.velocity;
            climb_vectors = None;
            considered_in_air = true;
            impulse_to_offset = Vector3::ZERO;
            slipping_vector = None;
            state.standing_on = None;
        }
        state.effective_velocity += impulse_to_offset;

        let velocity_on_plane = state
            .effective_velocity
            .reject_from(ctx.up_direction.adjust_precision());

        state.vertical_velocity = if let Some(climb_vectors) = &climb_vectors {
            state.effective_velocity.dot(climb_vectors.direction)
                * climb_vectors
                    .direction
                    .dot(ctx.up_direction.adjust_precision())
        } else {
            0.0
        };

        // #################################################################
        // ## NEW: Kart-Specific Dual Acceleration Logic
        // #################################################################

        let (walk_acceleration, walk_boost) = if considered_in_air {
            // ############
            // ## IN AIR ## - Use original logic with air_acceleration
            // ############
            let desired_boost = self.desired_velocity - velocity_on_plane;
            let safe_direction_coefficient = self
                .desired_velocity
                .normalize_or_zero()
                .dot(velocity_on_plane.normalize_or_zero());
            let direction_change_factor = 1.5 - 0.5 * safe_direction_coefficient;
            let max_acceleration = direction_change_factor * self.air_acceleration;

            if self.desired_velocity == Vector3::ZERO {
                (
                    Vector3::ZERO,
                    desired_boost.clamp_length_max(ctx.frame_duration * max_acceleration),
                )
            } else {
                (
                    (desired_boost / ctx.frame_duration).clamp_length_max(max_acceleration),
                    Vector3::ZERO,
                )
            }
        } else {
            // ##############
            // ## ON GROUND ## - Use new kart logic
            // ##############

            // Get the kart's current orientation (forward/right) on the ground plane
            // We use NEG_Z for forward, matching the `Turning` logic at the end of this function
            let kart_forward = ctx
                .tracker
                .rotation
                .mul_vec3(Vector3::NEG_Z)
                .reject_from(*ctx.up_direction)
                .normalize_or_zero();

            // Fallback in case kart is pointing straight up
            let (kart_forward, _kart_right) = if kart_forward == Vector3::ZERO {
                // Cannot determine orientation, use velocity as fallback
                (velocity_on_plane.normalize_or_zero(), Vector3::ZERO)
            } else {
                let kart_right = kart_forward.cross(*ctx.up_direction).normalize_or_zero();
                (kart_forward, kart_right)
            };

            // Calculate the total change in velocity we want
            let desired_boost = self.desired_velocity - velocity_on_plane;

            // Decompose the boost into forward (throttle/brake) and lateral (grip/turning) components
            let fwd_boost_vec = desired_boost.project_onto(kart_forward);
            let lat_boost_vec = desired_boost.reject_from(kart_forward);

            // Check if we are trying to reverse while still moving forward
            let current_fwd_speed = velocity_on_plane.dot(kart_forward);
            let desired_fwd_speed = self.desired_velocity.dot(kart_forward);
            let is_braking_to_reverse = (current_fwd_speed > 0.1) && (desired_fwd_speed < 0.0);

            if self.desired_velocity == Vector3::ZERO && slipping_vector.is_none() {
                // SPECIAL CASE: Braking to a full stop (Issue #39)
                // We use boost for a crisp stop.
                // We *do* want high-grip lateral braking here (to stop drifting).
                let fwd_boost =
                    fwd_boost_vec.clamp_length_max(ctx.frame_duration * self.acceleration);
                let lat_boost =
                    lat_boost_vec.clamp_length_max(ctx.frame_duration * self.turning_acceleration);
                (Vector3::ZERO, fwd_boost + lat_boost)
            } else {
                // NORMAL CASE: Accelerating, turning, or reversing
                let fwd_accel =
                    (fwd_boost_vec / ctx.frame_duration).clamp_length_max(self.acceleration);

                let lat_accel = if is_braking_to_reverse {
                    // We are braking to reverse.
                    // **DO NOT** apply high-grip turning acceleration.
                    // This prevents the "U-turn" motion.
                    // We still apply *some* lateral braking if the kart is drifting,
                    // but we cap it at the *low* throttle acceleration.
                    (lat_boost_vec / ctx.frame_duration).clamp_length_max(self.acceleration)
                } else {
                    // We are accelerating or turning normally.
                    // Apply high-grip turning acceleration.
                    (lat_boost_vec / ctx.frame_duration).clamp_length_max(self.turning_acceleration)
                };

                (fwd_accel + lat_accel, Vector3::ZERO)
            }
        };

        // ... (The rest of this block is the same) ...

        // Now, apply climb vector projection (if on ground and not slipping)
        let (walk_acceleration, walk_boost) =
            if let (Some(climb_vectors), None) = (&climb_vectors, slipping_vector) {
                (
                    climb_vectors.project(walk_acceleration),
                    climb_vectors.project(walk_boost),
                )
            } else {
                (walk_acceleration, walk_boost)
            };

        // Now, calculate slipping_boost (this logic is unchanged from original)
        let slipping_boost = 'slipping_boost: {
            // ... (slipping boost logic is unchanged) ...
            let Some(slipping_vector) = slipping_vector else {
                break 'slipping_boost Vector3::ZERO;
            };
            let vertical_velocity = if 0.0 <= state.vertical_velocity {
                ctx.tracker.gravity.dot(ctx.up_direction.adjust_precision()) * ctx.frame_duration
            } else {
                state.vertical_velocity
            };

            let Ok((slipping_direction, slipping_per_vertical_unit)) =
                Dir3::new_and_length(slipping_vector.f32())
            else {
                break 'slipping_boost Vector3::ZERO;
            };

            let required_veloicty_in_slipping_direction =
                slipping_per_vertical_unit.adjust_precision() * -vertical_velocity;

            let expected_velocity =
                velocity_on_plane + walk_acceleration * ctx.frame_duration + walk_boost;

            let expected_velocity_in_slipping_direction =
                expected_velocity.dot(slipping_direction.adjust_precision());

            let diff =
                required_veloicty_in_slipping_direction - expected_velocity_in_slipping_direction;

            if diff <= 0.0 {
                break 'slipping_boost Vector3::ZERO;
            }

            slipping_direction.adjust_precision() * diff
        };

        // Combine all forces
        let walk_vel_change = TnuaVelChange {
            acceleration: walk_acceleration,
            boost: walk_boost + slipping_boost,
        };

        // #################################################################
        // ## END of new logic
        // #################################################################

        let upward_impulse: TnuaVelChange = 'upward_impulse: {
            // ... (upward_impulse logic is unchanged) ...
            let should_disable_due_to_slipping =
                slipping_vector.is_some() && state.vertical_velocity <= 0.0;
            for _ in 0..2 {
                #[allow(clippy::unnecessary_cast)]
                match &mut state.airborne_timer {
                    None => {
                        if let (false, Some(sensor_output)) =
                            (should_disable_due_to_slipping, &ctx.proximity_sensor.output)
                        {
                            // not doing the jump calculation here
                            let spring_offset =
                                self.float_height - sensor_output.proximity.adjust_precision();
                            state.standing_offset =
                                -spring_offset * ctx.up_direction.adjust_precision();
                            break 'upward_impulse self.spring_force(state, &ctx, spring_offset);
                        } else {
                            state.airborne_timer = Some(Timer::from_seconds(
                                self.coyote_time as f32,
                                TimerMode::Once,
                            ));
                            continue;
                        }
                    }
                    Some(_) => {
                        if let (false, Some(sensor_output)) =
                            (should_disable_due_to_slipping, &ctx.proximity_sensor.output)
                        {
                            if sensor_output.proximity.adjust_precision() <= self.float_height {
                                state.airborne_timer = None;
                                continue;
                            }
                        }
                        if state.vertical_velocity <= 0.0 {
                            break 'upward_impulse TnuaVelChange::acceleration(
                                -self.free_fall_extra_gravity * ctx.up_direction.adjust_precision(),
                            );
                        } else {
                            break 'upward_impulse TnuaVelChange::ZERO;
                        }
                    }
                }
            }
            error!("Tnua could not decide on jump state");
            TnuaVelChange::ZERO
        };

        motor.lin = walk_vel_change + TnuaVelChange::boost(impulse_to_offset) + upward_impulse;
        let new_velocity = state.effective_velocity
            + motor.lin.boost
            + ctx.frame_duration * motor.lin.acceleration
            - impulse_to_offset;
        state.running_velocity = new_velocity.reject_from(ctx.up_direction.adjust_precision());

        // Tilt (unchanged)

        let torque_to_fix_tilt = {
            let tilted_up = ctx.tracker.rotation.mul_vec3(Vector3::Y);

            let rotation_required_to_fix_tilt =
                Quaternion::from_rotation_arc(tilted_up, ctx.up_direction.adjust_precision());

            let desired_angvel = (rotation_required_to_fix_tilt.xyz() / ctx.frame_duration)
                .clamp_length_max(self.tilt_offset_angvel);
            let angular_velocity_diff = desired_angvel - ctx.tracker.angvel;
            angular_velocity_diff.clamp_length_max(ctx.frame_duration * self.tilt_offset_angacl)
        };

        // Turning (unchanged)

        let desired_angvel = if let Some(desired_forward) = self.desired_forward {
            let current_forward = ctx.tracker.rotation.mul_vec3(Vector3::NEG_Z);
            let rotation_along_up_axis = rotation_arc_around_axis(
                ctx.up_direction,
                current_forward,
                desired_forward.adjust_precision(),
            )
            .unwrap_or(0.0);
            (rotation_along_up_axis / ctx.frame_duration)
                .clamp(-self.turning_angvel, self.turning_angvel)
        } else {
            0.0
        };

        // ... (rest of function is unchanged) ...
        let existing_angvel = ctx.tracker.angvel.dot(ctx.up_direction.adjust_precision());

        let torque_to_turn = desired_angvel - existing_angvel;

        let existing_turn_torque = torque_to_fix_tilt.dot(ctx.up_direction.adjust_precision());
        let torque_to_turn = torque_to_turn - existing_turn_torque;

        motor.ang = TnuaVelChange::boost(
            torque_to_fix_tilt + torque_to_turn * ctx.up_direction.adjust_precision(),
        );
    }

    // ... (rest of Trait impl is unchanged) ...

    fn proximity_sensor_cast_range(&self, _state: &Self::State) -> Float {
        self.float_height + self.cling_distance
    }

    fn displacement(&self, state: &Self::State) -> Option<Vector3> {
        match state.airborne_timer {
            None => Some(state.standing_offset),
            Some(_) => None,
        }
    }

    fn effective_velocity(&self, state: &Self::State) -> Vector3 {
        state.effective_velocity
    }

    fn vertical_velocity(&self, state: &Self::State) -> Float {
        state.vertical_velocity
    }

    fn neutralize(&mut self) {
        self.desired_velocity = Vector3::ZERO;
        self.desired_forward = None;
    }

    fn is_airborne(&self, state: &Self::State) -> bool {
        state
            .airborne_timer
            .as_ref()
            .is_some_and(|timer| timer.is_finished())
    }

    fn violate_coyote_time(&self, state: &mut Self::State) {
        if let Some(timer) = &mut state.airborne_timer {
            timer.set_duration(Duration::ZERO);
        }
    }
}

// CHANGED: Renamed
impl TnuaBuiltinKart {
    /// Calculate the vertical spring force that this basis would need to apply assuming its
    /// vertical distance from the vertical distance it needs to be at equals the `spring_offset`
    /// argument.
    ///
    /// Note: this is exposed so that actions like
    /// [`TnuaBuiltinCrouch`](crate::builtins::TnuaBuiltinCrouch) may rely on it.
    pub fn spring_force(
        &self,
        // CHANGED: Renamed
        state: &TnuaBuiltinKartState,
        ctx: &TnuaBasisContext,
        spring_offset: Float,
    ) -> TnuaVelChange {
        let spring_force: Float = spring_offset * self.spring_strength;

        let relative_velocity = state
            .effective_velocity
            .dot(ctx.up_direction.adjust_precision())
            - state.vertical_velocity;

        let gravity_compensation = -ctx.tracker.gravity;

        let dampening_boost = relative_velocity * self.spring_dampening;

        TnuaVelChange {
            acceleration: ctx.up_direction.adjust_precision() * spring_force + gravity_compensation,
            boost: ctx.up_direction.adjust_precision() * -dampening_boost,
        }
    }
}

#[derive(Debug, Clone)]
struct StandingOnState {
    entity: Entity,
    entity_linvel: Vector3,
}

// CHANGED: Renamed
#[derive(Default, Clone, Debug)]
pub struct TnuaBuiltinKartState {
    airborne_timer: Option<Timer>,
    /// The current distance of the character from the distance its supposed to float at.
    pub standing_offset: Vector3,
    standing_on: Option<StandingOnState>,
    effective_velocity: Vector3,
    vertical_velocity: Float,
    /// The velocity, perpendicular to the up direction, that the character is supposed to move at.
    ///
    /// If the character is standing on something else
    /// ([`standing_on_entity`](Self::standing_on_entity) returns `Some`) then the
    /// `running_velocity` will be relative to the velocity of that entity.
    pub running_velocity: Vector3,
}

// CHANGED: Renamed
impl TnuaBuiltinKartState {
    /// Returns the entity that the character currently stands on.
    pub fn standing_on_entity(&self) -> Option<Entity> {
        Some(self.standing_on.as_ref()?.entity)
    }

    pub fn reset_airborne_timer(&mut self) {
        self.airborne_timer = None;
    }
}

#[derive(Debug, Clone)]
struct ClimbVectors {
    direction: Vector3,
    sideways: Vector3,
}

impl ClimbVectors {
    fn project(&self, vector: Vector3) -> Vector3 {
        let axis_direction = vector.dot(self.direction) * self.direction;
        let axis_sideways = vector.dot(self.sideways) * self.sideways;
        axis_direction + axis_sideways
    }
}
