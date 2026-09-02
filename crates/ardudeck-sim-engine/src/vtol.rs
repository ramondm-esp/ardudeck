//! A VTOL vehicle: strips, rotors, and the coupling between them.
//!
//! Implements the existing `SimVehicle` trait, so nothing else in the engine
//! changes. The SIM_JSON FDM server, per-vehicle ports, `SharedWorld` wake
//! coupling, collision resolution and the WS telemetry stream all take this
//! vehicle exactly as they take `CopterVehicle`. Swarm was already built; this
//! is the second implementor the trait was left room for.
//!
//! `CopterVehicle` is untouched and stays the reference multirotor, because it
//! is a calibrated port of ArduPilot's own SITL model and every multirotor test
//! in this crate is measured against it.
//!
//! ## What couples to what
//!
//! The interaction terms are the reason this is one file rather than a sum of
//! independent parts, and they are what a transition actually consists of:
//!
//! - **Slipstream onto the wing.** Each strip is immersed in whatever rotor wake
//!   reaches it, at the immersion the current TILT produces. A lift rotor ahead
//!   of a wing keeps that wing flying at zero airspeed, and the same wing is
//!   what makes the rotor's download a penalty in hover.
//! - **Rotor inflow from the airframe's motion.** A rotor's local flow includes
//!   the body rates, so a pitching aircraft loads its front and rear rotors
//!   differently without a damping derivative anywhere.
//! - **Spool-up under real load.** Rotor speed is a state on a real inertia
//!   driven by real motor torque against the aerodynamic load the blade element
//!   pass just computed, not a commanded RPM.

use crate::aero::{surface_forces, no_induced, AeroOutput, FlowField};
use crate::airframe::Airframe;
use crate::bemt::{RotorOutput, RotorState};
use crate::copter::{initial_state, Environment, VehicleState, DEFAULT_ENVIRONMENT};
use crate::fdm_server::{HomeLocation, SimVehicle};
use crate::math::Vec3;
use crate::wake::RotorWake;

/// Largest integration sub-step (s).
///
/// Finer than the multirotor path uses. A wing adds much stiffer terms than a
/// rotor does: control-surface response and pitch damping act on a short time
/// constant, and semi-implicit Euler on a 400 Hz frame will ring on them. This
/// is the cheapest correct answer, and it is cheap: the strip sum is a few
/// thousand flops.
const MAX_SUBSTEP: f64 = 0.001;

/// Ground plane stiffness (N/m per kg) and damping, for the contact spring.
const GROUND_K: f64 = 900.0;
const GROUND_C: f64 = 60.0;
const GROUND_FRICTION: f64 = 0.6;

/// Per-step observability. Nothing here feeds back into the physics.
///
/// This exists because a partner comparing the sim to their flight test needs to
/// see WHICH surface or rotor disagreed. A total force tells them the sim is
/// wrong and nothing about where, which makes it unusable as a development tool
/// however accurate the total happens to be.
#[derive(Debug, Clone, Default)]
pub struct VtolDiagnostics {
    pub rotors: Vec<RotorOutput>,
    pub aero: AeroOutput,
    /// Airspeed magnitude (m/s).
    pub airspeed: f64,
    /// Whole-aircraft angle of attack and sideslip (rad), for the report only:
    /// no force is computed from either.
    pub alpha: f64,
    pub beta: f64,
    /// Fuselage parasitic drag, body frame (N).
    pub fuselage_drag_bf: Vec3,
    /// Total aerodynamic force from the strips, body frame (N).
    pub aero_force_bf: Vec3,
    /// Total rotor force, body frame (N).
    pub rotor_force_bf: Vec3,
    /// Fraction 0..1 of wing area stalled.
    pub stalled_fraction: f64,
    /// True while any rotor is in the vortex ring state.
    pub any_vrs: bool,
    /// Commanded tilt of each rotor (rad).
    pub tilt: Vec<f64>,
    /// Rotor speeds (rad/s).
    pub omega: Vec<f64>,
}

pub struct VtolVehicle {
    id: String,
    airframe: Airframe,
    state: VehicleState,
    rotors: Vec<RotorState>,
    home: HomeLocation,
    env: Environment,
    /// Terrain height under the vehicle (m above the home datum, +up).
    ground_height: f64,
    /// Pack voltage reported to SITL.
    voltage: f64,
    last_current: f64,
    last_throttle: f64,
    diag: VtolDiagnostics,
    /// Other vehicles' wake sources, set by `SharedWorld` each frame.
    neighbor_wake: Vec<RotorWake>,
    /// Contact force from the shared world's collision pass, world frame.
    contact_force: Vec3,
    /// Scratch, reused every step so the hot path allocates nothing.
    induced: Vec<Vec3>,
    wash: Vec<f64>,
}

impl VtolVehicle {
    pub fn new(id: impl Into<String>, airframe: Airframe, home: HomeLocation) -> VtolVehicle {
        let n_rot = airframe.rotors.len();
        let n_surf = airframe.surfaces.len();
        let voltage = airframe.voltage_max;
        VtolVehicle {
            id: id.into(),
            airframe,
            state: initial_state(),
            rotors: vec![RotorState::default(); n_rot],
            home,
            env: DEFAULT_ENVIRONMENT,
            ground_height: 0.0,
            voltage,
            last_current: 0.0,
            last_throttle: 0.0,
            diag: VtolDiagnostics::default(),
            neighbor_wake: Vec::new(),
            contact_force: Vec3::zero(),
            induced: vec![Vec3::zero(); n_surf],
            wash: vec![0.0; n_surf],
        }
    }

    pub fn airframe(&self) -> &Airframe {
        &self.airframe
    }

    pub fn diagnostics_full(&self) -> &VtolDiagnostics {
        &self.diag
    }

    pub fn state(&self) -> VehicleState {
        self.state
    }

    pub fn set_state(&mut self, s: VehicleState) {
        self.state = s;
    }

    pub fn set_environment(&mut self, env: Environment) {
        self.env = env;
    }

    pub fn set_ground_height(&mut self, h: f64) {
        self.ground_height = h;
    }

    /// Rotor speeds (rad/s), in spec order.
    ///
    /// Public because rotor speed is a genuine STATE that `VehicleState` does
    /// not carry, and a replay or a scenario restart that ignores it starts the
    /// model on a different spool-up path than the flight it is being compared
    /// against. That difference is small, physical, and entirely capable of
    /// swamping the model error it is meant to measure.
    pub fn rotor_speeds(&self) -> Vec<f64> {
        self.rotors.iter().map(|r| r.omega).collect()
    }

    /// Set the rotor speeds, from ESC telemetry or from a saved state.
    pub fn set_rotor_speeds(&mut self, omega: &[f64]) {
        for (i, r) in self.rotors.iter_mut().enumerate() {
            if let Some(w) = omega.get(i) {
                r.omega = w.max(0.0);
            }
        }
    }

    /// Spin the rotors up to where the given throttle would hold them unloaded,
    /// so a scenario can start in flight instead of spending its first second
    /// spooling up from rest.
    pub fn seed_rotors(&mut self, throttle: f64) {
        for (i, r) in self.airframe.rotors.iter().enumerate() {
            self.rotors[i].omega = r.no_load_omega(throttle, self.voltage);
        }
    }

    /// One physics sub-step. Pure in everything but `self.state`, `self.rotors`
    /// and the diagnostics.
    fn substep(&mut self, pwm: &[f64], dt: f64) {
        let att = self.state.attitude;
        // Velocity relative to the air mass, body frame. The wind is world NED.
        let v_air_world = self.state.velocity.sub(self.env.wind);
        let v_air_bf = att.rotate_world_to_body(v_air_world);
        let gyro = self.state.angular_velocity;

        // Neighbour wake at the CG, rotated into body frame: another vehicle's
        // downwash is flow this one's rotors and wing genuinely fly through.
        let mut neighbour_bf = Vec3::zero();
        if !self.neighbor_wake.is_empty() {
            let params = crate::wake::WakeParams::default();
            let mut w = Vec3::zero();
            for src in &self.neighbor_wake {
                w = w.add(crate::wake::wake_at(src, self.state.position, &params));
            }
            neighbour_bf = att.rotate_world_to_body(w);
        }

        // ─── Rotors ────────────────────────────────────────────────────────
        let n_rot = self.airframe.rotors.len();
        self.diag.rotors.clear();
        self.diag.tilt.clear();
        self.diag.omega.clear();
        let mut rotor_force = Vec3::zero();
        let mut rotor_moment = Vec3::zero();
        let mut current = 0.0;
        let mut any_vrs = false;
        let mut throttle_sum = 0.0;

        for i in 0..n_rot {
            let ch = self.airframe.rotor_channels[i];
            let throttle = self
                .airframe
                .servo_unit(pwm.get(ch.throttle).copied().unwrap_or(self.airframe.pwm.min));
            throttle_sum += throttle;
            let tilt = match ch.tilt {
                Some(c) => self
                    .airframe
                    .tilt_angle(i, pwm.get(c).copied().unwrap_or(self.airframe.pwm.min)),
                None => ch.tilt_min,
            };
            self.rotors[i].tilt = tilt;

            let out = self.airframe.rotors[i].forces(
                self.rotors[i],
                v_air_bf,
                gyro,
                self.env.air_density,
                &self.airframe.airfoils,
                neighbour_bf,
            );
            rotor_force = rotor_force.add(out.force_bf);
            rotor_moment = rotor_moment.add(out.moment_bf);
            current += out.current;
            any_vrs |= out.in_vrs;

            // Spool the rotor against the load the blade element pass just
            // computed, on its own inertia. This ordering matters: the rotor
            // responds to the load it is ACTUALLY under, so a rotor entering
            // prop wash or edgewise flow slows down for the right reason.
            self.rotors[i].omega = self.airframe.rotors[i].step_omega(
                self.rotors[i],
                throttle,
                self.voltage,
                out.torque,
                dt,
            );
            self.diag.tilt.push(tilt);
            self.diag.omega.push(self.rotors[i].omega);
            self.diag.rotors.push(out);
        }
        self.last_throttle = if n_rot > 0 { throttle_sum / n_rot as f64 } else { 0.0 };
        self.last_current = current;

        // ─── Slipstream onto the strips ────────────────────────────────────
        // Resolved per strip: immersion fraction and the wake velocity it sees.
        // Recomputed every sub-step because the immersion moves with tilt, which
        // IS the transition.
        for k in 0..self.airframe.surfaces.len() {
            let mut frac_sum = 0.0;
            let mut weighted = Vec3::zero();
            for i in 0..n_rot {
                let f = self.airframe.wash_fraction(k, i, self.rotors[i].tilt);
                if f <= 0.0 {
                    continue;
                }
                // The wake blows OPPOSITE the thrust, at the fully developed
                // slipstream speed the rotor solve reported.
                let axis = self.airframe.rotors[i].tilted_axis(self.rotors[i].tilt);
                let v = axis.scale(-self.diag.rotors[i].slipstream);
                weighted = weighted.add(v.scale(f));
                frac_sum += f;
            }
            self.wash[k] = frac_sum.min(1.0);
            self.induced[k] = if frac_sum > 1e-9 {
                weighted.scale(1.0 / frac_sum)
            } else {
                Vec3::zero()
            };
        }
        for (k, s) in self.airframe.surfaces.iter_mut().enumerate() {
            s.wash_fraction = self.wash[k];
        }

        // ─── Strips ────────────────────────────────────────────────────────
        let controls: Vec<f64> = (0..self.airframe.channel_limits.len())
            .map(|c| {
                self.airframe
                    .deflection(c, pwm.get(c).copied().unwrap_or(mid_pwm(&self.airframe)))
            })
            .collect();
        let flow = FlowField {
            velocity_air_bf: v_air_bf.sub(neighbour_bf),
            gyro,
            air_density: self.env.air_density,
            induced: &no_induced,
            induced_per_surface: Some(&self.induced),
        };
        let aero = surface_forces(
            &self.airframe.surfaces,
            &self.airframe.airfoils,
            &controls,
            &flow,
        );

        // ─── Fuselage ──────────────────────────────────────────────────────
        // Per-axis and signed by velocity, so a sideslip costs what it should.
        let q = 0.5 * self.env.air_density;
        let a = self.airframe.area_cd;
        let drag_bf = Vec3::new(
            q * a.x * v_air_bf.x * v_air_bf.x.abs(),
            q * a.y * v_air_bf.y * v_air_bf.y.abs(),
            q * a.z * v_air_bf.z * v_air_bf.z.abs(),
        );

        // ─── Sum and integrate ─────────────────────────────────────────────
        let force_bf = rotor_force.add(aero.force_bf).sub(drag_bf);
        let moment_bf = rotor_moment.add(aero.moment_bf);

        let mut force_world = att.rotate_body_to_world(force_bf);
        force_world.z += self.airframe.mass * self.env.gravity;
        force_world = force_world.add(self.contact_force);

        let (force_world, moment_bf) = self.apply_ground(force_world, moment_bf);

        let accel_world = force_world.scale(1.0 / self.airframe.mass);
        let velocity = self.state.velocity.add(accel_world.scale(dt));
        let position = self.state.position.add(velocity.scale(dt));

        // Rigid-body rotation with the gyroscopic term, which a wing makes worth
        // carrying: Ixx and Izz differ by a lot on a winged aircraft, so the
        // roll-yaw inertia coupling is real, where on a symmetric multirotor it
        // very nearly is not.
        let i = self.airframe.inertia;
        let w = self.state.angular_velocity;
        let gyro_term = Vec3::new(
            (i.y - i.z) * w.y * w.z,
            (i.z - i.x) * w.z * w.x,
            (i.x - i.y) * w.x * w.y,
        );
        let rot_accel = Vec3::new(
            (moment_bf.x + gyro_term.x) / i.x,
            (moment_bf.y + gyro_term.y) / i.y,
            (moment_bf.z + gyro_term.z) / i.z,
        );
        let angular_velocity = w.add(rot_accel.scale(dt));
        let attitude = att.integrate(angular_velocity, dt);

        // Specific force: what an accelerometer reads, so gravity is excluded.
        let accel_body = att.rotate_world_to_body(
            force_world
                .sub(Vec3::new(0.0, 0.0, self.airframe.mass * self.env.gravity))
                .scale(1.0 / self.airframe.mass),
        );

        self.state = VehicleState {
            position,
            velocity,
            attitude,
            angular_velocity,
            accel_body,
            current,
            timestamp: self.state.timestamp + dt,
            load: None,
        };

        self.diag.airspeed = v_air_bf.length();
        self.diag.alpha = if v_air_bf.x.abs() > 1e-6 || v_air_bf.z.abs() > 1e-6 {
            v_air_bf.z.atan2(v_air_bf.x)
        } else {
            0.0
        };
        self.diag.beta = if self.diag.airspeed > 1e-6 {
            (v_air_bf.y / self.diag.airspeed).clamp(-1.0, 1.0).asin()
        } else {
            0.0
        };
        self.diag.fuselage_drag_bf = drag_bf;
        self.diag.aero_force_bf = aero.force_bf;
        self.diag.rotor_force_bf = rotor_force;
        self.diag.stalled_fraction = aero.stalled_area_fraction;
        self.diag.any_vrs = any_vrs;
        self.diag.aero = aero;
    }

    /// Ground contact as a spring-damper with friction, resolved in world NED.
    ///
    /// A spring rather than a position clamp because a winged aircraft LANDS: it
    /// arrives with forward speed and has to roll out, and a clamp deletes the
    /// vertical momentum instead of resisting it, which hides both the bounce
    /// and the load.
    fn apply_ground(&self, mut force_world: Vec3, mut moment_bf: Vec3) -> (Vec3, Vec3) {
        // NED: position.z is negative above the datum, so penetration is when
        // -z falls below the terrain height.
        let agl = -self.state.position.z - self.ground_height;
        if agl > 0.0 {
            return (force_world, moment_bf);
        }
        let m = self.airframe.mass;
        let depth = -agl;
        let sink_rate = self.state.velocity.z;
        // Damping ADDS while penetrating. depth = -agl and agl = -z - h, so
        // d(depth)/dt is +velocity.z: a descending vehicle is sinking in, and
        // the resisting force has to grow with that. Subtracting it makes the
        // contact NEGATIVELY damped, which pumps energy in and launches the
        // aircraft off the ground on touchdown.
        let normal = (GROUND_K * m * depth + GROUND_C * m * sink_rate).max(0.0);
        force_world.z -= normal;

        let horiz = Vec3::new(self.state.velocity.x, self.state.velocity.y, 0.0);
        let speed = horiz.length();
        if speed > 1e-6 {
            let f = (GROUND_FRICTION * normal).min(m * speed / 0.02);
            force_world = force_world.sub(horiz.normalize().scale(f));
        }
        // On the ground the airframe cannot keep rotating freely about its
        // contact, so damp the body rates rather than let a wing on the deck
        // spin the aircraft up.
        moment_bf = moment_bf.sub(self.state.angular_velocity.scale(m * 0.5));
        (force_world, moment_bf)
    }
}

fn mid_pwm(a: &Airframe) -> f64 {
    0.5 * (a.pwm.min + a.pwm.max)
}

impl SimVehicle for VtolVehicle {
    fn id(&self) -> &str {
        &self.id
    }

    fn step(&mut self, pwm: &[f64], dt: f64) -> VehicleState {
        let clamped = dt.clamp(1e-4, 0.05);
        let substeps = ((clamped / MAX_SUBSTEP).ceil() as i64).max(1) as usize;
        let sub = clamped / substeps as f64;
        for _ in 0..substeps {
            self.substep(pwm, sub);
        }
        self.state
    }

    fn reset(&mut self) {
        self.state = initial_state();
        for r in self.rotors.iter_mut() {
            *r = RotorState::default();
        }
        self.contact_force = Vec3::zero();
        self.neighbor_wake.clear();
    }

    fn home(&self) -> HomeLocation {
        self.home
    }

    fn battery_voltage(&self) -> Option<f64> {
        Some(self.voltage)
    }

    fn battery_reading(&self) -> Option<(f64, f64)> {
        Some((self.voltage, self.last_current))
    }

    fn throttle(&self) -> f64 {
        self.last_throttle
    }

    fn motor_telemetry(&self) -> Option<(Vec<f64>, Vec<f64>)> {
        let thrust = self.diag.rotors.iter().map(|r| r.thrust).collect();
        let rpm = self.diag.omega.iter().map(|w| w * 60.0 / (2.0 * std::f64::consts::PI)).collect();
        Some((thrust, rpm))
    }

    // ─── Shared multi-vehicle world ────────────────────────────────────────

    fn set_neighbor_wake(&mut self, sources: Vec<RotorWake>) {
        self.neighbor_wake = sources;
    }

    fn set_contact_force(&mut self, force: Vec3) {
        self.contact_force = force;
    }

    fn rotor_wakes(&self) -> Vec<RotorWake> {
        let att = self.state.attitude;
        self.diag
            .rotors
            .iter()
            .enumerate()
            .filter(|(_, o)| o.slipstream > 0.1)
            .map(|(i, o)| {
                let r = &self.airframe.rotors[i];
                let axis_bf = r.tilted_axis(self.rotors[i].tilt);
                RotorWake {
                    origin: self.state.position.add(att.rotate_body_to_world(r.position)),
                    // The wake travels opposite the thrust.
                    axis: att.rotate_body_to_world(axis_bf.scale(-1.0)),
                    w: o.slipstream,
                    radius: r.radius,
                }
            })
            .collect()
    }

    fn rigid_state(&self) -> Option<(Vec3, Vec3, f64)> {
        // Bounding radius from the outermost rotor hub plus its own radius, or
        // from the wing tips on an unpowered airframe, which has no rotors to
        // measure and would otherwise report a constant.
        let rotor_reach = self
            .airframe
            .rotors
            .iter()
            .map(|r| r.position.length() + r.radius)
            .fold(0.0f64, f64::max);
        let wing_reach = self
            .airframe
            .surfaces
            .iter()
            .map(|s| s.position.length())
            .fold(0.0f64, f64::max);
        Some((self.state.position, self.state.velocity, rotor_reach.max(wing_reach).max(0.2)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::airframe::AirframeSpec;

    /// A 5 kg lift+cruise quadplane: four lift rotors, one pusher, a wing with
    /// elevons and a fin. Written the way a partner would write it, from
    /// geometry, with no coefficient anywhere.
    const QUADPLANE: &str = include_str!("test_quadplane.json");

    fn quadplane() -> VtolVehicle {
        let spec = AirframeSpec::from_json(QUADPLANE).expect("spec parses");
        let af = spec.build().expect("spec builds");
        VtolVehicle::new("q1", af, HomeLocation { lat: 42.09, lng: 19.09, alt: 10.0, heading: 0.0 })
    }

    /// PWM frame: lift throttle on 0-3, elevon 4, elevator 5, pusher 6.
    fn pwm(lift: f64, pusher: f64) -> Vec<f64> {
        let l = 1000.0 + 1000.0 * lift;
        let p = 1000.0 + 1000.0 * pusher;
        vec![l, l, l, l, 1500.0, 1500.0, p]
    }

    fn fly(v: &mut VtolVehicle, cmd: &[f64], secs: f64) {
        let dt = 0.0025;
        for _ in 0..(secs / dt) as usize {
            v.step(cmd, dt);
        }
    }

    /// Altitude above the datum (m). NED z is down.
    fn alt(v: &VtolVehicle) -> f64 {
        -v.state().position.z
    }

    // ─── The description ────────────────────────────────────────────────────

    #[test]
    fn the_airframe_builds_from_its_json() {
        let spec = AirframeSpec::from_json(QUADPLANE).expect("parses");
        let af = spec.build().expect("builds");
        // Wing 8x2, tailplane 4x2, fin 3x1 (not mirrored).
        assert_eq!(af.surfaces.len(), 16 + 8 + 3);
        assert_eq!(af.rotors.len(), 5);
        // Wing area: two panels of ~0.85 m x ~0.23 m mean chord.
        assert!((0.30..0.45).contains(&af.wing_area), "wing area {}", af.wing_area);
        // A 5 kg VTOL on four 15 inch discs: a normal disc loading.
        assert!((80.0..250.0).contains(&af.disc_loading()), "disc loading {}", af.disc_loading());
        assert!((100.0..250.0).contains(&af.wing_loading()), "wing loading {}", af.wing_loading());
    }

    /// A missing airfoil must fail the build, not silently substitute one. A
    /// default here is a sim flying a different aircraft than the customer
    /// described, and never saying so.
    #[test]
    fn an_undefined_airfoil_is_an_error_not_a_default() {
        let bad = QUADPLANE.replace("\"airfoil\": \"wing\"", "\"airfoil\": \"typo\"");
        let e = AirframeSpec::from_json(&bad).unwrap().build().unwrap_err();
        assert!(format!("{e}").contains("typo"), "{e}");
    }

    #[test]
    fn impossible_mass_and_inertia_are_rejected() {
        for bad in ["\"mass\": 0.0", "\"mass\": -5.0"] {
            let j = QUADPLANE.replace("\"mass\": 5.0", bad);
            assert!(AirframeSpec::from_json(&j).unwrap().build().is_err(), "{bad} accepted");
        }
        let j = QUADPLANE.replace("\"inertia\": [0.28, 0.32, 0.52]", "\"inertia\": [0.0, 0.32, 0.52]");
        assert!(AirframeSpec::from_json(&j).unwrap().build().is_err());
    }

    /// Elevons: one channel, opposite gains left and right. The mirrored panel
    /// must actually get the opposite sign or the aircraft has no roll control.
    #[test]
    fn mirrored_panels_get_the_opposite_control_gain() {
        let af = AirframeSpec::from_json(QUADPLANE).unwrap().build().unwrap();
        let gains: Vec<f64> = af
            .surfaces
            .iter()
            .filter_map(|s| s.control.filter(|c| c.channel == 4).map(|c| c.gain))
            .collect();
        assert!(gains.iter().any(|g| *g > 0.0) && gains.iter().any(|g| *g < 0.0),
            "elevon gains {gains:?}");
        // The elevator (channel 5) is symmetric: both sides the same sign.
        let el: Vec<f64> = af
            .surfaces
            .iter()
            .filter_map(|s| s.control.filter(|c| c.channel == 5).map(|c| c.gain))
            .collect();
        assert!(el.iter().all(|g| *g > 0.0), "elevator gains {el:?}");
    }

    /// Washout must actually reach the strips: the tip is rigged lower than the
    /// root, which is what makes a real wing drop gently instead of departing.
    #[test]
    fn twist_reaches_the_outboard_strips() {
        let af = AirframeSpec::from_json(QUADPLANE).unwrap().build().unwrap();
        let wing: Vec<f64> = af.surfaces[..16].iter().map(|s| s.incidence).collect();
        let root = wing[0];
        let tip = wing[7];
        assert!(tip < root, "tip {tip} should be rigged below root {root}");
        assert!(root > 0.0, "root incidence {root} should be positive");
    }

    // ─── Hover ──────────────────────────────────────────────────────────────

    /// Nothing works unless it holds itself up. Bisect for the throttle that
    /// hovers, which also proves thrust is monotone in throttle.
    #[test]
    fn it_hovers_at_a_findable_throttle() {
        let drift = |t: f64| {
            let mut v = quadplane();
            v.seed_rotors(t);
            v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -50.0), ..initial_state() });
            fly(&mut v, &pwm(t, 0.0), 2.0);
            alt(&v) - 50.0
        };
        assert!(drift(0.2) < 0.0, "20% throttle should sink");
        assert!(drift(1.0) > 0.0, "full throttle should climb");
        let (mut lo, mut hi) = (0.2, 1.0);
        for _ in 0..18 {
            let mid = 0.5 * (lo + hi);
            if drift(mid) < 0.0 { lo = mid } else { hi = mid }
        }
        let hover = 0.5 * (lo + hi);
        assert!((0.25..0.85).contains(&hover), "hover throttle {hover}");
        assert!(drift(hover).abs() < 1.0, "drifted {} m in 2 s at hover", drift(hover));
    }

    /// A hovering quadplane at equal throttle must be in MOMENT BALANCE. Not
    /// that it holds attitude for three seconds: no multirotor does that open
    /// loop, and asserting it would only be asserting that ArduPilot is absent.
    /// What has to hold is that the AIRFRAME introduces no bias, because a bias
    /// here is indistinguishable from bad tuning once the loop is closed, and it
    /// cannot be tuned out.
    ///
    /// Evaluated on a single sub-step from rest. Forces are computed at the
    /// state on entry, so one sub-step reads the pristine layout; running longer
    /// mixes in real dynamics (the pusher's torque starts a roll, which reaches
    /// the lift rotors as a genuine flow difference) and turns a clean symmetry
    /// check into a hunt for the right epsilon.
    #[test]
    fn a_hovering_quadplane_is_in_moment_balance() {
        let mut v = quadplane();
        v.seed_rotors(0.55);
        v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -50.0), ..initial_state() });
        v.step(&pwm(0.55, 0.0), 1e-4);
        let d = v.diagnostics_full();
        // Lift rotors only. The pusher is a SINGLE prop, so its reaction torque
        // genuinely rolls the aircraft with nothing to cancel it; that is real
        // and is asserted separately.
        let m: Vec3 = d.rotors[..4].iter().fold(Vec3::zero(), |a, r| a.add(r.moment_bf));
        assert!(d.rotors[..4].iter().all(|r| r.thrust > 1.0), "rotors not lifting");
        // Symmetric layout: no bias in any axis. The wing-mirroring bug this
        // caught showed up here at 2.6% of roll authority.
        assert!(m.x.abs() < 1e-9, "roll bias {}", m.x);
        assert!(m.y.abs() < 1e-9, "pitch bias {}", m.y);
        // Counter-rotating pairs: the reaction torques cancel outright.
        assert!(m.z.abs() < 1e-9, "yaw bias {}", m.z);
        // The wing is centred, so it contributes no rolling moment either.
        assert!(d.aero.moment_bf.x.abs() < 1e-9, "wing roll bias {}", d.aero.moment_bf.x);
    }

    /// A lone pusher prop rolls the aircraft against its own torque, with
    /// nothing to cancel it. Every single-engine aircraft does this and the
    /// customer's controller has to trim it out, so the sim has to show it.
    #[test]
    fn a_single_pusher_prop_rolls_the_airframe() {
        let mut v = quadplane();
        v.seed_rotors(0.0);
        v.set_state(VehicleState {
            position: Vec3::new(0.0, 0.0, -100.0),
            velocity: Vec3::new(20.0, 0.0, 0.0),
            ..initial_state()
        });
        fly(&mut v, &pwm(0.0, 0.9), 1.0);
        let pusher = v.diagnostics_full().rotors[4];
        assert!(pusher.torque > 0.0, "pusher not loaded: {}", pusher.torque);
        // Reaction opposes its rotation, about the thrust axis, which is roll.
        assert!(pusher.moment_bf.x < 0.0, "no roll reaction: {}", pusher.moment_bf.x);
    }

    /// The mirrored-panel bug this caught: the wing root offset was applied with
    /// the same sign to both sides, putting the whole wing off centre. It showed
    /// up as a slow roll in hover with no asymmetry visible anywhere.
    #[test]
    fn the_wing_is_centred_on_the_airframe() {
        let af = AirframeSpec::from_json(QUADPLANE).unwrap().build().unwrap();
        let y_sum: f64 = af.surfaces.iter().map(|s| s.position.y * s.area).sum();
        assert!(y_sum.abs() < 1e-9, "wing area centroid is off centre by {y_sum}");
        // The fin runs UP, not sideways: a vertical panel is a wing at 90 deg of
        // dihedral, and laying its strips along y puts the surface in the wrong
        // place while every force it produces still looks right on its own.
        let fin = &af.surfaces[af.surfaces.len() - 3..];
        assert!(fin.iter().all(|s| s.position.y.abs() < 1e-9), "fin is off the centreline");
        assert!(fin.iter().all(|s| s.position.z < 0.0), "fin does not extend upward");
        let spread = fin[2].position.z - fin[0].position.z;
        assert!(spread < -0.05, "fin strips are not stacked vertically: {spread}");
    }

    /// The rotors have to spool through a real inertia, not snap to a commanded
    /// speed, or an aborted transition cannot be modelled at all.
    #[test]
    fn rotors_spool_rather_than_snap() {
        let mut v = quadplane();
        v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -50.0), ..initial_state() });
        v.step(&pwm(1.0, 0.0), 0.0025);
        let early: f64 = v.diagnostics_full().omega[..4].iter().sum::<f64>() / 4.0;
        fly(&mut v, &pwm(1.0, 0.0), 1.0);
        let settled: f64 = v.diagnostics_full().omega[..4].iter().sum::<f64>() / 4.0;
        assert!(early < settled * 0.5, "snapped: {early} then {settled}");
        assert!(settled > 500.0, "never reached speed: {settled}");
    }

    // ─── Wing-borne flight and transition ───────────────────────────────────

    /// The wing must carry the aircraft at cruise. This is the claim the whole
    /// strip model exists to support, so it is asserted against weight.
    #[test]
    fn the_wing_carries_the_aircraft_at_cruise_speed() {
        let mut v = quadplane();
        v.set_state(VehicleState {
            position: Vec3::new(0.0, 0.0, -100.0),
            velocity: Vec3::new(22.0, 0.0, 0.0),
            ..initial_state()
        });
        v.step(&pwm(0.0, 0.6), 0.0025);
        let d = v.diagnostics_full();
        let weight = 5.0 * 9.80665;
        // Aerodynamic lift is -z in body frame; at level attitude that is up.
        let lift = -d.aero_force_bf.z;
        assert!(lift > 0.5 * weight, "wing lift {lift} N against {weight} N of weight");
        assert!(lift < 3.0 * weight, "implausibly high lift {lift} N");
        assert!(d.stalled_fraction < 0.05, "wing stalled in cruise: {}", d.stalled_fraction);
    }

    /// Lift goes as the square of speed, which is the property that makes a
    /// stall speed exist.
    #[test]
    fn wing_lift_grows_with_the_square_of_airspeed() {
        let lift_at = |u: f64| {
            let mut v = quadplane();
            v.set_state(VehicleState {
                position: Vec3::new(0.0, 0.0, -100.0),
                velocity: Vec3::new(u, 0.0, 0.0),
                ..initial_state()
            });
            v.step(&pwm(0.0, 0.0), 0.0025);
            -v.diagnostics_full().aero_force_bf.z
        };
        let a = lift_at(10.0);
        let b = lift_at(20.0);
        assert!(a > 0.0, "no lift at 10 m/s");
        assert!((3.4..4.6).contains(&(b / a)), "doubling speed gave {}x lift", b / a);
    }

    /// The transition itself: start hovering on the lift rotors, run the pusher
    /// up, and the wing takes the weight over as speed builds. This is the
    /// manoeuvre the customer is buying, so it is flown end to end rather than
    /// inferred from the pieces.
    #[test]
    fn it_transitions_from_rotor_borne_to_wing_borne_flight() {
        let mut v = quadplane();
        v.seed_rotors(0.55);
        v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -100.0), ..initial_state() });
        fly(&mut v, &pwm(0.55, 0.0), 1.0);

        let hover_wing = -v.diagnostics_full().aero_force_bf.z;
        assert!(hover_wing.abs() < 5.0, "wing should carry nothing in hover, got {hover_wing} N");

        // Run the pusher up and back the lift rotors off as speed builds.
        let dt = 0.0025;
        for i in 0..(12.0 / dt) as usize {
            let t = i as f64 * dt / 12.0;
            let lift = (0.55 * (1.0 - t)).max(0.0);
            fly_one(&mut v, &pwm(lift, 0.75), dt);
        }

        let d = v.diagnostics_full();
        let weight = 5.0 * 9.80665;
        assert!(d.airspeed > 15.0, "never accelerated: {} m/s", d.airspeed);
        let wing_lift = -d.aero_force_bf.z;
        assert!(wing_lift > 0.4 * weight, "wing took only {wing_lift} N of {weight} N");
        // And the lift rotors are done.
        let rotor_lift: f64 = -d.rotor_force_bf.z;
        assert!(rotor_lift < wing_lift, "rotors {rotor_lift} still out-lifting wing {wing_lift}");
        assert!(alt(&v) > 40.0, "fell out of the sky during transition: {} m", alt(&v));
    }

    fn fly_one(v: &mut VtolVehicle, cmd: &[f64], dt: f64) {
        v.step(cmd, dt);
    }

    /// The pusher must actually accelerate the aircraft.
    #[test]
    fn the_pusher_accelerates_the_aircraft() {
        let mut v = quadplane();
        v.set_state(VehicleState {
            position: Vec3::new(0.0, 0.0, -100.0),
            velocity: Vec3::new(18.0, 0.0, 0.0),
            ..initial_state()
        });
        fly(&mut v, &pwm(0.0, 0.9), 3.0);
        assert!(v.state().velocity.x > 18.0, "did not accelerate: {}", v.state().velocity.x);
    }

    // ─── Coupling ───────────────────────────────────────────────────────────

    /// A wing behind a tractor rotor must be immersed in its wake. This is the
    /// coupling the whole module is arranged around: it is what keeps a
    /// tiltrotor's wing flying at zero airspeed.
    ///
    /// Built as its own geometry rather than borrowed from the quadplane,
    /// because that airframe carries its rotors on booms that deliberately
    /// clear the wing, which is the normal lift+cruise layout. Asserting wash
    /// there would have been asserting a property the aircraft does not have.
    const TRACTOR: &str = r#"{
      "name": "tractor", "mass": 3.0, "inertia": [0.2, 0.2, 0.3],
      "airfoils": { "s": { "cl_alpha": 6.0, "alpha_stall_deg": 13.0, "alpha_stall_neg_deg": -13.0 } },
      "wings": [ { "name": "wing", "root": [0.0, 0.05, 0.0], "semi_span": 0.7,
                   "chord_root": 0.22, "chord_tip": 0.18, "airfoil": "s", "strips": 8 } ],
      "rotors": [ { "name": "tractor", "position": [0.45, 0.30, 0.0], "axis": [1,0,0],
                    "radius": 0.20, "chord_root": 0.03, "chord_tip": 0.018,
                    "pitch_root_deg": 26.0, "pitch_tip_deg": 10.0, "airfoil": "s",
                    "inertia": 6e-5, "kv": 500.0, "resistance": 0.07, "throttle_channel": 0 } ]
    }"#;

    #[test]
    fn a_wing_behind_a_tractor_rotor_is_immersed_in_its_wake() {
        let af = AirframeSpec::from_json(TRACTOR).unwrap().build().unwrap();
        let washed: Vec<f64> = (0..af.surfaces.len())
            .map(|k| af.wash_fraction(k, 0, 0.0))
            .collect();
        assert!(washed.iter().any(|f| *f > 0.9), "no strip fully immersed: {washed:?}");
        assert!(washed.iter().any(|f| *f == 0.0), "every strip immersed, wake is unbounded");
        // Partial immersion at the wake edge, so a tilt sweep does not switch
        // the wash on as a step and show up as a rolling transient.
        assert!(washed.iter().any(|f| *f > 0.0 && *f < 1.0), "no feathered edge: {washed:?}");
    }

    /// The complement, and a real property of the lift+cruise layout: its booms
    /// hold the lift rotors clear of the wing, so hover costs no download.
    #[test]
    fn the_quadplanes_booms_hold_its_rotors_clear_of_the_wing() {
        let af = AirframeSpec::from_json(QUADPLANE).unwrap().build().unwrap();
        let any = (0..af.surfaces.len())
            .any(|k| (0..4).any(|i| af.wash_fraction(k, i, 0.0) > 0.0));
        assert!(!any, "a lift rotor is blowing on the wing; that is a download penalty in hover");
    }

    /// Tilting a rotor sweeps its wake across the airframe. Freezing the wash
    /// geometry at build time would delete exactly this, and it is what a
    /// tiltrotor transition consists of.
    #[test]
    fn tilting_a_rotor_moves_its_wash() {
        let af = AirframeSpec::from_json(TRACTOR).unwrap().build().unwrap();
        let total = |tilt: f64| -> f64 {
            (0..af.surfaces.len()).map(|k| af.wash_fraction(k, 0, tilt)).sum()
        };
        let upright = total(0.0);
        let tilted = total(std::f64::consts::FRAC_PI_2);
        assert!((upright - tilted).abs() > 1e-9, "wash unchanged by tilt: {upright} vs {tilted}");
    }

    /// Fuselage drag has to be per-axis: a fuselage is far draggier sideways
    /// than forwards, and one scalar cannot say so.
    #[test]
    fn fuselage_drag_is_per_axis() {
        let drag = |vel: Vec3| {
            let mut v = quadplane();
            v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -100.0), velocity: vel, ..initial_state() });
            v.step(&pwm(0.0, 0.0), 0.0025);
            v.diagnostics_full().fuselage_drag_bf
        };
        let fwd = drag(Vec3::new(20.0, 0.0, 0.0)).x;
        let side = drag(Vec3::new(0.0, 20.0, 0.0)).y;
        assert!(fwd > 0.0 && side > 0.0);
        assert!(side > 2.0 * fwd, "sideways drag {side} should far exceed forward {fwd}");
        // Drag opposes motion in both directions.
        assert!(drag(Vec3::new(-20.0, 0.0, 0.0)).x < 0.0);
    }

    // ─── Ground and robustness ──────────────────────────────────────────────

    #[test]
    fn it_sits_on_the_ground_instead_of_falling_through_it() {
        let mut v = quadplane();
        v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -0.5), ..initial_state() });
        fly(&mut v, &pwm(0.0, 0.0), 4.0);
        let a = alt(&v);
        assert!(a > -0.2, "sank through the ground to {a} m");
        assert!(a < 0.2, "floating at {a} m with no throttle");
        assert!(v.state().velocity.length() < 1.0, "still moving: {:?}", v.state().velocity);
    }

    /// A NaN anywhere ends the run and takes the customer's test with it, so
    /// every corner a VTOL visits is swept.
    #[test]
    fn no_attitude_or_airspeed_produces_a_non_finite_state() {
        use std::f64::consts::PI;
        for (roll, pitch) in [(0.0, 0.0), (PI / 2.0, 0.0), (0.0, PI / 2.0), (PI, 0.0), (0.0, -PI / 2.0)] {
            for speed in [0.0, 5.0, 35.0, -20.0] {
                let mut v = quadplane();
                v.seed_rotors(0.6);
                v.set_state(VehicleState {
                    position: Vec3::new(0.0, 0.0, -200.0),
                    velocity: Vec3::new(speed, speed * 0.3, -speed * 0.2),
                    attitude: crate::math::Quat::from_euler(roll, pitch, 0.4),
                    angular_velocity: Vec3::new(2.0, -1.5, 1.0),
                    ..initial_state()
                });
                fly(&mut v, &pwm(0.7, 0.5), 0.5);
                let s = v.state();
                for c in [s.position.x, s.position.y, s.position.z,
                          s.velocity.x, s.velocity.y, s.velocity.z,
                          s.angular_velocity.x, s.angular_velocity.y, s.angular_velocity.z,
                          s.attitude.w, s.attitude.x, s.attitude.y, s.attitude.z] {
                    assert!(c.is_finite(), "non-finite at roll {roll} pitch {pitch} speed {speed}");
                }
            }
        }
    }

    #[test]
    fn reset_returns_it_to_the_initial_state() {
        let mut v = quadplane();
        v.seed_rotors(0.8);
        fly(&mut v, &pwm(0.8, 0.5), 1.0);
        assert!(v.state().position.length() > 0.0);
        v.reset();
        assert_eq!(v.state().position, Vec3::zero());
        assert_eq!(v.state().velocity, Vec3::zero());
    }

    // ─── Swarm ──────────────────────────────────────────────────────────────

    /// The vehicle has to satisfy the multi-vehicle hooks the shared world
    /// already drives, or swarm silently does nothing for VTOLs.
    #[test]
    fn it_publishes_wake_and_a_rigid_state_for_the_shared_world() {
        let mut v = quadplane();
        v.seed_rotors(0.7);
        v.set_state(VehicleState { position: Vec3::new(10.0, 20.0, -50.0), ..initial_state() });
        fly(&mut v, &pwm(0.7, 0.0), 0.5);

        let wakes = v.rotor_wakes();
        assert!(!wakes.is_empty(), "a hovering VTOL must publish wake sources");
        for w in &wakes {
            assert!(w.w > 0.0 && w.radius > 0.0);
            // The wake blows DOWNWARD from a lift rotor: NED +z.
            if w.axis.z > 0.5 {
                assert!(w.origin.z < 0.0, "wake origin above ground");
            }
            assert!(w.origin.x.is_finite() && w.axis.x.is_finite());
        }
        let (p, vel, r) = v.rigid_state().expect("rigid state");
        assert!((p.x - 10.0).abs() < 5.0 && r > 0.3, "bounds {r}");
        assert!(vel.z.is_finite());
    }

    /// A neighbour's downwash is flow this vehicle genuinely flies through, so
    /// it must reach both the rotors and the wing.
    #[test]
    fn a_neighbours_downwash_changes_the_flight() {
        let run = |wake: Vec<RotorWake>| {
            let mut v = quadplane();
            v.seed_rotors(0.6);
            v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -50.0), ..initial_state() });
            v.set_neighbor_wake(wake);
            fly(&mut v, &pwm(0.6, 0.0), 1.0);
            alt(&v)
        };
        let clean = run(Vec::new());
        // A big rotor directly above, blowing down onto it.
        let above = run(vec![RotorWake {
            origin: Vec3::new(0.0, 0.0, -54.0),
            axis: Vec3::new(0.0, 0.0, 1.0),
            w: 14.0,
            radius: 0.6,
        }]);
        assert!((clean - above).abs() > 1e-6, "downwash had no effect: {clean} vs {above}");
        assert!(above < clean, "being flown over should push it down: {above} vs {clean}");
    }

    #[test]
    fn contact_force_from_the_shared_world_is_applied() {
        let mut v = quadplane();
        v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -50.0), ..initial_state() });
        v.set_contact_force(Vec3::new(60.0, 0.0, 0.0));
        fly(&mut v, &pwm(0.0, 0.0), 0.5);
        assert!(v.state().velocity.x > 1.0, "contact force ignored: {}", v.state().velocity.x);
    }

    #[test]
    fn telemetry_reports_per_rotor_thrust_and_rpm() {
        let mut v = quadplane();
        v.seed_rotors(0.7);
        fly(&mut v, &pwm(0.7, 0.3), 0.5);
        let (thrust, rpm) = v.motor_telemetry().expect("telemetry");
        assert_eq!(thrust.len(), 5);
        assert_eq!(rpm.len(), 5);
        assert!(thrust[0] > 0.0 && rpm[0] > 1000.0, "thrust {} rpm {}", thrust[0], rpm[0]);
        assert!(v.battery_reading().unwrap().1 > 0.0, "no current drawn");
    }
}
