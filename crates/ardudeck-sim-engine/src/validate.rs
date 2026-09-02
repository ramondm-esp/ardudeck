//! Replay a real flight against the model, and report where they disagree.
//!
//! This is the part a partner actually bets an airframe on. A model is
//! trustworthy because someone showed its error against real flights, not
//! because of which equations are in it, and "reliable enough to test before a
//! first flight" is a claim about MEASURED ERROR BOUNDS or it is not a claim at
//! all.
//!
//! ## The trap this is built around
//!
//! The obvious harness feeds the logged servo outputs in, integrates for the
//! length of the flight, and compares trajectories. It does not work, and it
//! does not work for a reason no amount of model quality fixes: there is no
//! feedback. The real aircraft was being flown by a controller reacting to
//! disturbances the log does not contain. Two trajectories from the same initial
//! state under the same open-loop inputs diverge exponentially, so after thirty
//! seconds the comparison measures Lyapunov divergence and not the model. A
//! perfect model scores badly and a bad one scores no worse, which makes the
//! number actively misleading.
//!
//! So there are three modes, and the default is the one that isolates the model:
//!
//! - **Derivative** (default). At every logged sample, SET the state from the
//!   log and compare the instantaneous specific force and angular acceleration
//!   against what was measured. No integration at all, so no divergence, and the
//!   error is attributable to the force model alone. This is what to quote.
//! - **Window**. Re-initialise from the log every `horizon` seconds and compare
//!   at the end of each window. Answers "how far ahead can this predict", which
//!   is the question behind flying a new airframe in sim first.
//! - **FreeRun**. Integrate the whole log open loop. Included because it will be
//!   asked for, and reported with its divergence warning attached, because on
//!   its own it mostly measures the horizon rather than the model.

use crate::copter::VehicleState;
use crate::fdm_server::SimVehicle;
use crate::math::{Quat, Vec3};
use crate::vtol::VtolVehicle;

/// One logged sample: what the flight controller commanded and what the
/// aircraft actually did. This is the shape an ArduPilot `.bin` reduces to:
/// RCOU for the servo outputs, the EKF position and velocity, ATT for attitude,
/// IMU for specific force and angular rate.
#[derive(Debug, Clone)]
pub struct RecordFrame {
    /// Seconds since the start of the record.
    pub t: f64,
    /// Servo outputs (PWM), in the same channel order the airframe spec uses.
    pub pwm: Vec<f64>,
    /// Measured position, world NED relative to home (m).
    pub position: Vec3,
    /// Measured velocity, world NED (m/s).
    pub velocity: Vec3,
    pub attitude: Quat,
    /// Measured body rates (rad/s).
    pub gyro: Vec3,
    /// Measured specific force, body frame (m/s^2). What the accelerometer
    /// reads, so gravity is excluded. Directly logged, which is why the
    /// derivative mode compares against it rather than differentiating position
    /// twice and comparing against noise.
    pub accel_body: Vec3,
    /// Measured airspeed (m/s), if the aircraft carries a sensor.
    pub airspeed: Option<f64>,
    /// Wind estimate at this sample, world NED (m/s). The EKF's, when the log
    /// has one.
    pub wind: Option<Vec3>,
    /// Measured rotor speeds (rad/s), in airframe spec order, from ESC
    /// telemetry.
    ///
    /// Rotor speed is a STATE, and it is the one state a flight log most often
    /// omits. Without it the replay has to spool its own rotors, and the
    /// resulting difference is physical, not numerical: it swamped the model
    /// error by more than an order of magnitude the first time this harness was
    /// run against its own output. With ESC telemetry the comparison is
    /// state-complete; without it, `Replay::warmup` is what keeps the number
    /// honest. If a partner can log one extra field, this is the field.
    pub rpm: Option<Vec<f64>>,
}

#[derive(Debug, Clone)]
pub struct FlightRecord {
    pub name: String,
    pub frames: Vec<RecordFrame>,
}

/// How the replay is scored.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    /// Score the force model, with no integration. The default and the one to
    /// quote.
    Derivative,
    /// Re-initialise from the log every `horizon` seconds; score the prediction
    /// at the end of each window.
    Window { horizon: f64 },
    /// Integrate the whole record open loop. Diverges by construction.
    FreeRun,
}

impl Default for Mode {
    fn default() -> Self {
        Mode::Derivative
    }
}

/// Coarse flight phase, detected from the record rather than declared, so a
/// customer does not have to annotate their logs before getting a report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Phase {
    Hover,
    Transition,
    Cruise,
}

impl Phase {
    pub fn name(&self) -> &'static str {
        match self {
            Phase::Hover => "hover",
            Phase::Transition => "transition",
            Phase::Cruise => "cruise",
        }
    }
}

/// Airspeed below which the aircraft is rotor-borne, and above which it is
/// wing-borne. Between them is the transition, which is where a VTOL model is
/// hardest and where the error should be reported SEPARATELY: a report that
/// averages a long cruise with a four-second transition hides the only part
/// anyone is worried about.
const HOVER_MAX_MS: f64 = 6.0;
const CRUISE_MIN_MS: f64 = 16.0;

pub fn phase_of(airspeed: f64) -> Phase {
    if airspeed < HOVER_MAX_MS {
        Phase::Hover
    } else if airspeed > CRUISE_MIN_MS {
        Phase::Cruise
    } else {
        Phase::Transition
    }
}

/// Running error accumulator. RMS and worst case together, because a model can
/// have an excellent RMS and still be unusable if it is wrong by 3 g once per
/// flight, and that once is the event the customer cares about.
#[derive(Debug, Clone, Default)]
pub struct ErrorStat {
    pub n: usize,
    sum_sq: f64,
    pub worst: f64,
    pub worst_t: f64,
    sum_ref_sq: f64,
}

impl ErrorStat {
    pub fn push(&mut self, err: f64, reference: f64, t: f64) {
        if !err.is_finite() {
            return;
        }
        self.n += 1;
        self.sum_sq += err * err;
        self.sum_ref_sq += reference * reference;
        if err.abs() > self.worst {
            self.worst = err.abs();
            self.worst_t = t;
        }
    }

    pub fn rms(&self) -> f64 {
        if self.n == 0 {
            return 0.0;
        }
        (self.sum_sq / self.n as f64).sqrt()
    }

    /// Error as a fraction of the signal's own RMS. The number that survives
    /// being quoted without units: 4% of the measured accelerations is
    /// meaningful where "0.31 m/s^2" needs the reader to know how hard the
    /// aircraft was manoeuvring.
    pub fn normalised(&self) -> f64 {
        let r = if self.n == 0 { 0.0 } else { (self.sum_ref_sq / self.n as f64).sqrt() };
        if r < 1e-9 {
            return 0.0;
        }
        self.rms() / r
    }
}

/// Errors for one phase, or for the whole record.
#[derive(Debug, Clone, Default)]
pub struct Scores {
    /// Specific force per body axis (m/s^2).
    pub accel: [ErrorStat; 3],
    /// Angular acceleration per body axis (rad/s^2).
    pub ang_accel: [ErrorStat; 3],
    /// Position error (m). Window and FreeRun only; empty in Derivative mode,
    /// which never integrates.
    pub position: ErrorStat,
    /// Velocity error (m/s). Window and FreeRun only.
    pub velocity: ErrorStat,
    /// Attitude error (rad), as the angle of the rotation between measured and
    /// modelled. One number rather than three Euler errors, because Euler
    /// differences are meaningless near the poles and a VTOL goes there.
    pub attitude: ErrorStat,
}

impl Scores {
    pub fn samples(&self) -> usize {
        self.accel[0].n.max(self.position.n)
    }

    /// Worst normalised specific-force error across the three axes: the single
    /// figure of merit for the force model.
    pub fn accel_error(&self) -> f64 {
        self.accel.iter().map(|s| s.normalised()).fold(0.0, f64::max)
    }

    pub fn ang_accel_error(&self) -> f64 {
        self.ang_accel.iter().map(|s| s.normalised()).fold(0.0, f64::max)
    }
}

#[derive(Debug, Clone)]
pub struct Report {
    pub record: String,
    pub mode: Mode,
    pub duration: f64,
    pub overall: Scores,
    /// Scores split by detected flight phase. Reported separately because a
    /// four-second transition averaged into a five-minute cruise disappears,
    /// and the transition is the part a VTOL customer is worried about.
    pub by_phase: Vec<(Phase, Scores)>,
    /// Set when a mode was used whose number cannot be taken at face value.
    pub caveat: Option<String>,
}

impl Report {
    pub fn phase(&self, p: Phase) -> Option<&Scores> {
        self.by_phase.iter().find(|(q, _)| *q == p).map(|(_, s)| s)
    }

    /// Whether every phase's force error is within `tolerance` (as a fraction).
    /// Used to gate CI, so a fidelity regression fails the build instead of
    /// being discovered by a customer.
    pub fn within(&self, tolerance: f64) -> bool {
        self.by_phase
            .iter()
            .all(|(_, s)| s.samples() == 0 || s.accel_error() <= tolerance)
    }
}

impl std::fmt::Display for Report {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "validation: {} ({:?}, {:.1} s)", self.record, self.mode, self.duration)?;
        if let Some(c) = &self.caveat {
            writeln!(f, "  CAVEAT: {c}")?;
        }
        writeln!(f, "  {:<12} {:>7} {:>10} {:>10} {:>10}", "phase", "samples", "accel err", "ang err", "worst a")?;
        for (p, s) in &self.by_phase {
            if s.samples() == 0 {
                continue;
            }
            writeln!(
                f,
                "  {:<12} {:>7} {:>9.1}% {:>9.1}% {:>8.2} m/s2",
                p.name(),
                s.samples(),
                s.accel_error() * 100.0,
                s.ang_accel_error() * 100.0,
                s.accel.iter().map(|a| a.worst).fold(0.0, f64::max),
            )?;
        }
        writeln!(
            f,
            "  {:<12} {:>7} {:>9.1}% {:>9.1}%",
            "OVERALL",
            self.overall.samples(),
            self.overall.accel_error() * 100.0,
            self.overall.ang_accel_error() * 100.0
        )?;
        if self.overall.position.n > 0 {
            writeln!(
                f,
                "  position rms {:.2} m (worst {:.2} at t={:.1})   velocity rms {:.2} m/s   attitude rms {:.1} deg",
                self.overall.position.rms(),
                self.overall.position.worst,
                self.overall.position.worst_t,
                self.overall.velocity.rms(),
                self.overall.attitude.rms().to_degrees()
            )?;
        }
        Ok(())
    }
}

/// Seconds skipped at the start of a record whose frames carry no rotor speeds.
///
/// Long enough for the rotors to reach the speed the logged throttle implies.
/// Scoring across the spool-up would charge the model for a state the log never
/// recorded, which is not a modelling error and cannot be fixed by improving the
/// model.
pub const DEFAULT_WARMUP_S: f64 = 1.5;

/// Replay a record through a vehicle and score it, with the default warm-up.
pub fn replay(vehicle: &mut VtolVehicle, record: &FlightRecord, mode: Mode) -> Report {
    replay_with(vehicle, record, mode, DEFAULT_WARMUP_S)
}

/// Replay with an explicit warm-up period.
pub fn replay_with(
    vehicle: &mut VtolVehicle,
    record: &FlightRecord,
    mode: Mode,
    warmup: f64,
) -> Report {
    let mut overall = Scores::default();
    let mut phases: Vec<(Phase, Scores)> = vec![
        (Phase::Hover, Scores::default()),
        (Phase::Transition, Scores::default()),
        (Phase::Cruise, Scores::default()),
    ];
    let frames = &record.frames;
    let duration = frames.last().map(|f| f.t).unwrap_or(0.0) - frames.first().map(|f| f.t).unwrap_or(0.0);

    let mut window_start = f64::NEG_INFINITY;
    let inertia = vehicle.airframe().inertia;
    // Only frames with no logged rotor speed need the warm-up; a state-complete
    // record is scored from its first sample.
    let has_rpm = frames.first().map(|f| f.rpm.is_some()).unwrap_or(false);
    let score_from = frames.first().map(|f| f.t).unwrap_or(0.0) + if has_rpm { 0.0 } else { warmup.max(0.0) };

    for i in 0..frames.len().saturating_sub(1) {
        let fr = &frames[i];
        let next = &frames[i + 1];
        let dt = next.t - fr.t;
        if !(dt > 0.0) || !dt.is_finite() {
            continue;
        }

        // Fly the aircraft through the air mass the log says it was in. Skipping
        // this scores the model on a wind the aircraft never flew in, which
        // shows up as a bias in cruise and looks like a lift error.
        if let Some(w) = fr.wind {
            let mut env = crate::copter::DEFAULT_ENVIRONMENT;
            env.wind = w;
            vehicle.set_environment(env);
        }

        let reinit = match mode {
            Mode::Derivative => true,
            Mode::Window { horizon } => {
                if fr.t - window_start >= horizon {
                    window_start = fr.t;
                    true
                } else {
                    false
                }
            }
            Mode::FreeRun => i == 0,
        };
        // Rotor speed is a state and `VehicleState` does not carry it, so it is
        // restored separately when the log has it.
        if let Some(rpm) = &fr.rpm {
            vehicle.set_rotor_speeds(rpm);
        }
        if reinit {
            vehicle.set_state(VehicleState {
                position: fr.position,
                velocity: fr.velocity,
                attitude: fr.attitude,
                angular_velocity: fr.gyro,
                accel_body: fr.accel_body,
                current: 0.0,
                timestamp: fr.t,
                load: None,
            });
        }

        let before = vehicle.state();
        vehicle.step(&fr.pwm, dt);
        let after = vehicle.state();

        // Stepped, so the rotors keep evolving, but not scored: during warm-up
        // the model is flying a rotor state the log never recorded.
        if fr.t < score_from {
            continue;
        }

        let phase = phase_of(vehicle.diagnostics_full().airspeed);
        let slot = &mut phases.iter_mut().find(|(p, _)| *p == phase).unwrap().1;

        // Specific force: compare directly against the accelerometer, which is
        // logged. Differentiating position twice would compare the model against
        // differentiation noise.
        let modelled_a = after.accel_body;
        for k in 0..3 {
            let (m, r) = (axis(modelled_a, k), axis(fr.accel_body, k));
            slot.accel[k].push(m - r, r, fr.t);
            overall.accel[k].push(m - r, r, fr.t);
        }

        // Angular acceleration: from the model's own moment, and from the log's
        // gyro difference across this frame.
        let modelled_ang = after.angular_velocity.sub(before.angular_velocity).scale(1.0 / dt);
        let measured_ang = next.gyro.sub(fr.gyro).scale(1.0 / dt);
        let _ = inertia;
        for k in 0..3 {
            let (m, r) = (axis(modelled_ang, k), axis(measured_ang, k));
            slot.ang_accel[k].push(m - r, r, fr.t);
            overall.ang_accel[k].push(m - r, r, fr.t);
        }

        // Trajectory error, meaningful only where the model was allowed to
        // integrate. In Derivative mode the state is overwritten every sample,
        // so a position error there would be identically zero and would read as
        // a perfect score.
        if mode != Mode::Derivative {
            let pe = after.position.sub(next.position);
            slot.position.push(pe.length(), next.position.length(), fr.t);
            overall.position.push(pe.length(), next.position.length(), fr.t);
            let ve = after.velocity.sub(next.velocity);
            slot.velocity.push(ve.length(), next.velocity.length(), fr.t);
            overall.velocity.push(ve.length(), next.velocity.length(), fr.t);
            let ae = attitude_error(after.attitude, next.attitude);
            slot.attitude.push(ae, 1.0, fr.t);
            overall.attitude.push(ae, 1.0, fr.t);
        }
    }

    let caveat = match mode {
        Mode::FreeRun => Some(
            "FreeRun integrates open loop with no feedback, so the trajectory error \
             measures divergence from the absent controller as much as the model. \
             Quote the Derivative accel error instead."
                .to_string(),
        ),
        Mode::Window { horizon } if horizon > 10.0 => Some(format!(
            "a {horizon:.0} s window is long enough that divergence dominates; \
             prefer 1 to 5 s"
        )),
        _ => None,
    };

    Report { record: record.name.clone(), mode, duration, overall, by_phase: phases, caveat }
}

fn axis(v: Vec3, k: usize) -> f64 {
    match k {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// Angle of the rotation between two attitudes (rad). Convention-free, unlike
/// differencing Euler angles, which is meaningless near the poles and a VTOL
/// spends real time there.
pub fn attitude_error(a: Quat, b: Quat) -> f64 {
    let d = a.conjugate().multiply(b).normalize();
    2.0 * d.w.abs().clamp(0.0, 1.0).acos()
}

/// Record a flight FROM the model, for building a reference a later change can
/// be regressed against. This is how a fidelity regression gets caught by CI
/// rather than by a customer.
pub fn record_flight(
    vehicle: &mut VtolVehicle,
    name: &str,
    inputs: &dyn Fn(f64) -> Vec<f64>,
    duration: f64,
    dt: f64,
) -> FlightRecord {
    let mut frames = Vec::new();
    let n = (duration / dt).max(1.0) as usize;
    for i in 0..=n {
        let t = i as f64 * dt;
        let pwm = inputs(t);
        let s = vehicle.state();
        // Captured BEFORE the step, with everything else. Reading it afterwards
        // records the rotor speed at t+dt alongside a rigid-body state at t, and
        // a frame that mixes two instants is not a state the aircraft was ever
        // in. It scored as a model error of 2.3e-3 against a true floor of zero.
        let rpm = vehicle.rotor_speeds();
        vehicle.step(&pwm, dt);
        frames.push(RecordFrame {
            t,
            pwm: pwm.clone(),
            position: s.position,
            velocity: s.velocity,
            attitude: s.attitude,
            gyro: s.angular_velocity,
            // The specific force evaluated AT this frame's state, which is what
            // an accelerometer logs at time t. Taking the state's own
            // `accel_body` instead would record the force from the PREVIOUS
            // frame, and the whole harness would then be scoring the model
            // against itself one sample out of phase.
            accel_body: vehicle.state().accel_body,
            airspeed: Some(vehicle.diagnostics_full().airspeed),
            wind: None,
            rpm: Some(rpm),
        });
    }
    FlightRecord { name: name.to_string(), frames }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::airframe::AirframeSpec;
    use crate::copter::initial_state;
    use crate::fdm_server::HomeLocation;

    fn spec_json() -> String {
        // Reuse the quadplane the vehicle tests fly, so the harness is exercised
        // against the same airframe the physics is.
        include_str!("test_quadplane.json").to_string()
    }

    fn vehicle() -> VtolVehicle {
        let af = AirframeSpec::from_json(&spec_json()).unwrap().build().unwrap();
        VtolVehicle::new(
            "v",
            af,
            HomeLocation { lat: 42.09, lng: 19.09, alt: 10.0, heading: 0.0 },
        )
    }

    fn flying(v: &mut VtolVehicle) {
        v.seed_rotors(0.55);
        v.set_state(VehicleState {
            position: Vec3::new(0.0, 0.0, -120.0),
            ..initial_state()
        });
    }

    /// A transition profile: hover, run the pusher up while backing the lift
    /// rotors off, cruise. Spans all three phases so the report is exercised.
    fn transition_inputs(t: f64) -> Vec<f64> {
        let ramp = ((t - 2.0) / 10.0).clamp(0.0, 1.0);
        let lift = 1000.0 + 1000.0 * (0.55 * (1.0 - ramp));
        let push = 1000.0 + 1000.0 * (0.8 * ramp);
        vec![lift, lift, lift, lift, 1500.0, 1500.0, push]
    }

    // ─── Self-consistency ───────────────────────────────────────────────────

    /// The harness must score a model against its own flight as near-perfect.
    /// If this fails, every other number the harness produces is meaningless,
    /// because the floor is not zero.
    #[test]
    fn a_model_replayed_against_its_own_flight_scores_near_zero() {
        let mut rec_v = vehicle();
        flying(&mut rec_v);
        let record = record_flight(&mut rec_v, "self", &transition_inputs, 16.0, 0.0025);
        assert!(record.frames.len() > 6000, "short record: {}", record.frames.len());

        let mut v = vehicle();
        let r = replay(&mut v, &record, Mode::Derivative);
        assert!(r.overall.samples() > 6000);
        assert!(r.overall.accel_error() < 1e-9, "accel error {:.3e}", r.overall.accel_error());
        assert!(r.overall.ang_accel_error() < 1e-9, "ang error {:.3e}", r.overall.ang_accel_error());
        assert!(r.within(1e-6));
    }

    /// And it must score a DIFFERENT model as worse, or it has no discriminating
    /// power and the near-zero above is meaningless too.
    #[test]
    fn a_wrong_model_scores_worse_in_proportion_to_how_wrong_it_is() {
        let mut rec_v = vehicle();
        flying(&mut rec_v);
        let record = record_flight(&mut rec_v, "truth", &transition_inputs, 16.0, 0.0025);

        let err_for = |mass_scale: f64| {
            let mut spec = AirframeSpec::from_json(&spec_json()).unwrap();
            spec.mass *= mass_scale;
            let af = spec.build().unwrap();
            let mut v = VtolVehicle::new(
                "w",
                af,
                HomeLocation { lat: 42.09, lng: 19.09, alt: 10.0, heading: 0.0 },
            );
            replay(&mut v, &record, Mode::Derivative).overall.accel_error()
        };
        let small = err_for(1.05);
        let big = err_for(1.4);
        assert!(small > 1e-4, "a 5% mass error should be visible, got {small:.3e}");
        assert!(big > small * 2.0, "40% error {big:.3} should dwarf 5% error {small:.3}");
    }

    /// Wing area is a geometry number a partner can get wrong on their form.
    /// The harness has to catch that too, not just mass.
    #[test]
    fn a_geometry_error_is_visible_in_the_report() {
        let mut rec_v = vehicle();
        flying(&mut rec_v);
        let record = record_flight(&mut rec_v, "truth", &transition_inputs, 16.0, 0.0025);

        let mut spec = AirframeSpec::from_json(&spec_json()).unwrap();
        spec.wings[0].semi_span *= 1.25;
        let af = spec.build().unwrap();
        let mut v = VtolVehicle::new("w", af, HomeLocation { lat: 42.09, lng: 19.09, alt: 10.0, heading: 0.0 });
        let r = replay(&mut v, &record, Mode::Derivative);
        assert!(r.overall.accel_error() > 1e-3, "25% more span went unnoticed: {:.3e}", r.overall.accel_error());
        // And it should show up in CRUISE, where the wing carries the aircraft,
        // rather than in hover where the wing does almost nothing.
        // It has to be visible in CRUISE specifically, where the wing carries
        // the aircraft. Not asserted to EXCEED the hover error: errors are
        // normalised by each phase's own signal, and hover's specific force is
        // dominated by the steady 1 g that every model gets right, so the two
        // ratios are not on a common scale.
        let cruise = r.phase(Phase::Cruise).expect("cruise samples");
        assert!(cruise.samples() > 0, "record did not reach cruise");
        assert!(cruise.accel_error() > 1e-2,
            "25% more span barely showed in cruise: {:.4}", cruise.accel_error());
    }

    // ─── The mode design ────────────────────────────────────────────────────

    /// The claim the harness is built on: open-loop replay diverges, and the
    /// derivative mode does not. If FreeRun's trajectory error were comparable,
    /// the extra modes would be ceremony.
    #[test]
    fn free_run_diverges_where_the_derivative_mode_cannot() {
        let mut rec_v = vehicle();
        flying(&mut rec_v);
        let record = record_flight(&mut rec_v, "truth", &transition_inputs, 16.0, 0.0025);

        // A 10% mass error, replayed both ways.
        let build = || {
            let mut spec = AirframeSpec::from_json(&spec_json()).unwrap();
            spec.mass *= 1.10;
            VtolVehicle::new(
                "w",
                spec.build().unwrap(),
                HomeLocation { lat: 42.09, lng: 19.09, alt: 10.0, heading: 0.0 },
            )
        };
        let deriv = replay(&mut build(), &record, Mode::Derivative);
        let free = replay(&mut build(), &record, Mode::FreeRun);
        let window = replay(&mut build(), &record, Mode::Window { horizon: 1.0 });

        // Derivative never integrates, so it reports no trajectory error at all.
        assert_eq!(deriv.overall.position.n, 0, "derivative mode must not integrate");
        // FreeRun accumulates a large position error from a modest model error.
        assert!(free.overall.position.rms() > 1.0,
            "free run should drift: {:.2} m", free.overall.position.rms());
        // A short window keeps it bounded, which is what makes it quotable.
        assert!(window.overall.position.rms() < free.overall.position.rms(),
            "1 s window {:.2} m should beat free run {:.2} m",
            window.overall.position.rms(), free.overall.position.rms());
        // And FreeRun must carry its warning.
        assert!(free.caveat.is_some(), "free run reported without its caveat");
        assert!(deriv.caveat.is_none());
        assert!(replay(&mut build(), &record, Mode::Window { horizon: 30.0 }).caveat.is_some(),
            "an over-long window should be flagged");
    }

    /// The force-model error must NOT grow with record length, or the number is
    /// a function of how long the customer flew rather than of the model.
    #[test]
    fn the_derivative_error_does_not_grow_with_record_length() {
        let err_over = |secs: f64| {
            let mut rec_v = vehicle();
            flying(&mut rec_v);
            let record = record_flight(&mut rec_v, "t", &transition_inputs, secs, 0.0025);
            let mut spec = AirframeSpec::from_json(&spec_json()).unwrap();
            spec.mass *= 1.10;
            let mut v = VtolVehicle::new(
                "w",
                spec.build().unwrap(),
                HomeLocation { lat: 42.09, lng: 19.09, alt: 10.0, heading: 0.0 },
            );
            replay(&mut v, &record, Mode::Derivative).overall.accel_error()
        };
        let short = err_over(6.0);
        let long = err_over(18.0);
        assert!(short > 0.0 && long > 0.0);
        assert!((long / short) < 3.0, "error tripled with length: {short:.4} -> {long:.4}");
    }

    // ─── Reporting ──────────────────────────────────────────────────────────

    #[test]
    fn phases_are_detected_from_airspeed() {
        assert_eq!(phase_of(0.0), Phase::Hover);
        assert_eq!(phase_of(3.0), Phase::Hover);
        assert_eq!(phase_of(10.0), Phase::Transition);
        assert_eq!(phase_of(25.0), Phase::Cruise);
    }

    /// A transition record has to land samples in all three phases, or the
    /// per-phase split silently reports one bucket and looks fine.
    #[test]
    fn a_transition_record_populates_every_phase() {
        let mut rec_v = vehicle();
        flying(&mut rec_v);
        let record = record_flight(&mut rec_v, "t", &transition_inputs, 16.0, 0.0025);
        let mut v = vehicle();
        let r = replay(&mut v, &record, Mode::Derivative);
        for p in [Phase::Hover, Phase::Transition, Phase::Cruise] {
            assert!(r.phase(p).unwrap().samples() > 0, "no samples in {}", p.name());
        }
        let text = format!("{r}");
        assert!(text.contains("hover") && text.contains("transition") && text.contains("cruise"));
        assert!(text.contains("OVERALL"));
    }

    /// `within` is the CI gate: a fidelity regression has to fail the build
    /// rather than be found by a customer.
    #[test]
    fn within_gates_on_the_worst_phase_not_the_average() {
        let mut rec_v = vehicle();
        flying(&mut rec_v);
        let record = record_flight(&mut rec_v, "t", &transition_inputs, 16.0, 0.0025);
        let mut good = vehicle();
        assert!(replay(&mut good, &record, Mode::Derivative).within(0.01));

        let mut spec = AirframeSpec::from_json(&spec_json()).unwrap();
        spec.mass *= 1.5;
        let mut bad = VtolVehicle::new(
            "b",
            spec.build().unwrap(),
            HomeLocation { lat: 42.09, lng: 19.09, alt: 10.0, heading: 0.0 },
        );
        assert!(!replay(&mut bad, &record, Mode::Derivative).within(0.01));
    }

    #[test]
    fn error_stats_report_rms_and_the_worst_case_separately() {
        let mut s = ErrorStat::default();
        for _ in 0..999 {
            s.push(0.01, 1.0, 0.0);
        }
        // One big excursion barely moves the RMS but must be reported.
        s.push(5.0, 1.0, 42.0);
        assert!(s.rms() < 0.20, "one spike swamped the rms: {}", s.rms());
        assert_eq!(s.worst, 5.0);
        assert_eq!(s.worst_t, 42.0);
        assert!((s.normalised() - s.rms()).abs() < 1e-9, "reference rms is 1.0 here");
    }

    #[test]
    fn non_finite_samples_are_dropped_rather_than_poisoning_the_score() {
        let mut s = ErrorStat::default();
        s.push(1.0, 1.0, 0.0);
        s.push(f64::NAN, 1.0, 1.0);
        s.push(f64::INFINITY, 1.0, 2.0);
        assert_eq!(s.n, 1);
        assert!(s.rms().is_finite() && (s.rms() - 1.0).abs() < 1e-12);
    }

    /// Attitude error must be the rotation ANGLE, not an Euler difference:
    /// Euler differences blow up near the poles and a VTOL flies through them.
    #[test]
    fn attitude_error_is_convention_free() {
        use std::f64::consts::PI;
        let a = Quat::from_euler(0.0, 0.0, 0.0);
        assert!(attitude_error(a, a) < 1e-12);
        let b = Quat::from_euler(0.0, 0.0, 0.2);
        assert!((attitude_error(a, b) - 0.2).abs() < 1e-9, "{}", attitude_error(a, b));
        // Straight up, where roll and yaw are degenerate: two attitudes that are
        // the SAME rotation must score zero however their Euler angles read.
        let up1 = Quat::from_euler(0.0, PI / 2.0, 0.0);
        let up2 = Quat::from_euler(0.7, PI / 2.0, 0.7);
        assert!(attitude_error(up1, up2) < 1e-6,
            "gimbal-degenerate pair scored {}", attitude_error(up1, up2));
    }

    /// The aircraft has to be flown through the air mass the log says it was
    /// in. Ignoring the logged wind scores the model against a flight it never
    /// made, and in cruise that reads as a lift error.
    #[test]
    fn the_logged_wind_is_applied_during_replay() {
        let mut rec_v = vehicle();
        flying(&mut rec_v);
        let mut record = record_flight(&mut rec_v, "t", &transition_inputs, 8.0, 0.0025);
        let calm = replay(&mut vehicle(), &record, Mode::Derivative).overall.accel_error();
        for f in record.frames.iter_mut() {
            f.wind = Some(Vec3::new(-9.0, 0.0, 0.0));
        }
        let windy = replay(&mut vehicle(), &record, Mode::Derivative).overall.accel_error();
        assert!(windy > calm + 1e-6,
            "wind ignored: calm {calm:.3e} windy {windy:.3e}");
    }

    #[test]
    fn an_empty_or_degenerate_record_does_not_panic() {
        let empty = FlightRecord { name: "empty".into(), frames: Vec::new() };
        let r = replay(&mut vehicle(), &empty, Mode::Derivative);
        assert_eq!(r.overall.samples(), 0);
        assert!(r.within(0.0));
        // Frames with no time between them are skipped rather than dividing by
        // zero, which is what a log with duplicate timestamps looks like.
        let f = RecordFrame {
            t: 0.0,
            pwm: vec![1500.0; 7],
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            attitude: Quat::identity(),
            gyro: Vec3::zero(),
            accel_body: Vec3::zero(),
            airspeed: None,
            wind: None,
            rpm: None,
        };
        let dup = FlightRecord { name: "dup".into(), frames: vec![f.clone(), f.clone(), f] };
        let r = replay(&mut vehicle(), &dup, Mode::Derivative);
        assert_eq!(r.overall.samples(), 0);
    }
}
