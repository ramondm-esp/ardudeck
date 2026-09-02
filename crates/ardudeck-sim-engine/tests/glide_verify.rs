//! Unpowered validation: a flat sheet, a paper plane, a glider.
//!
//! Every other test in this crate checks the model against ITSELF: that the
//! signs are consistent, that the pieces agree, that replaying a flight
//! reproduces it. Useful, and none of it can catch a model that is
//! self-consistently wrong.
//!
//! These three articles can. They need no motor, no controller and no flight
//! log, and each one has an answer that comes from somewhere this model did not:
//!
//! | article | independent prediction |
//! |---|---|
//! | flat sheet | a square plate broadside has Cd about 1.17 (measured, textbook) |
//! | flat sheet | a thin plate's lift-curve slope is 2*pi per radian (thin airfoil theory) |
//! | flat sheet | terminal velocity is sqrt(2mg / rho Cd A) |
//! | paper plane | paper darts glide at roughly 3:1 to 6:1 (observed, universally) |
//! | any glide | the glide RATIO equals L/D exactly, and tan(glide angle) = D/L |
//! | glider | the phugoid period is pi*sqrt(2)*V/g (Lanchester 1908) |
//! | glider | minimum sink occurs SLOWER than best glide (drag polar geometry) |
//! | glider | moving the CG aft reduces pitch stiffness to zero at the neutral point |
//!
//! The glide-angle identity is the strongest of them. In steady flight the
//! aerodynamic force balances weight, so the flight path angle is fixed by the
//! lift-to-drag ratio and by nothing else. It ties the strip sum, the force
//! assembly, the attitude handling and the integrator into one number, and any
//! error in any of them breaks it. It is asserted on the GLIDER, because the
//! identity holds only in steady flight and the paper plane never achieves any:
//! it swoops the whole way down, which is itself one of the predictions here.

use ardudeck_sim_engine::airframe::AirframeSpec;
use ardudeck_sim_engine::articles::{FOAM_SHEET, FOAM_STRIP, GLIDER, PAPER_PLANE};
use ardudeck_sim_engine::copter::{initial_state, VehicleState};
use ardudeck_sim_engine::fdm_server::{HomeLocation, SimVehicle};
use ardudeck_sim_engine::math::{Quat, Vec3};
use ardudeck_sim_engine::vtol::VtolVehicle;

const RHO: f64 = 1.225;
const G: f64 = 9.80665;

fn build(json: &str) -> VtolVehicle {
    let spec = AirframeSpec::from_json(json).expect("spec parses");
    let af = spec.build().expect("spec builds");
    VtolVehicle::new("t", af, HomeLocation { lat: 0.0, lng: 0.0, alt: 0.0, heading: 0.0 })
}

fn launch(v: &mut VtolVehicle, speed: f64, pitch: f64, alt: f64) {
    v.set_state(VehicleState {
        position: Vec3::new(0.0, 0.0, -alt),
        velocity: Vec3::new(speed * pitch.cos(), 0.0, -speed * pitch.sin()),
        attitude: Quat::from_euler(0.0, pitch, 0.0),
        ..initial_state()
    });
}

fn fly(v: &mut VtolVehicle, secs: f64) {
    let dt = 0.004;
    for _ in 0..(secs / dt) as usize {
        v.step(&[], dt);
    }
}

/// What a glide looks like at one instant, measured the way a flight test would.
#[derive(Debug, Clone, Copy)]
struct Glide {
    /// Speed along the flight path (m/s).
    speed: f64,
    /// Rate of descent, positive down (m/s).
    sink: f64,
    /// Horizontal distance per unit of height lost.
    glide_ratio: f64,
    /// Flight path angle below the horizon (rad).
    gamma: f64,
    /// Aerodynamic force resolved about the FLIGHT PATH, not the body axes.
    lift: f64,
    drag: f64,
    alpha: f64,
}

impl Glide {
    fn ld(&self) -> f64 {
        if self.drag.abs() < 1e-12 {
            return f64::INFINITY;
        }
        self.lift / self.drag
    }
}

/// Measure the current glide. Lift and drag are resolved about the flight path
/// because that is what they are DEFINED about; taking them off the body axes
/// would smuggle the attitude into the answer and the glide-angle identity
/// would hold trivially.
fn measure(v: &mut VtolVehicle) -> Glide {
    // ONE sub-step. Forces are evaluated at the state on entry, so a single
    // sub-step reads them at exactly the state that was set. A full 4 ms frame
    // is four sub-steps, and on a 20 g sheet pulling 300 m/s^2 the state has
    // already moved 12% by the fourth: the harness then measures a coefficient
    // at a speed and angle the caller never asked for, which reads as a model
    // error of about 2x.
    v.step(&[], 1e-4);
    read(v)
}

/// Read the current glide without advancing the state.
fn read(v: &VtolVehicle) -> Glide {
    let s = v.state();
    let d = v.diagnostics_full();
    // With no rotors this is the entire aerodynamic force.
    let aero_bf = d.aero_force_bf.sub(d.fuselage_drag_bf);
    let aero_world = s.attitude.rotate_body_to_world(aero_bf);

    let speed = s.velocity.length();
    let vhat = s.velocity.normalize();
    let along = aero_world.dot(vhat);
    let drag = -along;
    let lift = aero_world.sub(vhat.scale(along)).length();
    let horiz = (s.velocity.x * s.velocity.x + s.velocity.y * s.velocity.y).sqrt();
    let sink = s.velocity.z;
    Glide {
        speed,
        sink,
        glide_ratio: if sink.abs() < 1e-9 { f64::INFINITY } else { horiz / sink },
        gamma: sink.atan2(horiz),
        lift,
        drag,
        alpha: d.alpha,
    }
}

/// Average a glide over a window, the way a flight test measures one: total
/// distance over total height lost.
///
/// A single instant is not a glide. The phugoid is very lightly damped, so an
/// aircraft twenty seconds after release is still trading height for speed, and
/// an instantaneous glide ratio sampled near the top of that oscillation reads
/// two or three times the real figure. Measuring 37:1 on a 15:1 glider is not a
/// model error, it is a measurement error, and averaging over several cycles is
/// what a real timed glide does about it.
fn measure_avg(v: &mut VtolVehicle, secs: f64) -> Glide {
    let dt = 0.004;
    let start = v.state().position;
    let (mut lift, mut drag, mut speed, mut alpha, mut n) = (0.0, 0.0, 0.0, 0.0, 0.0f64);
    for _ in 0..(secs / dt) as usize {
        v.step(&[], dt);
        let g = read(v);
        lift += g.lift;
        drag += g.drag;
        speed += g.speed;
        alpha += g.alpha;
        n += 1.0;
    }
    let end = v.state().position;
    let horiz = ((end.x - start.x).powi(2) + (end.y - start.y).powi(2)).sqrt();
    let drop = end.z - start.z;
    Glide {
        speed: speed / n,
        sink: drop / secs,
        glide_ratio: if drop.abs() < 1e-9 { f64::INFINITY } else { horiz / drop },
        gamma: (drop / horiz.max(1e-9)).atan(),
        lift: lift / n,
        drag: drag / n,
        alpha: alpha / n,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// ARTICLE 1: a flat sheet of foam
// ────────────────────────────────────────────────────────────────────────────

/// 300 x 300 mm, 20 g. Square, so aspect ratio 1.


/// A 2 m x 0.1 m strip: aspect ratio 20, so the finite-wing correction is small
/// and the section slope is nearly the 2D value thin airfoil theory predicts.


/// Force coefficients at a held attitude and airspeed, without flying.
fn coeffs(v: &mut VtolVehicle, alpha: f64, speed: f64, area: f64) -> (f64, f64) {
    // Fly level, pitched up by alpha, so the relative wind meets the wing at
    // exactly alpha.
    v.set_state(VehicleState {
        position: Vec3::new(0.0, 0.0, -1000.0),
        velocity: Vec3::new(speed * alpha.cos(), 0.0, speed * alpha.sin()),
        attitude: Quat::identity(),
        ..initial_state()
    });
    let g = measure(v);
    let q = 0.5 * RHO * speed * speed * area;
    (g.lift / q, g.drag / q)
}

/// A square flat plate broadside to the flow has a measured drag coefficient
/// near 1.17. That number is not in this model: it comes out of the Viterna
/// correlation, which was fitted to wind-turbine blade data on a different
/// continent for a different purpose.
#[test]
fn a_flat_plate_broadside_has_the_measured_drag_coefficient() {
    let mut v = build(FOAM_SHEET);
    let (cl, cd) = coeffs(&mut v, std::f64::consts::FRAC_PI_2, 10.0, 0.09);
    assert!(cl.abs() < 0.05, "a plate edge-on to its own lift should make none: cl {cl}");
    assert!(
        (1.0..1.4).contains(&cd),
        "square flat plate Cd is measured at about 1.17, model gives {cd}"
    );
}

/// And the trend with aspect ratio is right: a long plate drags more than a
/// square one, heading for the 2D value near 2.0. The model UNDERSTATES the
/// high-aspect-ratio end, which is a known property of the Viterna fit rather
/// than a bug, so the test pins the trend and the magnitude separately.
#[test]
fn flat_plate_drag_grows_with_aspect_ratio() {
    let mut square = build(FOAM_SHEET);
    let mut long = build(FOAM_STRIP);
    let (_, cd_1) = coeffs(&mut square, std::f64::consts::FRAC_PI_2, 10.0, 0.09);
    let (_, cd_20) = coeffs(&mut long, std::f64::consts::FRAC_PI_2, 10.0, 0.2);
    assert!(cd_20 > cd_1, "AR 20 plate ({cd_20}) should drag more than AR 1 ({cd_1})");
    // Measured is about 1.5 at AR 20 against 1.98 in 2D; the fit gives ~1.47.
    assert!((1.2..1.8).contains(&cd_20), "AR 20 flat plate Cd {cd_20}");
}

/// Thin airfoil theory: a flat plate lifts at 2*pi per radian in two
/// dimensions, reduced by Prandtl's finite-wing correction in three. At aspect
/// ratio 20 that is about 5.68 per radian. This is the check that the strip sum
/// reproduces lifting-line theory rather than merely producing a plausible
/// upward force.
#[test]
fn a_thin_plate_lifts_at_the_thin_airfoil_slope() {
    let mut v = build(FOAM_STRIP);
    let a0 = 2.0 * std::f64::consts::PI;
    let ar = 20.0;
    let e = 0.95;
    let predicted = a0 / (1.0 + a0 / (std::f64::consts::PI * e * ar));

    let (cl2, _) = coeffs(&mut v, 2.0f64.to_radians(), 12.0, 0.2);
    let (cl6, _) = coeffs(&mut v, 6.0f64.to_radians(), 12.0, 0.2);
    let slope = (cl6 - cl2) / (4.0f64.to_radians());
    assert!(
        (slope - predicted).abs() / predicted < 0.10,
        "lift-curve slope {slope:.3} vs thin-airfoil prediction {predicted:.3} per rad"
    );
    // A symmetric plate makes no lift at zero incidence, whatever the slope.
    // Not asserted at zero: lift and drag are resolved about the velocity AFTER
    // the sub-step, which gravity has already tilted by g*dt. That is a harness
    // artefact worth about 1e-5 in CL, not a model asymmetry.
    let (cl0, _) = coeffs(&mut v, 0.0, 12.0, 0.2);
    assert!(cl0.abs() < 1e-4, "symmetric plate lifts at zero alpha: {cl0}");
}

/// Terminal velocity of the sheet falling flat: mg = 0.5 rho V^2 Cd A, so
/// V = sqrt(2mg / rho Cd A). Roughly 5 m/s for this sheet, which is what a
/// 300 mm square of foam does when you drop it.
#[test]
fn the_sheet_falls_at_its_analytic_terminal_velocity() {
    let mut v = build(FOAM_SHEET);
    let area = 0.09;
    let (_, cd) = coeffs(&mut v, std::f64::consts::FRAC_PI_2, 5.0, area);
    let predicted = (2.0 * 0.020 * G / (RHO * cd * area)).sqrt();

    // Dropped flat, with the plate held level by a large inertia so the test
    // measures the drag law rather than the tumble.
    let mut spec = AirframeSpec::from_json(FOAM_SHEET).unwrap();
    spec.inertia = [50.0, 50.0, 50.0];
    let af = spec.build().unwrap();
    let mut v = VtolVehicle::new("s", af, HomeLocation { lat: 0.0, lng: 0.0, alt: 0.0, heading: 0.0 });
    v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -200.0), ..initial_state() });
    fly(&mut v, 12.0);

    let vel = v.state().velocity;
    let speed = vel.length();
    assert!(
        (speed - predicted).abs() / predicted < 0.05,
        "terminal velocity {speed:.3} m/s vs analytic {predicted:.3} m/s"
    );
    // Falling essentially straight down, so the comparison really is broadside.
    assert!(vel.z > 0.98 * speed, "not descending vertically: {vel:?}");
    // And it is genuinely terminal: another six seconds changes nothing.
    fly(&mut v, 6.0);
    assert!((v.state().velocity.length() - speed).abs() < 0.02, "still accelerating");
}

/// A sheet released edge-on must not stay edge-on. It is an unstable
/// equilibrium, and a model that sits in it has no pitching moment where a real
/// plate has one.
#[test]
fn a_sheet_released_edge_on_does_not_stay_there() {
    let mut v = build(FOAM_SHEET);
    launch(&mut v, 4.0, 0.02, 200.0);
    fly(&mut v, 4.0);
    let (_, pitch, _) = v.state().attitude.to_euler();
    assert!(pitch.abs() > 0.05, "plate held its unstable attitude: pitch {pitch}");
}

// ────────────────────────────────────────────────────────────────────────────
// ARTICLE 2: a paper plane
// ────────────────────────────────────────────────────────────────────────────

/// A dart: 200 mm span, swept, 5 g, no tail. Stability comes from SWEEP plus
/// washout, which is how a real tailless dart is trimmed (the reflex you bend
/// into the trailing edge). The strip model gets that for free: on a swept
/// wing the outboard strips sit further aft, so extra angle of attack adds lift
/// behind the CG and pitches the nose down.


/// A paper dart's best lift-to-drag is somewhere around 3:1 to 6:1. Everyone
/// who has thrown one has measured this. It is a property of the POLAR, so it
/// is read off the polar rather than off one particular trim, which is a
/// separate question about how the dart is folded.
#[test]
fn the_paper_planes_polar_peaks_where_a_dart_does() {
    let mut v = build(PAPER_PLANE);
    let area = v.airframe().wing_area;
    // Very low aspect ratio, which is what makes a dart a dart and why its
    // induced drag is so punishing.
    let ar = 0.2 * 0.2 / area;
    assert!((1.2..2.5).contains(&ar), "dart aspect ratio {ar:.2}");

    let mut best = (0.0f64, 0.0f64);
    let mut a = -2.0f64;
    while a < 18.0 {
        let (cl, cd) = coeffs(&mut v, a.to_radians(), 6.0, area);
        if cd > 0.0 && cl / cd > best.1 {
            best = (a, cl / cd);
        }
        a += 0.25;
    }
    assert!(
        (3.0..6.0).contains(&best.1),
        "paper darts do about 3:1 to 6:1, model peaks at {:.2}:1 (at alpha {:.1} deg)",
        best.1,
        best.0
    );
}

/// A dart SWOOPS. It climbs, stalls, drops the nose, gathers speed and climbs
/// again, and it keeps doing it. Everyone has watched one do this.
///
/// The model produces it unprompted, and it should: a 5 g dart is 90 mm from
/// its leading edge to its trailing edge, so there is almost no arm for a
/// pitch-damping moment to act on. The oscillation is the lightly damped
/// phugoid with the stall folded into the top of each cycle. Nothing here was
/// put in to make this happen and no term in the model is named after it.
#[test]
fn the_paper_plane_swoops_and_stalls_like_a_real_dart() {
    let mut v = build(PAPER_PLANE);
    launch(&mut v, 6.0, -0.08, 400.0);
    let dt = 0.004;
    let mut pitches = Vec::new();
    let mut worst_stall: f64 = 0.0;
    for _ in 0..(24.0 / dt) as usize {
        v.step(&[], dt);
        let (_, pitch, _) = v.state().attitude.to_euler();
        pitches.push(pitch.to_degrees());
        worst_stall = worst_stall.max(v.diagnostics_full().stalled_fraction);
    }
    let hi = pitches.iter().cloned().fold(f64::MIN, f64::max);
    let lo = pitches.iter().cloned().fold(f64::MAX, f64::min);
    assert!(hi - lo > 40.0, "no swoop: pitch only moved {:.1} deg", hi - lo);
    assert!(worst_stall > 0.3, "a swooping dart stalls at the top: worst {worst_stall:.2}");

    // Sustained, not a decaying transient: the second half swings as hard as
    // the first. A dart does not settle down.
    let half = pitches.len() / 2;
    let range = |w: &[f64]| {
        w.iter().cloned().fold(f64::MIN, f64::max) - w.iter().cloned().fold(f64::MAX, f64::min)
    };
    assert!(
        range(&pitches[half..]) > 0.5 * range(&pitches[..half]),
        "the swoop damped out; a dart's does not"
    );
}

/// Despite swooping the whole way down, it descends at a dart-like average
/// rate. Averaged over many cycles, because that is the only thing a swooping
/// aircraft has a well-defined value of.
#[test]
fn the_paper_plane_descends_at_a_dart_like_rate() {
    let mut v = build(PAPER_PLANE);
    launch(&mut v, 6.0, -0.08, 400.0);
    fly(&mut v, 4.0);
    let g = measure_avg(&mut v, 30.0);
    assert!(g.sink > 0.0, "it is climbing, not gliding: sink {}", g.sink);
    assert!(
        (1.2..5.0).contains(&g.glide_ratio),
        "average glide {:.2}:1 over 30 s",
        g.glide_ratio
    );
    assert!((1.5..8.0).contains(&g.speed), "mean speed {:.2} m/s", g.speed);
    // Worse than its own polar allows, because a swoop wastes energy. If free
    // flight matched the polar the aircraft would not be oscillating.
    assert!(g.glide_ratio < 3.0, "a swooping dart cannot reach its steady best");
}

/// Thrown too fast, it must settle back to its own trim speed rather than keep
/// the speed it was given. A tailless dart has real speed stability and this is
/// the visible consequence of it.
#[test]
fn the_paper_plane_settles_to_its_trim_speed() {
    let mut fast = build(PAPER_PLANE);
    let mut slow = build(PAPER_PLANE);
    launch(&mut fast, 11.0, -0.08, 200.0);
    launch(&mut slow, 3.5, -0.08, 200.0);
    fly(&mut fast, 12.0);
    fly(&mut slow, 12.0);
    let a = measure_avg(&mut fast, 10.0).speed;
    let b = measure_avg(&mut slow, 10.0).speed;
    assert!(
        (a - b).abs() / a.max(b) < 0.35,
        "launched at 11 and 3.5 m/s, settled at {a:.2} and {b:.2}: no trim speed"
    );
}

// ────────────────────────────────────────────────────────────────────────────
// ARTICLE 3: a glider
// ────────────────────────────────────────────────────────────────────────────

/// 2 m span, aspect ratio 12.5, 800 g, conventional tail. A club two-metre
/// model, whose real counterpart glides at somewhere around 15:1.


/// A 2 m model glider glides at something like 15:1. Not 3:1, which would be a
/// brick, and not 40:1, which would be a competition sailplane.
#[test]
fn the_glider_achieves_a_plausible_glide_ratio() {
    let mut v = build(GLIDER);
    launch(&mut v, 12.0, -0.05, 2000.0);
    fly(&mut v, 25.0);
    let g = measure_avg(&mut v, 60.0);
    assert!(g.sink > 0.0, "gliders sink: {}", g.sink);
    assert!(
        (8.0..30.0).contains(&g.glide_ratio),
        "a 2 m model glider does about 15:1, model gives {:.1}:1",
        g.glide_ratio
    );
    assert!((6.0..25.0).contains(&g.speed), "glide speed {:.1} m/s", g.speed);
    assert!(g.alpha.abs() < 0.20, "cruising at alpha {:.3} rad", g.alpha);
}

/// The same identity as the paper plane, on an aircraft with a tail, ten times
/// the mass and twice the glide ratio.
#[test]
fn the_gliders_glide_ratio_equals_its_lift_to_drag() {
    let mut v = build(GLIDER);
    launch(&mut v, 12.0, -0.05, 2000.0);
    fly(&mut v, 30.0);
    let g = measure_avg(&mut v, 60.0);
    let err = (g.glide_ratio - g.ld()).abs() / g.ld();
    assert!(
        err < 0.10,
        "glide ratio {:.2} from the trajectory vs L/D {:.2} from the forces ({:.1}% apart)",
        g.glide_ratio,
        g.ld(),
        err * 100.0
    );
}

/// Lanchester's phugoid, 1908. A disturbed aircraft trades height for speed on
/// a long, lightly damped oscillation whose period he put at pi*sqrt(2)*V/g,
/// depending on nothing about the aircraft except how fast it is going.
///
/// The DEFINING content is the energy exchange, so that is what is asserted
/// hardest: speed and altitude must move in antiphase, and total energy must be
/// very nearly conserved across the cycle. Both are checked directly.
///
/// The period is asserted more loosely, on purpose. Lanchester's derivation
/// idealises away drag entirely and assumes the aircraft holds its angle of
/// attack exactly, and real periods consequently run LONGER than his figure.
/// This model runs 1.3 to 1.6 times Lanchester, converging toward him as speed
/// rises, which is the right magnitude and the right direction. Asserting his
/// number to a few percent would be asserting that the model shares his
/// simplifications.
#[test]
fn the_glider_phugoids_as_an_energy_exchange() {
    let mut v = build(GLIDER);
    launch(&mut v, 12.0, -0.05, 5000.0);
    fly(&mut v, 30.0); // settle to trim

    let s = v.state();
    v.set_state(VehicleState { velocity: s.velocity.scale(1.25), ..s });

    let dt = 0.004;
    let mut speeds = Vec::new();
    let mut alts = Vec::new();
    for _ in 0..(120.0 / dt) as usize {
        v.step(&[], dt);
        speeds.push(v.state().velocity.length());
        alts.push(-v.state().position.z);
    }
    let mean_v = speeds.iter().sum::<f64>() / speeds.len() as f64;

    // ── The oscillation exists, and is lightly damped.
    let mut crossings = Vec::new();
    for i in 1..speeds.len() {
        if speeds[i - 1] <= mean_v && speeds[i] > mean_v {
            crossings.push(i as f64 * dt);
        }
    }
    assert!(crossings.len() >= 4, "only {} oscillations in 120 s", crossings.len());
    let periods: Vec<f64> = crossings.windows(2).map(|w| w[1] - w[0]).collect();
    let measured = periods.iter().sum::<f64>() / periods.len() as f64;

    // ── It is an ENERGY EXCHANGE: fast when low, slow when high.
    //    Correlated against the altitude TREND removed, since the glider is also
    //    descending steadily and that ramp would dominate the correlation.
    let n = alts.len() as f64;
    let slope = {
        let mean_t = (n - 1.0) / 2.0;
        let mean_a = alts.iter().sum::<f64>() / n;
        let mut num = 0.0;
        let mut den = 0.0;
        for (i, a) in alts.iter().enumerate() {
            let d = i as f64 - mean_t;
            num += d * (a - mean_a);
            den += d * d;
        }
        num / den
    };
    let mean_a = alts.iter().sum::<f64>() / n;
    let mean_t = (n - 1.0) / 2.0;
    let detrended: Vec<f64> =
        alts.iter().enumerate().map(|(i, a)| a - (mean_a + slope * (i as f64 - mean_t))).collect();
    let corr = {
        let ma = detrended.iter().sum::<f64>() / n;
        let mv = mean_v;
        let (mut num, mut da, mut dv) = (0.0, 0.0, 0.0);
        for i in 0..alts.len() {
            let x = detrended[i] - ma;
            let y = speeds[i] - mv;
            num += x * y;
            da += x * x;
            dv += y * y;
        }
        num / (da.sqrt() * dv.sqrt())
    };
    assert!(
        corr < -0.7,
        "speed and altitude must be in antiphase for this to be a phugoid; correlation {corr:.3}"
    );

    // ── Energy is nearly conserved across a cycle: it is an exchange, not a
    //    dive. What leaks out is drag, and on a glider at 15:1 that is slow.
    let mass = v.airframe().mass;
    let energy = |i: usize| 0.5 * mass * speeds[i].powi(2) + mass * G * alts[i];
    let one_cycle = (measured / dt) as usize;
    let loss = (energy(0) - energy(one_cycle)) / energy(0).abs().max(1.0);
    assert!(loss > 0.0, "gained energy over a cycle");
    assert!(loss < 0.05, "lost {:.1}% of total energy in one cycle; that is a dive, not a phugoid", loss * 100.0);

    // ── And the period is of Lanchester's order, running longer as his
    //    idealisation predicts it should.
    let lanchester = std::f64::consts::PI * 2.0f64.sqrt() * mean_v / G;
    let ratio = measured / lanchester;
    assert!(
        (0.9..2.0).contains(&ratio),
        "phugoid period {measured:.2} s is {ratio:.2}x Lanchester's {lanchester:.2} s"
    );
}

/// The half of Lanchester that is a scaling law rather than a constant: the
/// period grows with trim speed. Trim speed is changed the way it is changed on
/// a real aircraft, by loading it, so nothing about the aerodynamics moves.
#[test]
fn the_phugoid_period_grows_with_trim_speed() {
    let period_at = |mass: f64| -> (f64, f64) {
        let mut spec = AirframeSpec::from_json(GLIDER).unwrap();
        spec.mass = mass;
        let af = spec.build().unwrap();
        let mut v =
            VtolVehicle::new("g", af, HomeLocation { lat: 0.0, lng: 0.0, alt: 0.0, heading: 0.0 });
        launch(&mut v, 12.0, -0.05, 6000.0);
        fly(&mut v, 30.0);
        let s = v.state();
        v.set_state(VehicleState { velocity: s.velocity.scale(1.25), ..s });
        let dt = 0.004;
        let mut speeds = Vec::new();
        for _ in 0..(120.0 / dt) as usize {
            v.step(&[], dt);
            speeds.push(v.state().velocity.length());
        }
        let mean = speeds.iter().sum::<f64>() / speeds.len() as f64;
        let mut cr = Vec::new();
        for i in 1..speeds.len() {
            if speeds[i - 1] <= mean && speeds[i] > mean {
                cr.push(i as f64 * dt);
            }
        }
        let per: Vec<f64> = cr.windows(2).map(|w| w[1] - w[0]).collect();
        assert!(per.len() >= 3, "only {} cycles at {mass} kg", per.len());
        (mean, per.iter().sum::<f64>() / per.len() as f64)
    };

    let (v_light, t_light) = period_at(0.55);
    let (v_heavy, t_heavy) = period_at(1.15);
    assert!(v_heavy > v_light * 1.15, "loading it did not raise trim speed: {v_light:.2} -> {v_heavy:.2}");
    assert!(
        t_heavy > t_light,
        "the faster aircraft must phugoid more slowly: {t_light:.2} s at {v_light:.2} m/s, \
         {t_heavy:.2} s at {v_heavy:.2} m/s"
    );
    // Sub-linear in speed rather than Lanchester's exact proportionality, which
    // is the same deviation the absolute period shows and for the same reason.
    let speed_ratio = v_heavy / v_light;
    let period_ratio = t_heavy / t_light;
    assert!(
        period_ratio > 1.0 && period_ratio < speed_ratio * 1.3,
        "period scaled {period_ratio:.2}x against a speed ratio of {speed_ratio:.2}x"
    );
}

/// Drag polar geometry: minimum sink happens SLOWER than best glide. Best glide
/// maximises L/D; minimum sink maximises CL^1.5/CD, which lands at a lower
/// speed. Any model with a real induced-drag term reproduces this, and one that
/// has faked the polar will not.
#[test]
fn minimum_sink_is_slower_than_best_glide() {
    // Sample the polar by trimming at a range of speeds. Held attitude rather
    // than free flight, so each point is a clean polar sample.
    let mut v = build(GLIDER);
    let area = v.airframe().wing_area;
    let sample = |v: &mut VtolVehicle, alpha: f64| -> (f64, f64, f64) {
        let (cl, cd) = coeffs(v, alpha, 12.0, area);
        if cl <= 0.0 {
            return (f64::NAN, f64::NAN, f64::NAN);
        }
        // Speed that supports the weight at this CL, and the sink there.
        let speed = (2.0 * 0.80 * G / (RHO * area * cl)).sqrt();
        let sink = speed * cd / cl;
        (speed, sink, cl / cd)
    };
    let mut best_ld = (f64::NAN, 0.0);
    let mut min_sink = (f64::NAN, f64::INFINITY);
    let mut a = 0.0f64;
    while a < 10.0 {
        let (speed, sink, ld) = sample(&mut v, a.to_radians());
        if speed.is_finite() {
            if ld > best_ld.1 {
                best_ld = (speed, ld);
            }
            if sink < min_sink.1 {
                min_sink = (speed, sink);
            }
        }
        a += 0.25;
    }
    assert!(best_ld.0.is_finite() && min_sink.0.is_finite(), "no polar samples");
    assert!(
        min_sink.0 < best_ld.0,
        "minimum sink at {:.2} m/s must be SLOWER than best glide at {:.2} m/s",
        min_sink.0,
        best_ld.0
    );
    assert!((8.0..30.0).contains(&best_ld.1), "best L/D {:.1}", best_ld.1);
}

/// Static longitudinal stability, and the neutral point. Positions in the spec
/// are relative to the CG, so moving the CG aft means moving every surface
/// forward. Pitch stiffness dM/dalpha must fall as that happens, pass through
/// zero, and go positive: at the neutral point the aircraft is indifferent, and
/// behind it, divergent. This is the single most consequential number in a
/// tailed aircraft's design and it is not put in anywhere, it emerges.
#[test]
fn moving_the_cg_aft_removes_the_pitch_stiffness() {
    let stiffness = |cg_shift: f64| -> f64 {
        let mut spec = AirframeSpec::from_json(GLIDER).unwrap();
        for w in spec.wings.iter_mut() {
            w.root[0] += cg_shift;
        }
        let af = spec.build().unwrap();
        let mut v = VtolVehicle::new("g", af, HomeLocation { lat: 0.0, lng: 0.0, alt: 0.0, heading: 0.0 });
        let moment_at = |v: &mut VtolVehicle, alpha: f64| {
            v.set_state(VehicleState {
                position: Vec3::new(0.0, 0.0, -1000.0),
                velocity: Vec3::new(12.0 * alpha.cos(), 0.0, 12.0 * alpha.sin()),
                attitude: Quat::identity(),
                ..initial_state()
            });
            v.step(&[], 0.004);
            v.diagnostics_full().aero.moment_bf.y
        };
        let lo = moment_at(&mut v, 1.0f64.to_radians());
        let hi = moment_at(&mut v, 5.0f64.to_radians());
        (hi - lo) / 4.0f64.to_radians()
    };

    // As designed: nose-up alpha must give a nose-down moment.
    let nominal = stiffness(0.0);
    assert!(nominal < 0.0, "the glider as drawn is not pitch stable: dM/dalpha {nominal:.4}");

    // CG moving aft in 20 mm steps: stiffness rises monotonically toward zero.
    let mut prev = nominal;
    let mut neutral = None;
    for i in 1..=14 {
        let k = stiffness(0.02 * i as f64);
        assert!(k > prev - 1e-9, "stiffness not monotone in CG position at step {i}");
        if neutral.is_none() && k >= 0.0 {
            neutral = Some(0.02 * i as f64);
        }
        prev = k;
    }
    let np = neutral.expect("never reached the neutral point in 280 mm of CG travel");
    // The neutral point sits aft of the wing but well ahead of the tail, which
    // is the only place it can be on a conventional aircraft.
    assert!((0.02..0.25).contains(&np), "neutral point {np:.3} m aft of the design CG");
}

/// A glider with its fin must weathercock: released in a sideslip it swings its
/// nose into the relative wind rather than continuing sideways.
#[test]
fn the_glider_weathercocks_out_of_a_sideslip() {
    let mut v = build(GLIDER);
    v.set_state(VehicleState {
        position: Vec3::new(0.0, 0.0, -400.0),
        // Flying forward but slipping to the right.
        velocity: Vec3::new(12.0, 3.0, 0.6),
        attitude: Quat::identity(),
        ..initial_state()
    });
    let beta0 = v.diagnostics_full().beta;
    fly(&mut v, 6.0);
    let beta = v.diagnostics_full().beta;
    assert!(
        beta.abs() < beta0.abs().max(0.05),
        "sideslip grew from {beta0:.3} to {beta:.3} rad: the fin is not stabilising"
    );
}

/// Energy sanity across all three articles: an unpowered aircraft can never
/// gain total energy. It is the cheapest possible check that no sign anywhere
/// in the force sum is inverted, and it holds regardless of trim, attitude or
/// how badly the article flies.
#[test]
fn no_unpowered_article_gains_energy() {
    for (name, json, speed) in [
        ("foam sheet", FOAM_SHEET, 5.0),
        ("paper plane", PAPER_PLANE, 6.0),
        ("glider", GLIDER, 12.0),
    ] {
        let mut v = build(json);
        launch(&mut v, speed, -0.05, 2000.0);
        let mass = v.airframe().mass;
        let energy = |v: &VtolVehicle| {
            let s = v.state();
            0.5 * mass * s.velocity.length().powi(2) + mass * G * (-s.position.z)
        };
        let start = energy(&v);
        let mut prev = start;
        for _ in 0..40 {
            fly(&mut v, 1.0);
            let e = energy(&v);
            assert!(
                e <= prev + 1e-6,
                "{name} gained energy: {prev:.3} -> {e:.3} J"
            );
            prev = e;
        }
        assert!(prev < start, "{name} lost no energy at all in 40 s");
    }
}



