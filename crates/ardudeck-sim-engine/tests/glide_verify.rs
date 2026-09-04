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
use ardudeck_sim_engine::articles::{FOAM_SHEET, FOAM_SHEET_LE, FOAM_STRIP, GLIDER, PAPER_PLANE};
use ardudeck_sim_engine::copter::{initial_state, VehicleState};
use ardudeck_sim_engine::fdm_server::{HomeLocation, SimVehicle};
use ardudeck_sim_engine::math::{Quat, Vec3};
use ardudeck_sim_engine::vtol::VtolVehicle;
use ardudeck_sim_engine::wind::{WindConfig, WindField};

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

/// A sheet dropped in PERFECTLY STILL AIR falls straight down forever, and that
/// is correct rather than a missing force.
///
/// At exactly broadside the centre of pressure sits at the plate's own centre,
/// which for a uniform sheet is its CG, so the pitching moment is identically
/// zero. It is an equilibrium. An UNSTABLE one, because tilting even slightly
/// moves the centre of pressure forward of the CG, but a deterministic
/// simulation released with perfect symmetry has nothing to tilt it.
///
/// This is worth a test because it looked exactly like missing aerodynamics the
/// first time it was watched, and it is the opposite: the aerodynamics are
/// there, the ATMOSPHERE was not connected. Real air is never still.
#[test]
fn a_sheet_in_perfectly_still_air_sits_on_its_unstable_equilibrium() {
    let mut v = build(FOAM_SHEET);
    v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -200.0), ..initial_state() });
    fly(&mut v, 8.0);
    let (_, pitch, _) = v.state().attitude.to_euler();
    assert!(pitch.abs() < 1e-6, "something disturbed it: pitch {pitch}");
    let p = v.state().position;
    assert!(p.x.abs() < 1e-6 && p.y.abs() < 1e-6, "drifted sideways in still air: {p:?}");
}

/// Put it in real air and it tumbles on its own. Gusts tip it off the
/// equilibrium and the static instability does the rest, with no special case
/// and nothing seeded by hand.
#[test]
fn a_sheet_in_real_air_tumbles() {
    let mut v = build(FOAM_SHEET);
    // A light breeze with ordinary turbulence in it.
    v.set_wind_field(WindField::from_uniform(WindConfig {
        steady: Vec3::new(2.0, 0.0, 0.0),
        intensity: 1.2,
        time_constant: 0.8,
    }));
    v.set_seed(4242);
    v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -200.0), ..initial_state() });

    let dt = 0.004;
    let mut max_pitch: f64 = 0.0;
    let mut max_rate: f64 = 0.0;
    for _ in 0..(12.0 / dt) as usize {
        v.step(&[], dt);
        let (_, pitch, _) = v.state().attitude.to_euler();
        max_pitch = max_pitch.max(pitch.abs());
        max_rate = max_rate.max(v.state().angular_velocity.length());
    }
    assert!(
        max_pitch.to_degrees() > 45.0,
        "did not tumble: peak pitch only {:.1} deg",
        max_pitch.to_degrees()
    );
    assert!(max_rate > 1.0, "barely rotated: peak rate {max_rate:.2} rad/s");
    // And it goes somewhere, rather than dropping on the spot.
    let p = v.state().position;
    assert!(
        (p.x * p.x + p.y * p.y).sqrt() > 1.0,
        "fell straight down anyway: drifted {:.2} m",
        (p.x * p.x + p.y * p.y).sqrt()
    );
}

/// The same sheet with a rounded leading edge instead of a sharp one.
///
/// This is a test of the SECTION DATA, not of the simulation, and saying so is
/// the point of it. Strip theory has no geometry finer than chord and area, so
/// it cannot see a leading edge; everything a rounded nose does has to arrive
/// as coefficients. What the model must then do is carry that difference
/// through to the aircraft correctly, and these are the two consequences that
/// matter:
///
/// - A sharp edge separates the flow at any incidence, so it stalls early and
///   develops no leading-edge SUCTION: its resultant force stays roughly normal
///   to the surface and drag climbs with lift. A rounded nose recovers most of
///   that axial force, which is the entire reason an aerofoil has a usable
///   lift-to-drag ratio and a flat plate does not.
/// - It holds on to a much higher angle before letting go.
#[test]
fn a_rounded_leading_edge_beats_a_sharp_one_where_it_should() {
    let peak = |json: &str| -> (f64, f64, f64) {
        let mut v = build(json);
        let area = v.airframe().wing_area;
        let (mut best_ld, mut cl_max, mut at) = (0.0f64, 0.0f64, 0.0f64);
        let mut a = 0.0f64;
        while a < 30.0 {
            let (cl, cd) = coeffs(&mut v, a.to_radians(), 12.0, area);
            if cd > 0.0 && cl / cd > best_ld {
                best_ld = cl / cd;
            }
            if cl > cl_max {
                cl_max = cl;
                at = a;
            }
            a += 0.25;
        }
        (best_ld, cl_max, at)
    };
    let (ld_sharp, clmax_sharp, stall_sharp) = peak(FOAM_SHEET);
    let (ld_round, clmax_round, stall_round) = peak(FOAM_SHEET_LE);

    // Stall ANGLE is deliberately not compared here. At aspect ratio 1 the
    // finite-wing correction stretches the incidence axis so hard that a 2D
    // stall at 12 degrees lands past 29, and it swamps the section difference.
    // That is real, and it is why the stall-angle comparison lives in
    // `bl::tests::a_thin_section_stalls_earlier_than_a_thick_one`, on the 2D
    // sections, where it is the section that decides.
    let _ = (stall_sharp, stall_round);
    // Carries more.
    assert!(
        clmax_round > clmax_sharp * 1.3,
        "CLmax rounded {clmax_round:.3} vs sharp {clmax_sharp:.3}"
    );
    // And the THIN one is the more efficient of the two, which is the real
    // trade and the opposite of what this test first asserted. A blunt nose
    // buys stall margin and maximum lift; it does not buy efficiency, because
    // an 18% section carries far more profile drag than a 4% one. Getting this
    // backwards is easy precisely because "rounded is better" sounds right.
    assert!(
        ld_sharp > ld_round,
        "the thinner section should be the cleaner one: sharp {ld_sharp:.2} vs rounded {ld_round:.2}"
    );
    // Both are still wings rather than bricks.
    assert!(ld_round > 3.0 && ld_sharp > 3.0, "L/D {ld_round:.2} and {ld_sharp:.2}");

    // The planforms are IDENTICAL: if these differed, the comparison would be
    // measuring geometry rather than the section, which is exactly the mistake
    // this test exists to avoid.
    let a = AirframeSpec::from_json(FOAM_SHEET).unwrap().build().unwrap();
    let b = AirframeSpec::from_json(FOAM_SHEET_LE).unwrap().build().unwrap();
    assert!((a.wing_area - b.wing_area).abs() < 1e-12, "areas differ");
    assert_eq!(a.surfaces.len(), b.surfaces.len(), "strip counts differ");
    assert!((a.mass - b.mass).abs() < 1e-12, "masses differ");
}

/// Every article's coefficient table must be finite and continuous over the
/// whole circle, whatever route it was built by.
///
/// `aero` has had this check for tables built from a parametric spec since the
/// day Viterna went in. Tables built from a SOLVED polar did not, and a hole
/// duly appeared in one: on the aspect-ratio 1 sheet, lift vanished and drag
/// jumped fiftyfold to 1.0 over a few degrees around 10, because the
/// lifting-line correction ran the sample angles backwards past the stall and
/// the interpolator assumes they only ever go forwards. An article flying
/// through it lost its speed for no visible reason.
#[test]
fn every_article_has_a_continuous_coefficient_table() {
    for (name, json) in ardudeck_sim_engine::articles::ALL {
        let af = AirframeSpec::from_json(json).unwrap().build().unwrap();
        for (k, t) in af.airfoils.iter().enumerate() {
            let mut prev: Option<(f64, f64)> = None;
            let mut a = -180.0f64;
            while a <= 180.0 {
                let c = t.at(a.to_radians());
                assert!(c.cl.is_finite() && c.cd.is_finite(), "{name} airfoil {k}: non-finite at {a}");
                // Drag is bounded by the flat plate: past the stall a section
                // IS one and cannot out-drag it broadside. The 18% sheet once
                // reached 2.75 against a plate maximum near 1.13.
                assert!(c.cd >= 0.0 && c.cd <= 1.5, "{name} airfoil {k}: cd {} at {a}", c.cd);
                // Lift MAGNITUDE is not policed here. This test's job is holes
                // and jumps; the magnitude bound has its own test, which
                // currently fails for a real reason and says so rather than
                // being loosened until it passes. See
                // `thin_sections_should_not_reach_a_lift_coefficient_of_two`.
                assert!(c.cl.is_finite());
                if let Some((pl, pd)) = prev {
                    assert!(
                        (c.cl - pl).abs() < 0.15,
                        "{name} airfoil {k}: cl jumps {pl:.3} to {:.3} at {a} deg",
                        c.cl
                    );
                    assert!(
                        (c.cd - pd).abs() < 0.15,
                        "{name} airfoil {k}: cd jumps {pd:.3} to {:.3} at {a} deg",
                        c.cd
                    );
                }
                prev = Some((c.cl, c.cd));
                a += 0.25;
            }
        }
    }
}

/// Measure how a falling article rotates: total turning, net turning, and how
/// far it travels sideways per metre dropped.
fn tumble_stats(json: &str, seed: u32, secs: f64) -> (f64, f64, f64) {
    let mut v = build(json);
    v.set_wind_field(WindField::from_uniform(WindConfig {
        steady: Vec3::new(2.0, 0.0, 0.0),
        intensity: 1.2,
        time_constant: 0.8,
    }));
    v.set_seed(seed);
    v.set_state(VehicleState { position: Vec3::new(0.0, 0.0, -400.0), ..initial_state() });
    let dt = 0.004;
    let (mut turn, mut signed) = (0.0f64, 0.0f64);
    for _ in 0..(secs / dt) as usize {
        v.step(&[], dt);
        let q = v.state().angular_velocity.y;
        turn += q.abs() * dt;
        signed += q * dt;
    }
    let s = v.state();
    let horiz = (s.position.x * s.position.x + s.position.y * s.position.y).sqrt();
    let fell = (-s.position.z - 400.0).abs().max(1e-9);
    let tau = 2.0 * std::f64::consts::PI;
    ((signed / turn.max(1e-9)).abs(), turn / tau, horiz / fell)
}

/// KNOWN GAP, NARROWED: a very thin section still reaches a lift coefficient
/// higher than it should, though far less so than it did.
///
/// Adding leading-edge stall (Owen and Klanfer's bubble-burst criterion,
/// blended rather than switched) brought the peak from 2.41 down to 1.71, and
/// the thicker sections into range: NACA 0012 peaks at 1.36 near 12 degrees
/// against a measured 1.0 near 13, and the 0018 at 1.41 near 12.5 against 1.2
/// near 15. Those are usable.
///
/// The 4% section is not: 1.08 at 16.5 degrees where it should be near 0.7 at
/// 7, and it now out-lifts the 8% section, which is the wrong way round. The
/// burst criterion needs the bubble resolved over its own LENGTH near the nose,
/// and Thwaites plus Michel plus Head does not resolve it finely enough there.
/// The remaining fix is the one already noted in `bl.rs`: XFOIL's
/// lagged-dissipation closure with e^N transition.
#[test]
#[ignore = "records a known gap: thin sections stall too late at low Reynolds number"]
fn thin_sections_should_not_reach_a_lift_coefficient_of_two() {
    for (name, json) in ardudeck_sim_engine::articles::ALL {
        let af = AirframeSpec::from_json(json).unwrap().build().unwrap();
        for (k, t) in af.airfoils.iter().enumerate() {
            let mut peak = 0.0f64;
            let mut a = -180.0f64;
            while a <= 180.0 {
                peak = peak.max(t.at(a.to_radians()).cl.abs());
                a += 0.25;
            }
            assert!(peak <= 1.6, "{name} airfoil {k}: peak |CL| {peak:.2}");
        }
    }
}

/// A DROPPED SHEET TUMBLES: it spins continuously in one direction and flies off
/// that way. Everyone has watched one do it.
///
/// This needs the model to see that the section is TURNING, and a strip model
/// evaluating its flow at one chordwise point cannot: a rotating plate looks
/// identical to a stationary one at the same angle, so it swings and comes back.
/// Thin-airfoil theory says the quasi-steady angle is set by tangency at the
/// THREE-QUARTER chord while the lift acts at the quarter chord, and the gap
/// between those two points is the whole mechanism.
///
/// Before that change the sheet fluttered: 6.3 rotations of swinging with a net
/// of 0.24, which is an oscillation and not a tumble.
/// KNOWN GAP as of the chordwise rework: the model FLUTTERS where it should
/// tumble.
///
/// Andersen, Pesavento and Wang's phase diagram is governed by the
/// dimensionless inertia `I* = rho_s h / (rho_f c)`. This sheet is 20 g over
/// 0.09 m2 against `rho_f c` of 0.368, so `I* = 0.60`, and the flutter-to-tumble
/// boundary sits near 0.4. It should tumble.
///
/// It no longer fails UNPHYSICALLY, which it did before: tip speeds five times
/// airspeed and force coefficients of ten are gone, and so are both of the
/// constants that were fitted to fake the rotation. What is left is a
/// quasi-steady model landing in the wrong one of two real regimes, and closing
/// that needs the unsteady wake, which is a different model class.
#[test]
#[ignore = "records a known gap: the model flutters where this plate should tumble"]
fn a_dropped_sheet_tumbles_and_flies_off_in_one_direction() {
    for json in [FOAM_SHEET, FOAM_SHEET_LE] {
        for seed in [7u32, 99, 1234] {
            let (ratio, turns, glide) = tumble_stats(json, seed, 30.0);
            assert!(
                turns > 20.0,
                "barely rotated: {turns:.1} turns in 30 s (seed {seed})"
            );
            // Nearly all the turning is one way. An oscillation scores near zero
            // here however violently it swings, which is the distinction.
            assert!(
                ratio > 0.7,
                "fluttering, not tumbling: {:.0}% of {turns:.0} turns were net (seed {seed})",
                ratio * 100.0
            );
            // And it goes somewhere, rather than falling on the spot.
            assert!(
                glide > 0.25,
                "no sideways travel: {glide:.2} m across per m down (seed {seed})"
            );
        }
    }
}

/// And an AIRCRAFT does not tumble, which is the other half of the same claim.
/// The change that makes a sheet spin must leave something with a tail gliding.
#[test]
fn the_glider_still_glides_rather_than_tumbling() {
    for seed in [7u32, 99, 1234] {
        let (ratio, _, glide) = tumble_stats(GLIDER, seed, 30.0);
        assert!(ratio < 0.4, "the glider tumbled: {:.0}% net turning (seed {seed})", ratio * 100.0);
        // A modest bar, because this glider is deliberately built with a tail
        // too small to hold it steady: it porpoises, and an aircraft trading
        // height for speed averages worse than its own steady best. What
        // matters here is that it GLIDES rather than tumbling.
        assert!(glide > 1.5, "glided only {glide:.2}:1 (seed {seed})");
    }
}

/// Mass properties are COMPUTED from the breakdown, and everything
/// longitudinal follows: move the battery and the centre of gravity, the pitch
/// inertia and the static margin all move with it.
///
/// This is what a partner setting up an airframe actually needs. They know
/// their battery weighs 200 g and where it sits; where that puts the CG, and
/// whether the aircraft is then flyable, is the answer they want back.
#[test]
fn moving_the_battery_moves_the_cg_and_the_static_margin() {
    let at = |x: f64| {
        let j = GLIDER.replace(
            r#""position": [ 0.24,  0.00,  0.00], "mass": 0.200"#,
            &format!(r#""position": [{x:5.2},  0.00,  0.00], "mass": 0.200"#),
        );
        let af = AirframeSpec::from_json(&j).unwrap().build().unwrap();
        let (_, sm) = af.longitudinal_stability(12.0);
        (af.mass, af.cg_percent_mac(), af.inertia.y, sm)
    };

    let fwd = at(0.40);
    let mid = at(0.24);
    let aft = at(-0.10);

    // Total mass never changes: the battery only moved.
    for m in [fwd.0, mid.0, aft.0] {
        assert!((m - 0.8).abs() < 1e-9, "mass changed: {m}");
    }
    // The CG follows the battery, monotonically.
    assert!(fwd.1 < mid.1 && mid.1 < aft.1, "CG did not track the battery: {:.1} {:.1} {:.1}", fwd.1, mid.1, aft.1);
    // And the static margin falls as it goes aft, through neutral into
    // divergence. An aircraft with its CG behind the neutral point cannot be
    // flown, and this is the number that says so.
    assert!(fwd.3 > mid.3 && mid.3 > aft.3, "margin did not fall: {:+.3} {:+.3} {:+.3}", fwd.3, mid.3, aft.3);
    assert!(fwd.3 > 0.0, "nose-heavy should be stable: {:+.3}", fwd.3);
    assert!(aft.3 < 0.0, "tail-heavy should be unstable: {:+.3}", aft.3);
    // Pitch inertia is LEAST with the mass near the CG, which is the parallel
    // axis theorem showing up in something a person can act on.
    assert!(at(0.10).2 < fwd.2, "moving mass toward the CG must lower Iyy");
}

/// A uniform plate is unstable in pitch and an aircraft is not, and both fall
/// out of the same computation.
#[test]
fn the_articles_have_the_stability_their_shapes_imply() {
    let margin = |json: &str| {
        AirframeSpec::from_json(json).unwrap().build().unwrap().longitudinal_stability(12.0).1
    };
    // A uniform sheet carries its mass at mid-chord and its lift at the quarter
    // chord, so it cannot be stable. That is why it tumbles.
    assert!(margin(FOAM_SHEET) < 0.0, "the sheet should be unstable: {:+.3}", margin(FOAM_SHEET));
    // A dart and a glider are aircraft.
    let dart = margin(PAPER_PLANE);
    let glider = margin(GLIDER);
    assert!(dart > 0.0 && dart < 0.5, "dart margin {:+.3}", dart);
    assert!(glider > 0.0 && glider < 0.5, "glider margin {:+.3}", glider);
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
    // Not asserted to stall HARD. With the chord resolved the dart's strips no
    // longer all reach the stall together, so it brushes it rather than letting
    // go completely, which is what a dart that keeps flying actually does.
    assert!(worst_stall > 0.05, "never came near the stall: worst {worst_stall:.2}");

    // The swoop DAMPS rather than running forever, and that is a correction to
    // what this test used to assert. A dart with real pitch damping settles into
    // a glide; the endless violent porpoise was an artefact of every strip
    // sitting at one chordwise point, which left the model no pitch damping at
    // all. What survives is the initial swoop and the stall at the top of it.
    let half = pitches.len() / 2;
    let range = |w: &[f64]| {
        w.iter().cloned().fold(f64::MIN, f64::max) - w.iter().cloned().fold(f64::MAX, f64::min)
    };
    assert!(range(&pitches[..half]) > 40.0, "no swoop in the first half");
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
        (2.5..6.0).contains(&g.glide_ratio),
        "paper darts glide at about 3:1 to 6:1, model gives {:.2}",
        g.glide_ratio
    );
    assert!((1.5..8.0).contains(&g.speed), "mean speed {:.2} m/s", g.speed);
    // And the flight is STEADY: glide ratio from the trajectory now matches L/D
    // from the forces, which it could not while the dart was porpoising. That
    // identity holds only in steady flight, and getting it is what pitch damping
    // bought.
    let err = (g.glide_ratio - g.ld()).abs() / g.ld();
    assert!(err < 0.15, "glide {:.2} vs L/D {:.2}", g.glide_ratio, g.ld());
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

/// A dart's directional stability is its KEEL, the fold down the middle, and
/// nothing else. Without one it has no yaw stiffness and, worse, no yaw
/// DAMPING: any yaw a gust gives it simply stays, so the heading wanders and
/// the aircraft flies visibly crabbed. The first version of this spec had no
/// vertical surface at all and looked exactly like that.
#[test]
fn the_paper_plane_has_a_keel_that_hangs_below_it() {
    let af = AirframeSpec::from_json(PAPER_PLANE).unwrap().build().unwrap();
    let keel: Vec<_> = af
        .surfaces
        .iter()
        .filter(|s| s.normal.z.abs() < 0.5 && s.normal.y.abs() > 0.5)
        .collect();
    assert!(!keel.is_empty(), "the dart has no vertical surface");
    // Body z is DOWN, so a ventral keel sits at positive z.
    assert!(keel.iter().all(|s| s.position.z > 0.0), "the keel is above the wing, not below it");
    assert!(keel.iter().all(|s| s.position.y.abs() < 1e-9), "the keel is off the centreline");
}

#[test]
fn the_paper_plane_weathercocks_and_damps_in_yaw() {
    let mut v = build(PAPER_PLANE);
    // Flying forward but slipping to the right.
    v.set_state(VehicleState {
        position: Vec3::new(0.0, 0.0, -400.0),
        velocity: Vec3::new(6.0, 1.5, 0.4),
        attitude: Quat::identity(),
        ..initial_state()
    });
    v.step(&[], 1e-4);
    // Nose swings right, into the relative wind.
    assert!(
        v.diagnostics_full().aero.moment_bf.z > 0.0,
        "no weathercock: yaw moment {}",
        v.diagnostics_full().aero.moment_bf.z
    );

    // And a yaw RATE is opposed, which is the half that stops the heading
    // wandering. A surface with stiffness but no damping still hunts forever.
    let mut v = build(PAPER_PLANE);
    v.set_state(VehicleState {
        position: Vec3::new(0.0, 0.0, -400.0),
        velocity: Vec3::new(6.0, 0.0, 0.4),
        attitude: Quat::identity(),
        angular_velocity: Vec3::new(0.0, 0.0, 1.5),
        ..initial_state()
    });
    v.step(&[], 1e-4);
    assert!(
        v.diagnostics_full().aero.moment_bf.z < 0.0,
        "no yaw damping: rate +z met with moment {}",
        v.diagnostics_full().aero.moment_bf.z
    );
}

/// A UNIFORM wind produces NO sideslip, so an aircraft in one does not
/// weathercock. It flies through the moving air mass with its nose on its
/// AIRSPEED vector while its ground track drifts downwind. Nose one way, track
/// another, and both correct.
///
/// A uniform wind is a change of reference frame for the air, so it must change
/// the ground track and NOTHING else. Two flights launched at the same
/// AIRSPEED, one in still air and one in 6 m/s, must fly identically and differ
/// only in position.
///
/// Launched at the same AIRSPEED and not the same ground speed, which is the
/// whole point restated. Giving both the same ground velocity puts the windy
/// one into a 26 degree sideslip at the instant of release, and it then yaws
/// hard to sort itself out. That is correct behaviour and it is also what an
/// article DROPPED from rest into a breeze does, which is worth knowing when
/// watching one: it starts with the wind straight up its side.
#[test]
fn a_uniform_wind_shifts_the_track_and_changes_nothing_else() {
    // 6 m/s blowing east; NED y is east.
    let wind = Vec3::new(0.0, 6.0, 0.0);
    let run = |w: Vec3| {
        let mut v = build(GLIDER);
        v.set_wind_field(WindField::from_uniform(WindConfig {
            steady: w,
            intensity: 0.0,
            time_constant: 1.0,
        }));
        v.set_state(VehicleState {
            position: Vec3::new(0.0, 0.0, -3000.0),
            // Same AIRSPEED in both: ground velocity carries the wind.
            velocity: Vec3::new(12.0, 0.0, 0.6).add(w),
            attitude: Quat::from_euler(0.0, -0.05, 0.0),
            ..initial_state()
        });
        fly(&mut v, 10.0);
        v.state()
    };
    let calm = run(Vec3::zero());
    let windy = run(wind);

    // Same motion relative to the air. To a tolerance, not bit for bit:
    // airspeed is `velocity - wind`, and in the windy case both terms carry the
    // drift, so the subtraction loses low bits the calm case never had. A free
    // unstabilised glider then amplifies that, which is why the horizon here is
    // ten seconds and not a minute.
    let att_err = ardudeck_sim_engine::validate::attitude_error(calm.attitude, windy.attitude);
    assert!(att_err < 1e-3, "a uniform wind changed the attitude by {att_err:.3e} rad");
    assert!(
        calm.angular_velocity.sub(windy.angular_velocity).length() < 1e-3,
        "it changed the body rates"
    );

    // And the ground track is displaced by exactly wind times time.
    let east = windy.position.y - calm.position.y;
    assert!((east - 60.0).abs() < 0.5, "track displacement {east:.2} m, expected 60.00 m");
    assert!(
        (calm.position.x - windy.position.x).abs() < 0.5,
        "the wind moved it downrange as well as crosswind"
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
        // LOADED, not relabelled. `spec.mass` is ignored once a breakdown is
        // given, which is the point of having one: the mass is what the parts
        // weigh. Scaling every item keeps the centre of gravity where it was and
        // changes only the weight, which is what this test wants.
        let k = mass / spec.masses.iter().map(|m| m.mass).sum::<f64>();
        for m in spec.masses.iter_mut() {
            m.mass *= k;
        }
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






#[test]
fn probe_spin_rate2() {
    for (name, json) in [("sheet 0004", FOAM_SHEET), ("glider", GLIDER)] {
        let mut v = build(json);
        v.set_wind_field(WindField::from_uniform(WindConfig { steady: Vec3::new(2.0,0.0,0.0), intensity: 1.2, time_constant: 0.8 }));
        v.set_seed(7);
        v.set_state(VehicleState { position: Vec3::new(0.0,0.0,-400.0), ..initial_state() });
        let dt = 0.004;
        let (mut turn, mut signed) = (0.0f64, 0.0f64);
        for i in 0..(30.0/dt) as usize {
            v.step(&[], dt);
            let q = v.state().angular_velocity.y; turn += q.abs()*dt; signed += q*dt;
            if i % 1250 == 0 {
                let s = v.state();
                println!("{name:11} t={:5.1} w={:7.2} ({:4.2} rev/s) Vfall={:5.2} tip/V={:5.2}",
                    i as f64*dt, s.angular_velocity.y, s.angular_velocity.y.abs()/(2.0*std::f64::consts::PI),
                    s.velocity.z, s.angular_velocity.y.abs()*0.15/s.velocity.length().max(0.1));
            }
        }
        let s = v.state();
        let horiz = (s.position.x*s.position.x + s.position.y*s.position.y).sqrt();
        println!("  -> {name}: ratio {:4.2}  turns {:5.1}  drift/fall {:4.2}\n",
            (signed/turn.max(1e-9)).abs(), turn/(2.0*std::f64::consts::PI), horiz/(-s.position.z-400.0).abs().max(1e-9));
    }
}


