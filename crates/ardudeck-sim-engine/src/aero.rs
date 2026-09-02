//! Lifting-surface aerodynamics: strip theory over a discretised airframe.
//!
//! This is the fixed-wing half of a VTOL. `motor.rs` supplies rotor forces; this
//! supplies everything the airflow does to the structure, evaluated per STRIP
//! rather than as one set of whole-aircraft coefficients. The strip formulation is
//! what buys transition: local flow at a strip includes rotor slipstream, rotation
//! rate and wind, so a wing half-immersed in prop wash produces the rolling moment
//! it should instead of a whole-aircraft average that produces none.
//!
//! Body frame is FRD (x forward, y right, z down), matching `copter.rs` and
//! ArduPilot. Angle of attack follows the standard alpha = atan2(w, u).
//!
//! Coefficients run the FULL circle, -180..180 deg, because a VTOL hovers at
//! alpha = 90. Tables that stop at the stall are undefined exactly where
//! transition happens, which is the regime the customer cares about. The
//! extrapolation is Viterna-Corrigan (1981), the same method AeroDyn and QBlade
//! use for wind-turbine blades, chosen because that field has the same problem:
//! a real, measured need for coefficients at every angle.

use crate::math::Vec3;
use std::f64::consts::PI;

/// Table resolution over -180..180 deg. 1 deg is far finer than the underlying
/// data justifies; the cost is 361 entries per airfoil and the benefit is that
/// linear interpolation never has to cross the stall peak in one step.
pub const TABLE_STEP_DEG: f64 = 1.0;
const TABLE_N: usize = 361;

/// Aspect ratio above which the Viterna CD_max correlation saturates. Viterna &
/// Corrigan give CD_max = 1.11 + 0.018*AR only up to AR 50; beyond it the flat
/// plate value stops growing.
const VITERNA_AR_MAX: f64 = 50.0;

/// Drag multiplier at exactly 180 deg, relative to the section's own minimum
/// drag. A backwards aerofoil is attached but blunt-first, so it is draggier
/// than the same section flying forwards.
const REVERSED_CD_FACTOR: f64 = 2.0;

/// Recover a plausible minimum drag from the stall-point drag, so the fully
/// reversed anchor does not need another parameter threaded through. The stall
/// point already carries cd_min plus the lift-induced rise; a quarter of it is a
/// bounded, monotone stand-in that can never exceed the flat-plate value.
fn cd_min_of(cd_stall: f64, cd_max: f64) -> f64 {
    (cd_stall * 0.25).clamp(0.005, cd_max)
}

// ─── Airfoil coefficients ───────────────────────────────────────────────────

/// Parametric section description. Everything a partner can state about a wing
/// without running a panel code, which is the level of detail an airframe
/// datasheet actually carries.
///
/// A measured or AVL-derived polar is strictly better and is supported by
/// building `AirfoilTable` from samples instead (`AirfoilTable::from_samples`);
/// this exists so an airframe can be described from its drawing on day one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirfoilSpec {
    /// Lift-curve slope in the linear region, per radian. Thin-airfoil theory
    /// gives 2*pi for a 2D section; a finite wing is lower and the caller should
    /// pass the 3D value (see `finite_wing_slope`).
    pub cl_alpha: f64,
    /// Zero-lift angle of attack, radians. Negative for a cambered section.
    pub alpha_0: f64,
    /// Positive stall angle, radians, measured from zero lift.
    pub alpha_stall: f64,
    /// Negative stall angle, radians (typically shallower in magnitude than the
    /// positive one on a cambered section).
    pub alpha_stall_neg: f64,
    /// Minimum profile drag coefficient.
    pub cd_min: f64,
    /// Quadratic drag rise with lift: cd = cd_min + cd_k * (cl - cl_at_cd_min)^2.
    /// Carries both profile drag rise and induced drag when the caller folds
    /// 1/(pi*e*AR) into it.
    pub cd_k: f64,
    /// Section pitching moment about the quarter chord in the linear region.
    pub cm_0: f64,
    /// Aspect ratio, used only for the Viterna flat-plate CD_max correlation.
    pub aspect_ratio: f64,
}

impl Default for AirfoilSpec {
    /// A mildly cambered general-aviation-like section on a moderate wing. Chosen
    /// so an airframe that omits a field still flies rather than producing zeros.
    fn default() -> Self {
        AirfoilSpec {
            cl_alpha: 5.0,
            alpha_0: -0.035,
            alpha_stall: 0.26,
            alpha_stall_neg: -0.22,
            cd_min: 0.02,
            cd_k: 0.05,
            cm_0: -0.05,
            aspect_ratio: 6.0,
        }
    }
}

/// Prandtl finite-wing correction to the 2D lift-curve slope. `a0` is the 2D
/// slope (2*pi for thin-airfoil theory), `e` the Oswald / span efficiency.
///
/// Callers should use this rather than passing 2*pi to `AirfoilSpec`, because a
/// strip model discretises the SPAN but not the trailing vorticity: nothing in
/// the strip sum knows the wing is finite, so the downwash correction has to be
/// baked into the section slope.
pub fn finite_wing_slope(a0: f64, aspect_ratio: f64, e: f64) -> f64 {
    if aspect_ratio <= 0.0 {
        return a0;
    }
    a0 / (1.0 + a0 / (PI * e * aspect_ratio))
}

/// Induced-drag factor 1/(pi*e*AR), to be added into `AirfoilSpec::cd_k` for the
/// same reason `finite_wing_slope` exists.
pub fn induced_drag_k(aspect_ratio: f64, e: f64) -> f64 {
    if aspect_ratio <= 0.0 {
        return 0.0;
    }
    1.0 / (PI * e * aspect_ratio)
}

/// CL / CD / CM sampled every `TABLE_STEP_DEG` over the full -180..180 circle.
#[derive(Debug, Clone)]
pub struct AirfoilTable {
    cl: Vec<f64>,
    cd: Vec<f64>,
    cm: Vec<f64>,
}

/// One evaluated section: the three coefficients at an angle of attack.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coeffs {
    pub cl: f64,
    pub cd: f64,
    pub cm: f64,
}

impl AirfoilTable {
    /// Build the full-circle table from a parametric spec: linear (with a smooth
    /// approach to the stall peak) inside the stall angles, Viterna-Corrigan
    /// outside them.
    pub fn from_spec(s: &AirfoilSpec) -> AirfoilTable {
        let mut cl = Vec::with_capacity(TABLE_N);
        let mut cd = Vec::with_capacity(TABLE_N);
        let mut cm = Vec::with_capacity(TABLE_N);

        let ar = s.aspect_ratio.clamp(0.1, VITERNA_AR_MAX);
        // Viterna & Corrigan (1981) eq. 4: the flat-plate drag the section reaches
        // at 90 deg. Everything post-stall is anchored to this one number.
        let cd_max = 1.11 + 0.018 * ar;

        // The stall points the extrapolation is matched to, in coefficient space.
        let pos = stall_point(s, s.alpha_stall);
        let neg = stall_point(s, s.alpha_stall_neg);

        for i in 0..TABLE_N {
            let a_deg = -180.0 + TABLE_STEP_DEG * i as f64;
            let a = a_deg.to_radians();
            let c = section_coeffs(s, a, cd_max, &pos, &neg);
            cl.push(c.cl);
            cd.push(c.cd);
            cm.push(c.cm);
        }
        AirfoilTable { cl, cd, cm }
    }

    /// Build from measured or panel-code samples: `(alpha_rad, cl, cd, cm)`
    /// tuples, which need not be evenly spaced but MUST be sorted ascending and
    /// span -180..180. This is the path an AVL / XFOIL / wind-tunnel polar takes,
    /// and it is the one a validated airframe should end up on.
    pub fn from_samples(samples: &[(f64, f64, f64, f64)]) -> AirfoilTable {
        let mut cl = Vec::with_capacity(TABLE_N);
        let mut cd = Vec::with_capacity(TABLE_N);
        let mut cm = Vec::with_capacity(TABLE_N);
        for i in 0..TABLE_N {
            let a = (-180.0 + TABLE_STEP_DEG * i as f64).to_radians();
            let (l, d, m) = interp_samples(samples, a);
            cl.push(l);
            cd.push(d);
            cm.push(m);
        }
        AirfoilTable { cl, cd, cm }
    }

    /// Linear interpolation at an arbitrary angle. `alpha` is wrapped into
    /// -pi..pi first, so a tumbling airframe never indexes off the end.
    pub fn at(&self, alpha: f64) -> Coeffs {
        let a_deg = wrap_pi(alpha).to_degrees();
        let x = (a_deg + 180.0) / TABLE_STEP_DEG;
        let i = (x.floor() as isize).clamp(0, TABLE_N as isize - 1) as usize;
        let j = (i + 1).min(TABLE_N - 1);
        let f = (x - i as f64).clamp(0.0, 1.0);
        Coeffs {
            cl: self.cl[i] + (self.cl[j] - self.cl[i]) * f,
            cd: self.cd[i] + (self.cd[j] - self.cd[i]) * f,
            cm: self.cm[i] + (self.cm[j] - self.cm[i]) * f,
        }
    }
}

/// CL and CD at a stall angle, from the linear model, used as the Viterna match
/// point. Kept separate because both signs need it and getting the two ends from
/// different formulas is how post-stall curves end up discontinuous.
fn stall_point(s: &AirfoilSpec, alpha_stall: f64) -> (f64, f64, f64) {
    let cl = s.cl_alpha * (alpha_stall - s.alpha_0);
    let cd = s.cd_min + s.cd_k * cl * cl;
    (alpha_stall, cl, cd)
}

/// The full-circle section model at one angle.
fn section_coeffs(
    s: &AirfoilSpec,
    alpha: f64,
    cd_max: f64,
    pos: &(f64, f64, f64),
    neg: &(f64, f64, f64),
) -> Coeffs {
    let a = wrap_pi(alpha);

    // Attached flow: the linear region, which is the only part with real data
    // behind it.
    if a <= pos.0 && a >= neg.0 {
        let cl = s.cl_alpha * (a - s.alpha_0);
        let cd = s.cd_min + s.cd_k * cl * cl;
        return Coeffs { cl, cd, cm: s.cm_0 };
    }

    // Post-stall. Viterna is derived for the first quadrant, so fold the angle
    // into 0..90 and restore the sign afterwards. Reflecting about 90 deg (a
    // reversed-flow section) rather than deriving a separate model is what
    // AeroDyn does, and it is why the curve stays continuous at 90 and 180.
    let sign = if a >= 0.0 { 1.0 } else { -1.0 };
    let match_point = if a >= 0.0 { pos } else { neg };
    let mag = a.abs();
    // Reflected angle: past 90 deg the section is flying backwards, and the
    // magnitude that matters is the angle from the REVERSED chord.
    let (folded, reversed) = if mag <= PI / 2.0 { (mag, false) } else { (PI - mag, true) };

    let (a_s, cl_s, cd_s) = (match_point.0.abs(), match_point.1.abs(), match_point.2);

    // Viterna carries a 1/sin(alpha) term, so it diverges as the folded angle
    // approaches zero, which is |alpha| -> 180: fully reversed flow. That region
    // is not post-stall at all, it is attached flow over a backwards aerofoil, so
    // Viterna has no business there. Below the stall angle on the reversed side,
    // interpolate between the Viterna value at the match point and the physical
    // values at exactly 180 (no lift, and the drag of a trailing-edge-first
    // section). Anchoring both ends keeps the full circle continuous.
    let (cl_v, cd_v) = if reversed && folded < a_s {
        let (cl_s_v, cd_s_v) = viterna(a_s, a_s, cl_s, cd_s, cd_max);
        // A reversed section is draggy even when attached; 2x the forward
        // minimum is the usual order for a blunt trailing edge leading.
        let cd_180 = cd_s.min(cd_max) * 0.0 + REVERSED_CD_FACTOR * cd_min_of(cd_s, cd_max);
        let f = (folded / a_s.max(1e-6)).clamp(0.0, 1.0);
        (cl_s_v * f, cd_180 + (cd_s_v - cd_180) * f)
    } else {
        viterna(folded, a_s, cl_s, cd_s, cd_max)
    };

    // A reversed section still produces lift toward its suction side, but a
    // trailing-edge-first aerofoil is a poor one, so the reversed branch is
    // scaled down. 0.8 is the flat-plate-like value AeroDyn's reversed branch
    // converges to and it keeps CL continuous through 90 deg (where CL is ~0).
    let cl = sign * cl_v * if reversed { -0.8 } else { 1.0 };
    let cd = cd_v;

    // Centre of pressure migrates from the quarter chord toward mid-chord as the
    // flow separates, so the moment about c/4 grows nose-down with normal force.
    // -0.25 * CN is the flat-plate limit (CP at c/2, a quarter chord aft).
    let cn = cl * folded.cos() + cd * folded.sin();
    let blend = ((mag - a_s) / (PI / 2.0 - a_s).max(1e-6)).clamp(0.0, 1.0);
    let cm = s.cm_0 * (1.0 - blend) - 0.25 * cn * blend;

    Coeffs { cl, cd, cm }
}

/// Viterna-Corrigan post-stall extrapolation, first quadrant only.
///
/// Viterna, L.A. and Corrigan, R.D., "Fixed Pitch Rotor Performance of Large
/// Horizontal Axis Wind Turbines", NASA CP-2230 (1981). `alpha`, `alpha_stall`
/// in radians and both in 0..pi/2; `cl_stall`/`cd_stall` are the coefficients at
/// the match point.
pub fn viterna(alpha: f64, alpha_stall: f64, cl_stall: f64, cd_stall: f64, cd_max: f64) -> (f64, f64) {
    let sa = alpha_stall.sin();
    let ca = alpha_stall.cos();
    // A stall angle at or past 90 deg leaves no post-stall region to fill and
    // divides by a vanishing cosine; fall back to the pure flat plate.
    if ca.abs() < 1e-6 || sa.abs() < 1e-6 {
        return (cd_max * 0.5 * (2.0 * alpha).sin(), cd_max * alpha.sin().powi(2));
    }
    let a1 = cd_max / 2.0;
    let b1 = cd_max;
    let a2 = (cl_stall - cd_max * sa * ca) * sa / (ca * ca);
    let b2 = (cd_stall - cd_max * sa * sa) / ca;

    let s = alpha.sin();
    let c = alpha.cos();
    // sin(alpha) in the denominator is why the match point must be off zero; the
    // caller guarantees that by construction (alpha >= alpha_stall > 0).
    let cl = a1 * (2.0 * alpha).sin() + a2 * c * c / s.max(1e-6);
    let cd = b1 * s * s + b2 * c;
    (cl, cd.max(0.0))
}

fn interp_samples(samples: &[(f64, f64, f64, f64)], a: f64) -> (f64, f64, f64) {
    if samples.is_empty() {
        return (0.0, 0.0, 0.0);
    }
    if a <= samples[0].0 {
        let s = samples[0];
        return (s.1, s.2, s.3);
    }
    if a >= samples[samples.len() - 1].0 {
        let s = samples[samples.len() - 1];
        return (s.1, s.2, s.3);
    }
    let k = samples.partition_point(|s| s.0 < a).max(1);
    let (a0, l0, d0, m0) = samples[k - 1];
    let (a1, l1, d1, m1) = samples[k];
    let f = if (a1 - a0).abs() < 1e-12 { 0.0 } else { (a - a0) / (a1 - a0) };
    (l0 + (l1 - l0) * f, d0 + (d1 - d0) * f, m0 + (m1 - m0) * f)
}

/// Wrap an angle into -pi..pi.
pub fn wrap_pi(a: f64) -> f64 {
    let mut x = (a + PI) % (2.0 * PI);
    if x < 0.0 {
        x += 2.0 * PI;
    }
    x - PI
}

// ─── Control surfaces ───────────────────────────────────────────────────────

/// A hinged control surface on a strip. Deflection shifts the section's
/// zero-lift angle, which is thin-airfoil theory's result for a plain flap and
/// is why an elevon needs no separate lift curve of its own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlLink {
    /// Index into the control-deflection vector passed to `surface_forces`.
    pub channel: usize,
    /// Sign and gearing applied to that channel's deflection for THIS strip. An
    /// elevon pair is one channel with +1 on one wing and -1 on the other for
    /// roll, so the same mechanism covers ailerons, elevons and ruddervators.
    pub gain: f64,
    /// Hinge position as a fraction of chord (0.25 = a quarter-chord flap).
    pub chord_fraction: f64,
    /// Mechanical deflection limit, radians.
    pub max_deflect: f64,
}

/// Thin-airfoil plain-flap effectiveness tau: the fraction of a flap deflection
/// that appears as a shift in the zero-lift angle.
///
/// tau = 1 - (theta_f - sin theta_f)/pi with theta_f = acos(2*E - 1), E the flap
/// chord fraction. This is a real result, not a fitted curve: it is why a 25%
/// flap is about 55% effective and why making a control surface bigger has badly
/// diminishing returns.
pub fn flap_effectiveness(chord_fraction: f64) -> f64 {
    let e = chord_fraction.clamp(0.0, 1.0);
    let theta = (2.0 * e - 1.0).clamp(-1.0, 1.0).acos();
    1.0 - (theta - theta.sin()) / PI
}

// ─── Surfaces ───────────────────────────────────────────────────────────────

/// One strip of lifting surface, in body frame.
#[derive(Debug, Clone, Copy)]
pub struct Surface {
    /// Quarter-chord midpoint of the strip relative to the CG, body frame (m).
    pub position: Vec3,
    /// Planform area of the strip (m^2).
    pub area: f64,
    /// Streamwise chord of the strip (m). Sets the pitching-moment arm.
    pub chord: f64,
    /// Unit chordwise direction: where the section points at zero incidence.
    /// Body +x for a conventional wing.
    pub forward: Vec3,
    /// Unit normal on the suction side: the direction positive lift acts at
    /// alpha = 0. Body -z (up) for a conventional wing.
    pub normal: Vec3,
    /// Built-in rig angle about the span axis, radians, positive leading edge up.
    /// Carries both wing incidence and washout.
    pub incidence: f64,
    /// Index into the airfoil table array.
    pub airfoil: usize,
    /// Hinged control surface on this strip, if any.
    pub control: Option<ControlLink>,
    /// Fraction 0..1 of this strip that lies in rotor slipstream. Applied to the
    /// slipstream velocity the strip sees, so a partially washed strip is not
    /// treated as fully immersed.
    pub wash_fraction: f64,
}

impl Surface {
    /// A strip on a conventional level wing: chord along body +x, lift up.
    pub fn wing_strip(position: Vec3, area: f64, chord: f64, airfoil: usize) -> Surface {
        Surface {
            position,
            area,
            chord,
            forward: Vec3::new(1.0, 0.0, 0.0),
            normal: Vec3::new(0.0, 0.0, -1.0),
            incidence: 0.0,
            airfoil,
            control: None,
            wash_fraction: 0.0,
        }
    }

    /// A strip on a vertical surface (fin / rudder): chord along body +x, lift to
    /// the LEFT so a positive sideslip produces a restoring yaw moment.
    pub fn fin_strip(position: Vec3, area: f64, chord: f64, airfoil: usize) -> Surface {
        Surface {
            position,
            area,
            chord,
            forward: Vec3::new(1.0, 0.0, 0.0),
            normal: Vec3::new(0.0, -1.0, 0.0),
            incidence: 0.0,
            airfoil,
            control: None,
            wash_fraction: 0.0,
        }
    }

    /// Span axis, from the chordwise and normal directions. Right wing for a
    /// conventional strip, and the axis a positive (nose-up) pitching moment
    /// acts about.
    pub fn span_axis(&self) -> Vec3 {
        self.forward.cross(self.normal).normalize()
    }
}

/// Per-strip diagnostics. Pure observability: nothing here feeds back into the
/// physics. This is the "physics X-ray" for aero, and it is what a partner needs
/// to see when the sim and their flight test disagree, because a total force
/// tells you nothing about WHICH surface was wrong.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceDiag {
    pub alpha: f64,
    /// Dynamic pressure at the strip (Pa), after slipstream.
    pub q: f64,
    pub cl: f64,
    pub cd: f64,
    pub force_bf: Vec3,
    /// True once the strip is past its table's linear region.
    pub stalled: bool,
}

/// Total aerodynamic force and moment from a set of strips.
#[derive(Debug, Clone, Default)]
pub struct AeroOutput {
    /// Body-frame force at the CG (N).
    pub force_bf: Vec3,
    /// Body-frame moment about the CG (N m).
    pub moment_bf: Vec3,
    /// Per-strip diagnostics, in the order the strips were supplied.
    pub diag: Vec<SurfaceDiag>,
    /// Fraction 0..1 of total strip AREA that is stalled. A single number a
    /// partner can put on a plot next to their flight log.
    pub stalled_area_fraction: f64,
}

/// Everything the strip sum needs about the airflow, so the caller owns where
/// wind, wake and slipstream come from and this stays pure.
pub struct FlowField<'a> {
    /// Vehicle velocity relative to the air mass, body frame (m/s).
    pub velocity_air_bf: Vec3,
    /// Body rates (rad/s).
    pub gyro: Vec3,
    pub air_density: f64,
    /// Extra local flow at a body-frame point, body frame: rotor slipstream,
    /// neighbour wake, gust structure. Returns the velocity of the AIR relative
    /// to the airframe, so a downward prop wash on a wing below the disc is +z.
    pub induced: &'a dyn Fn(Vec3) -> Vec3,
}

/// Zero induced flow, for tests and for a strip set with no rotors near it.
pub fn no_induced(_p: Vec3) -> Vec3 {
    Vec3::zero()
}

/// Integrate forces over the strips.
///
/// `controls` is indexed by `ControlLink::channel` and carries deflections in
/// radians, already mixed by the caller (the flight controller's mixer output,
/// not raw stick).
pub fn surface_forces(
    surfaces: &[Surface],
    airfoils: &[AirfoilTable],
    controls: &[f64],
    flow: &FlowField,
) -> AeroOutput {
    let mut force = Vec3::zero();
    let mut moment = Vec3::zero();
    let mut diag = Vec::with_capacity(surfaces.len());
    let mut area_total = 0.0;
    let mut area_stalled = 0.0;

    for s in surfaces {
        // Local flow: vehicle motion, the strip's own velocity from body rates,
        // and whatever the caller induces there. The rotation term is why a strip
        // model damps in roll and pitch without a separate damping derivative.
        let v_rot = flow.gyro.cross(s.position);
        let v_ind = if s.wash_fraction > 0.0 {
            (flow.induced)(s.position).scale(s.wash_fraction)
        } else {
            Vec3::zero()
        };
        // `induced` is the air's velocity relative to the airframe; the strip's
        // velocity relative to the AIR is therefore minus that.
        let v = flow.velocity_air_bf.add(v_rot).sub(v_ind);

        // Rig the section: incidence rotates the chord line about the span axis.
        let span = s.span_axis();
        let (fwd, nrm) = rotate_about(s.forward, s.normal, span, s.incidence);

        let u = v.dot(fwd);
        let w = -v.dot(nrm);
        let v_plane_sq = u * u + w * w;
        if v_plane_sq < 1e-9 {
            diag.push(SurfaceDiag {
                alpha: 0.0,
                q: 0.0,
                cl: 0.0,
                cd: 0.0,
                force_bf: Vec3::zero(),
                stalled: false,
            });
            area_total += s.area;
            continue;
        }
        // Strip theory: only the component in the section plane produces section
        // forces. The spanwise component is dropped (independence principle).
        let alpha = w.atan2(u);
        let q = 0.5 * flow.air_density * v_plane_sq;

        let table = &airfoils[s.airfoil.min(airfoils.len().saturating_sub(1))];
        let mut c = table.at(alpha);

        // Control deflection as a zero-lift shift, evaluated by re-reading the
        // table at the equivalent angle rather than by adding a delta-CL. Doing
        // it at the table keeps the control effective in the linear region and
        // correctly INEFFECTIVE once the strip has stalled, which is the whole
        // reason to model post-stall at all.
        let mut deflect = 0.0;
        if let Some(link) = &s.control {
            let raw = controls.get(link.channel).copied().unwrap_or(0.0) * link.gain;
            deflect = raw.clamp(-link.max_deflect, link.max_deflect);
            if deflect != 0.0 {
                let tau = flap_effectiveness(link.chord_fraction);
                c = table.at(alpha + tau * deflect);
                // Hinge-gap and separation drag, quadratic in deflection. Small,
                // but it is what makes a hard aileron input cost airspeed.
                c.cd += 0.02 * deflect * deflect / link.chord_fraction.max(0.05);
            }
        }

        let l = q * s.area * c.cl;
        let d = q * s.area * c.cd;
        let m = q * s.area * s.chord * c.cm;

        // In-plane velocity direction, and lift normal to it.
        let v_ip = fwd.scale(u).sub(nrm.scale(w));
        let drag_dir = v_ip.normalize();
        let lift_dir = span.cross(drag_dir).normalize();

        let f = lift_dir.scale(l).sub(drag_dir.scale(d));
        force = force.add(f);
        moment = moment.add(span.scale(m)).add(s.position.cross(f));

        // "Stalled" is defined off the table, not off a fixed angle, so a section
        // with a wide linear region is not reported as stalled early.
        let stalled = is_stalled(table, alpha + deflect);
        area_total += s.area;
        if stalled {
            area_stalled += s.area;
        }
        diag.push(SurfaceDiag { alpha, q, cl: c.cl, cd: c.cd, force_bf: f, stalled });
    }

    AeroOutput {
        force_bf: force,
        moment_bf: moment,
        diag,
        stalled_area_fraction: if area_total > 0.0 { area_stalled / area_total } else { 0.0 },
    }
}

/// A strip counts as stalled when moving one more degree into the angle no longer
/// increases lift. Reading the slope off the table means this works for a
/// measured polar as well as a parametric one.
fn is_stalled(table: &AirfoilTable, alpha: f64) -> bool {
    let step = TABLE_STEP_DEG.to_radians();
    let here = table.at(alpha).cl;
    let further = table.at(alpha + step * alpha.signum()).cl;
    (further.abs() < here.abs() - 1e-9) && here.abs() > 1e-6
}

/// Rotate a (forward, normal) pair about the span axis by `angle`, Rodrigues.
/// Positive angle raises the leading edge, i.e. adds incidence.
fn rotate_about(fwd: Vec3, nrm: Vec3, axis: Vec3, angle: f64) -> (Vec3, Vec3) {
    if angle == 0.0 {
        return (fwd, nrm);
    }
    let rot = |v: Vec3| {
        let (s, c) = angle.sin_cos();
        v.scale(c).add(axis.cross(v).scale(s)).add(axis.scale(axis.dot(v) * (1.0 - c)))
    };
    // Negative angle about the span axis pitches the chord line UP, because the
    // span axis points out the right wing and a nose-up rotation about it is
    // positive: incidence is a nose-up rig, so the chord rotates with +angle.
    (rot(fwd).normalize(), rot(nrm).normalize())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sym() -> AirfoilSpec {
        AirfoilSpec { alpha_0: 0.0, cm_0: 0.0, ..AirfoilSpec::default() }
    }

    fn flow(v: Vec3) -> FlowField<'static> {
        FlowField {
            velocity_air_bf: v,
            gyro: Vec3::zero(),
            air_density: 1.225,
            induced: &no_induced,
        }
    }

    /// One 10 m^2 wing at the CG, so force is isolated from moment.
    fn one_wing() -> Vec<Surface> {
        vec![Surface::wing_strip(Vec3::zero(), 10.0, 1.0, 0)]
    }

    #[test]
    fn symmetric_section_has_no_lift_at_zero_alpha() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let out = surface_forces(&one_wing(), &t, &[], &flow(Vec3::new(30.0, 0.0, 0.0)));
        assert!(out.force_bf.z.abs() < 1e-6, "lift {}", out.force_bf.z);
        // Drag opposes motion: body +x forward means drag is -x.
        assert!(out.force_bf.x < 0.0, "drag {}", out.force_bf.x);
    }

    /// The sign that everything else rests on: nose-up alpha lifts UP, which in
    /// FRD is negative z. Getting this backwards inverts the whole aircraft and
    /// still produces a stable-looking trim, so it is asserted directly.
    #[test]
    fn positive_alpha_lifts_upward() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        // +z velocity component = descending relative to the air = air from below.
        let out = surface_forces(&one_wing(), &t, &[], &flow(Vec3::new(30.0, 0.0, 3.0)));
        assert!(out.diag[0].alpha > 0.0, "alpha {}", out.diag[0].alpha);
        assert!(out.force_bf.z < 0.0, "expected upward lift, got z {}", out.force_bf.z);
    }

    #[test]
    fn negative_alpha_lifts_downward() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let out = surface_forces(&one_wing(), &t, &[], &flow(Vec3::new(30.0, 0.0, -3.0)));
        assert!(out.force_bf.z > 0.0, "expected downward lift, got z {}", out.force_bf.z);
    }

    #[test]
    fn lift_grows_with_the_square_of_speed() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let a = surface_forces(&one_wing(), &t, &[], &flow(Vec3::new(10.0, 0.0, 1.0)));
        let b = surface_forces(&one_wing(), &t, &[], &flow(Vec3::new(20.0, 0.0, 2.0)));
        // Same alpha, double the speed: four times the force.
        let r = b.force_bf.z / a.force_bf.z;
        assert!((r - 4.0).abs() < 1e-6, "ratio {r}");
    }

    // ─── The full circle ────────────────────────────────────────────────────

    /// A VTOL hovers at alpha = 90 and tumbles through 180 in a bad transition.
    /// The table must be finite and continuous everywhere, or the FDM detonates
    /// exactly when the customer is testing the case they bought this for.
    #[test]
    fn table_is_finite_and_continuous_over_the_whole_circle() {
        for spec in [sym(), AirfoilSpec::default()] {
            let t = AirfoilTable::from_spec(&spec);
            let mut prev: Option<Coeffs> = None;
            let mut a_deg: f64 = -180.0;
            while a_deg <= 180.0 {
                let c = t.at(a_deg.to_radians());
                assert!(c.cl.is_finite() && c.cd.is_finite() && c.cm.is_finite(), "non-finite at {a_deg}");
                assert!(c.cd >= 0.0, "negative drag {} at {a_deg}", c.cd);
                assert!(c.cl.abs() < 3.0, "runaway cl {} at {a_deg}", c.cl);
                assert!(c.cd < 3.0, "runaway cd {} at {a_deg}", c.cd);
                if let Some(p) = prev {
                    // No step in the coefficients larger than what a real polar
                    // shows per quarter degree. This is the assertion that caught
                    // Viterna's 1/sin singularity at 180.
                    assert!((c.cl - p.cl).abs() < 0.15, "cl jump {} -> {} at {a_deg}", p.cl, c.cl);
                    assert!((c.cd - p.cd).abs() < 0.15, "cd jump {} -> {} at {a_deg}", p.cd, c.cd);
                }
                prev = Some(c);
                a_deg += 0.25;
            }
        }
    }

    /// At 90 deg the section is a flat plate: all drag, no lift, and the drag
    /// coefficient is Viterna's CD_max for the aspect ratio.
    #[test]
    fn ninety_degrees_is_a_flat_plate() {
        let s = sym();
        let t = AirfoilTable::from_spec(&s);
        let c = t.at(PI / 2.0);
        let cd_max = 1.11 + 0.018 * s.aspect_ratio;
        assert!(c.cl.abs() < 0.02, "cl at 90 deg {}", c.cl);
        assert!((c.cd - cd_max).abs() < 0.05, "cd at 90 deg {} vs cd_max {cd_max}", c.cd);
    }

    #[test]
    fn one_eighty_is_reversed_attached_flow_not_a_flat_plate() {
        let t = AirfoilTable::from_spec(&sym());
        let c = t.at(PI);
        assert!(c.cl.abs() < 0.05, "cl at 180 {}", c.cl);
        // Draggier than flying forwards, far less draggy than broadside.
        assert!(c.cd > 0.005 && c.cd < 0.3, "cd at 180 {}", c.cd);
    }

    #[test]
    fn viterna_reproduces_its_match_point_exactly() {
        let (cl, cd) = viterna(0.26, 0.26, 1.3, 0.045, 1.2);
        assert!((cl - 1.3).abs() < 1e-9, "cl {cl}");
        assert!((cd - 0.045).abs() < 1e-9, "cd {cd}");
    }

    #[test]
    fn lift_peaks_at_the_stall_angle() {
        let s = sym();
        let t = AirfoilTable::from_spec(&s);
        let at_stall = t.at(s.alpha_stall).cl;
        for d in [2.0f64, 5.0, 10.0, 20.0] {
            let past = t.at(s.alpha_stall + d.to_radians()).cl;
            assert!(past < at_stall, "cl rose past stall at +{d} deg: {past} vs {at_stall}");
        }
        let before = t.at(s.alpha_stall - 5.0f64.to_radians()).cl;
        assert!(before < at_stall, "cl should still be climbing before stall");
    }

    /// A cambered section lifts at zero alpha and stalls asymmetrically.
    #[test]
    fn camber_shifts_the_zero_lift_angle() {
        let t = AirfoilTable::from_spec(&AirfoilSpec::default());
        assert!(t.at(0.0).cl > 0.1, "cambered section should lift at alpha 0");
        assert!(t.at(AirfoilSpec::default().alpha_0).cl.abs() < 1e-6, "no lift at alpha_0");
    }

    // ─── Moments and rate damping ───────────────────────────────────────────

    /// A tail behind the CG must pitch the nose DOWN at positive alpha. That is
    /// static longitudinal stability, and it is the single check that says the
    /// moment signs are self-consistent.
    #[test]
    fn tail_behind_the_cg_is_pitch_stable() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let tail = vec![Surface::wing_strip(Vec3::new(-1.5, 0.0, 0.0), 1.0, 0.3, 0)];
        let up = surface_forces(&tail, &t, &[], &flow(Vec3::new(30.0, 0.0, 3.0)));
        assert!(up.moment_bf.y < 0.0, "nose-up alpha needs a nose-down moment, got {}", up.moment_bf.y);
        let down = surface_forces(&tail, &t, &[], &flow(Vec3::new(30.0, 0.0, -3.0)));
        assert!(down.moment_bf.y > 0.0, "and the reverse, got {}", down.moment_bf.y);
    }

    /// Roll damping falls out of the rotation term with no damping derivative
    /// anywhere in the code. If this fails, `gyro.cross(position)` has the wrong
    /// sign and the aircraft will diverge in roll.
    #[test]
    fn a_roll_rate_is_damped_by_the_wings() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let wings = vec![
            Surface::wing_strip(Vec3::new(0.0, 1.5, 0.0), 1.0, 0.3, 0),
            Surface::wing_strip(Vec3::new(0.0, -1.5, 0.0), 1.0, 0.3, 0),
        ];
        let f = FlowField {
            velocity_air_bf: Vec3::new(30.0, 0.0, 0.0),
            gyro: Vec3::new(1.0, 0.0, 0.0),
            air_density: 1.225,
            induced: &no_induced,
        };
        let out = surface_forces(&wings, &t, &[], &f);
        assert!(out.moment_bf.x < 0.0, "roll rate +x needs a -x moment, got {}", out.moment_bf.x);
    }

    #[test]
    fn a_pitch_rate_is_damped_by_the_tail() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let tail = vec![Surface::wing_strip(Vec3::new(-1.5, 0.0, 0.0), 1.0, 0.3, 0)];
        let f = FlowField {
            velocity_air_bf: Vec3::new(30.0, 0.0, 0.0),
            gyro: Vec3::new(0.0, 1.0, 0.0),
            air_density: 1.225,
            induced: &no_induced,
        };
        let out = surface_forces(&tail, &t, &[], &f);
        assert!(out.moment_bf.y < 0.0, "pitch rate +y needs a -y moment, got {}", out.moment_bf.y);
    }

    /// Weathercock stability: a fin behind the CG yaws the nose into a sideslip.
    #[test]
    fn fin_behind_the_cg_is_yaw_stable() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let fin = vec![Surface::fin_strip(Vec3::new(-1.5, 0.0, -0.3), 0.5, 0.3, 0)];
        // +y velocity = slipping to the right = air arriving from the right.
        let out = surface_forces(&fin, &t, &[], &flow(Vec3::new(30.0, 3.0, 0.0)));
        assert!(out.moment_bf.z > 0.0, "should yaw nose right into the slip, got {}", out.moment_bf.z);
    }

    // ─── Control surfaces ───────────────────────────────────────────────────

    /// Thin-airfoil theory: a 25% chord flap is roughly 60% effective, and the
    /// curve is strongly concave, which is why bigger control surfaces pay so
    /// poorly.
    #[test]
    fn flap_effectiveness_matches_thin_airfoil_theory() {
        let t25 = flap_effectiveness(0.25);
        assert!((0.55..0.68).contains(&t25), "tau(0.25) = {t25}");
        assert!(flap_effectiveness(0.0).abs() < 1e-9);
        assert!((flap_effectiveness(1.0) - 1.0).abs() < 1e-9);
        // Doubling the flap from 25% to 50% is far less than double the effect.
        assert!(flap_effectiveness(0.5) < 2.0 * t25);
        // Monotone.
        let mut prev = -1.0;
        for i in 0..=20 {
            let v = flap_effectiveness(i as f64 / 20.0);
            assert!(v >= prev - 1e-12, "not monotone at {i}");
            prev = v;
        }
    }

    #[test]
    fn elevator_deflection_pitches_the_aircraft() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let mut tail = Surface::wing_strip(Vec3::new(-1.5, 0.0, 0.0), 1.0, 0.3, 0);
        tail.control = Some(ControlLink { channel: 0, gain: 1.0, chord_fraction: 0.35, max_deflect: 0.5 });
        let s = vec![tail];
        let f = flow(Vec3::new(30.0, 0.0, 0.0));
        let neutral = surface_forces(&s, &t, &[0.0], &f);
        // Trailing edge DOWN on the tail lifts the tail, which pitches nose down.
        let down = surface_forces(&s, &t, &[0.2], &f);
        assert!(neutral.moment_bf.y.abs() < 1e-9);
        assert!(down.moment_bf.y < 0.0, "moment {}", down.moment_bf.y);
        let up = surface_forces(&s, &t, &[-0.2], &f);
        assert!(up.moment_bf.y > 0.0, "moment {}", up.moment_bf.y);
    }

    /// Opposite gains on the two wings turn one channel into ailerons. This is
    /// the whole elevon mechanism, so it is asserted rather than assumed.
    #[test]
    fn opposite_gains_make_one_channel_roll() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let mk = |y: f64, gain: f64| {
            let mut s = Surface::wing_strip(Vec3::new(0.0, y, 0.0), 1.0, 0.3, 0);
            s.control = Some(ControlLink { channel: 0, gain, chord_fraction: 0.25, max_deflect: 0.5 });
            s
        };
        let wings = vec![mk(1.5, 1.0), mk(-1.5, -1.0)];
        let out = surface_forces(&wings, &t, &[0.2], &flow(Vec3::new(30.0, 0.0, 0.0)));
        assert!(out.moment_bf.x.abs() > 1e-3, "should roll, got {}", out.moment_bf.x);
        // Pure roll: the two wings' lift changes cancel in heave.
        assert!(out.force_bf.z.abs() < 1e-6, "should not heave, got {}", out.force_bf.z);
    }

    /// Controls must lose authority once the strip has stalled. This is the
    /// reason the deflection is applied by re-reading the table rather than by
    /// adding a delta-CL, and it is the difference between a sim that warns you
    /// about a departure and one that lets you fly out of it.
    #[test]
    fn control_authority_collapses_past_the_stall() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let mut w = Surface::wing_strip(Vec3::zero(), 10.0, 1.0, 0);
        w.control = Some(ControlLink { channel: 0, gain: 1.0, chord_fraction: 0.25, max_deflect: 0.4 });
        let s = vec![w];
        let authority = |v: Vec3| {
            let a = surface_forces(&s, &t, &[0.0], &flow(v)).force_bf.z;
            let b = surface_forces(&s, &t, &[0.3], &flow(v)).force_bf.z;
            (b - a).abs()
        };
        let attached = authority(Vec3::new(30.0, 0.0, 1.0));
        // 45 deg: deep post-stall, well inside the Viterna branch.
        let stalled = authority(Vec3::new(30.0, 0.0, 30.0));
        assert!(stalled < attached * 0.5, "attached {attached}, stalled {stalled}");
    }

    #[test]
    fn deflection_is_clamped_to_the_mechanical_limit() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let mut w = Surface::wing_strip(Vec3::zero(), 10.0, 1.0, 0);
        w.control = Some(ControlLink { channel: 0, gain: 1.0, chord_fraction: 0.25, max_deflect: 0.2 });
        let s = vec![w];
        let f = flow(Vec3::new(30.0, 0.0, 0.0));
        let at_limit = surface_forces(&s, &t, &[0.2], &f).force_bf.z;
        let way_past = surface_forces(&s, &t, &[5.0], &f).force_bf.z;
        assert!((at_limit - way_past).abs() < 1e-9);
    }

    // ─── Slipstream ─────────────────────────────────────────────────────────

    /// A wing in prop wash sees more dynamic pressure than the airframe's own
    /// airspeed, which is how a VTOL keeps control authority at zero airspeed.
    #[test]
    fn slipstream_raises_dynamic_pressure_on_a_washed_strip() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let mut w = Surface::wing_strip(Vec3::new(0.5, 1.0, 0.0), 1.0, 0.3, 0);
        w.wash_fraction = 1.0;
        let s = vec![w];
        // 15 m/s of air moving backwards over the wing (body -x), i.e. a tractor
        // prop ahead of it, with the airframe barely moving.
        let wash = |_p: Vec3| Vec3::new(-15.0, 0.0, 0.0);
        let f = FlowField {
            velocity_air_bf: Vec3::new(2.0, 0.0, 0.5),
            gyro: Vec3::zero(),
            air_density: 1.225,
            induced: &wash,
        };
        let with_wash = surface_forces(&s, &t, &[], &f);
        let without = surface_forces(&s, &t, &[], &flow(Vec3::new(2.0, 0.0, 0.5)));
        assert!(with_wash.diag[0].q > without.diag[0].q * 20.0,
            "washed q {} vs clean {}", with_wash.diag[0].q, without.diag[0].q);
    }

    #[test]
    fn wash_fraction_scales_the_induced_flow() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let mk = |frac: f64| {
            let mut w = Surface::wing_strip(Vec3::zero(), 1.0, 0.3, 0);
            w.wash_fraction = frac;
            vec![w]
        };
        let wash = |_p: Vec3| Vec3::new(-20.0, 0.0, 0.0);
        let f = FlowField {
            velocity_air_bf: Vec3::new(5.0, 0.0, 0.0),
            gyro: Vec3::zero(),
            air_density: 1.225,
            induced: &wash,
        };
        let q_full = surface_forces(&mk(1.0), &t, &[], &f).diag[0].q;
        let q_half = surface_forces(&mk(0.5), &t, &[], &f).diag[0].q;
        let q_none = surface_forces(&mk(0.0), &t, &[], &f).diag[0].q;
        assert!(q_full > q_half && q_half > q_none, "{q_full} {q_half} {q_none}");
    }

    // ─── Hover and robustness ───────────────────────────────────────────────

    /// The case that decides whether a VTOL sim is usable at all: hanging on the
    /// rotors at zero airspeed, and descending vertically at 90 deg alpha.
    #[test]
    fn hover_and_vertical_flight_are_well_behaved() {
        let t = vec![AirfoilTable::from_spec(&AirfoilSpec::default())];
        let s = one_wing();
        for v in [
            Vec3::zero(),
            Vec3::new(0.0, 0.0, 5.0),   // vertical descent: alpha = +90
            Vec3::new(0.0, 0.0, -5.0),  // vertical climb: alpha = -90
            Vec3::new(-20.0, 0.0, 0.0), // flying backwards: alpha = 180
            Vec3::new(0.0, 20.0, 0.0),  // pure sideways
        ] {
            let out = surface_forces(&s, &t, &[], &flow(v));
            assert!(out.force_bf.x.is_finite() && out.force_bf.y.is_finite() && out.force_bf.z.is_finite(),
                "non-finite force at v {v:?}");
            assert!(out.moment_bf.x.is_finite() && out.moment_bf.y.is_finite() && out.moment_bf.z.is_finite(),
                "non-finite moment at v {v:?}");
        }
        // At rest there is no air load at all.
        let still = surface_forces(&s, &t, &[], &flow(Vec3::zero()));
        assert_eq!(still.force_bf, Vec3::zero());
    }

    /// Vertical descent through a wing is nearly pure drag, and it is large: this
    /// is the download that makes a tailsitter descend badly, so it must not be
    /// silently zero.
    #[test]
    fn vertical_descent_is_mostly_drag_on_the_wing() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let out = surface_forces(&one_wing(), &t, &[], &flow(Vec3::new(0.0, 0.0, 5.0)));
        // Descending (+z): the wing pushes back up (-z), broadside.
        assert!(out.force_bf.z < 0.0, "z {}", out.force_bf.z);
        let cd_like = out.force_bf.z.abs() / (0.5 * 1.225 * 25.0 * 10.0);
        assert!(cd_like > 0.9, "broadside drag coefficient only {cd_like}");
    }

    #[test]
    fn stalled_area_fraction_reports_the_wing_state() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let s = one_wing();
        assert_eq!(surface_forces(&s, &t, &[], &flow(Vec3::new(30.0, 0.0, 0.5))).stalled_area_fraction, 0.0);
        assert_eq!(surface_forces(&s, &t, &[], &flow(Vec3::new(30.0, 0.0, 30.0))).stalled_area_fraction, 1.0);
    }

    #[test]
    fn incidence_adds_angle_of_attack() {
        let t = vec![AirfoilTable::from_spec(&sym())];
        let mut w = Surface::wing_strip(Vec3::zero(), 10.0, 1.0, 0);
        w.incidence = 3.0f64.to_radians();
        let rigged = surface_forces(&vec![w], &t, &[], &flow(Vec3::new(30.0, 0.0, 0.0)));
        assert!(rigged.force_bf.z < 0.0, "3 deg of rig should lift, got {}", rigged.force_bf.z);
        let plain = surface_forces(&one_wing(), &t, &[], &flow(Vec3::new(30.0, 0.0, 0.0)));
        assert!(plain.force_bf.z.abs() < 1e-6);
    }

    // ─── Finite-wing corrections ────────────────────────────────────────────

    #[test]
    fn finite_wing_slope_is_below_the_two_dimensional_value() {
        let a0 = 2.0 * PI;
        let a3 = finite_wing_slope(a0, 6.0, 0.85);
        assert!(a3 < a0 && a3 > 0.6 * a0, "AR 6 slope {a3} vs 2D {a0}");
        // A higher aspect ratio recovers more of the 2D slope.
        assert!(finite_wing_slope(a0, 20.0, 0.85) > a3);
        assert!((finite_wing_slope(a0, f64::INFINITY, 0.85) - a0).abs() < 1e-9);
    }

    #[test]
    fn induced_drag_falls_with_aspect_ratio() {
        assert!(induced_drag_k(6.0, 0.85) > induced_drag_k(20.0, 0.85));
        assert_eq!(induced_drag_k(0.0, 0.85), 0.0);
    }

    // ─── Tables from samples ────────────────────────────────────────────────

    #[test]
    fn sampled_polar_round_trips() {
        let samples = vec![
            (-PI, 0.0, 0.04, 0.0),
            (-0.2, -1.0, 0.03, -0.05),
            (0.0, 0.0, 0.02, -0.05),
            (0.2, 1.0, 0.03, -0.05),
            (PI, 0.0, 0.04, 0.0),
        ];
        let t = AirfoilTable::from_samples(&samples);
        assert!((t.at(0.0).cl).abs() < 0.02);
        assert!((t.at(0.2).cl - 1.0).abs() < 0.05, "cl {}", t.at(0.2).cl);
        assert!((t.at(0.1).cl - 0.5).abs() < 0.05, "midpoint should interpolate");
        // Outside the samples it holds the end value rather than extrapolating.
        assert!(t.at(4.0).cl.is_finite());
    }

    /// Wrapping must preserve the ANGLE, not a chosen representative: the
    /// interval is half open, so pi and -pi are both correct answers for pi and
    /// asserting either one is asserting an implementation detail.
    #[test]
    fn wrap_pi_folds_any_angle() {
        for a in [0.0, PI, -PI, 3.0 * PI, 2.5 * PI, -7.3, 100.0] {
            let w = wrap_pi(a);
            assert!(w >= -PI - 1e-12 && w <= PI + 1e-12, "wrap({a}) = {w} out of range");
            let turns = (a - w) / (2.0 * PI);
            assert!((turns - turns.round()).abs() < 1e-9, "wrap({a}) = {w} is not the same angle");
        }
        // And the table must agree at both ends, or the half-open interval would
        // put a discontinuity at exactly 180 deg.
        let t = AirfoilTable::from_spec(&sym());
        assert!((t.at(PI).cl - t.at(-PI).cl).abs() < 1e-9);
        assert!((t.at(PI).cd - t.at(-PI).cd).abs() < 1e-9);
    }
}
