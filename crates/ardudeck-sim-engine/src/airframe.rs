//! The airframe a partner describes, and the strips and rotors it builds into.
//!
//! This is the seam the whole model is worth having for: everything downstream
//! is derived from GEOMETRY a customer can read off their own drawing, not from
//! coefficients someone tuned until the sim matched a video. A description that
//! comes from the aircraft extrapolates to conditions it has never flown, which
//! is the entire point of testing before a first flight. A curve fit to logs
//! does not, however well it matches the logs it was fitted to.
//!
//! Deliberately NOT in this file: any whole-aircraft coefficient. There is no
//! CL_alpha for the vehicle, no Cm_q, no dihedral effect. Every one of those
//! falls out of the strip sum, which is why a wing half-immersed in prop wash
//! rolls and a whole-aircraft derivative set cannot make it.

use crate::aero::{
    finite_wing_slope, induced_drag_k, AirfoilSpec, AirfoilTable, ControlLink, Surface,
};
use crate::bemt::{MotorDrive, Rotor};
use crate::math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::f64::consts::PI;

/// How far downstream a rotor's slipstream is treated as coherent, in disc
/// radii. Past this it has mixed out enough that a wing in it is not meaningfully
/// blown. Short, because the terms that matter are wings sitting a chord or two
/// behind a lift rotor.
const WAKE_LENGTH_RADII: f64 = 6.0;
/// Downstream distance, in radii, over which the wake contracts to its fully
/// developed radius. Momentum theory gives the far-wake area as half the disc,
/// so the radius contracts by 1/sqrt(2).
const WAKE_CONTRACT_RADII: f64 = 2.0;

// ─── The description ────────────────────────────────────────────────────────

/// A complete airframe. This is the JSON a partner writes.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AirframeSpec {
    pub name: String,
    /// All-up mass (kg).
    pub mass: f64,
    /// Roll, pitch, yaw moments of inertia about the CG (kg m^2).
    pub inertia: [f64; 3],
    /// Named airfoil sections, referenced by wings and rotors.
    #[serde(default)]
    pub airfoils: HashMap<String, SectionSpec>,
    #[serde(default)]
    pub wings: Vec<WingSpec>,
    #[serde(default)]
    pub rotors: Vec<RotorSpec>,
    #[serde(default)]
    pub fuselage: FuselageSpec,
    /// Servo PWM band, for mapping SITL's output to deflections and throttle.
    #[serde(default)]
    pub pwm: PwmSpec,
    /// Full pack voltage (V).
    #[serde(default = "default_voltage")]
    pub voltage_max: f64,
}

fn default_voltage() -> f64 {
    22.2
}

/// A named 2D section. Either parametric, or a measured / panel-code polar.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum SectionSpec {
    /// A section by SHAPE. `naca` is a 4-digit designation ("0012", "2412");
    /// `reynolds` the chord Reynolds number to solve it at, which matters:
    /// stall angle and drag both move with it, and a 2 m model glider flies at
    /// a few hundred thousand where a light aircraft flies at a few million.
    ///
    /// The polar is SOLVED from these coordinates by `panel` and `bl`, so
    /// camber, thickness, leading-edge radius, stall angle and drag all come
    /// from the shape. This is the route a wing should use; the two below exist
    /// for measured data and for describing a section that has no shape.
    Naca { naca: String, #[serde(default)] reynolds: Option<f64> },
    /// `(alpha_deg, cl, cd, cm)` samples, sorted ascending. The path an AVL,
    /// XFOIL or wind-tunnel polar takes, and where a validated airframe should
    /// end up: it replaces the parametric guess with measurement.
    Polar { polar: Vec<[f64; 4]> },
    Parametric(ParametricSection),
}

/// Section described by the numbers a datasheet carries. Angles in DEGREES here
/// and radians everywhere inside, because a person writing this file thinks in
/// degrees and every unit error in this kind of code is at that boundary.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParametricSection {
    /// 2D lift-curve slope per radian. Thin-airfoil theory gives 2*pi; the
    /// finite-wing correction is applied per wing from its aspect ratio, so this
    /// stays the SECTION value.
    #[serde(default = "default_cl_alpha")]
    pub cl_alpha: f64,
    #[serde(default)]
    pub alpha_0_deg: f64,
    #[serde(default = "default_stall")]
    pub alpha_stall_deg: f64,
    #[serde(default = "default_stall_neg")]
    pub alpha_stall_neg_deg: f64,
    #[serde(default = "default_cd_min")]
    pub cd_min: f64,
    #[serde(default)]
    pub cd_k: f64,
    #[serde(default)]
    pub cm_0: f64,
}

fn default_cl_alpha() -> f64 {
    2.0 * PI
}
fn default_stall() -> f64 {
    14.0
}
fn default_stall_neg() -> f64 {
    -12.0
}
fn default_cd_min() -> f64 {
    0.012
}

impl Default for ParametricSection {
    fn default() -> Self {
        ParametricSection {
            cl_alpha: default_cl_alpha(),
            alpha_0_deg: 0.0,
            alpha_stall_deg: default_stall(),
            alpha_stall_neg_deg: default_stall_neg(),
            cd_min: default_cd_min(),
            cd_k: 0.0,
            cm_0: 0.0,
        }
    }
}

/// One lifting panel. `mirror` builds the matching panel on the other side, so a
/// wing is one entry rather than two that can drift apart.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WingSpec {
    pub name: String,
    /// Root quarter-chord position relative to the CG, body FRD (m).
    pub root: [f64; 3],
    /// Semi-span of ONE panel (m), root to tip.
    ///
    /// NEGATIVE on a `vertical` panel means it hangs DOWNWARD from its root,
    /// which is where a paper dart's keel and many a model's ventral fin
    /// actually are. The magnitude sets the area either way.
    pub semi_span: f64,
    pub chord_root: f64,
    pub chord_tip: f64,
    #[serde(default)]
    pub sweep_deg: f64,
    #[serde(default)]
    pub dihedral_deg: f64,
    /// Rig angle at the root, positive leading edge up.
    #[serde(default)]
    pub incidence_deg: f64,
    /// Washout: incidence CHANGE at the tip, normally negative so the tip stalls
    /// last. This is why a real wing drops a wing gently instead of departing,
    /// and a strip model reproduces it for free where a whole-aircraft model
    /// cannot represent it at all.
    #[serde(default)]
    pub twist_deg: f64,
    pub airfoil: String,
    /// Strips per panel. 6 to 10 resolves the spanwise loading and the partial
    /// prop-wash immersion that the whole model is here for.
    #[serde(default = "default_strips")]
    pub strips: usize,
    /// A fin rather than a wing: the panel stands vertically and its lift acts
    /// sideways.
    #[serde(default)]
    pub vertical: bool,
    /// Build the mirrored panel on -y. Always true for a wing or tailplane,
    /// false for a single centreline fin.
    #[serde(default = "default_true")]
    pub mirror: bool,
    #[serde(default = "default_oswald")]
    pub oswald: f64,
    #[serde(default)]
    pub controls: Vec<ControlSpec>,
}

fn default_strips() -> usize {
    8
}
fn default_true() -> bool {
    true
}
fn default_oswald() -> f64 {
    0.85
}

/// A hinged surface occupying part of a panel's span.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlSpec {
    pub name: String,
    /// Servo output index (0-based) this surface follows.
    pub channel: usize,
    /// Gearing and sign on the panel built at +y.
    #[serde(default = "default_one")]
    pub gain: f64,
    /// Gearing and sign on the MIRRORED panel. -1 against +1 makes an aileron
    /// or an elevon; +1 against +1 makes a flap or an elevator. One channel plus
    /// two gains covers every mixed surface a VTOL uses.
    #[serde(default = "default_one")]
    pub mirror_gain: f64,
    /// Inboard and outboard ends as a fraction of the semi-span.
    #[serde(default)]
    pub span_start: f64,
    #[serde(default = "default_one")]
    pub span_end: f64,
    #[serde(default = "default_chord_fraction")]
    pub chord_fraction: f64,
    #[serde(default = "default_max_deflect")]
    pub max_deflect_deg: f64,
}

fn default_one() -> f64 {
    1.0
}
fn default_chord_fraction() -> f64 {
    0.25
}
fn default_max_deflect() -> f64 {
    25.0
}

/// One rotor, its motor, and how the flight controller drives it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RotorSpec {
    pub name: String,
    pub position: [f64; 3],
    /// Thrust axis at zero tilt. `[0,0,-1]` lifts, `[1,0,0]` pushes.
    pub axis: [f64; 3],
    /// Axis tilt rotates about. A lift rotor tilting FORWARD uses `[0,-1,0]`;
    /// see `bemt::Rotor::tilt_axis`.
    #[serde(default)]
    pub tilt_axis: [f64; 3],
    pub radius: f64,
    #[serde(default = "default_blades")]
    pub blades: usize,
    pub chord_root: f64,
    pub chord_tip: f64,
    pub pitch_root_deg: f64,
    pub pitch_tip_deg: f64,
    pub airfoil: String,
    /// +1 or -1. The sum over all rotors should be near zero on a multirotor or
    /// it cannot hold heading without a permanent yaw offset.
    #[serde(default = "default_one")]
    pub spin: f64,
    /// Rotor plus prop polar inertia (kg m^2).
    pub inertia: f64,
    pub kv: f64,
    pub resistance: f64,
    #[serde(default)]
    pub no_load_current: f64,
    /// Servo output index (0-based) carrying this rotor's throttle.
    pub throttle_channel: usize,
    /// Servo output driving tilt, if this rotor tilts. `None` is a fixed rotor,
    /// which is the lift+cruise case.
    #[serde(default)]
    pub tilt_channel: Option<usize>,
    /// Tilt at minimum and maximum servo output (deg).
    #[serde(default)]
    pub tilt_min_deg: f64,
    #[serde(default = "default_tilt_max")]
    pub tilt_max_deg: f64,
}

fn default_blades() -> usize {
    2
}
fn default_tilt_max() -> f64 {
    90.0
}

/// Fuselage and everything else that is drag but not lift.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuselageSpec {
    /// Cd*A per body axis (m^2): forward, sideways, vertical. Sideways and
    /// vertical are much larger than forward on any real fuselage, and keeping
    /// them separate is what makes a sideslip cost something.
    #[serde(default = "default_area_cd")]
    pub area_cd: [f64; 3],
}

/// No fuselage unless one is described.
///
/// This used to default to a plausible small-aircraft fuselage, which is the
/// same silent-default failure the airfoil lookup already refuses to make: an
/// airframe that omits the block then flies with someone else's fuselage bolted
/// to it and is never told. It cost an hour on a bare flat plate whose drag
/// coefficient came out at 2.07 against a table value of 1.47, with the whole
/// difference being an invented fuselage. A missing fuselage means no fuselage.
fn default_area_cd() -> [f64; 3] {
    [0.0, 0.0, 0.0]
}

impl Default for FuselageSpec {
    fn default() -> Self {
        FuselageSpec { area_cd: default_area_cd() }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PwmSpec {
    pub min: f64,
    pub max: f64,
}

impl Default for PwmSpec {
    fn default() -> Self {
        PwmSpec { min: 1000.0, max: 2000.0 }
    }
}

// ─── The built airframe ─────────────────────────────────────────────────────

/// How the flight controller's servo outputs reach one rotor.
#[derive(Debug, Clone, Copy)]
pub struct RotorChannels {
    pub throttle: usize,
    pub tilt: Option<usize>,
    pub tilt_min: f64,
    pub tilt_max: f64,
}

/// A control surface's servo mapping, so a deflection can be produced from PWM.
#[derive(Debug, Clone, Copy)]
pub struct ChannelLimit {
    pub max_deflect: f64,
}

/// The runtime airframe: strips, rotors, tables, and the servo mapping.
#[derive(Debug, Clone)]
pub struct Airframe {
    pub name: String,
    pub mass: f64,
    pub inertia: Vec3,
    pub voltage_max: f64,
    pub pwm: PwmSpec,
    pub area_cd: Vec3,
    pub airfoils: Vec<AirfoilTable>,
    /// The SHAPE behind each airfoil table, when it came from one. Kept so a
    /// renderer can draw the section the physics is actually flying rather than
    /// a flat plate standing in for it: the sheet articles are a NACA 0004 and
    /// a 0018, and drawing both as the same zero-thickness quad hides the only
    /// difference between them. `None` where a section was given as bare
    /// coefficients or a raw polar and has no shape to draw.
    pub sections: Vec<Option<crate::panel::Section>>,
    pub surfaces: Vec<Surface>,
    pub rotors: Vec<Rotor>,
    pub rotor_channels: Vec<RotorChannels>,
    /// Per control channel, the largest deflection any surface on it allows.
    /// Used to turn a normalised servo output into radians.
    pub channel_limits: Vec<f64>,
    /// Reference wing area (m^2): the first non-vertical panel in the spec,
    /// both sides. Reporting only, and the denominator of `wing_loading`.
    pub wing_area: f64,
}

#[derive(Debug)]
pub enum BuildError {
    UnknownAirfoil(String),
    /// Neither a rotor nor a lifting surface: nothing to simulate.
    NothingToFly,
    BadMass(f64),
    BadInertia([f64; 3]),
}

impl std::fmt::Display for BuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BuildError::UnknownAirfoil(n) => write!(f, "airfoil '{n}' is referenced but not defined"),
            BuildError::NothingToFly => {
                write!(f, "airframe has neither rotors nor lifting surfaces")
            }
            BuildError::BadMass(m) => write!(f, "mass must be positive, got {m}"),
            BuildError::BadInertia(i) => write!(f, "inertia must be positive, got {i:?}"),
        }
    }
}

impl std::error::Error for BuildError {}

impl AirframeSpec {
    /// The SHAPE of the first non-vertical wing's section, when it has one.
    ///
    /// For drawing. A section given as a parametric guess or a raw polar has no
    /// shape to return, and returning `None` for those is the honest answer:
    /// only a wing that names geometry has geometry to show.
    pub fn first_wing_section(&self) -> Option<(crate::panel::Section, f64, f64, f64)> {
        let w = self.wings.iter().find(|w| !w.vertical)?;
        let sec = match self.airfoils.get(&w.airfoil)? {
            SectionSpec::Naca { naca, .. } => {
                let d: Vec<u32> = naca.chars().filter_map(|c| c.to_digit(10)).collect();
                if d.len() < 4 {
                    return None;
                }
                crate::panel::Section::naca4(
                    d[0] as f64,
                    d[1] as f64,
                    (d[2] * 10 + d[3]) as f64,
                    160,
                )
            }
            _ => return None,
        };
        Some((sec, w.chord_root, w.root[0], w.root[2]))
    }

    pub fn from_json(s: &str) -> Result<AirframeSpec, serde_json::Error> {
        serde_json::from_str(s)
    }

    /// Build the runtime airframe. Fails loudly on a missing airfoil or an
    /// impossible mass rather than substituting a default, because a silent
    /// default here is a sim that flies a different aircraft than the customer
    /// described and never says so.
    pub fn build(&self) -> Result<Airframe, BuildError> {
        if !(self.mass > 0.0) || !self.mass.is_finite() {
            return Err(BuildError::BadMass(self.mass));
        }
        if self.inertia.iter().any(|i| !(*i > 0.0) || !i.is_finite()) {
            return Err(BuildError::BadInertia(self.inertia));
        }
        // An UNPOWERED airframe is legitimate and is the most useful thing to
        // validate against: a glider's steady glide ratio is exactly its L/D,
        // its phugoid period has a closed form, and a flat plate has a textbook
        // drag coefficient. None of those need a motor, a controller or a log,
        // and all of them are predictions from theory this model did not supply.
        // Requiring a rotor made the best available test articles undescribable.
        if self.rotors.is_empty() && self.wings.is_empty() {
            return Err(BuildError::NothingToFly);
        }

        // Airfoils first: everything else indexes into this.
        let mut names: Vec<&String> = self.airfoils.keys().collect();
        names.sort(); // stable indices across runs, so a bake is reproducible.
        let index: HashMap<&str, usize> =
            names.iter().enumerate().map(|(i, n)| (n.as_str(), i)).collect();
        let mut airfoils = Vec::with_capacity(names.len());
        let mut sections: Vec<Option<crate::panel::Section>> = Vec::with_capacity(names.len());
        for n in &names {
            airfoils.push(build_table(&self.airfoils[*n], 6.0));
            sections.push(shape_of(&self.airfoils[*n]));
        }
        let lookup = |n: &str| -> Result<usize, BuildError> {
            index.get(n).copied().ok_or_else(|| BuildError::UnknownAirfoil(n.to_string()))
        };

        let mut surfaces = Vec::new();
        let mut channel_limits: Vec<f64> = Vec::new();
        // Reference area is the MAIN wing only, by convention, so wing loading
        // means what a reader expects. Summing the tailplane and fin into it
        // gives a number that is defensible and answers a question nobody asked.
        let mut wing_area = 0.0;
        let mut reference_taken = false;

        for w in &self.wings {
            // A wing's SECTION slope has to be corrected for its own aspect
            // ratio, because the strip sum discretises the span but knows
            // nothing about the trailing vorticity that makes a finite wing
            // shallower. Each panel therefore gets its own table.
            let ar = wing_aspect_ratio(w);
            let base = lookup(&w.airfoil)?;
            let corrected = correct_for_wing(&self.airfoils[&w.airfoil], ar, w.oswald);
            let table_index = airfoils.len();
            airfoils.push(corrected);
            sections.push(shape_of(&self.airfoils[&w.airfoil]));
            let _ = base;

            let is_reference = !w.vertical && !reference_taken;
            for side in [1.0f64, -1.0] {
                if side < 0.0 && !w.mirror {
                    continue;
                }
                let (mut strips, area) = build_panel(w, side, table_index, &mut channel_limits);
                if is_reference {
                    wing_area += area;
                }
                surfaces.append(&mut strips);
            }
            if is_reference {
                reference_taken = true;
            }
        }

        let mut rotors = Vec::with_capacity(self.rotors.len());
        let mut rotor_channels = Vec::with_capacity(self.rotors.len());
        for r in &self.rotors {
            let airfoil = lookup(&r.airfoil)?;
            let tilt_axis = if r.tilt_axis == [0.0, 0.0, 0.0] {
                Vec3::new(0.0, -1.0, 0.0)
            } else {
                Vec3::new(r.tilt_axis[0], r.tilt_axis[1], r.tilt_axis[2])
            };
            rotors.push(Rotor {
                position: Vec3::new(r.position[0], r.position[1], r.position[2]),
                axis: Vec3::new(r.axis[0], r.axis[1], r.axis[2]).normalize(),
                tilt_axis,
                radius: r.radius,
                blades: r.blades,
                chord_root: r.chord_root,
                chord_tip: r.chord_tip,
                pitch_root: r.pitch_root_deg.to_radians(),
                pitch_tip: r.pitch_tip_deg.to_radians(),
                airfoil,
                spin: if r.spin >= 0.0 { 1.0 } else { -1.0 },
                inertia: r.inertia,
                motor: MotorDrive {
                    kv: r.kv,
                    resistance: r.resistance,
                    no_load_current: r.no_load_current,
                    voltage_max: self.voltage_max,
                },
            });
            rotor_channels.push(RotorChannels {
                throttle: r.throttle_channel,
                tilt: r.tilt_channel,
                tilt_min: r.tilt_min_deg.to_radians(),
                tilt_max: r.tilt_max_deg.to_radians(),
            });
        }

        let f = &self.fuselage;
        Ok(Airframe {
            name: self.name.clone(),
            mass: self.mass,
            inertia: Vec3::new(self.inertia[0], self.inertia[1], self.inertia[2]),
            voltage_max: self.voltage_max,
            pwm: self.pwm,
            area_cd: Vec3::new(f.area_cd[0], f.area_cd[1], f.area_cd[2]),
            airfoils,
            sections,
            surfaces,
            rotors,
            rotor_channels,
            channel_limits,
            wing_area,
        })
    }
}

/// The coordinates behind a section spec, when it has any.
fn shape_of(s: &SectionSpec) -> Option<crate::panel::Section> {
    match s {
        SectionSpec::Naca { naca, .. } => {
            let d: Vec<u32> = naca.chars().filter_map(|c| c.to_digit(10)).collect();
            if d.len() < 4 {
                return None;
            }
            Some(crate::panel::Section::naca4(
                d[0] as f64,
                d[1] as f64,
                (d[2] * 10 + d[3]) as f64,
                80,
            ))
        }
        _ => None,
    }
}

fn wing_aspect_ratio(w: &WingSpec) -> f64 {
    let mean_chord = 0.5 * (w.chord_root + w.chord_tip);
    if mean_chord <= 0.0 {
        return 1.0;
    }
    let span = if w.mirror { 2.0 * w.semi_span.abs() } else { w.semi_span.abs() };
    (span / mean_chord).max(0.1)
}

/// Reynolds number assumed when a section does not give one. A 200 mm chord at
/// 20 m/s, which is a model aeroplane.
const DEFAULT_RE: f64 = 2.7e5;

/// Solve a NACA section into a polar and correct it to a finite wing.
///
/// The panel and boundary-layer solve is TWO-DIMENSIONAL. Lifting-line theory
/// turns that into a wing: at a given lift the 3D wing needs more incidence by
/// `CL/(pi e AR)`, and carries induced drag of `CL^2/(pi e AR)`. Applying it to
/// the samples rather than to a slope means it stays correct through the stall,
/// where there is no slope to correct.
fn solve_naca(code: &str, re: f64, ar: f64, e: f64) -> AirfoilTable {
    let digits: Vec<u32> = code.chars().filter_map(|c| c.to_digit(10)).collect();
    let sec = if digits.len() >= 4 {
        crate::panel::Section::naca4(
            digits[0] as f64,
            digits[1] as f64,
            (digits[2] * 10 + digits[3]) as f64,
            200,
        )
    } else {
        crate::panel::Section::naca4(0.0, 0.0, 12.0, 200)
    };
    let two_d = crate::bl::polar(&sec, re, -22.0, 22.0, 0.5);
    if two_d.len() < 3 {
        return AirfoilTable::from_polar(&two_d, ar);
    }

    // Truncate to the ATTACHED range before correcting, because lifting-line
    // theory is only defined there and applying it past the stall breaks the
    // table outright.
    //
    // The correction shifts each sample to `alpha + CL/(pi e AR)`. Past the
    // stall CL FALLS, so the shifted angle runs backwards and the samples stop
    // being sorted, which the interpolator assumes they are. On an aspect-ratio
    // 1 sheet that put a hole at 10 degrees where lift vanished and drag jumped
    // fiftyfold to 1.0, and an article flying through it lost its speed for no
    // visible reason. Past the peak is Viterna's job anyway.
    let i_pos = two_d
        .iter()
        .enumerate()
        .filter(|(_, s)| s.0 > 0.0)
        .max_by(|a, b| a.1 .1.total_cmp(&b.1 .1))
        .map(|(i, _)| i)
        .unwrap_or(two_d.len() - 1);
    let i_neg = two_d
        .iter()
        .enumerate()
        .filter(|(_, s)| s.0 < 0.0)
        .min_by(|a, b| a.1 .1.total_cmp(&b.1 .1))
        .map(|(i, _)| i)
        .unwrap_or(0);
    let (lo, hi) = (i_neg.min(i_pos), i_neg.max(i_pos));

    let k = if ar > 0.0 { 1.0 / (PI * e * ar) } else { 0.0 };
    let mut three_d: Vec<(f64, f64, f64, f64)> = two_d[lo..=hi]
        .iter()
        .map(|&(a, cl, cd, cm)| (a + cl * k, cl, cd + cl * cl * k, cm))
        .collect();
    // Belt and braces: drop anything that still fails to advance, so the
    // interpolator can never be handed an unsorted list again.
    three_d.dedup_by(|b, a| b.0 <= a.0);
    AirfoilTable::from_polar(&three_d, ar)
}

fn build_table(s: &SectionSpec, ar: f64) -> AirfoilTable {
    match s {
        SectionSpec::Naca { naca, reynolds } => {
            solve_naca(naca, reynolds.unwrap_or(DEFAULT_RE), ar, 0.85)
        }
        SectionSpec::Polar { polar } => {
            let samples: Vec<(f64, f64, f64, f64)> = polar
                .iter()
                .map(|p| (p[0].to_radians(), p[1], p[2], p[3]))
                .collect();
            AirfoilTable::from_samples(&samples)
        }
        SectionSpec::Parametric(p) => AirfoilTable::from_spec(&AirfoilSpec {
            cl_alpha: p.cl_alpha,
            alpha_0: p.alpha_0_deg.to_radians(),
            alpha_stall: p.alpha_stall_deg.to_radians(),
            alpha_stall_neg: p.alpha_stall_neg_deg.to_radians(),
            cd_min: p.cd_min,
            cd_k: p.cd_k,
            cm_0: p.cm_0,
            aspect_ratio: ar,
        }),
    }
}

/// The same section, corrected to a given wing's aspect ratio.
///
/// A measured polar is passed through unchanged: it is data, and silently
/// bending measured numbers to a formula is exactly what makes a sim
/// untrustworthy. The correction is applied only where the section was itself a
/// parametric guess.
fn correct_for_wing(s: &SectionSpec, ar: f64, e: f64) -> AirfoilTable {
    match s {
        SectionSpec::Naca { naca, reynolds } => {
            solve_naca(naca, reynolds.unwrap_or(DEFAULT_RE), ar, e)
        }
        SectionSpec::Polar { .. } => build_table(s, ar),
        SectionSpec::Parametric(p) => AirfoilTable::from_spec(&AirfoilSpec {
            cl_alpha: finite_wing_slope(p.cl_alpha, ar, e),
            alpha_0: p.alpha_0_deg.to_radians(),
            alpha_stall: p.alpha_stall_deg.to_radians(),
            alpha_stall_neg: p.alpha_stall_neg_deg.to_radians(),
            cd_min: p.cd_min,
            cd_k: p.cd_k + induced_drag_k(ar, e),
            cm_0: p.cm_0,
            aspect_ratio: ar,
        }),
    }
}

/// Cut one panel into strips. Returns the strips and the panel's area.
fn build_panel(
    w: &WingSpec,
    side: f64,
    airfoil: usize,
    channel_limits: &mut Vec<f64>,
) -> (Vec<Surface>, f64) {
    let n = w.strips.max(1);
    // A vertical panel may hang downward (negative semi_span); the sign steers
    // it, the magnitude is the area. Taking the raw value as a width would give
    // a downward fin negative area and therefore negative lift.
    let span_len = w.semi_span.abs();
    let span_sign = if w.semi_span < 0.0 { -1.0 } else { 1.0 };
    // The root offset MIRRORS with the panel. Applying it with the same sign to
    // both sides puts the whole wing off centre, which shows up as a slow roll
    // with no asymmetry anywhere a reader would think to look.
    let root = Vec3::new(w.root[0], w.root[1] * side, w.root[2]);
    let sweep = w.sweep_deg.to_radians();
    let dihedral = w.dihedral_deg.to_radians();
    let mut out = Vec::with_capacity(n);
    let mut area = 0.0;

    for i in 0..n {
        // Mid-strip station as a fraction of the semi-span.
        let t = (i as f64 + 0.5) / n as f64;
        let chord = w.chord_root + (w.chord_tip - w.chord_root) * t;
        let width = span_len / n as f64;
        let a = chord * width;
        area += a;

        // Quarter-chord position. A HORIZONTAL panel runs its span out the wing,
        // back with sweep and up with dihedral. A VERTICAL panel runs its span
        // UPWARD: a fin is a wing at 90 degrees of dihedral, so laying its
        // strips out along y puts the whole surface in the wrong place while
        // every force it produces still looks correct in isolation. Twin fins
        // still get their root offset mirrored, so both march up from their own
        // boom.
        let y = span_len * t;
        let pos = if w.vertical {
            // Body z is DOWN, so subtracting rises. `span_sign` flips it for a
            // VENTRAL fin, which is where a paper dart's keel actually is.
            Vec3::new(root.x - y * sweep.tan(), root.y, root.z - y * span_sign)
        } else {
            Vec3::new(
                root.x - y * sweep.tan(),
                root.y + side * y * dihedral.cos(),
                root.z - y * dihedral.sin(),
            )
        };

        let (forward, normal) = if w.vertical {
            // A fin: lift acts to the left, so a slip from the right yaws the
            // nose right (see `aero::Surface::fin_strip`).
            (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0))
        } else {
            let c = dihedral.cos();
            let s = dihedral.sin() * side;
            (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, -s, -c).normalize())
        };

        let mut surf = Surface {
            position: pos,
            area: a,
            chord,
            forward,
            normal,
            incidence: (w.incidence_deg + w.twist_deg * t).to_radians(),
            airfoil,
            control: None,
            wash_fraction: 0.0,
        };

        for c in &w.controls {
            if t >= c.span_start && t <= c.span_end {
                let max = c.max_deflect_deg.to_radians();
                if channel_limits.len() <= c.channel {
                    channel_limits.resize(c.channel + 1, 0.0);
                }
                channel_limits[c.channel] = channel_limits[c.channel].max(max);
                surf.control = Some(ControlLink {
                    channel: c.channel,
                    gain: if side >= 0.0 { c.gain } else { c.mirror_gain },
                    chord_fraction: c.chord_fraction,
                    max_deflect: max,
                });
                break;
            }
        }
        out.push(surf);
    }
    (out, area)
}

// ─── Slipstream geometry ────────────────────────────────────────────────────

impl Airframe {
    /// Fraction 0..1 of a strip immersed in one rotor's slipstream, at the
    /// rotor's current tilt.
    ///
    /// Recomputed per step rather than baked, because a tilting rotor's wake
    /// SWEEPS ACROSS the wing during transition. That sweep is the aerodynamic
    /// event a tiltrotor transition consists of, and freezing the geometry at
    /// build time would delete it.
    pub fn wash_fraction(&self, surface: usize, rotor: usize, tilt: f64) -> f64 {
        let s = &self.surfaces[surface];
        let r = &self.rotors[rotor];
        let axis = r.tilted_axis(tilt);
        let d = s.position.sub(r.position);
        // The wake travels opposite the thrust, so downstream is -axis.
        let along = -d.dot(axis);
        if along <= 0.0 || along > WAKE_LENGTH_RADII * r.radius {
            return 0.0;
        }
        let radial = d.sub(axis.scale(d.dot(axis))).length();
        // Momentum theory: the far wake has half the disc area, so its radius
        // contracts by 1/sqrt(2). Contraction is spread over the near wake.
        let c = (along / (WAKE_CONTRACT_RADII * r.radius)).clamp(0.0, 1.0);
        let r_wake = r.radius * (1.0 - c * (1.0 - std::f64::consts::FRAC_1_SQRT_2));
        // Strips have width, so immersion is partial at the wake edge. Half a
        // strip width of feather stops the wash switching on as a step, which
        // would appear as a rolling moment transient during a tilt sweep.
        let feather = (s.area / s.chord.max(1e-6)) * 0.5;
        if radial <= r_wake - feather {
            1.0
        } else if radial >= r_wake + feather {
            0.0
        } else {
            let t = (r_wake + feather - radial) / (2.0 * feather).max(1e-9);
            t * t * (3.0 - 2.0 * t)
        }
    }

    /// Total rotor disc area (m^2).
    pub fn disc_area(&self) -> f64 {
        self.rotors.iter().map(|r| r.disc_area()).sum()
    }

    /// Disc loading at all-up weight (N/m^2). The number that most directly
    /// predicts how a VTOL hovers, so it is worth having on the report.
    pub fn disc_loading(&self) -> f64 {
        let a = self.disc_area();
        if a <= 0.0 {
            return 0.0;
        }
        self.mass * 9.80665 / a
    }

    /// Wing loading at all-up weight (N/m^2), or 0 with no wings.
    pub fn wing_loading(&self) -> f64 {
        if self.wing_area <= 0.0 {
            return 0.0;
        }
        self.mass * 9.80665 / self.wing_area
    }

    /// Normalised 0..1 output from a servo PWM value.
    pub fn servo_unit(&self, pwm: f64) -> f64 {
        let span = self.pwm.max - self.pwm.min;
        if span <= 0.0 {
            return 0.0;
        }
        ((pwm - self.pwm.min) / span).clamp(0.0, 1.0)
    }

    /// Control deflection in radians from a servo PWM value: centre is neutral,
    /// the ends are the surface's mechanical limits.
    pub fn deflection(&self, channel: usize, pwm: f64) -> f64 {
        let limit = self.channel_limits.get(channel).copied().unwrap_or(0.0);
        (self.servo_unit(pwm) * 2.0 - 1.0) * limit
    }

    /// Tilt angle in radians from a servo PWM value.
    pub fn tilt_angle(&self, rotor: usize, pwm: f64) -> f64 {
        let ch = &self.rotor_channels[rotor];
        let u = self.servo_unit(pwm);
        ch.tilt_min + (ch.tilt_max - ch.tilt_min) * u
    }
}
