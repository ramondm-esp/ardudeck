//! The unpowered test articles, as data.
//!
//! One source of truth, shared by `tests/glide_verify.rs` and by the game, so
//! the thing you watch fly is byte for byte the thing the validation suite
//! measured. Keeping a second copy in the renderer is how a demo ends up
//! showing an aircraft that no test has ever checked.
//!
//! Every number here is GEOMETRY or a section property. There is no coefficient
//! fitted to make any of these behave, which is the point: a flat plate's drag,
//! a dart's glide ratio and a glider's phugoid all have answers that come from
//! somewhere other than this model, and they are what the suite checks against.

/// 300 x 300 mm of foam, 20 g. Aspect ratio 1. Its quarter chord sits ahead of
/// its own mid-chord CG, which is why a dropped sheet tumbles rather than
/// planing.
pub const FOAM_SHEET: &str = r#"{
  "name": "foam-sheet", "mass": 0.020, "inertia": [3.0e-4, 3.0e-4, 6.0e-4],
  "airfoils": { "plate": { "naca": "0004", "reynolds": 2.0e5 } },
  "wings": [ { "name": "sheet", "root": [0.075, 0.0, 0.0], "semi_span": 0.15,
               "chord_root": 0.30, "chord_tip": 0.30, "airfoil": "plate", "strips": 6 } ],
  "rotors": [],
  "fuselage": { "area_cd": [0.0, 0.0, 0.0] }
}"#;

/// 2 m x 100 mm strip, aspect ratio 20: near enough two-dimensional to compare
/// against thin airfoil theory directly.
pub const FOAM_STRIP: &str = r#"{
  "name": "foam-strip", "mass": 0.040, "inertia": [1.0e-3, 1.0e-3, 2.0e-3],
  "airfoils": { "plate": { "cl_alpha": 6.28318, "alpha_0_deg": 0.0,
                           "alpha_stall_deg": 10.0, "alpha_stall_neg_deg": -10.0,
                           "cd_min": 0.02, "cd_k": 0.0, "cm_0": 0.0 } },
  "wings": [ { "name": "sheet", "root": [0.0, 0.0, 0.0], "semi_span": 1.0,
               "chord_root": 0.10, "chord_tip": 0.10, "airfoil": "plate",
               "strips": 10, "oswald": 0.95 } ],
  "rotors": []
}"#;

/// The SAME 300 x 300 sheet, with a rounded leading edge.
///
/// Identical planform, area, mass and inertia. The only difference is the
/// section, and that is the point of having it: strip theory has no geometry
/// finer than chord and area, so it CANNOT see a leading edge. Everything a
/// rounded nose does aerodynamically has to arrive as coefficients.
///
/// What actually changes, and why:
///
/// The pair is a NACA 0004 against a NACA 0018: the same planform wearing a
/// nearly sharp nose and a distinctly blunt one. Nothing about their behaviour
/// is stated anywhere. Both polars are SOLVED from the coordinates by `panel`
/// and `bl`, so the stall angle, the drag and the lift curve all follow from
/// the shape, and the thin one lets go first because its leading edge makes a
/// suction peak the boundary layer cannot survive.
pub const FOAM_SHEET_LE: &str = r#"{
  "name": "sheet, rounded LE", "mass": 0.020, "inertia": [3.0e-4, 3.0e-4, 6.0e-4],
  "airfoils": { "rounded": { "naca": "0018", "reynolds": 2.0e5 } },
  "wings": [ { "name": "sheet", "root": [0.075, 0.0, 0.0], "semi_span": 0.15,
               "chord_root": 0.30, "chord_tip": 0.30, "airfoil": "rounded", "strips": 6 } ],
  "rotors": [],
  "fuselage": { "area_cd": [0.0, 0.0, 0.0] }
}"#;

/// A dart. Tailless, swept, trimmed by washout, and lightly enough damped that
/// it swoops the whole way down like the real thing.
pub const PAPER_PLANE: &str = r#"{
  "name": "paper-dart", "mass": 0.005, "inertia": [2.0e-5, 4.0e-5, 5.0e-5],
  "airfoils": { "paper": { "cl_alpha": 5.0, "alpha_0_deg": 0.0,
                           "alpha_stall_deg": 14.0, "alpha_stall_neg_deg": -14.0,
                           "cd_min": 0.025, "cd_k": 0.015, "cm_0": 0.0 } },
  "wings": [ { "name": "wing", "root": [0.024, 0.006, 0.0], "semi_span": 0.10,
               "chord_root": 0.20, "chord_tip": 0.06, "sweep_deg": 42.0,
               "incidence_deg": 3.0, "twist_deg": -18.0,
               "airfoil": "paper", "strips": 8, "oswald": 0.80 },
    { "name": "keel", "root": [-0.01, 0.0, 0.0], "semi_span": -0.035,
      "chord_root": 0.16, "chord_tip": 0.07, "sweep_deg": 30.0,
      "airfoil": "paper", "strips": 4, "vertical": true, "mirror": false } ],
  "rotors": [],
  "fuselage": { "area_cd": [0.0006, 0.0025, 0.0012] }
}"#;

/// A 2 m model glider, about 14:1.
pub const GLIDER: &str = r#"{
  "name": "glider-2m", "mass": 0.80, "inertia": [0.045, 0.030, 0.070],
  "masses": [
    { "name": "wing left",  "position": [ 0.00, -0.45, -0.04], "mass": 0.150 },
    { "name": "wing right", "position": [ 0.00,  0.45, -0.04], "mass": 0.150 },
    { "name": "boom",       "position": [-0.31,  0.00,  0.00], "mass": 0.100 },
    { "name": "tail",       "position": [-0.62,  0.00, -0.02], "mass": 0.050 },
    { "name": "battery",    "position": [ 0.24,  0.00,  0.00], "mass": 0.200 },
    { "name": "nose, rx",   "position": [ 0.10,  0.00,  0.00], "mass": 0.150 }
  ],
  "airfoils": {
    "wing": { "cl_alpha": 6.10, "alpha_0_deg": -2.0, "alpha_stall_deg": 12.0,
              "alpha_stall_neg_deg": -10.0, "cd_min": 0.011, "cd_k": 0.006, "cm_0": -0.05 },
    "tail": { "cl_alpha": 5.9, "alpha_0_deg": 0.0, "alpha_stall_deg": 12.0,
              "alpha_stall_neg_deg": -12.0, "cd_min": 0.012, "cd_k": 0.0, "cm_0": 0.0 }
  },
  "wings": [
    { "name": "wing", "root": [0.012, 0.03, -0.04], "semi_span": 1.0,
      "chord_root": 0.18, "chord_tip": 0.14, "incidence_deg": 2.5, "twist_deg": -1.5,
      "dihedral_deg": 4.0, "airfoil": "wing", "strips": 10, "oswald": 0.90 },
    { "name": "tailplane", "root": [-0.62, 0.02, -0.02], "semi_span": 0.18,
      "chord_root": 0.11, "chord_tip": 0.09, "incidence_deg": -1.6,
      "airfoil": "tail", "strips": 5 },
    { "name": "fin", "root": [-0.66, 0.0, -0.02], "semi_span": 0.15,
      "chord_root": 0.12, "chord_tip": 0.08, "airfoil": "tail", "strips": 4,
      "vertical": true, "mirror": false }
  ],
  "rotors": [],
  "fuselage": { "area_cd": [0.0035, 0.020, 0.012] }
}"#;

/// Every article, in the order a demo should show them: simplest first.
pub const ALL: &[(&str, &str)] = &[
    ("sheet, sharp LE", FOAM_SHEET),
    ("sheet, rounded LE", FOAM_SHEET_LE),
    ("paper plane", PAPER_PLANE),
    ("glider", GLIDER),
];
