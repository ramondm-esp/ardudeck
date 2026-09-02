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
  "airfoils": { "plate": { "cl_alpha": 6.28318, "alpha_0_deg": 0.0,
                           "alpha_stall_deg": 22.0, "alpha_stall_neg_deg": -22.0,
                           "cd_min": 0.02, "cd_k": 0.0, "cm_0": 0.0 } },
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
               "airfoil": "paper", "strips": 8, "oswald": 0.80 } ],
  "rotors": [],
  "fuselage": { "area_cd": [0.0006, 0.0025, 0.0012] }
}"#;

/// A 2 m model glider, about 14:1.
pub const GLIDER: &str = r#"{
  "name": "glider-2m", "mass": 0.80, "inertia": [0.045, 0.030, 0.070],
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
    ("foam sheet", FOAM_SHEET),
    ("paper plane", PAPER_PLANE),
    ("glider", GLIDER),
];
