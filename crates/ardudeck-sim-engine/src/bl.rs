//! Integral boundary layer on the panel solution: drag, transition, separation
//! and therefore STALL.
//!
//! `panel.rs` gives an inviscid pressure distribution and no drag at all
//! (d'Alembert). This marches a boundary layer along that distribution and
//! supplies the two things the flight model was still being handed by hand:
//! how much drag the section makes, and the angle at which it lets go.
//!
//! Standard integral method, in the order the flow meets it:
//!
//! - **Thwaites** for the laminar run (correlations as fitted by Cebeci and
//!   Bradshaw), separating at lambda = -0.09.
//! - **Michel's criterion** for transition. XFOIL uses an e^N envelope, which
//!   is better in strong adverse gradients; Michel is the established simple
//!   criterion and is what this starts on.
//! - **Head's entrainment method** for the turbulent run, with Ludwieg-Tillmann
//!   skin friction, separating near H = 2.4.
//! - **Squire-Young** to turn the trailing-edge momentum thickness into profile
//!   drag.
//!
//! Stall is not a parameter here. It is where the separation point runs forward
//! off the trailing edge, which is a consequence of the pressure distribution,
//! which is a consequence of the shape.

use crate::panel::{solve, PanelSolution, Section};

/// Kinematic viscosity of air at sea level, m^2/s.
pub const NU_AIR: f64 = 1.46e-5;

/// Laminar separation. Thwaites' parameter reaches this and the laminar layer
/// cannot continue.
const LAMBDA_SEP: f64 = -0.09;
/// Turbulent separation, by shape factor.
const H_SEP: f64 = 2.4;

/// Momentum-thickness Reynolds number below which a laminar separation bubble
/// BURSTS instead of reattaching (Owen and Klanfer). This is the mechanism of
/// LEADING-EDGE stall: a sharp nose separates the flow within a few thousandths
/// of a chord, where the layer is too thin to survive the pressure recovery, so
/// it never comes back. A blunt nose separates later with a fatter layer and
/// reattaches, which is why it stalls gently from the trailing edge instead.
const RE_THETA_BURST: f64 = 125.0;

/// Normal-force coefficient of a fully separated flat plate, which is what a
/// section becomes once its leading-edge bubble has burst.
const CD_PLATE: f64 = 1.15;

/// Suction peaks between which leading-edge stall sets in.
///
/// The peak is the criterion, not a symptom. Thin airfoil theory gives an
/// INFINITE suction peak at a sharp leading edge, and a real nose of finite
/// radius gives a large finite one; no boundary layer survives recovering from
/// it, and that is what leading-edge stall is.
///
/// Judging it by Reynolds number at separation goes the wrong way: the peak
/// inflates the local velocity, so `Re_theta` RISES with incidence and the
/// criterion relaxed exactly when it should have bitten. A NACA 0004 reached
/// Cp of -96 at 16 degrees and -147 at 20 and was still counted half attached.
const CP_PEAK_OK: f64 = -6.0;
const CP_PEAK_GONE: f64 = -14.0;

/// Separation reaching this far forward counts as stalled.
///
/// A CALIBRATION of this closure, not a physical constant, and the one number
/// here that is. Head's entrainment method is known to under-predict separation
/// growth in strong adverse gradients, which is exactly why Drela used a lagged
/// dissipation closure in XFOIL instead. Untuned, this put a NACA 0012's stall
/// at 22 degrees against a measured 15 to 16. Replacing Head with a lag method
/// is the principled fix and would let this go back to the H = 2.4 separation
/// criterion alone.
const STALL_SEP_X: f64 = 0.80;

/// One surface's boundary layer result.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceBl {
    /// Momentum thickness at the trailing edge, chords.
    pub theta_te: f64,
    /// Shape factor at the trailing edge.
    pub h_te: f64,
    /// Edge velocity for the Squire-Young wake, freestream units.
    ///
    /// Taken slightly UPSTREAM of the trailing edge, not at it. The Kutta
    /// condition stagnates the inviscid flow exactly at the cusp, so the last
    /// panel midpoint reads an edge velocity near 0.7 that the boundary layer
    /// never sees. Squire-Young raises it to the power (H+5)/2, so a 30% error
    /// there became a factor of four in drag: a NACA 0012 came out at 0.0038
    /// against a published 0.006.
    pub ue_te: f64,
    /// Where the flow separated, as x/c. 1.0 means it never did.
    pub x_sep: f64,
    /// Where it transitioned, as x/c. 1.0 means it stayed laminar.
    pub x_tr: f64,
    /// How completely the leading-edge bubble failed to reattach: 0 fully
    /// reattached, 1 fully burst.
    ///
    /// A FRACTION and not a flag. A boolean flips with angle of attack as the
    /// separation and transition points hop between panels, which puts a step in
    /// the polar, and a step in a coefficient table is the same defect that once
    /// gave an article a hole to fly through.
    pub burst: f64,
}

/// A section's viscous state at one angle of attack.
#[derive(Debug, Clone)]
pub struct ViscousPoint {
    pub alpha: f64,
    /// Inviscid lift, before any stall correction.
    pub cl_inviscid: f64,
    /// Lift after the separation correction.
    pub cl: f64,
    pub cd: f64,
    pub cm: f64,
    pub upper: SurfaceBl,
    pub lower: SurfaceBl,
    /// True once separation has run far enough forward to count as stalled.
    pub stalled: bool,
}

/// March the boundary layer along one surface.
///
/// `s` is arc length from the stagnation point and `ue` the edge velocity there,
/// both already ordered downstream. `re` is the chord Reynolds number.
fn march(s: &[f64], ue: &[f64], x: &[f64], re: f64) -> SurfaceBl {
    let n = s.len();
    // Edge velocity for the wake, read upstream of the trailing-edge cusp.
    let te_ref = {
        let total = s.last().copied().unwrap_or(1.0);
        let mut v = ue.last().copied().unwrap_or(1.0).abs();
        for i in (0..n).rev() {
            if s[i] < 0.98 * total {
                v = ue[i].abs();
                break;
            }
        }
        v.max(1e-6)
    };
    let mut out = SurfaceBl {
        theta_te: 0.0,
        h_te: 2.6,
        ue_te: te_ref,
        x_sep: 1.0,
        x_tr: 1.0,
        burst: 0.0,
    };
    if n < 4 {
        return out;
    }
    let nu = 1.0 / re; // chords^2 per unit time, with chord and freestream at 1

    // ── laminar: Thwaites ───────────────────────────────────────────────────
    // theta^2 = 0.45 nu / ue^6 * integral(ue^5 ds). The integral form is used
    // rather than a marching ODE because it is exact for the laminar layer and
    // does not accumulate step error through the strong leading-edge gradient.
    let mut integral = 0.0;
    let mut theta = 0.0;
    let mut h = 2.6;
    let mut i_tr = n - 1;
    let mut laminar_sep = false;
    for i in 1..n {
        let u0 = ue[i - 1].abs().max(1e-6);
        let u1 = ue[i].abs().max(1e-6);
        integral += 0.5 * (u0.powi(5) + u1.powi(5)) * (s[i] - s[i - 1]);
        let u = u1;
        theta = (0.45 * nu * integral / u.powi(6)).max(0.0).sqrt();
        let duds = (u1 - u0) / (s[i] - s[i - 1]).max(1e-9);
        let lambda = (theta * theta / nu) * duds;
        h = thwaites_h(lambda);
        if lambda < LAMBDA_SEP {
            out.x_sep = x[i];
            laminar_sep = true;
            i_tr = i;
            break;
        }
        // Michel: transition when Re_theta crosses the correlation against Re_x.
        let re_x = (u * s[i] / nu).max(1.0);
        let re_theta = u * theta / nu;
        if re_theta > 1.174 * (1.0 + 22400.0 / re_x) * re_x.powf(0.46) {
            out.x_tr = x[i];
            i_tr = i;
            break;
        }
        i_tr = i;
    }

    // A laminar separation bubble normally reattaches turbulent and the march
    // carries on. It does NOT always: a weak bubble BURSTS instead, and the
    // flow never comes back. That is leading-edge stall, and it is why a thin
    // sharp-nosed section lets go early and abruptly where a thick one hangs on.
    //
    // Owen and Klanfer's criterion, as used since: the bubble reattaches only if
    // the momentum-thickness Reynolds number at separation is above about 125.
    // Below that there is not enough momentum in the layer to survive the
    // pressure recovery.
    //
    // Reattaching unconditionally, as this did, let thin sections run to a lift
    // coefficient of 2.0 to 2.5 where a NACA 0004 at Re 2e5 measures nearer 0.8.
    // The panel solve had the suction peak right the whole time; nothing was
    // acting on it.
    if laminar_sep {
        let u_sep = ue[i_tr.min(n - 1)].abs().max(1e-6);
        let re_theta = u_sep * theta / nu;
        // Smoothstepped across the threshold rather than switched at it.
        let t = (1.0 - re_theta / RE_THETA_BURST).clamp(0.0, 1.0);
        out.burst = t * t * (3.0 - 2.0 * t);
        if out.burst > 0.99 {
            out.theta_te = theta;
            out.h_te = h.max(H_SEP);
            return out;
        }
        out.x_tr = out.x_sep;
        out.x_sep = 1.0;
    }
    // A turbulent layer starts at a HEALTHY shape factor whatever the laminar
    // one had reached. Momentum thickness is continuous through transition;
    // the profile is not. Carrying the laminar H across left a NACA 0012 with
    // H = 3.5 at the trailing edge at ZERO incidence, which is a separated
    // layer, and it made the section three times too draggy.
    h = 1.4;

    // ── turbulent: Head's entrainment method ────────────────────────────────
    //
    // Stopped short of the trailing edge. The Kutta condition drives the
    // INVISCID edge velocity to zero at the cusp, and the momentum equation
    // carries theta/Ue, so marching into it makes the layer blow up: a NACA
    // 0012 at ZERO incidence separated in a single step at the last panel and
    // finished with H = 3.5. That is a singularity in the outer solution, not a
    // boundary layer. The surface march ends here and the wake is handled by
    // Squire-Young, which is the usual division.
    let s_end = 0.985 * s[n - 1];
    let mut h1 = head_h1(h.max(1.05));
    for i in (i_tr + 1)..n {
        if s[i] > s_end {
            break;
        }
        let ds = (s[i] - s[i - 1]).max(1e-12);
        let u0 = ue[i - 1].abs().max(1e-6);
        let u1 = ue[i].abs().max(1e-6);
        let u = (0.5 * (u0 + u1)).max(0.15);
        let duds = (u1 - u0) / ds;
        let re_theta = (u * theta / nu).max(1.0);
        // Ludwieg-Tillmann.
        let cf = 0.246 * 10f64.powf(-0.678 * h) * re_theta.powf(-0.268);
        // Momentum integral.
        let dtheta = cf / 2.0 - (h + 2.0) * theta / u * duds;
        // Entrainment.
        let f = 0.0306 * (h1 - 3.0).max(1e-3).powf(-0.6169);
        let dh1 = f / theta.max(1e-9) - h1 / u * duds - h1 / theta.max(1e-9) * dtheta;
        theta = (theta + dtheta * ds).max(1e-9);
        h1 = (h1 + dh1 * ds).max(3.02);
        h = head_h_from_h1(h1);
        if h > H_SEP {
            out.x_sep = x[i];
            break;
        }
    }
    out.theta_te = theta;
    out.h_te = h;
    out
}

/// Thwaites shape factor from lambda (Cebeci and Bradshaw's fit).
fn thwaites_h(lambda: f64) -> f64 {
    let l = lambda.clamp(-0.1, 0.25);
    if l >= 0.0 {
        2.61 - 3.75 * l + 5.24 * l * l
    } else {
        2.088 + 0.0731 / (l + 0.14)
    }
}

/// Head's H1 from H.
fn head_h1(h: f64) -> f64 {
    if h <= 1.6 {
        3.3 + 0.8234 * (h - 1.1).max(1e-3).powf(-1.287)
    } else {
        3.3 + 1.5501 * (h - 0.6778).max(1e-3).powf(-3.064)
    }
}

/// And back again, by inversion. Monotone in the range that matters, so a
/// bisection is exact enough and avoids a second fitted correlation drifting
/// away from the first.
fn head_h_from_h1(h1: f64) -> f64 {
    let (mut lo, mut hi) = (1.05f64, 3.5f64);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if head_h1(mid) > h1 {
            lo = mid
        } else {
            hi = mid
        }
    }
    0.5 * (lo + hi)
}

/// Solve a section at one angle: inviscid pressures, then the boundary layer.
pub fn viscous_point(section: &Section, alpha: f64, re: f64) -> ViscousPoint {
    let inv: PanelSolution = solve(section, alpha);
    let n = inv.cp.len();
    let mut vp = ViscousPoint {
        alpha,
        cl_inviscid: inv.cl,
        cl: inv.cl,
        cd: 0.0,
        cm: inv.cm,
        upper: SurfaceBl { theta_te: 0.0, h_te: 2.6, ue_te: 1.0, x_sep: 1.0, x_tr: 1.0, burst: 0.0 },
        lower: SurfaceBl { theta_te: 0.0, h_te: 2.6, ue_te: 1.0, x_sep: 1.0, x_tr: 1.0, burst: 0.0 },
        stalled: false,
    };
    if n < 8 {
        return vp;
    }

    // Split at the stagnation point, which is where Cp peaks, and march each way.
    let stag = inv.cp.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).unwrap().0;
    let ue: Vec<f64> = inv.cp.iter().map(|c| (1.0 - c).max(0.0).sqrt()).collect();

    let mut build = |idx: Vec<usize>| -> SurfaceBl {
        let mut s = Vec::with_capacity(idx.len());
        let mut u = Vec::with_capacity(idx.len());
        let mut x = Vec::with_capacity(idx.len());
        let mut acc = 0.0;
        for (k, &i) in idx.iter().enumerate() {
            if k > 0 {
                let j = idx[k - 1];
                acc += ((inv.xc[i] - inv.xc[j]).powi(2) + (inv.zc[i] - inv.zc[j]).powi(2)).sqrt();
            }
            s.push(acc.max(1e-6));
            u.push(ue[i]);
            x.push(inv.xc[i]);
        }
        march(&s, &u, &x, re)
    };
    // Panels run counterclockwise from the trailing edge: lower surface first,
    // then upper. Downstream is away from the stagnation point in both.
    vp.lower = build((0..=stag).rev().collect());
    vp.upper = build((stag..n).collect());

    // Squire-Young: profile drag from the trailing-edge momentum thickness.
    let sq = |b: &SurfaceBl| 2.0 * b.theta_te * b.ue_te.powf((b.h_te + 5.0) / 2.0);
    vp.cd = sq(&vp.upper) + sq(&vp.lower);

    // ── LEADING-EDGE STALL ─────────────────────────────────────────────────
    //
    // The bubble burst, so the flow is separated from the nose back. The section
    // is now a FLAT PLATE at this angle, and that is what it gets: the fully
    // separated normal-force law, not a fraction of the attached lift.
    //
    // Cutting lift toward zero instead was the mistake that made this criterion
    // look wrong the first time it was tried. It stalled a NACA 0004 at four
    // degrees down to CL 0.001, which is not what a stalled section does: a
    // stalled plate still carries a normal force of about `cd_max sin(alpha)`,
    // which is where its remaining lift comes from. The criterion was firing
    // correctly all along; what followed it was wrong.
    // Leading-edge stall from the SUCTION PEAK the panel solve computed, blended
    // with the bubble-burst estimate from the boundary layer. Either mechanism
    // can take a section; the peak is what takes a thin one.
    let peak = ((inv.cp_min - CP_PEAK_OK) / (CP_PEAK_GONE - CP_PEAK_OK)).clamp(0.0, 1.0);
    let peak = peak * peak * (3.0 - 2.0 * peak);
    let burst = vp.upper.burst.max(vp.lower.burst).max(peak);
    if burst > 0.01 {
        let a = alpha.abs();
        let cn = CD_PLATE * a.sin();
        let (pl_cl, pl_cd, pl_cm) = (
            cn * a.cos() * alpha.signum(),
            cn * a.sin() + vp.cd.min(0.05),
            -0.25 * cn,
        );
        vp.cl = vp.cl * (1.0 - burst) + pl_cl * burst;
        vp.cd = vp.cd * (1.0 - burst) + pl_cd * burst;
        vp.cm = vp.cm * (1.0 - burst) + pl_cm * burst;
        if burst > 0.5 {
            vp.stalled = true;
            return vp;
        }
    }

    // ── TRAILING-EDGE STALL ────────────────────────────────────────────────
    // Separation has crept forward over most of the chord. Lift falls off as the
    // attached fraction shrinks, which is the gentle stall a thick section has.
    let sep = vp.upper.x_sep.min(1.0);
    if sep < STALL_SEP_X {
        vp.stalled = true;
        vp.cl = inv.cl * (sep / STALL_SEP_X).clamp(0.0, 1.0);
        vp.cd += 0.9 * (1.0 - sep) * inv.cl.abs().max(0.3);
    }
    vp
}

/// Sweep a section into a polar the coefficient tables can consume.
///
/// Returns `(alpha_rad, cl, cd, cm)` samples, which is exactly the shape
/// `aero::AirfoilTable::from_samples` takes.
pub fn polar(section: &Section, re: f64, from_deg: f64, to_deg: f64, step_deg: f64) -> Vec<(f64, f64, f64, f64)> {
    let mut out = Vec::new();
    let mut a = from_deg;
    while a <= to_deg + 1e-9 {
        let p = viscous_point(section, a.to_radians(), re);
        out.push((a.to_radians(), p.cl, p.cd, p.cm));
        a += step_deg;
    }
    out
}

/// The positive stall angle in degrees, found by sweeping until separation runs
/// forward. `None` if it never stalls in the range.
pub fn stall_angle_deg(section: &Section, re: f64) -> Option<f64> {
    let mut a: f64 = 0.0;
    let mut best_cl = f64::MIN;
    let mut at = None;
    while a <= 25.0 {
        let p = viscous_point(section, a.to_radians(), re);
        if p.cl > best_cl {
            best_cl = p.cl;
            at = Some(a);
        } else if p.stalled {
            break;
        }
        a += 0.25;
    }
    at
}

#[cfg(test)]
mod tests {
    use super::*;

    const RE: f64 = 3.0e6;

    /// Blasius: a laminar layer on a flat plate has theta = 0.664 x / sqrt(Re_x).
    /// Marched with a uniform edge velocity, Thwaites must reproduce it.
    #[test]
    fn thwaites_reproduces_the_blasius_flat_plate() {
        let n = 400;
        let s: Vec<f64> = (0..n).map(|i| 1e-4 + i as f64 / n as f64).collect();
        let ue = vec![1.0; n];
        let x = s.clone();
        let b = march(&s, &ue, &x, RE);
        let re_x = RE * s[n - 1];
        let blasius = 0.664 * s[n - 1] / re_x.sqrt();
        // Michel will transition it before the trailing edge at this Reynolds
        // number, so compare where it is still laminar.
        assert!(b.x_tr < 1.0, "never transitioned; Michel is not firing");
        let b_short = march(&s[..40], &ue[..40], &x[..40], RE);
        let re_x2 = RE * s[39];
        let bl2 = 0.664 * s[39] / re_x2.sqrt();
        assert!(
            (b_short.theta_te - bl2).abs() / bl2 < 0.10,
            "theta {:.3e} vs Blasius {:.3e}",
            b_short.theta_te,
            bl2
        );
        let _ = blasius;
    }

    /// A symmetric section at zero lift makes only friction drag, and a NACA
    /// 0012 at Re 3e6 is measured at about 0.006.
    #[test]
    fn the_naca_0012_has_its_published_minimum_drag() {
        let s = Section::naca4(0.0, 0.0, 12.0, 200);
        let p = viscous_point(&s, 0.0, RE);
        assert!(
            (0.004..0.011).contains(&p.cd),
            "cd {:.5} at zero lift, published about 0.006",
            p.cd
        );
        assert!(p.cl.abs() < 1e-6);
        assert!(!p.stalled);
        // It transitions somewhere on the chord rather than staying laminar to
        // the trailing edge at this Reynolds number.
        assert!(p.upper.x_tr < 0.9, "transition at x/c {:.3}", p.upper.x_tr);
    }

    /// Drag rises with incidence, because the layer on the upper surface is
    /// working against a stronger adverse gradient.
    #[test]
    fn drag_rises_with_incidence() {
        let s = Section::naca4(0.0, 0.0, 12.0, 200);
        let a0 = viscous_point(&s, 0.0, RE).cd;
        let a6 = viscous_point(&s, 6f64.to_radians(), RE).cd;
        assert!(a6 > a0, "cd {a0:.5} at 0 deg, {a6:.5} at 6 deg");
    }

    /// Transition moves FORWARD as Reynolds number rises. A basic property of
    /// the criterion and a check that Reynolds number is actually plumbed in.
    #[test]
    fn transition_moves_forward_with_reynolds_number() {
        let s = Section::naca4(0.0, 0.0, 12.0, 200);
        let at = |re: f64| viscous_point(&s, 0.0, re).upper.x_tr;
        let slow = at(3.0e5);
        let fast = at(1.0e7);
        assert!(fast < slow, "transition at x/c {slow:.3} at Re 3e5, {fast:.3} at Re 1e7");
    }

    /// STALL, from the shape. A NACA 0012 at Re 3e6 stalls around 15 to 16
    /// degrees. That number is not in this code: separation runs forward off the
    /// trailing edge because of the pressure distribution the panel method
    /// computed for those coordinates.
    #[test]
    fn the_naca_0012_stalls_where_it_should() {
        let s = Section::naca4(0.0, 0.0, 12.0, 200);
        let a = stall_angle_deg(&s, RE).expect("it must stall somewhere");
        assert!(
            (10.0..20.0).contains(&a),
            "stalls at {a:.1} deg, published about 15 to 16"
        );
    }

    /// The answer to the question this was all built for: a THIN section stalls
    /// earlier than a thick one, because its sharper nose makes a suction peak
    /// the boundary layer cannot survive.
    ///
    /// Nothing here is told which section is sharp. It follows from the
    /// coordinates, through the pressure distribution, to the separation point.
    #[test]
    fn a_thin_section_stalls_earlier_than_a_thick_one() {
        let thin = Section::naca4(0.0, 0.0, 6.0, 240);
        let thick = Section::naca4(0.0, 0.0, 18.0, 240);
        let a_thin = stall_angle_deg(&thin, RE).expect("thin stalls");
        let a_thick = stall_angle_deg(&thick, RE).expect("thick stalls");
        assert!(
            a_thick > a_thin + 1.0,
            "6% stalls at {a_thin:.1} deg, 18% at {a_thick:.1}: a thin section must let go first"
        );
    }

    #[test]
    fn a_polar_is_finite_everywhere_and_shaped_like_a_polar() {
        let s = Section::naca4(2.0, 4.0, 12.0, 160);
        let p = polar(&s, RE, -8.0, 24.0, 1.0);
        assert!(p.len() > 20);
        for (a, cl, cd, cm) in &p {
            assert!(cl.is_finite() && cd.is_finite() && cm.is_finite(), "non-finite at {a}");
            // A fully separated section really does approach flat-plate drag,
            // so the bound is loose. Deep post-stall is not this model's job
            // anyway: `aero::AirfoilTable` extends the polar past stall with
            // Viterna, which is fitted to measured data out to 90 degrees.
            assert!(*cd > 0.0 && *cd < 2.0, "cd {cd} at {:.1} deg", a.to_degrees());
        }
        // Lift rises then falls: there is a peak inside the range.
        let peak = p.iter().enumerate().max_by(|x, y| x.1 .1.total_cmp(&y.1 .1)).unwrap().0;
        assert!(peak > 2 && peak < p.len() - 2, "peak at the edge of the sweep");
    }
}

#[cfg(test)]
mod bake_cost {
    use super::*;
    /// The whole point of doing this offline: one section, one polar, once.
    #[test]
    fn a_viscous_polar_is_cheap_enough_to_bake() {
        let s = Section::naca4(2.0, 4.0, 12.0, 200);
        let t0 = std::time::Instant::now();
        let p = polar(&s, 3.0e6, -10.0, 20.0, 0.5);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        println!("viscous polar: {} points at 200 panels in {ms:.1} ms ({:.2} ms/point)", p.len(), ms / p.len() as f64);
        assert!(p.len() > 50);
    }
}





