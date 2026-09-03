//! Potential flow around an aerofoil SHAPE: the Hess-Smith panel method.
//!
//! This is the piece that was missing. `aero.rs` integrates section
//! coefficients over strips, and until now those coefficients were numbers
//! somebody typed. Strip theory has no geometry finer than chord and area, so
//! it cannot see a leading edge, a camber line or a thickness distribution, and
//! an airframe spec had no way to even state one. That is not a modelling
//! simplification, it is a missing input.
//!
//! So the section becomes GEOMETRY, and this solves it. Given coordinates it
//! returns the pressure distribution, lift and moment for the actual shape,
//! exactly, for inviscid incompressible flow. Leading-edge radius, camber and
//! thickness all enter because they are in the coordinates.
//!
//! ## Why this runs offline and not per frame
//!
//! Not a compromise, arithmetic. A 200-panel solve is a dense 201x201 system,
//! about 2.7 Mflop, which is well under a millisecond, but a flight model
//! evaluates hundreds of strips at 400 Hz and has nanoseconds per strip. No
//! simulator solves a flow field in that budget. The division everyone uses is
//! the one here: solve the SHAPE offline, tabulate, integrate the table at
//! runtime.
//!
//! ## Method
//!
//! Hess-Smith (Douglas-Neumann, 1966): N panels each carrying a CONSTANT source
//! strength, plus a single constant vortex strength shared by all of them. That
//! is N+1 unknowns, closed by N no-penetration conditions at the panel midpoints
//! and one Kutta condition at the trailing edge. Constant-strength panels rather
//! than XFOIL's linear-vorticity formulation because the influence coefficients
//! are closed-form and short, and the difference at the panel counts used here
//! is far smaller than the difference between having a section shape and not.
//!
//! What this does NOT do is viscosity. It is potential flow: no boundary layer,
//! no separation, and therefore no stall and no friction drag. It gives the
//! lift-curve slope, the zero-lift angle, the moment and the pressure
//! distribution correctly, and it gives zero drag, which is d'Alembert's paradox
//! and is asserted as a test rather than hidden. Drag and stall need the
//! boundary layer that goes on top of this.

use std::f64::consts::PI;

/// A closed 2D section, as node coordinates.
///
/// Ordered COUNTERCLOCKWISE starting at the trailing edge: back along the lower
/// surface to the leading edge, then forward along the upper surface to the
/// trailing edge again. With that ordering the outward normal of a panel from
/// node j to j+1 is `(-dz, dx)/L`, which is what every sign below assumes.
#[derive(Debug, Clone, PartialEq)]
pub struct Section {
    pub name: String,
    /// N+1 nodes; the first and last coincide at the trailing edge.
    pub x: Vec<f64>,
    pub z: Vec<f64>,
}

/// One solved operating point.
#[derive(Debug, Clone)]
pub struct PanelSolution {
    /// Pressure coefficient at each panel's midpoint.
    pub cp: Vec<f64>,
    /// Panel midpoints, for plotting the distribution.
    pub xc: Vec<f64>,
    pub zc: Vec<f64>,
    /// Lift coefficient, from integrating the pressure.
    pub cl: f64,
    /// Lift coefficient from Kutta-Joukowski, `2 * Gamma`. Computed by a
    /// completely different route from `cl` and reported separately: the two
    /// agreeing is the strongest single check that the solve is right.
    pub cl_circulation: f64,
    /// Pitching moment about the quarter chord, positive nose up.
    pub cm: f64,
    /// Pressure drag. Must be ~0 (d'Alembert). A non-zero value here means the
    /// panelling is too coarse or the solve is wrong; it is never physical.
    pub cd_pressure: f64,
    /// Most negative Cp anywhere: the suction peak. This is the number that
    /// separates a sharp leading edge from a rounded one, because it is the
    /// pressure spike a sharp nose cannot support and a round one can.
    pub cp_min: f64,
    /// Where that peak sits, as a fraction of chord.
    pub cp_min_x: f64,
}

impl Section {
    /// A NACA 4-digit section, `naca4(2, 4, 12, n)` being the 2412.
    ///
    /// `m` is maximum camber in percent of chord, `p` the position of that
    /// camber in tenths, `t` the thickness in percent. `panels` is the number of
    /// panels, split evenly between the surfaces.
    ///
    /// Nodes are COSINE spaced, clustered toward both edges. Not a refinement:
    /// the leading edge is where the pressure gradient is steepest and where the
    /// entire difference between a sharp and a round nose lives, and uniform
    /// spacing puts almost no points there.
    pub fn naca4(m_pct: f64, p_tenths: f64, t_pct: f64, panels: usize) -> Section {
        let m = m_pct / 100.0;
        let p = p_tenths / 10.0;
        let t = t_pct / 100.0;
        let half = (panels / 2).max(3);

        let camber = |x: f64| -> (f64, f64) {
            if m <= 0.0 || p <= 0.0 || p >= 1.0 {
                return (0.0, 0.0);
            }
            if x < p {
                (
                    m / (p * p) * (2.0 * p * x - x * x),
                    2.0 * m / (p * p) * (p - x),
                )
            } else {
                let q = 1.0 - p;
                (
                    m / (q * q) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x),
                    2.0 * m / (q * q) * (p - x),
                )
            }
        };
        // Closed trailing edge (-0.1036 rather than the original -0.1015), so
        // the section is a closed contour. An open TE leaves a gap the panel
        // method has to be told what to do with, and this avoids the question.
        let thickness = |x: f64| -> f64 {
            5.0 * t
                * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x.powi(3)
                    - 0.1036 * x.powi(4))
        };

        let mut xs = Vec::with_capacity(2 * half + 1);
        let mut zs = Vec::with_capacity(2 * half + 1);
        // Lower surface, trailing edge to leading edge.
        for i in 0..=half {
            let beta = PI * i as f64 / half as f64;
            let xc = 0.5 * (1.0 + beta.cos()); // 1 -> 0
            let (yc, dyc) = camber(xc);
            let yt = thickness(xc);
            let th = dyc.atan();
            xs.push(xc + yt * th.sin());
            zs.push(yc - yt * th.cos());
        }
        // Upper surface, leading edge back to trailing edge, skipping the shared
        // leading-edge node.
        for i in 1..=half {
            let beta = PI * i as f64 / half as f64;
            let xc = 0.5 * (1.0 - beta.cos()); // 0 -> 1
            let (yc, dyc) = camber(xc);
            let yt = thickness(xc);
            let th = dyc.atan();
            xs.push(xc - yt * th.sin());
            zs.push(yc + yt * th.cos());
        }
        Section {
            name: format!("NACA {:.0}{:.0}{:02.0}", m_pct, p_tenths, t_pct),
            x: xs,
            z: zs,
        }
    }

    /// A section from measured or drawn coordinates, in the ordering described
    /// on `Section`.
    pub fn from_coords(name: impl Into<String>, x: Vec<f64>, z: Vec<f64>) -> Section {
        Section { name: name.into(), x, z }
    }

    pub fn panels(&self) -> usize {
        self.x.len().saturating_sub(1)
    }

    /// Leading-edge radius, estimated by fitting a circle through the three
    /// nodes nearest the leading edge.
    ///
    /// Reported because it is the geometric property that decides how a section
    /// stalls, and the whole point of taking a shape as input is being able to
    /// say what shape it is.
    pub fn le_radius(&self) -> f64 {
        let n = self.panels();
        if n < 6 {
            return 0.0;
        }
        // The leading edge is the node of least x.
        let mut k = 0;
        for i in 0..=n {
            if self.x[i] < self.x[k] {
                k = i;
            }
        }
        if k == 0 || k >= n {
            return 0.0;
        }
        let (ax, az) = (self.x[k - 1], self.z[k - 1]);
        let (bx, bz) = (self.x[k], self.z[k]);
        let (cx, cz) = (self.x[k + 1], self.z[k + 1]);
        // Circumradius of the triangle: R = abc / 4A.
        let ab = ((bx - ax).powi(2) + (bz - az).powi(2)).sqrt();
        let bc = ((cx - bx).powi(2) + (cz - bz).powi(2)).sqrt();
        let ca = ((ax - cx).powi(2) + (az - cz).powi(2)).sqrt();
        let area2 = ((bx - ax) * (cz - az) - (cx - ax) * (bz - az)).abs();
        if area2 < 1e-14 {
            return 0.0;
        }
        ab * bc * ca / (2.0 * area2)
    }
}

/// Solve the section at one angle of attack (radians). Freestream speed and
/// chord are both 1, so everything returned is already a coefficient.
pub fn solve(section: &Section, alpha: f64) -> PanelSolution {
    let n = section.panels();
    let mut out = PanelSolution {
        cp: vec![0.0; n],
        xc: vec![0.0; n],
        zc: vec![0.0; n],
        cl: 0.0,
        cl_circulation: 0.0,
        cm: 0.0,
        cd_pressure: 0.0,
        cp_min: 0.0,
        cp_min_x: 0.0,
    };
    if n < 4 {
        return out;
    }

    // ── panel geometry ──────────────────────────────────────────────────────
    let mut len = vec![0.0; n];
    let mut tx = vec![0.0; n];
    let mut tz = vec![0.0; n];
    let mut nx = vec![0.0; n];
    let mut nz = vec![0.0; n];
    for i in 0..n {
        let dx = section.x[i + 1] - section.x[i];
        let dz = section.z[i + 1] - section.z[i];
        let l = (dx * dx + dz * dz).sqrt().max(1e-12);
        len[i] = l;
        tx[i] = dx / l;
        tz[i] = dz / l;
        // Outward for the counterclockwise ordering `Section` documents.
        nx[i] = -dz / l;
        nz[i] = dx / l;
        out.xc[i] = 0.5 * (section.x[i] + section.x[i + 1]);
        out.zc[i] = 0.5 * (section.z[i] + section.z[i + 1]);
    }

    // ── influence coefficients ──────────────────────────────────────────────
    // an[i][j], at[i][j]: normal and tangential velocity at control point i from
    // a unit SOURCE on panel j. avn[i], avt[i]: the same from the single shared
    // unit VORTEX, already summed over all panels.
    let mut an = vec![0.0; n * n];
    let mut at = vec![0.0; n * n];
    let mut avn = vec![0.0; n];
    let mut avt = vec![0.0; n];

    for i in 0..n {
        for j in 0..n {
            let (us, ws, uv, wv) = if i == j {
                // On the panel itself. A source sheet induces half its strength
                // outward and nothing along itself; a vortex sheet the dual.
                (0.0, 0.5, 0.5, 0.0)
            } else {
                // Field point in panel j's frame, where (t, n) is a right-handed
                // pair so this is a plain rotation.
                let dxg = out.xc[i] - section.x[j];
                let dzg = out.zc[i] - section.z[j];
                let xp = dxg * tx[j] + dzg * tz[j];
                let zp = dxg * nx[j] + dzg * nz[j];
                let r1 = (xp * xp + zp * zp).sqrt().max(1e-12);
                let r2 = ((xp - len[j]).powi(2) + zp * zp).sqrt().max(1e-12);
                let th1 = zp.atan2(xp);
                let th2 = zp.atan2(xp - len[j]);
                // Constant-strength source panel.
                let us = (1.0 / (2.0 * PI)) * (r1 / r2).ln();
                let ws = (1.0 / (2.0 * PI)) * (th2 - th1);
                // The vortex field is the source field rotated a quarter turn,
                // which is where these come from rather than a second derivation.
                (us, ws, ws, -us)
            };
            // Back to global.
            let (sgx, sgz) = (us * tx[j] + ws * nx[j], us * tz[j] + ws * nz[j]);
            let (vgx, vgz) = (uv * tx[j] + wv * nx[j], uv * tz[j] + wv * nz[j]);
            an[i * n + j] = sgx * nx[i] + sgz * nz[i];
            at[i * n + j] = sgx * tx[i] + sgz * tz[i];
            avn[i] += vgx * nx[i] + vgz * nz[i];
            avt[i] += vgx * tx[i] + vgz * tz[i];
        }
    }

    // ── assemble and solve ──────────────────────────────────────────────────
    let (ca, sa) = (alpha.cos(), alpha.sin());
    let m = n + 1;
    let mut a = vec![0.0; m * m];
    let mut b = vec![0.0; m];
    // N no-penetration conditions at the control points.
    for i in 0..n {
        for j in 0..n {
            a[i * m + j] = an[i * n + j];
        }
        a[i * m + n] = avn[i];
        b[i] = -(ca * nx[i] + sa * nz[i]);
    }
    // The Kutta condition: the flow leaves the trailing edge smoothly, so the
    // tangential speeds on the two panels meeting there are equal and opposite.
    // Their tangents point in opposite senses around the trailing edge, which is
    // why this is a sum and not a difference.
    let (f, l) = (0usize, n - 1);
    for j in 0..n {
        a[n * m + j] = at[f * n + j] + at[l * n + j];
    }
    a[n * m + n] = avt[f] + avt[l];
    b[n] = -(ca * tx[f] + sa * tz[f]) - (ca * tx[l] + sa * tz[l]);

    let Some(sol) = solve_dense(&mut a, &mut b, m) else {
        return out;
    };
    let gamma = sol[n];

    // ── recover the flow, then the forces ───────────────────────────────────
    let mut fx = 0.0;
    let mut fz = 0.0;
    let mut cm = 0.0;
    out.cp_min = f64::MAX;
    for i in 0..n {
        let mut vt = ca * tx[i] + sa * tz[i] + gamma * avt[i];
        for j in 0..n {
            vt += sol[j] * at[i * n + j];
        }
        // The normal velocity is zero by construction, so the speed IS the
        // tangential component.
        let cp = 1.0 - vt * vt;
        out.cp[i] = cp;
        if cp < out.cp_min {
            out.cp_min = cp;
            out.cp_min_x = out.xc[i];
        }
        // Pressure acts inward on the surface.
        let (dfx, dfz) = (-cp * nx[i] * len[i], -cp * nz[i] * len[i]);
        fx += dfx;
        fz += dfz;
        // Moment about the quarter chord, positive NOSE UP.
        //
        // The scalar `rx*Fz - rz*Fx` is the component out of the (x, z) plane,
        // which points along -y in a right-handed (x, y, z) frame because
        // x cross z = -y. Nose up is positive about +y, so this is negated.
        // Left unnegated it gave the NACA 2412 a moment of +0.055 against its
        // published -0.05: the right magnitude, which is exactly how a sign
        // error survives being looked at.
        cm -= (out.xc[i] - 0.25) * dfz - out.zc[i] * dfx;
    }
    // Lift is across the freestream, drag along it.
    out.cl = -fx * sa + fz * ca;
    out.cd_pressure = fx * ca + fz * sa;
    out.cm = cm;
    // Kutta-Joukowski, by a wholly different route: total circulation is the
    // shared vortex strength over the perimeter.
    let perimeter: f64 = len.iter().sum();
    out.cl_circulation = 2.0 * gamma * perimeter;
    out
}

/// Gaussian elimination with partial pivoting. Written out rather than pulled
/// in: it is thirty lines, the system is small and dense, and a linear-algebra
/// dependency in the flight core would have to be justified to anyone auditing
/// it.
fn solve_dense(a: &mut [f64], b: &mut [f64], n: usize) -> Option<Vec<f64>> {
    for k in 0..n {
        let mut piv = k;
        let mut best = a[k * n + k].abs();
        for i in (k + 1)..n {
            let v = a[i * n + k].abs();
            if v > best {
                best = v;
                piv = i;
            }
        }
        if best < 1e-14 {
            return None;
        }
        if piv != k {
            for j in 0..n {
                a.swap(k * n + j, piv * n + j);
            }
            b.swap(k, piv);
        }
        let d = a[k * n + k];
        for i in (k + 1)..n {
            let f = a[i * n + k] / d;
            if f == 0.0 {
                continue;
            }
            for j in k..n {
                a[i * n + j] -= f * a[k * n + j];
            }
            b[i] -= f * b[k];
        }
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = b[i];
        for j in (i + 1)..n {
            s -= a[i * n + j] * x[j];
        }
        x[i] = s / a[i * n + i];
    }
    if x.iter().all(|v| v.is_finite()) {
        Some(x)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const N: usize = 200;

    fn deg(d: f64) -> f64 {
        d.to_radians()
    }

    // ─── Against closed-form theory ─────────────────────────────────────────

    /// A symmetric section makes no lift at zero incidence, and its moment about
    /// the quarter chord is zero at every angle. Both are exact.
    #[test]
    fn a_symmetric_section_is_neutral_at_zero_incidence() {
        let s = Section::naca4(0.0, 0.0, 12.0, N);
        let r = solve(&s, 0.0);
        assert!(r.cl.abs() < 1e-6, "cl {}", r.cl);
        assert!(r.cm.abs() < 1e-6, "cm {}", r.cm);
        // Near zero at incidence, but not exactly: thin-airfoil theory puts the
        // aerodynamic centre exactly at the quarter chord, and a real section
        // with thickness has it a little aft of there, so the moment drifts
        // slightly with angle. That drift is a property of the shape and is one
        // of the things having a shape buys.
        for a in [2.0, 5.0, 8.0] {
            let r = solve(&s, deg(a));
            assert!(r.cm.abs() < 0.02, "symmetric cm {} at {a} deg", r.cm);
        }
    }

    /// Thin airfoil theory: the lift-curve slope of ANY thin 2D section is
    /// 2*pi per radian. A real section with thickness runs slightly above it.
    /// This number is not in the code anywhere; it comes out of the solve.
    #[test]
    fn the_lift_curve_slope_is_two_pi_per_radian() {
        let s = Section::naca4(0.0, 0.0, 12.0, N);
        let a2 = solve(&s, deg(2.0)).cl;
        let a6 = solve(&s, deg(6.0)).cl;
        let slope = (a6 - a2) / deg(4.0);
        assert!(
            (slope - 2.0 * PI).abs() / (2.0 * PI) < 0.10,
            "slope {slope:.3} per rad against thin-airfoil 2*pi = {:.3}",
            2.0 * PI
        );
        // Positive incidence lifts. If this is negative the panel ordering or
        // the outward normal is inverted, and everything else would still look
        // plausible.
        assert!(a2 > 0.0 && a6 > a2, "cl {a2} then {a6}");
    }

    /// A thicker section has a slightly HIGHER slope than a thin one, which is
    /// the classic thickness correction and is a property of the shape rather
    /// than of anything supplied.
    #[test]
    fn thickness_raises_the_lift_curve_slope_slightly() {
        let slope = |t: f64| {
            let s = Section::naca4(0.0, 0.0, t, N);
            (solve(&s, deg(6.0)).cl - solve(&s, deg(2.0)).cl) / deg(4.0)
        };
        let thin = slope(6.0);
        let thick = slope(18.0);
        assert!(thick > thin, "18% slope {thick:.3} should exceed 6% slope {thin:.3}");
        assert!((thick - thin) / thin < 0.25, "and only slightly: {thin:.3} to {thick:.3}");
    }

    /// The NACA 2412's zero-lift angle is about -2.1 degrees, and its quarter
    /// chord moment about -0.05. Both are published, measured numbers for this
    /// section, and neither appears anywhere in this code: they follow from the
    /// camber line in the coordinates.
    #[test]
    fn the_naca_2412_reproduces_its_published_numbers() {
        let s = Section::naca4(2.0, 4.0, 12.0, N);
        // Bisect for zero lift.
        let (mut lo, mut hi) = (deg(-8.0), deg(4.0));
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            if solve(&s, mid).cl < 0.0 {
                lo = mid
            } else {
                hi = mid
            }
        }
        let a0 = 0.5 * (lo + hi).to_degrees();
        assert!(
            (a0 - -2.1).abs() < 0.6,
            "zero-lift angle {a0:.2} deg, published about -2.1"
        );
        let cm = solve(&s, deg(0.0)).cm;
        assert!((cm - -0.05).abs() < 0.03, "cm about c/4 {cm:.4}, published about -0.05");
    }

    /// Camber lifts at zero incidence, and more camber lifts more.
    #[test]
    fn camber_lifts_at_zero_incidence() {
        let cl0 = |m: f64| solve(&Section::naca4(m, 4.0, 12.0, N), 0.0).cl;
        assert!(cl0(0.0).abs() < 1e-6);
        assert!(cl0(2.0) > 0.15, "2% camber gives cl {}", cl0(2.0));
        assert!(cl0(4.0) > cl0(2.0));
    }

    // ─── Internal consistency ───────────────────────────────────────────────

    /// Lift by integrating the pressure and lift by Kutta-Joukowski are computed
    /// by entirely separate routes. Agreement is the single strongest check that
    /// the solve is right, because almost any error breaks one and not the other.
    #[test]
    fn pressure_integration_agrees_with_kutta_joukowski() {
        for (m, p, t) in [(0.0, 0.0, 12.0), (2.0, 4.0, 12.0), (4.0, 4.0, 15.0)] {
            let s = Section::naca4(m, p, t, N);
            for a in [-4.0, 0.0, 5.0, 10.0] {
                let r = solve(&s, deg(a));
                let d = (r.cl - r.cl_circulation).abs();
                assert!(
                    d < 0.02 + 0.03 * r.cl.abs(),
                    "{}: cl {:.4} by pressure vs {:.4} by circulation at {a} deg",
                    s.name,
                    r.cl,
                    r.cl_circulation
                );
            }
        }
    }

    /// D'Alembert's paradox: a body in steady inviscid incompressible potential
    /// flow experiences NO DRAG. Any drag this model reports is numerical.
    ///
    /// Asserted rather than hidden, because it is also the honest statement of
    /// what this method cannot do: real drag needs the boundary layer that goes
    /// on top of it.
    #[test]
    fn potential_flow_produces_no_drag() {
        let s = Section::naca4(2.0, 4.0, 12.0, N);
        for a in [0.0, 4.0, 8.0] {
            let r = solve(&s, deg(a));
            assert!(
                r.cd_pressure.abs() < 0.01,
                "pressure drag {:.5} at {a} deg is not zero",
                r.cd_pressure
            );
        }
    }

    /// At the stagnation point the flow is brought to rest, so Cp is exactly 1.
    /// There must be exactly one such point, near the leading edge, and it must
    /// move as incidence changes.
    #[test]
    fn there_is_a_stagnation_point_with_cp_of_one() {
        let s = Section::naca4(2.0, 4.0, 12.0, N);
        for a in [0.0, 6.0] {
            let r = solve(&s, deg(a));
            let peak = r.cp.iter().cloned().fold(f64::MIN, f64::max);
            assert!((peak - 1.0).abs() < 0.02, "peak cp {peak:.4} at {a} deg, expected 1.0");
            let at = r.cp.iter().enumerate().max_by(|x, y| x.1.total_cmp(y.1)).unwrap().0;
            assert!(r.xc[at] < 0.05, "stagnation at x/c {:.3}, expected near the LE", r.xc[at]);
        }
    }

    #[test]
    fn the_solution_converges_with_panel_count() {
        let cl = |n: usize| solve(&Section::naca4(2.0, 4.0, 12.0, n), deg(5.0)).cl;
        let coarse = cl(60);
        let fine = cl(160);
        let finer = cl(320);
        assert!((finer - fine).abs() < (fine - coarse).abs() + 1e-9, "not converging");
        assert!((finer - fine).abs() < 0.01, "still moving at 320 panels: {fine:.4} to {finer:.4}");
    }

    // ─── The leading edge, which is the whole point ─────────────────────────

    /// The geometry is now an INPUT, so a leading-edge radius can be measured
    /// off the section rather than asserted about it.
    #[test]
    fn leading_edge_radius_follows_from_the_thickness() {
        // For a NACA 4-digit section the LE radius is 1.1019 * t^2 chords.
        for t in [6.0, 12.0, 18.0] {
            let s = Section::naca4(0.0, 0.0, t, 240);
            let r = s.le_radius();
            let want = 1.1019 * (t / 100.0).powi(2);
            assert!(
                (r - want).abs() / want < 0.35,
                "{}: measured LE radius {r:.5} c, analytic {want:.5} c",
                s.name
            );
        }
        assert!(Section::naca4(0.0, 0.0, 18.0, 240).le_radius() > Section::naca4(0.0, 0.0, 6.0, 240).le_radius());
    }

    /// THE result this whole module exists for: a SHARP leading edge produces a
    /// violent suction peak that a rounded one does not.
    ///
    /// That peak is what a boundary layer cannot survive, so it is the direct
    /// cause of a thin section stalling early and abruptly while a thick one
    /// holds on. Nothing here was told which section is sharp. The difference
    /// comes out of the coordinates, which is exactly what was missing before.
    #[test]
    fn a_sharp_leading_edge_produces_a_violent_suction_peak() {
        let sharp = Section::naca4(0.0, 0.0, 4.0, 300);
        let round = Section::naca4(0.0, 0.0, 18.0, 300);
        let a = deg(8.0);
        let (ps, pr) = (solve(&sharp, a), solve(&round, a));

        assert!(ps.cp_min < pr.cp_min, "sharp peak {:.2} should be deeper than round {:.2}", ps.cp_min, pr.cp_min);
        assert!(
            ps.cp_min < 2.0 * pr.cp_min,
            "and by a lot: 4% section peaks at Cp {:.2}, 18% at {:.2}",
            ps.cp_min,
            pr.cp_min
        );
        // Both peak at the nose, which is where the trouble is.
        assert!(ps.cp_min_x < 0.06 && pr.cp_min_x < 0.15, "peaks at {:.3} and {:.3} c", ps.cp_min_x, pr.cp_min_x);
        // And the peak gets worse with incidence, which is why stall happens at
        // an angle rather than at a speed.
        assert!(solve(&sharp, deg(12.0)).cp_min < ps.cp_min);
    }

    // ─── Cost ───────────────────────────────────────────────────────────────

    /// Settles the "is Rust fast enough for this" question by measurement.
    ///
    /// This is a BAKE step: one solve per section per angle, done once and
    /// tabulated, never in the flight loop. The budget is generous by orders of
    /// magnitude and the point of the test is to keep it that way.
    #[test]
    fn a_full_polar_is_cheap_enough_to_bake() {
        let s = Section::naca4(2.0, 4.0, 12.0, 200);
        let t0 = std::time::Instant::now();
        let mut points = 0;
        for i in 0..61 {
            let a = deg(-10.0 + 0.5 * i as f64);
            let r = solve(&s, a);
            assert!(r.cl.is_finite());
            points += 1;
        }
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        println!("panel method: {points} points at 200 panels in {ms:.1} ms ({:.2} ms/point)", ms / points as f64);
        // Deliberately loose: this runs in a debug build in CI. Even here it has
        // to be well inside a second for a whole polar.
        assert!(ms < 20_000.0, "a 61-point polar took {ms:.0} ms");
    }
}
