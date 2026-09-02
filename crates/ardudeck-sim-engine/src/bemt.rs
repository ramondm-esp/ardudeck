//! Blade element momentum theory rotors, for vehicles that fly edgewise.
//!
//! `motor.rs` stays exactly as it is: it is a faithful port of ArduPilot's
//! SIM_Motor.cpp and every multirotor test in this crate is calibrated against
//! it. This module is the rotor model for aircraft that spend real time with the
//! disc at an angle to the flow, where the ported model is not merely less
//! accurate but structurally unable to be right:
//!
//! - Momentum theory has NO advance ratio. It computes one inflow from the axial
//!   component alone, so a rotor translating at 25 m/s edgewise is treated as a
//!   rotor in hover. The entire transition corridor is that case.
//! - It produces no in-plane H-force and no hub moment, because it has no
//!   azimuth. Both are first-order effects on a tiltrotor and both are what makes
//!   a real transition need coordinated control rather than a throttle ramp.
//! - It is undefined in descent. The momentum equation has no solution through
//!   the vortex ring state, and VRS is the failure mode a VTOL customer most
//!   needs to see coming.
//!
//! Structure follows Leishman, *Principles of Helicopter Aerodynamics* (2nd ed.):
//! Glauert's inflow equation for a rotor at incidence, Drees' linear inflow for
//! the forward-flight gradient, the standard empirical curve fit through the
//! vortex ring state where momentum theory has no solution, and blade element
//! integration over radius and azimuth using the same full-circle airfoil tables
//! `aero.rs` builds.

use crate::aero::AirfoilTable;
use crate::math::Vec3;
use std::f64::consts::PI;

/// Radial stations per blade. The integrand is smooth in r away from the tip, so
/// the tip-loss factor does more for accuracy here than more stations would.
pub const RADIAL_STATIONS: usize = 10;
/// Azimuth stations. Only needed at all because edgewise flow makes the blade
/// loading vary around the disc; 12 resolves the once-per-rev asymmetry that
/// produces the H-force and hub moment without resolving harmonics we do not
/// model anyway.
pub const AZIMUTH_STATIONS: usize = 12;

/// Root cutout as a fraction of radius: the hub and blade grip carry no lift.
const ROOT_CUTOUT: f64 = 0.15;

/// Maximum inflow iterations. Glauert's equation converges in a handful of
/// fixed-point steps under relaxation; the cap only bounds pathological input.
const INFLOW_ITERS: usize = 40;
const INFLOW_TOL: f64 = 1e-6;
/// Under-relaxation on the inflow fixed point. Undamped iteration oscillates
/// near the VRS boundary instead of converging.
const INFLOW_RELAX: f64 = 0.35;
/// Outer passes coupling blade-element thrust to the inflow solve.
const INFLOW_OUTER_PASSES: usize = 8;

// ─── Rotor description ──────────────────────────────────────────────────────

/// A rotor's geometry, blades and drive. Physical quantities only: nothing here
/// is a tuning knob, so a partner can fill it in from their own build sheet.
#[derive(Debug, Clone)]
pub struct Rotor {
    /// Hub position relative to the CG, body frame (m).
    pub position: Vec3,
    /// Unit thrust axis in body frame at zero tilt. Body -z (up) for a lift
    /// rotor, body +x (forward) for a pusher.
    pub axis: Vec3,
    /// Axis the rotor tilts about, body frame. Body +y (the pitch axis) tilts a
    /// lift rotor forward into a pusher, which is the tiltrotor case.
    pub tilt_axis: Vec3,
    pub radius: f64,
    pub blades: usize,
    /// Blade chord at the root and tip (m); linear taper between them.
    pub chord_root: f64,
    pub chord_tip: f64,
    /// Blade pitch at the root and tip (rad), giving linear twist. A propeller
    /// carries large twist (high root, low tip); a helicopter blade carries
    /// little.
    pub pitch_root: f64,
    pub pitch_tip: f64,
    /// Index into the airfoil table array used for the blade sections.
    pub airfoil: usize,
    /// Rotation sense about `axis`: +1 or -1. Sets the reaction torque sign and
    /// which side of the disc is advancing.
    pub spin: f64,
    /// Polar moment of inertia of rotor plus prop (kg m^2). Sets spool-up rate,
    /// which is a real handling factor on a VTOL: lift rotors that spool down
    /// slowly are what makes a botched transition recoverable.
    pub inertia: f64,
    pub motor: MotorDrive,
}

/// Brushless motor and ESC, as the two constants a datasheet actually gives.
#[derive(Debug, Clone, Copy)]
pub struct MotorDrive {
    /// Motor velocity constant, RPM per volt, no load.
    pub kv: f64,
    /// Winding resistance (ohm).
    pub resistance: f64,
    /// No-load current at the reference voltage (A).
    pub no_load_current: f64,
    /// Full pack voltage (V).
    pub voltage_max: f64,
}

impl MotorDrive {
    /// Torque constant Kt (N m / A) from Kv. Kt = 60/(2*pi*Kv) in SI, an
    /// identity for any permanent-magnet machine, not a fit.
    pub fn kt(&self) -> f64 {
        if self.kv <= 0.0 {
            return 0.0;
        }
        60.0 / (2.0 * PI * self.kv)
    }
}

/// Live rotor state, integrated by the caller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotorState {
    /// Angular speed (rad/s), always non-negative; `Rotor::spin` carries sense.
    pub omega: f64,
    /// Commanded tilt about `tilt_axis` (rad). 0 = the rotor's `axis` as given.
    pub tilt: f64,
}

impl Default for RotorState {
    fn default() -> Self {
        RotorState { omega: 0.0, tilt: 0.0 }
    }
}

/// One rotor's contribution, body frame.
#[derive(Debug, Clone, Copy, Default)]
pub struct RotorOutput {
    /// Total force at the hub: thrust along the tilted axis plus the in-plane
    /// H-force (m).
    pub force_bf: Vec3,
    /// Moment about the CG: reaction torque, hub moment, and the hub force's
    /// moment arm.
    pub moment_bf: Vec3,
    /// Thrust along the tilted axis (N).
    pub thrust: f64,
    /// In-plane force magnitude, opposing the edgewise flow (N).
    pub h_force: f64,
    /// Aerodynamic torque resisting rotation (N m), always >= 0.
    pub torque: f64,
    /// Converged mean induced velocity through the disc (m/s).
    pub induced_velocity: f64,
    /// Advance ratio, edgewise speed over tip speed.
    pub advance_ratio: f64,
    /// Wake skew angle from the disc axis (rad). 0 = axial, pi/2 = fully edgewise.
    pub skew: f64,
    /// Pack current this rotor draws (A).
    pub current: f64,
    /// True while the operating point sits inside the vortex ring state, where
    /// momentum theory has no solution and the empirical fit is in use.
    pub in_vrs: bool,
    /// Fully developed slipstream speed (m/s), the wake source a wing downstream
    /// samples and what a neighbour vehicle sees.
    pub slipstream: f64,
}

// ─── Inflow ─────────────────────────────────────────────────────────────────

/// Where on the induced-velocity curve a rotor is operating.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowState {
    /// Climb or hover: momentum theory's normal working state.
    Normal,
    /// Descent between hover and the windmill brake: momentum theory has no
    /// valid solution and the empirical fit is used.
    VortexRing,
    /// Descending fast enough that the wake is fully above the disc again.
    WindmillBrake,
}

/// Non-dimensional induced velocity vi/vh in AXIAL flow, from the climb ratio
/// Vc/vh (positive = climbing).
///
/// Three branches, because there is no single valid expression:
/// - Vc/vh >= 0: momentum theory, normal working state.
/// - Vc/vh <= -2: momentum theory, windmill brake state.
/// - between: the flow is recirculating, momentum theory's control volume
///   assumption is violated outright, and the accepted substitute is a curve fit
///   to measured data (Leishman 2.14.3, coefficients from the collected
///   autorotation and descent measurements). This region IS the vortex ring
///   state, and modelling it is the difference between a sim that shows a VTOL
///   settling into its own wake and one that lets it descend forever.
pub fn axial_inflow_ratio(vc_over_vh: f64) -> (f64, FlowState) {
    let x = vc_over_vh;
    if x >= 0.0 {
        let h = x / 2.0;
        (-h + (h * h + 1.0).sqrt(), FlowState::Normal)
    } else if x <= -2.0 {
        let h = x / 2.0;
        (-h - (h * h - 1.0).sqrt(), FlowState::WindmillBrake)
    } else {
        // Empirical fit over -2 <= Vc/vh <= 0. Matches 1.0 at hover by
        // construction and is within 3% of the windmill branch at -2, so the
        // curve is continuous across both joins.
        let k = [1.0, -1.125, -1.372, -1.718, -0.655];
        let mut v = 0.0;
        let mut p = 1.0;
        for c in k {
            v += c * p;
            p *= x;
        }
        (v, FlowState::VortexRing)
    }
}

/// Solve Glauert's inflow equation for a rotor at incidence:
///
/// `vi = vh^2 / sqrt(v_par^2 + (v_axial + vi)^2)`
///
/// where `v_par` is the in-plane (edgewise) speed and `v_axial` is the component
/// through the disc, positive in the thrust direction (so a climbing lift rotor
/// is positive). This is the one equation that covers hover, axial climb and
/// high-speed forward flight, and reduces to `vi = vh` at rest.
///
/// In near-axial descent it hands over to `axial_inflow_ratio`, because Glauert's
/// equation inherits momentum theory's failure through the vortex ring state.
pub fn glauert_inflow(vh: f64, v_par: f64, v_axial: f64) -> (f64, FlowState) {
    if vh <= 1e-9 {
        return (0.0, FlowState::Normal);
    }
    // Edgewise flow breaks up the recirculating wake, so VRS only exists close to
    // axial. mu > ~0.1 of the hover induced velocity is enough to escape it,
    // which is why forward flight is the standard VRS recovery.
    let near_axial = v_par < 0.5 * vh;
    if near_axial && v_axial < 0.0 {
        let (ratio, state) = axial_inflow_ratio(v_axial / vh);
        return (ratio * vh, state);
    }
    let mut vi = vh;
    for _ in 0..INFLOW_ITERS {
        let denom = (v_par * v_par + (v_axial + vi) * (v_axial + vi)).sqrt().max(1e-9);
        let next = vh * vh / denom;
        let step = next - vi;
        vi += step * INFLOW_RELAX;
        if step.abs() < INFLOW_TOL {
            break;
        }
    }
    (vi.max(0.0), FlowState::Normal)
}

/// Drees' linear inflow model: the fore-aft and lateral gradients that make the
/// inflow non-uniform in forward flight. `chi` is the wake skew angle (0 axial,
/// pi/2 fully edgewise), `mu` the advance ratio.
///
/// Returns `(k_x, k_y)` for `lambda(r, psi) = lambda_0 * (1 + k_x * x * cos(psi)
/// + k_y * x * sin(psi))`. Without this the disc is uniformly loaded and produces
/// no hub moment in forward flight, which is a first-order error on a tiltrotor.
pub fn drees_inflow_gradient(chi: f64, mu: f64) -> (f64, f64) {
    let s = chi.sin();
    if s.abs() < 1e-6 {
        return (0.0, 0.0);
    }
    let k_x = (4.0 / 3.0) * ((1.0 - chi.cos() - 1.8 * mu * mu) / s);
    (k_x, -2.0 * mu)
}

/// Prandtl tip-loss factor at radial station `x` for a rotor with `b` blades at
/// inflow angle `phi`. Lift falls to zero at the tip because the pressure
/// equalises around it; without this a BEMT rotor overpredicts thrust by roughly
/// the tip-loss fraction, several percent, systematically.
pub fn prandtl_tip_loss(x: f64, b: usize, phi: f64) -> f64 {
    let s = phi.abs().sin();
    if s < 1e-6 || x >= 1.0 {
        return if x >= 1.0 { 0.0 } else { 1.0 };
    }
    let f = (b as f64 / 2.0) * (1.0 - x) / (x.max(1e-6) * s);
    (2.0 / PI) * (-f).exp().clamp(0.0, 1.0).acos()
}

// ─── The rotor solve ────────────────────────────────────────────────────────

impl Rotor {
    pub fn disc_area(&self) -> f64 {
        PI * self.radius * self.radius
    }

    /// Thrust axis after tilt, body frame. Rodrigues about `tilt_axis`.
    pub fn tilted_axis(&self, tilt: f64) -> Vec3 {
        if tilt == 0.0 {
            return self.axis.normalize();
        }
        let k = self.tilt_axis.normalize();
        let v = self.axis.normalize();
        let (s, c) = tilt.sin_cos();
        v.scale(c).add(k.cross(v).scale(s)).add(k.scale(k.dot(v) * (1.0 - c))).normalize()
    }

    fn chord_at(&self, x: f64) -> f64 {
        self.chord_root + (self.chord_tip - self.chord_root) * x
    }

    fn pitch_at(&self, x: f64) -> f64 {
        self.pitch_root + (self.pitch_tip - self.pitch_root) * x
    }

    /// Blade element integration at a known rotor speed and known local flow.
    ///
    /// `velocity_air_bf` is the vehicle's velocity relative to the air in body
    /// frame; `gyro` the body rates, so the hub's own velocity from rotation is
    /// included. `induced_ext` is any extra local flow at the hub (a neighbour's
    /// wake), as the air's velocity relative to the airframe.
    pub fn forces(
        &self,
        state: RotorState,
        velocity_air_bf: Vec3,
        gyro: Vec3,
        air_density: f64,
        airfoils: &[AirfoilTable],
        induced_ext: Vec3,
    ) -> RotorOutput {
        let axis = self.tilted_axis(state.tilt);
        let omega = state.omega.max(0.0);
        let r = self.radius;
        let tip_speed = omega * r;

        // Flow the hub sees, relative to the air.
        let v_hub = velocity_air_bf.add(gyro.cross(self.position)).sub(induced_ext);
        // Positive when the rotor is climbing into its own thrust direction:
        // `axis` IS that direction, so the projection needs no negation. Getting
        // this backwards inverts climb and descent, which silently swaps the
        // normal working state for the vortex ring state.
        let v_axial = v_hub.dot(axis);
        let v_par_vec = v_hub.sub(axis.scale(v_hub.dot(axis)));
        let v_par = v_par_vec.length();

        let mut out = RotorOutput {
            advance_ratio: if tip_speed > 1e-6 { v_par / tip_speed } else { 0.0 },
            ..Default::default()
        };
        if omega < 1e-6 || self.blades == 0 {
            return out;
        }

        // The hover induced velocity depends on thrust and thrust depends on the
        // inflow, so seed from a pass at zero inflow and iterate. The coupling is
        // negative feedback (more inflow, less alpha, less thrust) so it is a
        // contraction, but it starts far from the answer and needs enough passes
        // for the reported vi and thrust to be mutually consistent, which is what
        // `hover_thrust_agrees_with_momentum_theory` checks.
        let mut vi = 0.0;
        let mut state_flow = FlowState::Normal;
        let mut integ = self.integrate(omega, v_axial, v_par, 0.0, 0.0, 0.0, air_density, airfoils);
        for _ in 0..INFLOW_OUTER_PASSES {
            let t = integ.thrust.max(0.0);
            let vh = (t / (2.0 * air_density * self.disc_area())).max(0.0).sqrt();
            let (v, st) = glauert_inflow(vh, v_par, v_axial);
            vi = v;
            state_flow = st;
            // Wake skew: 0 when the flow is axial, pi/2 when fully edgewise.
            let chi = v_par.atan2((v_axial + vi).abs().max(1e-9));
            let (kx, ky) = drees_inflow_gradient(chi, out.advance_ratio);
            integ = self.integrate(omega, v_axial, v_par, vi, kx, ky, air_density, airfoils);
            out.skew = chi;
        }

        let thrust = integ.thrust;
        let par_dir = if v_par > 1e-9 { v_par_vec.normalize() } else { Vec3::zero() };
        let lat = if v_par > 1e-9 { axis.cross(par_dir) } else { Vec3::zero() };
        let force = axis
            .scale(thrust)
            .add(par_dir.scale(integ.f_par))
            .add(lat.scale(integ.f_lat));

        // Reaction torque opposes rotation, so it is along -spin about the axis.
        let reaction = axis.scale(-self.spin * integ.torque);
        let hub = par_dir.scale(integ.m_par).add(lat.scale(integ.m_lat));

        out.force_bf = force;
        out.moment_bf = reaction.add(hub).add(self.position.cross(force));
        out.thrust = thrust;
        // Reported positive when the in-plane force opposes the edgewise flow,
        // i.e. when it is drag, which is the only case anyone reads it for.
        out.h_force = -integ.f_par;
        out.torque = integ.torque;
        out.induced_velocity = vi;
        out.in_vrs = state_flow == FlowState::VortexRing;
        // Far-wake speed is twice the induced velocity at the disc (momentum
        // theory), on top of whatever the rotor is already moving through.
        out.slipstream = v_axial + 2.0 * vi;

        let kt = self.motor.kt();
        out.current = if kt > 1e-9 {
            integ.torque / kt + self.motor.no_load_current
        } else {
            0.0
        };
        out
    }

    /// One blade element pass at a fixed inflow. Split out because the inflow
    /// solve calls it repeatedly.
    #[allow(clippy::too_many_arguments)]
    fn integrate(
        &self,
        omega: f64,
        v_axial: f64,
        v_par: f64,
        vi: f64,
        kx: f64,
        ky: f64,
        rho: f64,
        airfoils: &[AirfoilTable],
    ) -> Integrated {
        let table = &airfoils[self.airfoil.min(airfoils.len().saturating_sub(1))];
        let r = self.radius;
        let b = self.blades as f64;
        let dx = (1.0 - ROOT_CUTOUT) / RADIAL_STATIONS as f64;
        let dpsi = 2.0 * PI / AZIMUTH_STATIONS as f64;

        let mut thrust = 0.0;
        let mut torque = 0.0;
        let mut f_par = 0.0;
        let mut f_lat = 0.0;
        let mut m_par = 0.0;
        let mut m_lat = 0.0;

        for i in 0..RADIAL_STATIONS {
            let x = ROOT_CUTOUT + dx * (i as f64 + 0.5);
            let rad = x * r;
            let c = self.chord_at(x);
            let theta = self.pitch_at(x);

            for j in 0..AZIMUTH_STATIONS {
                let psi = dpsi * (j as f64 + 0.5);
                let (sp, cp) = psi.sin_cos();

                // Tangential: rotation plus the edgewise component, which adds on
                // the advancing side and subtracts on the retreating side. This
                // asymmetry is the entire source of the H-force and hub moment.
                let u_t = omega * rad + self.spin * v_par * sp;
                // Perpendicular: through-disc flow, with Drees' linear gradient.
                let lambda = vi * (1.0 + kx * x * cp + ky * x * sp);
                // Leishman's inflow ratio: the through-disc flow is the climb
                // rate PLUS the induced velocity, both measured in the direction
                // that reduces the blade's angle of attack. A climbing rotor
                // therefore unloads, which is the correct behaviour.
                let u_p = lambda + v_axial;
                let u_sq = u_t * u_t + u_p * u_p;
                if u_sq < 1e-9 {
                    continue;
                }
                let phi = u_p.atan2(u_t);
                let alpha = theta - phi;
                let cf = table.at(alpha);

                let tl = prandtl_tip_loss(x, self.blades, phi);
                let q = 0.5 * rho * u_sq * c;
                let dl = q * cf.cl * tl;
                let dd = q * cf.cd;

                let (sphi, cphi) = phi.sin_cos();
                // Normal to the disc, and in the plane of rotation.
                let dfn = dl * cphi - dd * sphi;
                let dft = dl * sphi + dd * cphi;

                // Average over azimuth (dpsi/2pi) and integrate over the blade
                // span, times the blade count.
                let w = b * (dpsi / (2.0 * PI)) * dx * r;
                thrust += dfn * w;
                torque += dft * rad * w;
                // In-plane force, resolved in the (par_dir, lat) basis that psi
                // is measured in. A blade at azimuth psi points along
                // d = par*cos + lat*sin and moves along spin*(-par*sin +
                // lat*cos); the tangential air load opposes that motion, so the
                // reaction on the airframe is -dft along it.
                f_par += dft * self.spin * sp * w;
                f_lat += -dft * self.spin * cp * w;
                // Hub moment from the once-per-rev loading asymmetry:
                // (rad*d) x (dfn*axis), with par x axis = -lat, lat x axis = par.
                m_par += dfn * rad * sp * w;
                m_lat += -dfn * rad * cp * w;
            }
        }
        Integrated { thrust, torque, f_par, f_lat, m_par, m_lat }
    }

    /// Step the rotor speed one tick under motor torque and aerodynamic load.
    ///
    /// A first-order BLDC model on a real inertia, rather than a commanded RPM,
    /// because spool-up lag is a handling factor on a VTOL: how fast the lift
    /// rotors come back is what decides whether an aborted transition is
    /// recoverable, and a model that snaps to the commanded speed cannot show it.
    pub fn step_omega(&self, state: RotorState, throttle: f64, pack_voltage: f64, load_torque: f64, dt: f64) -> f64 {
        let kt = self.motor.kt();
        if kt <= 1e-9 || self.inertia <= 1e-9 {
            return state.omega;
        }
        let v = (pack_voltage * throttle.clamp(0.0, 1.0)).max(0.0);
        let back_emf = state.omega * kt;
        let current = ((v - back_emf) / self.motor.resistance.max(1e-6)).max(-200.0);
        let q_motor = kt * (current - self.motor.no_load_current.copysign(current.max(0.0) + 1e-9));
        let alpha = (q_motor - load_torque) / self.inertia;
        (state.omega + alpha * dt).max(0.0)
    }

    /// Rotor speed that a throttle setting reaches with no aerodynamic load, for
    /// seeding a trim state rather than spooling up from rest every reset.
    pub fn no_load_omega(&self, throttle: f64, pack_voltage: f64) -> f64 {
        let kt = self.motor.kt();
        if kt <= 1e-9 {
            return 0.0;
        }
        (pack_voltage * throttle.clamp(0.0, 1.0) / kt).max(0.0)
    }
}

/// One blade element pass, in the (par_dir, lat) in-plane basis. Keeping the
/// in-plane terms as vector components rather than as signed scalars is what
/// makes the H-force and hub-moment signs derivable instead of guessed.
struct Integrated {
    thrust: f64,
    torque: f64,
    /// In-plane force along the edgewise flow direction (N). Negative is drag.
    f_par: f64,
    /// In-plane force perpendicular to it (N).
    f_lat: f64,
    m_par: f64,
    m_lat: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aero::{AirfoilSpec, AirfoilTable};

    /// A thin, low-camber blade section. Propeller blades run at low Reynolds
    /// number and stall early, which matters: a heavily loaded lift rotor really
    /// does stall its root.
    fn blade_airfoil() -> Vec<AirfoilTable> {
        vec![AirfoilTable::from_spec(&AirfoilSpec {
            cl_alpha: 5.7,
            alpha_0: -0.05,
            alpha_stall: 0.22,
            alpha_stall_neg: -0.20,
            cd_min: 0.015,
            cd_k: 0.04,
            cm_0: -0.03,
            aspect_ratio: 8.0,
        })]
    }

    /// A 15 inch two-blade lift rotor, roughly a 5 kg quadplane's.
    fn lift_rotor() -> Rotor {
        Rotor {
            position: Vec3::new(0.35, 0.35, 0.0),
            axis: Vec3::new(0.0, 0.0, -1.0),
            tilt_axis: Vec3::new(0.0, 1.0, 0.0),
            radius: 0.19,
            blades: 2,
            chord_root: 0.030,
            chord_tip: 0.018,
            pitch_root: 0.42,
            pitch_tip: 0.16,
            airfoil: 0,
            spin: 1.0,
            inertia: 6.0e-5,
            motor: MotorDrive { kv: 400.0, resistance: 0.08, no_load_current: 0.7, voltage_max: 22.2 },
        }
    }

    fn still() -> (Vec3, Vec3) {
        (Vec3::zero(), Vec3::zero())
    }

    fn run(r: &Rotor, omega: f64, v: Vec3) -> RotorOutput {
        r.forces(
            RotorState { omega, tilt: 0.0 },
            v,
            Vec3::zero(),
            1.225,
            &blade_airfoil(),
            Vec3::zero(),
        )
    }

    // ─── Momentum-theory agreement ──────────────────────────────────────────

    /// The converged state must be self-consistent: the inflow the solver
    /// reports has to be the one that thrust implies, T = 2*rho*A*vi^2. This is
    /// the check that the blade element pass and the inflow solve actually agree
    /// rather than each being separately plausible.
    #[test]
    fn hover_thrust_agrees_with_momentum_theory() {
        let r = lift_rotor();
        for omega in [400.0, 700.0, 1100.0] {
            let out = run(&r, omega, Vec3::zero());
            assert!(out.thrust > 0.0, "no thrust at {omega} rad/s");
            let implied = 2.0 * 1.225 * r.disc_area() * out.induced_velocity.powi(2);
            let err = (implied - out.thrust).abs() / out.thrust;
            assert!(err < 0.05, "omega {omega}: thrust {} vs momentum {implied} ({:.1}%)",
                out.thrust, err * 100.0);
        }
    }

    /// Thrust goes as the square of rotor speed, the defining property of a
    /// fixed-pitch rotor.
    #[test]
    fn thrust_scales_with_the_square_of_rotor_speed() {
        let r = lift_rotor();
        let a = run(&r, 500.0, Vec3::zero()).thrust;
        let b = run(&r, 1000.0, Vec3::zero()).thrust;
        let ratio = b / a;
        assert!((3.6..4.4).contains(&ratio), "doubling omega gave {ratio}x thrust");
    }

    #[test]
    fn a_stopped_rotor_produces_nothing() {
        let out = run(&lift_rotor(), 0.0, Vec3::new(20.0, 0.0, 0.0));
        assert_eq!(out.thrust, 0.0);
        assert_eq!(out.torque, 0.0);
        assert_eq!(out.force_bf, Vec3::zero());
    }

    /// The rotor is lifting a plausible aircraft, not producing a number that
    /// merely has the right units. A 15 inch prop at ~7000 rpm should make
    /// something in the region of 10 N.
    #[test]
    fn thrust_is_physically_plausible_for_the_geometry() {
        let out = run(&lift_rotor(), 730.0, Vec3::zero());
        assert!((4.0..30.0).contains(&out.thrust), "15 inch prop at 7000 rpm gave {} N", out.thrust);
        assert!(out.torque > 0.0 && out.torque < 2.0, "torque {} N m", out.torque);
        // Figure of merit: ideal power over actual. A real prop is 0.4 to 0.8.
        let ideal = out.thrust * out.induced_velocity;
        let actual = out.torque * 730.0;
        let fm = ideal / actual;
        assert!((0.3..0.9).contains(&fm), "figure of merit {fm}");
    }

    // ─── Axial flow: climb, descent, vortex ring ────────────────────────────

    #[test]
    fn hover_inflow_ratio_is_one() {
        let (v, s) = axial_inflow_ratio(0.0);
        assert!((v - 1.0).abs() < 1e-12, "{v}");
        assert_eq!(s, FlowState::Normal);
    }

    #[test]
    fn climbing_reduces_induced_velocity() {
        let (hover, _) = axial_inflow_ratio(0.0);
        for c in [0.5, 1.0, 3.0] {
            let (v, s) = axial_inflow_ratio(c);
            assert!(v < hover, "climb {c} gave vi/vh {v}");
            assert_eq!(s, FlowState::Normal);
        }
    }

    /// The whole reason this module exists in place of momentum theory. Between
    /// hover and the windmill brake state the rotor is recirculating its own
    /// wake, momentum theory has NO solution, and the induced velocity is higher
    /// than either branch predicts.
    #[test]
    fn the_vortex_ring_state_is_modelled_between_the_two_momentum_branches() {
        let (_, s) = axial_inflow_ratio(-1.0);
        assert_eq!(s, FlowState::VortexRing);
        // Peak induced velocity sits inside the region, above the hover value.
        let mut worst: f64 = 0.0;
        let mut worst_at = 0.0;
        let mut x = -1.99;
        while x < 0.0 {
            let (v, _) = axial_inflow_ratio(x);
            if v > worst {
                worst = v;
                worst_at = x;
            }
            x += 0.01;
        }
        assert!(worst > 1.05, "VRS should raise vi above hover, peak {worst}");
        assert!((-1.6..-0.4).contains(&worst_at), "VRS peak at Vc/vh = {worst_at}");
    }

    /// The curve fit exists to bridge two branches, so it has to actually MEET
    /// them. A discontinuity here is a force step in the FDM during a descent,
    /// which is exactly when a customer is watching.
    #[test]
    fn the_induced_velocity_curve_is_continuous_across_both_joins() {
        let mut prev: Option<f64> = None;
        let mut x = -4.0;
        while x <= 4.0 {
            let (v, _) = axial_inflow_ratio(x);
            assert!(v.is_finite() && v >= 0.0, "vi/vh = {v} at {x}");
            if let Some(p) = prev {
                assert!((v - p).abs() < 0.05, "jump {p} -> {v} at Vc/vh = {x}");
            }
            prev = Some(v);
            x += 0.005;
        }
    }

    #[test]
    fn windmill_brake_state_is_reached_in_fast_descent() {
        let (_, s) = axial_inflow_ratio(-3.0);
        assert_eq!(s, FlowState::WindmillBrake);
    }

    /// Forward flight is the standard VRS recovery, and it has to be the
    /// recovery in the model too: edgewise flow blows the recirculating wake
    /// away, so the same descent rate is safe once you are moving.
    #[test]
    fn edgewise_flow_escapes_the_vortex_ring_state() {
        let vh = 8.0;
        let descent = -1.0 * vh;
        let (_, slow) = glauert_inflow(vh, 0.1, descent);
        assert_eq!(slow, FlowState::VortexRing, "near-axial descent should be VRS");
        let (_, fast) = glauert_inflow(vh, 20.0, descent);
        assert_eq!(fast, FlowState::Normal, "edgewise flow should clear it");
    }

    #[test]
    fn glauert_reduces_to_hover_at_rest() {
        let (vi, _) = glauert_inflow(7.0, 0.0, 0.0);
        assert!((vi - 7.0).abs() < 1e-3, "vi {vi}");
    }

    /// Glauert's high-speed asymptote: vi -> vh^2 / V. This is what makes a
    /// rotor cheap to hold up once it is moving, and momentum theory cannot
    /// produce it at all.
    #[test]
    fn glauert_approaches_the_high_speed_asymptote() {
        let vh = 7.0;
        let v = 60.0;
        let (vi, _) = glauert_inflow(vh, v, 0.0);
        let asymptote = vh * vh / v;
        assert!((vi - asymptote).abs() / asymptote < 0.05, "vi {vi} vs {asymptote}");
    }

    #[test]
    fn induced_velocity_falls_monotonically_with_forward_speed() {
        let vh = 7.0;
        let mut prev = f64::INFINITY;
        for v in [0.0, 2.0, 5.0, 10.0, 20.0, 40.0, 80.0] {
            let (vi, _) = glauert_inflow(vh, v, 0.0);
            assert!(vi < prev + 1e-9, "vi rose at v = {v}: {vi} after {prev}");
            prev = vi;
        }
    }

    // ─── Edgewise flow ──────────────────────────────────────────────────────

    /// A rotor translating edgewise drags. Momentum theory produces no in-plane
    /// force at all, so this is a term the ported model is structurally missing.
    #[test]
    fn edgewise_flow_produces_drag_on_the_disc() {
        let r = lift_rotor();
        let hover = run(&r, 800.0, Vec3::zero());
        assert!(hover.h_force.abs() < 1e-6, "hover H-force {}", hover.h_force);
        let moving = run(&r, 800.0, Vec3::new(20.0, 0.0, 0.0));
        assert!(moving.h_force > 0.0, "edgewise H-force should be drag, got {}", moving.h_force);
        // And it must actually oppose the motion in body frame.
        assert!(moving.force_bf.x < 0.0, "in-plane force x {}", moving.force_bf.x);
        assert!(moving.advance_ratio > 0.1, "advance ratio {}", moving.advance_ratio);
    }

    #[test]
    fn advance_ratio_is_edgewise_speed_over_tip_speed() {
        let r = lift_rotor();
        let omega = 800.0;
        let out = run(&r, omega, Vec3::new(15.0, 0.0, 0.0));
        assert!((out.advance_ratio - 15.0 / (omega * r.radius)).abs() < 1e-9);
    }

    /// Wake skew goes from axial to fully edgewise, and it is what Drees' inflow
    /// gradient keys on.
    #[test]
    fn wake_skew_grows_from_axial_to_edgewise() {
        let r = lift_rotor();
        let axial = run(&r, 800.0, Vec3::zero()).skew;
        let fast = run(&r, 800.0, Vec3::new(30.0, 0.0, 0.0)).skew;
        assert!(axial < 0.05, "hover skew {axial}");
        assert!(fast > 1.0 && fast < PI / 2.0 + 1e-9, "edgewise skew {fast}");
    }

    /// The loading asymmetry has to produce a hub moment, because that is the
    /// disturbance a tiltrotor's controller has to fly against in transition.
    #[test]
    fn edgewise_flow_produces_a_hub_moment() {
        let mut r = lift_rotor();
        r.position = Vec3::zero(); // isolate the hub moment from the arm.
        let hover = run(&r, 800.0, Vec3::zero());
        let moving = run(&r, 800.0, Vec3::new(20.0, 0.0, 0.0));
        let hub_hover = (hover.moment_bf.x.powi(2) + hover.moment_bf.y.powi(2)).sqrt();
        let hub_moving = (moving.moment_bf.x.powi(2) + moving.moment_bf.y.powi(2)).sqrt();
        assert!(hub_hover < 1e-6, "hover hub moment {hub_hover}");
        assert!(hub_moving > 1e-4, "edgewise hub moment {hub_moving}");
    }

    #[test]
    fn drees_gradient_vanishes_in_axial_flow_and_grows_with_skew() {
        let (kx0, ky0) = drees_inflow_gradient(0.0, 0.0);
        assert!(kx0.abs() < 1e-9 && ky0.abs() < 1e-9);
        let (kx, ky) = drees_inflow_gradient(1.2, 0.25);
        assert!(kx > 0.5, "kx {kx}");
        assert!(ky < 0.0, "ky {ky}");
    }

    /// A propeller unloads as the aircraft speeds up: the blades see a smaller
    /// angle of attack. This is why a fixed-pitch pusher has a top speed.
    #[test]
    fn a_pusher_prop_unloads_with_forward_speed() {
        let mut r = lift_rotor();
        r.axis = Vec3::new(1.0, 0.0, 0.0);
        r.position = Vec3::new(-0.5, 0.0, 0.0);
        let mut prev = f64::INFINITY;
        for v in [0.0, 5.0, 10.0, 20.0, 30.0] {
            // +x velocity with the axis along +x is axial climb for this rotor.
            let t = run(&r, 900.0, Vec3::new(v, 0.0, 0.0)).thrust;
            assert!(t < prev, "thrust rose at {v} m/s: {t} after {prev}");
            prev = t;
        }
    }

    // ─── Tilt ───────────────────────────────────────────────────────────────

    /// The tiltrotor case: rotating the axis 90 deg about the pitch axis turns a
    /// lift rotor into a pusher, with no other change.
    #[test]
    fn tilting_ninety_degrees_turns_lift_into_thrust() {
        let r = lift_rotor();
        let up = r.tilted_axis(0.0);
        assert!((up.z + 1.0).abs() < 1e-9, "untilted axis {up:?}");
        let fwd = r.tilted_axis(PI / 2.0);
        assert!((fwd.x - 1.0).abs() < 1e-6 && fwd.z.abs() < 1e-6, "tilted axis {fwd:?}");
        let af = blade_airfoil();
        let (v, g) = still();
        let hover = r.forces(RotorState { omega: 800.0, tilt: 0.0 }, v, g, 1.225, &af, Vec3::zero());
        let cruise = r.forces(RotorState { omega: 800.0, tilt: PI / 2.0 }, v, g, 1.225, &af, Vec3::zero());
        assert!(hover.force_bf.z < -1.0, "lift {}", hover.force_bf.z);
        assert!(cruise.force_bf.x > 1.0, "thrust {}", cruise.force_bf.x);
        // Same rotor, same speed: the same total force, pointed elsewhere.
        assert!((hover.thrust - cruise.thrust).abs() < 1e-6);
    }

    #[test]
    fn partial_tilt_splits_the_force() {
        let r = lift_rotor();
        let af = blade_airfoil();
        let out = r.forces(RotorState { omega: 800.0, tilt: PI / 4.0 }, Vec3::zero(), Vec3::zero(), 1.225, &af, Vec3::zero());
        assert!(out.force_bf.x > 0.0 && out.force_bf.z < 0.0, "force {:?}", out.force_bf);
        assert!((out.force_bf.x + out.force_bf.z).abs() < 1e-6, "45 deg should split evenly");
    }

    // ─── Torque, current and spool-up ───────────────────────────────────────

    /// Reaction torque opposes rotation. Two rotors spinning opposite ways must
    /// cancel, which is the entire basis of yaw control on a multirotor.
    #[test]
    fn counter_rotating_pairs_cancel_in_yaw() {
        let mut cw = lift_rotor();
        cw.position = Vec3::zero();
        let mut ccw = cw.clone();
        ccw.spin = -1.0;
        let a = run(&cw, 800.0, Vec3::zero());
        let b = run(&ccw, 800.0, Vec3::zero());
        assert!(a.moment_bf.z.abs() > 1e-4, "no reaction torque");
        assert!((a.moment_bf.z + b.moment_bf.z).abs() < 1e-9, "pair does not cancel");
    }

    #[test]
    fn current_rises_with_load() {
        let r = lift_rotor();
        let light = run(&r, 400.0, Vec3::zero()).current;
        let heavy = run(&r, 1000.0, Vec3::zero()).current;
        assert!(heavy > light && light > 0.0, "light {light} heavy {heavy}");
    }

    /// Spool-up takes real time, set by the rotor's own inertia. A model that
    /// snaps to the commanded speed cannot show why an aborted transition is
    /// survivable or not, which is a handling question a VTOL customer will ask.
    #[test]
    fn the_rotor_spools_up_over_time_not_instantly() {
        let r = lift_rotor();
        let target = r.no_load_omega(1.0, 22.2);
        assert!(target > 500.0, "no-load omega {target}");
        let mut s = RotorState { omega: 0.0, tilt: 0.0 };
        let dt = 0.0025;
        // One tick must not get anywhere near the target.
        s.omega = r.step_omega(s, 1.0, 22.2, 0.0, dt);
        assert!(s.omega < target * 0.5, "snapped to {} in one tick", s.omega);
        let mut ticks = 1;
        while s.omega < target * 0.9 && ticks < 4000 {
            s.omega = r.step_omega(s, 1.0, 22.2, 0.0, dt);
            ticks += 1;
        }
        assert!(ticks < 4000, "never spooled up");
        let secs = ticks as f64 * dt;
        assert!((0.005..2.0).contains(&secs), "spool-up took {secs} s");
    }

    #[test]
    fn aerodynamic_load_holds_the_rotor_below_its_no_load_speed() {
        let r = lift_rotor();
        let target = r.no_load_omega(1.0, 22.2);
        let mut s = RotorState { omega: target, tilt: 0.0 };
        let load = run(&r, target, Vec3::zero()).torque;
        assert!(load > 0.0);
        for _ in 0..400 {
            s.omega = r.step_omega(s, 1.0, 22.2, load, 0.0025);
        }
        assert!(s.omega < target, "loaded rotor held at no-load speed {}", s.omega);
    }

    #[test]
    fn zero_throttle_spools_down() {
        let r = lift_rotor();
        let mut s = RotorState { omega: 800.0, tilt: 0.0 };
        for _ in 0..200 {
            s.omega = r.step_omega(s, 0.0, 22.2, 0.05, 0.0025);
        }
        assert!(s.omega < 800.0, "did not slow: {}", s.omega);
        assert!(s.omega >= 0.0, "went negative: {}", s.omega);
    }

    // ─── Tip loss ───────────────────────────────────────────────────────────

    #[test]
    fn prandtl_tip_loss_falls_to_zero_at_the_tip() {
        assert_eq!(prandtl_tip_loss(1.0, 2, 0.1), 0.0);
        let mid = prandtl_tip_loss(0.5, 2, 0.1);
        let near_tip = prandtl_tip_loss(0.97, 2, 0.1);
        assert!(mid > 0.9, "mid-span loss {mid}");
        assert!(near_tip < mid, "tip {near_tip} not below mid {mid}");
        // More blades, less relative tip loss per blade.
        assert!(prandtl_tip_loss(0.9, 6, 0.1) > prandtl_tip_loss(0.9, 2, 0.1));
    }

    // ─── Robustness ─────────────────────────────────────────────────────────

    /// The FDM must not produce a non-finite force in any attitude, because a
    /// single NaN propagates into the state and ends the run. These are the
    /// corners a VTOL actually visits.
    #[test]
    fn every_flow_regime_is_finite() {
        let r = lift_rotor();
        let af = blade_airfoil();
        for v in [
            Vec3::zero(),
            Vec3::new(0.0, 0.0, -30.0), // hard climb
            Vec3::new(0.0, 0.0, 30.0),  // hard descent, through VRS
            Vec3::new(50.0, 0.0, 0.0),  // fully edgewise
            Vec3::new(-50.0, 0.0, 0.0), // backwards
            Vec3::new(30.0, 30.0, 30.0),
        ] {
            for tilt in [0.0, PI / 4.0, PI / 2.0, PI] {
                for omega in [0.0, 1.0, 800.0, 4000.0] {
                    let o = r.forces(RotorState { omega, tilt }, v, Vec3::new(3.0, -2.0, 1.0), 1.225, &af, Vec3::zero());
                    for c in [o.force_bf.x, o.force_bf.y, o.force_bf.z,
                              o.moment_bf.x, o.moment_bf.y, o.moment_bf.z,
                              o.thrust, o.torque, o.induced_velocity, o.current] {
                        assert!(c.is_finite(), "non-finite at v {v:?} tilt {tilt} omega {omega}");
                    }
                    assert!(o.torque >= 0.0, "negative torque at v {v:?}");
                }
            }
        }
    }

    /// Descending vertically must eventually be flagged as VRS. This is the
    /// single output a customer will look for when asking whether the sim knows
    /// about settling with power.
    #[test]
    fn a_vertical_descent_enters_the_vortex_ring_state() {
        let r = lift_rotor();
        let hover_vi = run(&r, 800.0, Vec3::zero()).induced_velocity;
        assert!(hover_vi > 1.0, "hover vi {hover_vi}");
        // Descend at about the hover induced velocity: the classic VRS entry.
        let out = run(&r, 800.0, Vec3::new(0.0, 0.0, hover_vi));
        assert!(out.in_vrs, "descent at vh should be VRS, vi {}", out.induced_velocity);
        // And moving forward at the same descent rate gets out of it.
        let escaped = run(&r, 800.0, Vec3::new(25.0, 0.0, hover_vi));
        assert!(!escaped.in_vrs, "forward flight should clear VRS");
    }

    #[test]
    fn slipstream_is_reported_for_downstream_surfaces() {
        let out = run(&lift_rotor(), 800.0, Vec3::zero());
        // Far-wake speed is twice the disc induced velocity in hover.
        assert!((out.slipstream - 2.0 * out.induced_velocity).abs() < 1e-9);
        assert!(out.slipstream > 2.0, "slipstream {}", out.slipstream);
    }

    #[test]
    fn kt_follows_from_kv() {
        let m = MotorDrive { kv: 400.0, resistance: 0.08, no_load_current: 0.7, voltage_max: 22.2 };
        assert!((m.kt() - 60.0 / (2.0 * PI * 400.0)).abs() < 1e-12);
        assert_eq!(MotorDrive { kv: 0.0, ..m }.kt(), 0.0);
    }
}
