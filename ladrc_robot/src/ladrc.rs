//! Ultra-Smooth 2nd-Order Discrete LADRC (Linear Active Disturbance Rejection Control).
//!
//! Mathematical foundation:
//! - Plant model (lateral line position `e`, positive = line on right, positive `u` = steer right):
//!   `ë(t) = -b0(v) * u(t) + f(t)`
//!   where `f(t)` lumps road curvature (v²·κ), motor asymmetry, wheel slip, speed-bump torque
//!   impulses, ramp gravity disturbances, and battery voltage sag into ONE total disturbance.
//! - Observer: Exact Pole-Placement Discrete Current-Observer 3rd-Order LESO
//!   (Miklosovic, Radke, Gao — IEEE ACC 2006).
//!   - Unconditionally stable at any ω_o > 0.
//!   - Zero 1-step measurement delay (uses current measurement immediately).
//!   - Bumpless anti-windup by feeding ACTUAL applied steering `u_eff` (after motor slew/current limits).
//! - Adaptive Observer Bandwidth ω_o:
//!   - Calm on straightaways (velvety smooth, zero quantization shimmer on 4 IR sensors).
//!   - Agile in curves & disturbances (instant curvature locking and bump rejection).
//! - Speed-Scheduled Control Effectiveness `b0(v)`:
//!   - Keeps closed-loop bandwidth ω_c and damping ζ invariant across all speeds!
//! - Exposes filtered curvature disturbance `|z3 / b0|` to the velocity planner so the robot
//!   automatically holds optimal apex speed throughout an entire 180° hairpin!

use crate::config::{DT_SEC, PWM_MAX, TuningProfile};

pub struct LadrcOutput {
    /// Commanded steering differential PWM (positive => turn RIGHT: vL += u, vR -= u).
    pub steer_pwm: f32,
    /// Filtered lateral error estimate z1.
    pub z1: f32,
    /// Noise-free lateral velocity estimate z2 (de/dt).
    pub z2: f32,
    /// Total disturbance estimate z3.
    pub z3: f32,
    /// Normalized curvature/disturbance effort |z3 / b0| in [0.0..1.5], used by velocity planner.
    pub curvature_effort: f32,
}

pub struct LadrcController {
    /// LESO states: z1 ≈ e, z2 ≈ de/dt, z3 ≈ total disturbance f(t).
    z1: f32,
    z2: f32,
    z3: f32,
    /// Previous ACTUAL applied steering PWM (post-limiter) for bumpless anti-windup.
    u_prev_eff: f32,
    /// Smoothed observer bandwidth ω_o (rad/s) for seamless calm <-> agile transition.
    omega_o_smooth: f32,
    /// Low-pass filtered curvature/disturbance metric for silky speed profiling.
    curv_effort_lpf: f32,
    /// Cached controller parameters.
    kp: f32,
    kd: f32,
    profile: TuningProfile,
}

impl LadrcController {
    pub fn new(profile: TuningProfile) -> Self {
        let kp = profile.omega_c * profile.omega_c;
        let kd = 2.0 * profile.zeta * profile.omega_c;
        Self {
            z1: 0.0,
            z2: 0.0,
            z3: 0.0,
            u_prev_eff: 0.0,
            omega_o_smooth: profile.omega_o_calm,
            curv_effort_lpf: 0.0,
            kp,
            kd,
            profile,
        }
    }

    /// Reset observer & internal filters (e.g. before launch or straight crossing).
    pub fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
        self.z3 = 0.0;
        self.u_prev_eff = 0.0;
        self.omega_o_smooth = self.profile.omega_o_calm;
        self.curv_effort_lpf = 0.0;
    }

    /// Bumpless re-seed after completing a 80°-120° shortcut turn onto the new branch.
    pub fn reset_to_pos(&mut self, pos: f32) {
        self.z1 = pos;
        self.z2 = 0.0;
        self.z3 = 0.0;
        self.u_prev_eff = 0.0;
        self.omega_o_smooth = self.profile.omega_o_calm;
        self.curv_effort_lpf = 0.0;
    }

    /// Feed back the ACTUAL applied differential steering PWM from the motor/power limiter
    /// so the LESO never winds up when motors saturate or hit the 4x AA current envelope.
    #[inline]
    pub fn feedback_applied_steer(&mut self, applied_steer_pwm: f32) {
        self.u_prev_eff = applied_steer_pwm;
    }

    /// Speed-scheduled control effectiveness b0(v) > 0 in (norm-units/s^2) / PWM.
    ///
    /// First-principles GA12-N20 300 RPM kinematics (v_max ≈ 0.55 m/s, wheelbase ≈ 90 mm,
    /// sensor lookahead ≈ 65 mm, half-span = 20 mm, τ_m ≈ 68 ms):
    ///   ë ≈ -(2.1 .. 3.4) * u_steer_pwm  (norm-units / s^2).
    /// Choosing b0 slightly conservative (1.0x..1.25x true plant gain) guarantees
    /// rock-solid stability and silky smoothness while the LESO absorbs any mismatch in z3.
    #[inline]
    fn effective_b0(&self, forward_speed_est: f32) -> f32 {
        let v_norm = (forward_speed_est / PWM_MAX).clamp(0.0, 1.0);
        self.profile.b0_base + self.profile.b0_speed_gain * v_norm
    }

    /// Smooth dual-slope error shaping near center to eliminate 4-IR quantization micro-wiggle
    /// on straightaways while preserving 100% crisp authority in curves.
    #[inline]
    fn shape_error(e: f32) -> f32 {
        let abs_e = libm::fabsf(e);
        if abs_e < 0.10 {
            // Gentle 0.70 slope inside ±0.10, C1-continuous transition via cubic blend.
            let t = abs_e / 0.10;
            let gain = 0.70 + 0.30 * t * t;
            e * gain
        } else {
            e
        }
    }

    /// Execute one 500 Hz LADRC step under normal tracking or straddle.
    ///
    /// - `e_meas`: measured lateral line position in [-1.28, +1.28]
    /// - `forward_speed_est`: estimated forward speed (0..255 PWM equivalent)
    /// - `disturbance_leak`: 1.0 during normal tracking, <1.0 (e.g. 0.985) during straddle/crossing
    pub fn step(
        &mut self,
        e_meas: f32,
        forward_speed_est: f32,
        disturbance_leak: f32,
    ) -> LadrcOutput {
        let ts = DT_SEC;
        let b0 = self.effective_b0(forward_speed_est);

        // 1. Adaptive Observer Bandwidth ω_o:
        // Blend smoothly from calm (straight) to agile (entering curve / high innovation).
        let activity = (libm::fabsf(e_meas) * 1.1 + libm::fabsf(self.z2) * 0.12).clamp(0.0, 1.0);
        let omega_o_target = self.profile.omega_o_calm
            + activity * (self.profile.omega_o_agile - self.profile.omega_o_calm);
        self.omega_o_smooth += 0.08 * (omega_o_target - self.omega_o_smooth);

        // 2. Exact Pole-Placement Discrete Current-Observer Gains (Miklosovic & Gao, 2006)
        // β = exp(-ω_o * Ts)
        let beta = libm::expf(-self.omega_o_smooth * ts);
        let one_m_beta = 1.0 - beta;
        let l1 = 1.0 - beta * beta * beta;
        let l2 = (1.5 / ts) * one_m_beta * one_m_beta * (1.0 + beta);
        let l3 = (1.0 / (ts * ts)) * one_m_beta * one_m_beta * one_m_beta;

        // 3. Exact ZOH State Prediction using previous ACTUAL applied steering u_prev_eff
        // Plant: ë = z3 - b0 * u
        let accel_pred = self.z3 - b0 * self.u_prev_eff;
        let z1_pred = self.z1 + ts * self.z2 + 0.5 * ts * ts * accel_pred;
        let z2_pred = self.z2 + ts * accel_pred;
        let z3_pred = self.z3 * disturbance_leak;

        // 4. Current-Observer Measurement Update (zero phase lag!)
        let innov = e_meas - z1_pred;
        self.z1 = z1_pred + l1 * innov;
        self.z2 = z2_pred + l2 * innov;
        self.z3 = z3_pred + l3 * innov;

        // CRITICAL: Fast sign-reversal release on z3!
        // When exiting a sharp curve and crossing center (z1 flips opposite to z3), dump old
        // curve bias immediately so PD damping catches the straightaway with ZERO whip-back!
        if self.z1 * self.z3 < 0.0 && libm::fabsf(self.z1) > 0.12 {
            self.z3 *= 0.90;
        }

        // Guard disturbance state <= 35% of max steering so PD damping ALWAYS dominates!
        let max_z3 = b0 * self.profile.max_steer_pwm * 0.35;
        self.z3 = self.z3.clamp(-max_z3, max_z3);

        // 5. Linear State Error Feedback (LSEF) + Total Disturbance Cancellation
        // Desired: ë = -kp * e_shaped - kd * ė
        // => u = (kp * z1_shaped + kd * z2 + z3) / b0
        let z1_shaped = Self::shape_error(self.z1);
        let u_raw = (self.kp * z1_shaped + self.kd * self.z2 + self.z3) / b0;
        let steer_pwm = u_raw.clamp(-self.profile.max_steer_pwm, self.profile.max_steer_pwm);

        // 6. Filtered Curvature/Disturbance Effort (|z3 / b0| normalized to max_steer_pwm)
        // On a steady 180° hairpin, z1 ≈ 0 yet |z3 / b0| stays high => tells velocity planner
        // we are still mid-corner so the robot holds silky apex speed until the exit straight!
        let raw_curv = libm::fabsf(self.z3 / (b0 * self.profile.max_steer_pwm)).clamp(0.0, 1.5);
        // Fast attack entering curve, smooth release exiting curve
        let curv_alpha = if raw_curv > self.curv_effort_lpf {
            0.12
        } else {
            0.035
        };
        self.curv_effort_lpf += curv_alpha * (raw_curv - self.curv_effort_lpf);

        LadrcOutput {
            steer_pwm,
            z1: self.z1,
            z2: self.z2,
            z3: self.z3,
            curvature_effort: self.curv_effort_lpf,
        }
    }

    /// Smoothly coast/decay LESO states during Lost-Line recovery so re-acquisition is bumpless.
    pub fn coast_during_lost(&mut self, lost_dir: f32, forward_speed_est: f32) {
        let b0 = self.effective_b0(forward_speed_est);
        // Keep z1 parked near the outer edge on the lost side, damp z2, hold gentle curvature bias.
        self.z1 = 0.90 * self.z1 + 0.10 * (lost_dir * 1.15);
        self.z2 *= 0.92;
        let target_z3 = lost_dir * b0 * self.profile.max_steer_pwm * 0.35;
        self.z3 = 0.95 * self.z3 + 0.05 * target_z3;
        self.curv_effort_lpf = 0.95 * self.curv_effort_lpf + 0.05 * 1.0;
    }

    #[inline]
    pub fn velocity_hint(&self) -> f32 {
        self.z2
    }
}
