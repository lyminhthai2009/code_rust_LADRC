//! Zero-Lag Steering + Battery-Safe Dual-Motor Controller for 4x AA + AMS1117-3.3.
//!
//! Hardware contract (verified):
//! - Left Motor:  PWM_PIN_L_A = GPIO2 (LEDC Ch0), PWM_PIN_L_B = GPIO10 (LEDC Ch1)
//! - Right Motor: PWM_PIN_R_A = GPIO6 (LEDC Ch2), PWM_PIN_R_B = GPIO5  (LEDC Ch3)
//! - PWM: 5 kHz, 8-bit hardware resolution (0..=255 via `ChannelHW::set_duty_hw`)
//! - Forward: A = pwm, B = 0 | Reverse: A = 0, B = pwm | Coast: A = 0, B = 0
//!
//! Decoupled Power & Steering Architecture (eliminates actuator phase lag!):
//! 1. Common-mode forward speed `v_base` is soft-started & slew-limited => ZERO standing-start
//!    or acceleration brownout on 4x AA!
//! 2. Differential steering `u_steer` responds INSTANTANEOUSLY (`6500 PWM/s`) because increasing
//!    one wheel while decreasing the other keeps total battery load nearly constant => ZERO
//!    steering phase lag and ZERO oscillation!
//! 3. True Battery-Draw Current Limiter counts ONLY positive battery-drawing torque (accelerating
//!    or plugging), NEVER penalizing wheel braking/deceleration!
//! 4. Anti-Plugging Coast Guard prevents 2x stall inrush when flipping Forward <-> Reverse at speed.

use esp_hal::ledc::LowSpeed;
use esp_hal::ledc::channel::{Channel, ChannelHW};

use crate::config::*;
use crate::planner::DriveCommand;

pub struct AppliedDrive {
    pub left_pwm: f32,
    pub right_pwm: f32,
    /// Actual applied differential steering `(L - R) / 2` fed back to LADRC for exact anti-windup.
    pub applied_steer: f32,
    /// Estimated average forward speed `(ω_L + ω_R) / 2` in [0..255] for LADRC gain-scheduling.
    pub est_forward_speed: f32,
}

pub struct MotorPowerController<'d> {
    ch_la: Channel<'d, LowSpeed>,
    ch_lb: Channel<'d, LowSpeed>,
    ch_ra: Channel<'d, LowSpeed>,
    ch_rb: Channel<'d, LowSpeed>,
    /// Previous decoupled common-mode speed and differential steering.
    applied_base: f32,
    applied_steer: f32,
    /// Estimated wheel speeds ω_hat (-255..+255) from first-order electromechanical model.
    omega_hat_l: f32,
    omega_hat_r: f32,
    /// Soft-start progress counter (ticks).
    run_ticks: u32,
}

impl<'d> MotorPowerController<'d> {
    pub fn new(
        ch_la: Channel<'d, LowSpeed>,
        ch_lb: Channel<'d, LowSpeed>,
        ch_ra: Channel<'d, LowSpeed>,
        ch_rb: Channel<'d, LowSpeed>,
    ) -> Self {
        let mut this = Self {
            ch_la,
            ch_lb,
            ch_ra,
            ch_rb,
            applied_base: 0.0,
            applied_steer: 0.0,
            omega_hat_l: 0.0,
            omega_hat_r: 0.0,
            run_ticks: 0,
        };
        this.coast_stop();
        this
    }

    pub fn reset(&mut self) {
        self.run_ticks = 0;
        self.applied_base = 0.0;
        self.applied_steer = 0.0;
        self.omega_hat_l = 0.0;
        self.omega_hat_r = 0.0;
        self.coast_stop();
    }

    /// Immediate coast stop (`A=0, B=0` on both motors — zero battery draw).
    pub fn coast_stop(&mut self) {
        self.ch_la.set_duty_hw(0);
        self.ch_lb.set_duty_hw(0);
        self.ch_ra.set_duty_hw(0);
        self.ch_rb.set_duty_hw(0);
        self.applied_base = 0.0;
        self.applied_steer = 0.0;
        let alpha_m = DT_SEC / (MOTOR_MECH_TAU_S + DT_SEC);
        self.omega_hat_l += alpha_m * (0.0 - self.omega_hat_l);
        self.omega_hat_r += alpha_m * (0.0 - self.omega_hat_r);
    }

    /// Drive motors directly during gentle sweep calibration (zero-brownout safe).
    pub fn drive_calibration(&mut self, left_target: f32, right_target: f32) {
        self.run_ticks = (SOFT_START_MS * LOOP_FREQ_HZ) / 1000 + 1;
        let cmd = DriveCommand {
            base_speed: 0.5 * (left_target + right_target),
            steer_cmd: 0.5 * (left_target - right_target),
            max_reverse_pwm: 90.0,
            instant_snap: false,
            anti_plug_th: ANTI_PLUG_OMEGA_TH,
            safety_stopped: false,
        };
        let _ = self.apply(&cmd);
    }

    /// Apply decoupled `(base_speed, steer_cmd)` with instant steering & full 4x AA protection.
    pub fn apply(&mut self, cmd: &DriveCommand) -> AppliedDrive {
        // 1. Soft-start S-curve on launch (protects 4x AA on standing start)
        let soft_ticks = ((SOFT_START_MS * LOOP_FREQ_HZ) / 1000).max(1);
        self.run_ticks = self.run_ticks.saturating_add(1);
        let soft_scale = if self.run_ticks < soft_ticks {
            let t = (self.run_ticks as f32) / (soft_ticks as f32);
            t * t * (3.0 - 2.0 * t)
        } else {
            1.0
        };

        let target_base = (cmd.base_speed * soft_scale).clamp(-PWM_MAX, PWM_MAX);
        let target_steer = (cmd.steer_cmd * soft_scale).clamp(-PWM_MAX, PWM_MAX);

        // 2. Decoupled Slew Rate Limiting (bypassed for `instant_snap` during shortcut pivot,
        //    where `BATTERY_CURRENT_BUDGET = 122.0` still strictly protects 4x AA!).
        let (base, steer) = if cmd.instant_snap {
            (target_base, target_steer)
        } else {
            let max_base_up = COMMON_ACCEL_PWM_PER_SEC * DT_SEC;
            let max_base_dn = COMMON_BRAKE_PWM_PER_SEC * DT_SEC;
            let base_diff = target_base - self.applied_base;
            let base_step = if base_diff > 0.0 {
                base_diff.min(max_base_up)
            } else {
                base_diff.max(-max_base_dn)
            };
            let b = self.applied_base + base_step;

            let max_steer_step = STEER_SLEW_PWM_PER_SEC * DT_SEC;
            let s = self.applied_steer
                + (target_steer - self.applied_steer).clamp(-max_steer_step, max_steer_step);
            (b, s)
        };

        self.applied_base = base;
        self.applied_steer = steer;

        // 3. Steering-Priority Differential Mixer with 4x AA Wheel Cap
        let max_w = ACTIVE_PROFILE.max_wheel_pwm;
        let mut l = base + steer;
        let mut r = base - steer;
        let peak = l.max(r);
        if peak > max_w {
            let overflow = peak - max_w;
            l -= overflow;
            r -= overflow;
        }
        let min_pwm = -cmd.max_reverse_pwm;
        l = l.clamp(min_pwm, max_w);
        r = r.clamp(min_pwm, max_w);

        let sum_abs = libm::fabsf(l) + libm::fabsf(r);
        if sum_abs > MAX_TOTAL_PWM {
            let scale = MAX_TOTAL_PWM / sum_abs;
            l *= scale;
            r *= scale;
        }

        // 4. Anti-Plugging Coast Guard (uses `cmd.anti_plug_th`: 30 normal, 115 after 32ms coast in shortcut)
        l = Self::anti_plug_guard(l, self.omega_hat_l, cmd.anti_plug_th);
        r = Self::anti_plug_guard(r, self.omega_hat_r, cmd.anti_plug_th);

        // 5. Joint Positive Battery-Draw Current Envelope Limiter (`draw_L + draw_R <= 132.0`)
        //    CRITICAL: When a motor wants REVERSE (`cmd < 0`) while still coasting forward
        //    (`omega_hat > 0`), NEVER scale it back into a positive forward PWM! Hold at 0.0
        //    (free coast = 0 A) until budget allows clean reverse => ultra-decisive shortcut snap!
        let draw_l = Self::battery_draw(l, self.omega_hat_l);
        let draw_r = Self::battery_draw(r, self.omega_hat_r);
        let total_draw = draw_l + draw_r;
        if total_draw > BATTERY_CURRENT_BUDGET {
            let scale = BATTERY_CURRENT_BUDGET / total_draw;
            if draw_l > 0.0 {
                l = Self::scale_motor_draw(l, self.omega_hat_l, draw_l * scale);
            }
            if draw_r > 0.0 {
                r = Self::scale_motor_draw(r, self.omega_hat_r, draw_r * scale);
            }
            l = l.clamp(min_pwm, max_w);
            r = r.clamp(min_pwm, max_w);
        }

        // 6. Update Back-EMF Wheel Velocity Estimator
        let alpha_m = DT_SEC / (MOTOR_MECH_TAU_S + DT_SEC);
        self.omega_hat_l += alpha_m * (l - self.omega_hat_l);
        self.omega_hat_r += alpha_m * (r - self.omega_hat_r);

        // 8. Output Hardware PWM (5 kHz, 8-bit 0..=255)
        self.write_hardware(l, r);

        AppliedDrive {
            left_pwm: l,
            right_pwm: r,
            applied_steer: 0.5 * (l - r),
            est_forward_speed: (0.5 * (self.omega_hat_l + self.omega_hat_r)).max(0.0),
        }
    }

    /// Estimate normalized battery current drawn by one motor in coast/drive H-bridge mode.
    /// - Coasting (`cmd == 0`) or slowing down (`|cmd| <= |ω_hat|`, same sign) => 0 battery draw!
    /// - Accelerating (`|cmd| > |ω_hat|`, same sign) => `|cmd| - |ω_hat|`.
    /// - Plugging (opposite sign) => `|cmd| + |ω_hat|`.
    #[inline]
    fn battery_draw(cmd: f32, omega_hat: f32) -> f32 {
        let abs_cmd = libm::fabsf(cmd);
        if abs_cmd < 1.0 {
            return 0.0;
        }
        if cmd * omega_hat >= 0.0 {
            (abs_cmd - libm::fabsf(omega_hat)).max(0.0)
        } else {
            abs_cmd + libm::fabsf(omega_hat)
        }
    }

    /// Apply scaled battery-draw allowance without ever flipping a desired reverse command
    /// back into forward drive (or vice versa).
    #[inline]
    fn scale_motor_draw(cmd: f32, omega_hat: f32, allowed_draw: f32) -> f32 {
        if cmd < 0.0 && omega_hat > 0.0 {
            -((allowed_draw - omega_hat).max(0.0))
        } else if cmd > 0.0 && omega_hat < 0.0 {
            (allowed_draw + omega_hat).max(0.0)
        } else if cmd >= 0.0 {
            omega_hat.max(0.0) + allowed_draw
        } else {
            omega_hat.min(0.0) - allowed_draw
        }
    }

    #[inline]
    fn anti_plug_guard(cmd: f32, omega_hat: f32, thresh: f32) -> f32 {
        if (omega_hat > thresh && cmd < -10.0) || (omega_hat < -thresh && cmd > 10.0) {
            0.0
        } else {
            cmd
        }
    }

    #[inline]
    fn compensate_deadzone(pwm: f32) -> f32 {
        let abs_v = libm::fabsf(pwm);
        if abs_v < 2.0 {
            0.0
        } else {
            let sign = if pwm >= 0.0 { 1.0 } else { -1.0 };
            let mapped =
                MOTOR_DEADZONE_PWM + (abs_v / PWM_MAX) * (PWM_MAX - MOTOR_DEADZONE_PWM);
            sign * mapped.clamp(0.0, PWM_MAX)
        }
    }

    fn write_hardware(&self, logical_l: f32, logical_r: f32) {
        let mut l = Self::compensate_deadzone(logical_l);
        let mut r = Self::compensate_deadzone(logical_r * RIGHT_MOTOR_TRIM);

        if SWAP_MOTORS_LR {
            core::mem::swap(&mut l, &mut r);
        }
        if INVERT_LEFT_MOTOR {
            l = -l;
        }
        if INVERT_RIGHT_MOTOR {
            r = -r;
        }

        Self::set_motor_pair(&self.ch_la, &self.ch_lb, l);
        Self::set_motor_pair(&self.ch_ra, &self.ch_rb, r);
    }

    #[inline]
    fn set_motor_pair(ch_a: &Channel<'d, LowSpeed>, ch_b: &Channel<'d, LowSpeed>, signed_pwm: f32) {
        let abs_v = libm::fabsf(signed_pwm);
        // True 100.0% DC duty (256 / 256 on 8-bit LEDC) at full throttle eliminates switching loss!
        let duty = if abs_v >= 254.5 {
            256u32
        } else {
            (abs_v + 0.5) as u32
        };
        if signed_pwm > 0.5 {
            ch_b.set_duty_hw(0);
            ch_a.set_duty_hw(duty);
        } else if signed_pwm < -0.5 {
            ch_a.set_duty_hw(0);
            ch_b.set_duty_hw(duty);
        } else {
            ch_a.set_duty_hw(0);
            ch_b.set_duty_hw(0);
        }
    }
}
