//! Curvature-Adaptive Velocity Planner & 80°-120° X-Crossing Shortcut State Machine.
//!
//! Handles:
//! 1. High-speed 255 LADRC line following + curvature-adaptive cornering.
//! 2. Arrow-straight crossing lock (`StraightLock`) across forbidden-shortcut X-intersections
//!    (Vòng tròn 1 entry & exit), immune to 80°-120° oblique entry pull!
//! 3. Decisive, angle-agnostic (75°-125°), zero-brownout shortcut turns (`ShortcutRight` /
//!    `ShortcutLeft`) using 3-phase sensor-guided zero-overshoot pivot (`LeaveOldLine` ->
//!    `AcquireBranch` -> bumpless LADRC catch -> rocket back to 255!).
//! 4. Automatic crossing counter reset whenever robot is lifted (`LineState::Lifted`).

use crate::config::*;
use crate::ladrc::LadrcController;
use crate::sensor::LineState;

pub struct DriveCommand {
    /// Target common-mode forward speed (PWM units, -255..255).
    pub base_speed: f32,
    /// Target differential steering (PWM units, positive => turn RIGHT).
    pub steer_cmd: f32,
    /// Max allowed reverse PWM on inner wheel (positive magnitude).
    pub max_reverse_pwm: f32,
    /// Bypass base/steer slew rate for instant decisive shortcut snap (still protected by 122 AA budget!).
    pub instant_snap: bool,
    /// Anti-plugging threshold for reverse engagement (30 normal, 115 in shortcut Phase 2 after coast).
    pub anti_plug_th: f32,
    /// Safety stop (robot lifted off track or lost for > 2.0 s).
    pub safety_stopped: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CrossingMode {
    None,
    /// Lock arrow-straight across a non-shortcut X-crossing.
    StraightLock { ticks: u32 },
    /// Phase 1: Snap-turn until the 3 non-turning sensors see a clean white gap >= 6 ms
    /// (100% physically clear of old straight line => IMPOSSIBLE to abort back straight!).
    ShortcutLeave {
        dir_sign: i8,
        ticks: u32,
        white_ticks: u32,
    },
    /// Phase 2: High-speed snap-pivot until outer sensor S4 (or S3/S2) hits the new branch!
    ShortcutAcquire { dir_sign: i8, ticks: u32 },
    /// Phase 3: Crisp Counter-Brake (Outer = 0 COAST, Inner = +150) for 20-60 ms to kill
    /// rotational momentum DEAD-CENTER on the new branch with ZERO overshoot!
    ShortcutBrake { dir_sign: i8, ticks: u32 },
}

pub struct MotionPlanner {
    profile: TuningProfile,
    /// Smoothed base forward speed (PWM units).
    base_speed_lpf: f32,
    /// Smoothed recovery ramp (0..1).
    recovery_ramp: f32,
    /// Number of confirmed X-crossings encountered in the current run (0-based index into CROSSING_PLAN).
    cross_count: usize,
    /// Active crossing maneuver state machine.
    cross_mode: CrossingMode,
}

impl MotionPlanner {
    pub fn new(profile: TuningProfile) -> Self {
        Self {
            profile,
            base_speed_lpf: 0.0,
            recovery_ramp: 0.0,
            cross_count: 0,
            cross_mode: CrossingMode::None,
        }
    }

    pub fn reset(&mut self) {
        self.base_speed_lpf = 0.0;
        self.recovery_ramp = 0.0;
        self.cross_count = 0;
        self.cross_mode = CrossingMode::None;
    }

    pub fn cross_count(&self) -> usize {
        self.cross_count
    }

    pub fn is_in_shortcut(&self) -> bool {
        matches!(
            self.cross_mode,
            CrossingMode::ShortcutLeave { .. }
                | CrossingMode::ShortcutAcquire { .. }
                | CrossingMode::ShortcutBrake { .. }
        )
    }

    /// Compute optimal `(base_speed, steer_cmd)` for the current 500 Hz tick.
    pub fn compute(
        &mut self,
        line_state: LineState,
        sensor_weights: [f32; 4],
        ladrc: &mut LadrcController,
        est_forward_speed: f32,
    ) -> DriveCommand {
        let tick_ms = 1000 / LOOP_FREQ_HZ;

        // 0. Lifted-in-air ALWAYS overrides everything: stop safely & reset crossing counter
        //    so placing the robot back on START begins cleanly from Crossing #1!
        if matches!(line_state, LineState::Lifted) {
            self.reset();
            ladrc.reset();
            return DriveCommand {
                base_speed: 0.0,
                steer_cmd: 0.0,
                max_reverse_pwm: 0.0,
                instant_snap: false,
                anti_plug_th: ANTI_PLUG_OMEGA_TH,
                safety_stopped: true,
            };
        }

        // 1. Trigger a new crossing maneuver ONLY when idle (`CrossingMode::None`)
        if self.cross_mode == CrossingMode::None
            && let LineState::Crossing {
                is_new_event: true, ..
            } = line_state
        {
            let idx = self.cross_count;
            self.cross_count = self.cross_count.saturating_add(1);
            let action = if idx < CROSSING_PLAN.len() {
                CROSSING_PLAN[idx]
            } else {
                CrossingAction::Straight
            };

            ladrc.reset();
            self.recovery_ramp = 0.0;
            self.cross_mode = match action {
                CrossingAction::Straight => CrossingMode::StraightLock { ticks: 0 },
                CrossingAction::ShortcutRight => CrossingMode::ShortcutLeave {
                    dir_sign: 1,
                    ticks: 0,
                    white_ticks: 0,
                },
                CrossingAction::ShortcutLeft => CrossingMode::ShortcutLeave {
                    dir_sign: -1,
                    ticks: 0,
                    white_ticks: 0,
                },
            };
        }

        // 2. Execute Active Crossing Maneuver (StraightLock / ShortcutLeave / ShortcutAcquire)
        match self.cross_mode {
            CrossingMode::StraightLock { ticks } => {
                let next_ticks = ticks + 1;
                let elapsed = next_ticks * tick_ms;
                let wings_clear = sensor_weights[0] < 0.25 && sensor_weights[3] < 0.25;
                let still_crossing = matches!(line_state, LineState::Crossing { .. });

                if (elapsed >= 44 && !still_crossing && wings_clear) || elapsed >= MAX_CROSS_LOCK_MS {
                    self.cross_mode = CrossingMode::None;
                } else {
                    self.cross_mode = CrossingMode::StraightLock { ticks: next_ticks };
                    let target_base = self
                        .profile
                        .v_crossing
                        .min(self.base_speed_lpf.max(self.profile.v_curve_min));
                    self.base_speed_lpf += 0.10 * (target_base - self.base_speed_lpf);
                    return DriveCommand {
                        base_speed: self.base_speed_lpf,
                        steer_cmd: 0.0,
                        max_reverse_pwm: 0.0,
                        instant_snap: false,
                        anti_plug_th: ANTI_PLUG_OMEGA_TH,
                        safety_stopped: false,
                    };
                }
            }

            CrossingMode::ShortcutLeave {
                dir_sign,
                ticks,
                white_ticks,
            } => {
                let next_ticks = ticks + 1;
                let elapsed = next_ticks * tick_ms;

                // Require the 3 non-turning sensors to clear the old straight line into the white gap!
                // Right turn: S1, S2, S3 all < 0.20 (even if S4 already sees the incoming 80° branch!).
                // Left turn:  S2, S3, S4 all < 0.20.
                let old_line_cleared = if dir_sign > 0 {
                    sensor_weights[0] < 0.20 && sensor_weights[1] < 0.20 && sensor_weights[2] < 0.20
                } else {
                    sensor_weights[1] < 0.20 && sensor_weights[2] < 0.20 && sensor_weights[3] < 0.20
                };

                let next_white = if elapsed >= SHORTCUT_LEAVE_MIN_MS && old_line_cleared {
                    white_ticks + 1
                } else {
                    0
                };

                if next_white >= 3 || elapsed >= SHORTCUT_LEAVE_MAX_MS {
                    self.cross_mode = CrossingMode::ShortcutAcquire { dir_sign, ticks: 0 };
                } else {
                    self.cross_mode = CrossingMode::ShortcutLeave {
                        dir_sign,
                        ticks: next_ticks,
                        white_ticks: next_white,
                    };
                }

                // Phase 1 (0..32 ms): Instant strong snap L=205, R=0 (0 A inrush!)
                // Phase 2 (>32 ms):   Hard on-spot pivot L=185, R=-145 (Δ=330 PWM, bounded by 132 AA budget!)
                let (outer, inner_rev, plug_th) = if elapsed < SHORTCUT_BRAKE_MS {
                    (SHORTCUT_BRAKE_OUTER_PWM, 0.0, ANTI_PLUG_OMEGA_TH)
                } else {
                    (
                        SHORTCUT_OUTER_PWM,
                        SHORTCUT_INNER_REV_PWM,
                        SHORTCUT_ANTI_PLUG_TH,
                    )
                };
                let base = 0.5 * (outer - inner_rev);
                let steer = (dir_sign as f32) * 0.5 * (outer + inner_rev);
                self.base_speed_lpf = base;
                return DriveCommand {
                    base_speed: base,
                    steer_cmd: steer,
                    max_reverse_pwm: inner_rev,
                    instant_snap: true,
                    anti_plug_th: plug_th,
                    safety_stopped: false,
                };
            }

            CrossingMode::ShortcutAcquire { dir_sign, ticks } => {
                let next_ticks = ticks + 1;
                let elapsed = next_ticks * tick_ms;

                let outer_w = if dir_sign > 0 {
                    sensor_weights[3]
                } else {
                    sensor_weights[0]
                };
                // Trigger Counter-Brake the instant outer sensor S4 (or S3/S2) touches the new branch!
                let branch_hit = outer_w >= 0.42
                    || sensor_weights[2] >= 0.35
                    || sensor_weights[1] >= 0.35;

                if branch_hit || elapsed >= SHORTCUT_ACQUIRE_MAX_MS {
                    self.cross_mode = CrossingMode::ShortcutBrake { dir_sign, ticks: 0 };
                    // Immediately start Counter-Brake on this exact tick: Outer = 0 (COAST), Inner = +150!
                    let base = 0.5 * SHORTCUT_COUNTER_BRAKE_PWM;
                    let steer = -(dir_sign as f32) * 0.5 * SHORTCUT_COUNTER_BRAKE_PWM;
                    self.base_speed_lpf = base;
                    return DriveCommand {
                        base_speed: base,
                        steer_cmd: steer,
                        max_reverse_pwm: 0.0,
                        instant_snap: true,
                        anti_plug_th: SHORTCUT_ANTI_PLUG_TH,
                        safety_stopped: false,
                    };
                } else {
                    self.cross_mode = CrossingMode::ShortcutAcquire {
                        dir_sign,
                        ticks: next_ticks,
                    };
                    let base = 0.5 * (SHORTCUT_OUTER_PWM - SHORTCUT_INNER_REV_PWM);
                    let steer =
                        (dir_sign as f32) * 0.5 * (SHORTCUT_OUTER_PWM + SHORTCUT_INNER_REV_PWM);
                    self.base_speed_lpf = base;
                    return DriveCommand {
                        base_speed: base,
                        steer_cmd: steer,
                        max_reverse_pwm: SHORTCUT_INNER_REV_PWM,
                        instant_snap: true,
                        anti_plug_th: SHORTCUT_ANTI_PLUG_TH,
                        safety_stopped: false,
                    };
                }
            }

            CrossingMode::ShortcutBrake { dir_sign, ticks } => {
                let next_ticks = ticks + 1;
                let elapsed = next_ticks * tick_ms;

                // Release counter-brake into full-speed LADRC once centered on S2/S3 (or max 60 ms)!
                let centered = if dir_sign > 0 {
                    (sensor_weights[1] >= 0.30 && sensor_weights[2] >= 0.30)
                        || sensor_weights[1] >= 0.45
                } else {
                    (sensor_weights[1] >= 0.30 && sensor_weights[2] >= 0.30)
                        || sensor_weights[2] >= 0.45
                };

                if (elapsed >= SHORTCUT_COUNTER_BRAKE_MIN_MS && centered)
                    || elapsed >= SHORTCUT_COUNTER_BRAKE_MAX_MS
                {
                    self.cross_mode = CrossingMode::None;
                    let sum_w = sensor_weights[0]
                        + sensor_weights[1]
                        + sensor_weights[2]
                        + sensor_weights[3];
                    let pos = if sum_w >= LINE_PRESENT_SUM {
                        let wsum = sensor_weights[0] * SENSOR_POS_NORM[0]
                            + sensor_weights[1] * SENSOR_POS_NORM[1]
                            + sensor_weights[2] * SENSOR_POS_NORM[2]
                            + sensor_weights[3] * SENSOR_POS_NORM[3];
                        wsum / sum_w.max(SENSOR_DENOM_FLOOR)
                    } else {
                        0.0
                    };
                    ladrc.reset_to_pos(pos);
                    self.base_speed_lpf = 185.0;
                } else {
                    self.cross_mode = CrossingMode::ShortcutBrake {
                        dir_sign,
                        ticks: next_ticks,
                    };
                    // Counter-Brake: Outer = 0 (COAST => 0 A!), Inner = +150 => stops spin DEAD-CENTER!
                    let base = 0.5 * SHORTCUT_COUNTER_BRAKE_PWM;
                    let steer = -(dir_sign as f32) * 0.5 * SHORTCUT_COUNTER_BRAKE_PWM;
                    self.base_speed_lpf = base;
                    return DriveCommand {
                        base_speed: base,
                        steer_cmd: steer,
                        max_reverse_pwm: 0.0,
                        instant_snap: true,
                        anti_plug_th: SHORTCUT_ANTI_PLUG_TH,
                        safety_stopped: false,
                    };
                }
            }

            CrossingMode::None => {}
        }

        // 3. Normal High-Speed 255 LADRC Line Tracking & Recovery
        match line_state {
            LineState::Tracking { pos, strength: _ } => {
                self.recovery_ramp = 0.0;
                let out = ladrc.step(pos, est_forward_speed, 1.0);

                // Wide 0.35 deadband so straights & gentle/medium sweeps stay LOCKED at 255/255!
                let err_term = libm::fabsf(out.z1) * self.profile.slow_k_err;
                let vel_term = libm::fabsf(out.z2) * self.profile.slow_k_vel;
                let curv_term = out.curvature_effort * self.profile.slow_k_curv;
                let slow_raw = err_term + vel_term + curv_term;
                let slow_factor = ((slow_raw - 0.35) / 0.65).clamp(0.0, 1.0);

                let target_base = self.profile.v_max
                    - (self.profile.v_max - self.profile.v_curve_min) * slow_factor;

                let alpha = if target_base < self.base_speed_lpf {
                    0.20
                } else {
                    0.075
                };
                self.base_speed_lpf += alpha * (target_base - self.base_speed_lpf);

                DriveCommand {
                    base_speed: self.base_speed_lpf,
                    steer_cmd: out.steer_pwm,
                    max_reverse_pwm: self.profile.max_reverse_tracking,
                    instant_snap: false,
                    anti_plug_th: ANTI_PLUG_OMEGA_TH,
                    safety_stopped: false,
                }
            }

            LineState::Crossing { .. } => {
                self.recovery_ramp = 0.0;
                let out = ladrc.step(0.0, est_forward_speed, 0.95);
                let target_base = self
                    .profile
                    .v_crossing
                    .min(self.base_speed_lpf.max(self.profile.v_curve_min));
                self.base_speed_lpf += 0.08 * (target_base - self.base_speed_lpf);

                DriveCommand {
                    base_speed: self.base_speed_lpf,
                    steer_cmd: (out.steer_pwm * 0.15).clamp(-25.0, 25.0),
                    max_reverse_pwm: 0.0,
                    instant_snap: false,
                    anti_plug_th: ANTI_PLUG_OMEGA_TH,
                    safety_stopped: false,
                }
            }

            LineState::CenterStraddle {
                pos_hold,
                elapsed_ms: _,
            } => {
                self.recovery_ramp = 0.0;
                let out = ladrc.step(pos_hold, est_forward_speed, 0.98);

                let target_base = self.profile.v_straddle;
                self.base_speed_lpf += 0.06 * (target_base - self.base_speed_lpf);

                DriveCommand {
                    base_speed: self.base_speed_lpf,
                    steer_cmd: out.steer_pwm.clamp(-70.0, 70.0),
                    max_reverse_pwm: 0.0,
                    instant_snap: false,
                    anti_plug_th: ANTI_PLUG_OMEGA_TH,
                    safety_stopped: false,
                }
            }

            LineState::Lost { dir, elapsed_ms } => {
                if elapsed_ms >= LOST_SAFETY_STOP_MS {
                    self.base_speed_lpf = 0.0;
                    self.recovery_ramp = 0.0;
                    return DriveCommand {
                        base_speed: 0.0,
                        steer_cmd: 0.0,
                        max_reverse_pwm: 0.0,
                        instant_snap: false,
                        anti_plug_th: ANTI_PLUG_OMEGA_TH,
                        safety_stopped: true,
                    };
                }

                ladrc.coast_during_lost(dir, est_forward_speed);

                self.recovery_ramp = (self.recovery_ramp + 0.035).min(1.0);
                let outer = self.profile.v_curve_min
                    + self.recovery_ramp
                        * (self.profile.recovery_outer_pwm - self.profile.v_curve_min);
                let inner = if elapsed_ms < 70 {
                    0.0
                } else {
                    let rev_t = (((elapsed_ms - 70) as f32) / 90.0).min(1.0);
                    rev_t * self.profile.recovery_inner_pwm
                };

                let base = 0.5 * (outer + inner);
                let steer = dir * 0.5 * (outer - inner);
                self.base_speed_lpf = base;

                DriveCommand {
                    base_speed: base,
                    steer_cmd: steer,
                    max_reverse_pwm: libm::fabsf(self.profile.recovery_inner_pwm),
                    instant_snap: false,
                    anti_plug_th: ANTI_PLUG_OMEGA_TH,
                    safety_stopped: false,
                }
            }

            LineState::Lifted => unreachable!(),
        }
    }
}
