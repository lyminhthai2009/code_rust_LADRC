//! 4-IR Isosceles Trapezoid Sensor Fusion, Online Auto-Calibration & Obstacle Classifier.
//!
//! Specifically engineered for the COBOT 4-IR trapezoid layout:
//! - Outer base S1..S4 = 40.0 mm  (x = -1.0000, +1.0000)
//! - Inner base S2..S3 = 22.5 mm  (x = -0.5625, +0.5625)
//! - Forward lookahead of S2, S3 = 4.84 mm ahead of S1, S4
//!
//! Upgraded using live ESP32-C3 hardware telemetry:
//! - Calibrated to real 12-bit ADC range (White ≈ 4095, Black ≈ 1900-2050, Air ≈ 1680-1850).
//! - Fast bidirectional online adaptation so black reaches full 1.00 on ANY arena contrast.
//! - Lifted-in-air auto-stop (`LineState::Lifted`) prevents wheels spinning when picked up!

use crate::config::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LineState {
    /// Normal continuous tracking on the black line.
    Tracking {
        /// Normalized lateral error in [-1.25, +1.25].
        /// Positive => line is to the RIGHT of robot center => steer RIGHT.
        pos: f32,
        /// Sum of active normalized sensor weights (tracks line width & confidence).
        strength: f32,
    },
    /// X-Crossing detected (75°-125° oblique-capable, debounced against speed bumps).
    /// `is_new_event` is `true` on the exact tick a new crossing is confirmed (with 650 ms cooldown).
    Crossing {
        elapsed_ms: u32,
        is_new_event: bool,
        wings_clear: bool,
    },
    /// Thin black line sitting inside the 22.5 mm gap between S2 and S3,
    /// OR briefly crossing the red START/FINISH bar while centered.
    CenterStraddle { pos_hold: f32, elapsed_ms: u32 },
    /// Line exited past S1 (dir = -1.0) or S4 (dir = +1.0) in a sharp curve.
    Lost { dir: f32, elapsed_ms: u32 },
    /// Robot lifted off the track (all 4 sensors continuously dark beyond `LIFTED_STOP_MS`).
    Lifted,
}

pub struct SensorProcessor {
    white_ref: [f32; 4],
    black_ref: [f32; 4],
    /// Whether startup/put-down already locked onto a real arena surface (buttonless auto-calib!).
    arena_seeded: bool,
    /// One-shot early-run darkest-line tracker (first 300 ms on main mat) to lock glossy black ink
    /// on a brand-new competition arena with zero buttons and zero ongoing drift!
    early_run_ticks: u32,
    early_min_black: f32,
    early_black_locked: bool,
    weights: [f32; 4],
    last_raw: [u16; 4],
    last_valid_pos: f32,
    last_side: f32,
    line_width_ema: f32,
    /// Continuous ticks where all 4 sensors read uniformly air-dark (Lifted-in-air).
    all_dark_ticks: u32,
    /// Crossing confirmation debounce & post-crossing cooldown (ticks).
    cross_confirm_ticks: u32,
    cross_cooldown_ticks: u32,
    straddle_ticks: u32,
    lost_ticks: u32,
}

impl SensorProcessor {
    pub const fn new() -> Self {
        Self {
            white_ref: [INIT_WHITE_ADC; 4],
            black_ref: [INIT_BLACK_ADC; 4],
            arena_seeded: false,
            early_run_ticks: 0,
            early_min_black: 4095.0,
            early_black_locked: false,
            weights: [0.0; 4],
            last_raw: [INIT_WHITE_ADC as u16; 4],
            last_valid_pos: 0.0,
            last_side: 1.0,
            line_width_ema: 1.0,
            all_dark_ticks: 0,
            cross_confirm_ticks: 0,
            cross_cooldown_ticks: 0,
            straddle_ticks: 0,
            lost_ticks: 0,
        }
    }

    /// Median of 3 raw ADC readings: completely eliminates PWM switching & optical glitches.
    #[inline(always)]
    pub fn median3(a: u16, b: u16, c: u16) -> u16 {
        let max_ab = if a > b { a } else { b };
        let min_ab = if a < b { a } else { b };
        if c > max_ab {
            max_ab
        } else if c < min_ab {
            min_ab
        } else {
            c
        }
    }

    #[inline(always)]
    fn order_channels(raw_gpio: [u16; 4]) -> [u16; 4] {
        if MIRROR_SENSORS_LR {
            [raw_gpio[3], raw_gpio[2], raw_gpio[1], raw_gpio[0]]
        } else {
            raw_gpio
        }
    }

    /// Buttonless instant arena-baseline seeding during startup settle (or right after put-down!).
    /// Directly locks onto ANY competition arena's true white & black reflectance without BOOT button!
    pub fn seed_startup_sample(&mut self, raw_gpio: [u16; 4]) {
        let ordered = Self::order_channels(raw_gpio);
        self.last_raw = ordered;

        let mut brightest = 0u16;
        let mut darkest = 4095u16;
        for &v in &ordered {
            if v > brightest {
                brightest = v;
            }
            if v < darkest {
                darkest = v;
            }
        }
        let b_f = brightest as f32;
        let d_f = darkest as f32;

        if IR_WHITE_IS_HIGH && b_f > 1400.0 {
            if !self.arena_seeded {
                // First valid surface sample: snap immediately to THIS arena's true white!
                self.white_ref = [b_f; 4];
                if b_f - d_f > MIN_CAL_SPAN {
                    self.black_ref = [d_f; 4];
                    self.early_black_locked = true;
                } else {
                    self.black_ref = [(b_f * 0.45).min(INIT_BLACK_ADC); 4];
                }
                self.arena_seeded = true;
            } else {
                for i in 0..4 {
                    let v = ordered[i] as f32;
                    if b_f > self.white_ref[i] {
                        self.white_ref[i] = 0.70 * self.white_ref[i] + 0.30 * b_f;
                    } else {
                        self.white_ref[i] = 0.92 * self.white_ref[i] + 0.08 * b_f.max(v);
                    }
                    if b_f - d_f > MIN_CAL_SPAN {
                        let target_black = if v < b_f - 0.55 * (b_f - d_f) { v } else { d_f };
                        self.black_ref[i] = 0.80 * self.black_ref[i] + 0.20 * target_black;
                        self.early_black_locked = true;
                    }
                }
            }
        }
    }

    pub fn sweep_calibrate_reset(&mut self) {
        self.white_ref = [0.0; 4];
        self.black_ref = [4095.0; 4];
    }

    pub fn sweep_calibrate_sample(&mut self, raw_gpio: [u16; 4]) {
        let ordered = Self::order_channels(raw_gpio);
        self.last_raw = ordered;
        for i in 0..4 {
            let v = ordered[i] as f32;
            if IR_WHITE_IS_HIGH {
                if v > self.white_ref[i] {
                    self.white_ref[i] = v;
                }
                if v < self.black_ref[i] {
                    self.black_ref[i] = v;
                }
            } else {
                if v < self.white_ref[i] {
                    self.white_ref[i] = v;
                }
                if v > self.black_ref[i] {
                    self.black_ref[i] = v;
                }
            }
        }
    }

    pub fn sweep_calibrate_finish(&mut self) {
        for i in 0..4 {
            if IR_WHITE_IS_HIGH {
                if self.white_ref[i] - self.black_ref[i] < MIN_CAL_SPAN {
                    self.white_ref[i] = INIT_WHITE_ADC;
                    self.black_ref[i] = INIT_BLACK_ADC;
                }
            } else if self.black_ref[i] - self.white_ref[i] < MIN_CAL_SPAN {
                self.white_ref[i] = INIT_BLACK_ADC;
                self.black_ref[i] = INIT_WHITE_ADC;
            }
        }
    }

    /// Process one 500 Hz sensor frame and return the classified `LineState`.
    pub fn update(&mut self, raw_gpio: [u16; 4], eso_vel_hint: f32) -> LineState {
        let ordered = Self::order_channels(raw_gpio);
        self.last_raw = ordered;

        // Find brightest and darkest channels in this frame (for safe online calibration!)
        let mut frame_max = ordered[0];
        let mut frame_min = ordered[0];
        for &v in &ordered[1..] {
            if v > frame_max {
                frame_max = v;
            }
            if v < frame_min {
                frame_min = v;
            }
        }
        let fmax = frame_max as f32;
        let fmin = frame_min as f32;
        let spread = fmax - fmin;

        // If the robot was turned on in the air and just placed onto the arena floor, seed
        // arena white/black immediately on first floor contact!
        if IR_WHITE_IS_HIGH && !self.arena_seeded && fmax > 2200.0 && spread > MIN_CAL_SPAN {
            self.white_ref = [fmax; 4];
            self.black_ref = [fmin; 4];
            self.arena_seeded = true;
            self.early_black_locked = true;
        }

        let has_valid_surface_contrast =
            IR_WHITE_IS_HIGH && fmax > 2200.0 && spread > MIN_CAL_SPAN;

        // 1. Safe Online Arena Adaptation (Buttonless!)
        //    - White expands upward immediately on brighter mat regions.
        //    - Black expands downward immediately on darker ink (> 300 ADC).
        //    - One-shot early-run lock (first 300 ms on main mat only) adapts upward if the
        //      competition arena uses glossy brighter black ink (e.g. 2450 vs 1950), then locks
        //      forever so gray edges in curves can NEVER poison `black_ref`!
        let mut raw_black_glob = [0.0f32; 4];
        let mut avg_span = 0.0f32;
        for i in 0..4 {
            let v = ordered[i] as f32;
            if IR_WHITE_IS_HIGH {
                if v > self.white_ref[i] {
                    self.white_ref[i] += CAL_EXPAND_ALPHA * (v - self.white_ref[i]);
                }

                if has_valid_surface_contrast {
                    let is_dark_ch = v < fmax - 0.65 * spread;
                    if is_dark_ch && v < self.black_ref[i] && v > 250.0 {
                        self.black_ref[i] += CAL_EXPAND_ALPHA * (v - self.black_ref[i]);
                    }
                }

                let span = (self.white_ref[i] - self.black_ref[i]).max(MIN_CAL_SPAN);
                avg_span += span;
                raw_black_glob[i] = ((self.white_ref[i] - v) / span).clamp(0.0, 1.0);
            } else {
                if v < self.white_ref[i] {
                    self.white_ref[i] += CAL_EXPAND_ALPHA * (v - self.white_ref[i]);
                }
                if v > self.black_ref[i] {
                    self.black_ref[i] += CAL_EXPAND_ALPHA * (v - self.black_ref[i]);
                }
                let span = (self.black_ref[i] - self.white_ref[i]).max(MIN_CAL_SPAN);
                avg_span += span;
                raw_black_glob[i] = ((v - self.white_ref[i]) / span).clamp(0.0, 1.0);
            }
        }
        avg_span *= 0.25;

        // One-shot early-run glossy-black lock (runs ONLY once in first 300 ms if startup saw all-white)
        if IR_WHITE_IS_HIGH && !self.early_black_locked && has_valid_surface_contrast {
            let avg_white =
                0.25 * (self.white_ref[0] + self.white_ref[1] + self.white_ref[2] + self.white_ref[3]);
            if fmax >= 0.88 * avg_white {
                if fmin < self.early_min_black {
                    self.early_min_black = fmin;
                }
                self.early_run_ticks = self.early_run_ticks.saturating_add(1);
                if self.early_run_ticks >= 150 {
                    // 300 ms elapsed on main white mat: lock black_ref once if brighter ink detected
                    if self.early_min_black < avg_white - MIN_CAL_SPAN {
                        for b in &mut self.black_ref {
                            if self.early_min_black > *b + 120.0 {
                                *b = self.early_min_black;
                            }
                        }
                    }
                    self.early_black_locked = true;
                }
            }
        }

        // 2. Bridge-Proof, 5mm-Bump-Proof & Arena-Invariant X-Crossing & Air Detector
        let tick_ms = 1000 / LOOP_FREQ_HZ;
        let confirm_needed = (CROSS_CONFIRM_MS / tick_ms).max(1);
        let cooldown_ticks_max = (CROSS_COOLDOWN_MS / tick_ms).max(1);

        let raw_sum_glob =
            raw_black_glob[0] + raw_black_glob[1] + raw_black_glob[2] + raw_black_glob[3];
        if self.cross_cooldown_ticks > 0 {
            self.cross_cooldown_ticks -= 1;
        }

        let min_b_glob = raw_black_glob[0]
            .min(raw_black_glob[1])
            .min(raw_black_glob[2])
            .min(raw_black_glob[3]);

        // Lifted-in-air check: ALL 4 deeply dark AND almost zero spread (<= 14% span)
        // => Dark Bridge / 5mm Speed Bump with black line NEVER falsely triggers [AIR]!
        let is_air_dark =
            min_b_glob >= AIR_MIN_ALL_BLACK && spread <= AIR_MAX_SPREAD_RATIO * avg_span;

        // X-Crossing check: ALL 4 uniformly solid matte black!
        // => Rejects Bridge background AND 5mm blue Speed Bumps where outer sensors > line!
        let is_crossing_raw = min_b_glob >= CROSSING_MIN_ALL_BLACK
            && raw_sum_glob >= CROSSING_MIN_SUM
            && spread <= CROSSING_MAX_SPREAD_RATIO * avg_span;

        // 3. LOCAL-CONTRAST TRACKING NORMALIZATION (100% Immune to Bridge Bg & 5mm Speed Bumps!)
        //    Uses THIS frame's brightest sensor `fmax` as the local background (so grey/wood Bridge
        //    or lifted 5mm blue Speed Bump automatically reads 0.00!), divided by `max(fmax - black_ref, 420)`
        //    to preserve smooth analog linearity on tiny line edges!
        let inv_scale = 1.0 / (BLACK_SAT_CEIL - BLACK_NOISE_FLOOR);
        let mut sum_w = 0.0f32;
        for i in 0..4 {
            let v = ordered[i] as f32;
            let lb = if IR_WHITE_IS_HIGH {
                let local_span = (fmax - self.black_ref[i]).max(LOCAL_CONTRAST_SPAN_MIN);
                ((fmax - v) / local_span).clamp(0.0, 1.0)
            } else {
                let local_span = (self.black_ref[i] - fmin).max(LOCAL_CONTRAST_SPAN_MIN);
                ((v - fmin) / local_span).clamp(0.0, 1.0)
            };
            let clean = ((lb - BLACK_NOISE_FLOOR) * inv_scale).clamp(0.0, 1.0);
            self.weights[i] = clean;
            sum_w += clean;
        }

        // 4. Lifted-In-Air Auto-Stop vs Confirmed X-Crossing Event
        if is_air_dark {
            self.all_dark_ticks = self.all_dark_ticks.saturating_add(1);
            if self.all_dark_ticks * tick_ms >= LIFTED_STOP_MS {
                self.cross_confirm_ticks = 0;
                self.cross_cooldown_ticks = 0;
                self.straddle_ticks = 0;
                self.lost_ticks = 0;
                // Re-arm buttonless arena seeding so putting the robot down on a new table
                // immediately adapts to that table's surface!
                self.arena_seeded = false;
                self.early_black_locked = false;
                self.early_run_ticks = 0;
                self.early_min_black = 4095.0;
                return LineState::Lifted;
            }
        } else {
            self.all_dark_ticks = 0;
        }

        if is_crossing_raw {
            self.cross_confirm_ticks = self.cross_confirm_ticks.saturating_add(1);
            // Only enter Crossing state AFTER full confirmation (6 ms = 3 ticks)!
            // 1-2 tick flickers fall through to normal smooth LADRC Tracking (ZERO curve stutter!).
            if self.cross_confirm_ticks >= confirm_needed {
                self.straddle_ticks = 0;
                self.lost_ticks = 0;
                let is_new_event = if self.cross_cooldown_ticks == 0 {
                    self.cross_cooldown_ticks = cooldown_ticks_max;
                    true
                } else {
                    false
                };
                let wings_clear = self.weights[0] < 0.25 && self.weights[3] < 0.25;
                return LineState::Crossing {
                    elapsed_ms: self.cross_confirm_ticks * tick_ms,
                    is_new_event,
                    wings_clear,
                };
            }
        } else {
            self.cross_confirm_ticks = 0;
        }

        // 5. Normal Tracking State
        if sum_w >= LINE_PRESENT_SUM {
            self.straddle_ticks = 0;
            self.lost_ticks = 0;

            let weighted_sum = self.weights[0] * SENSOR_POS_NORM[0]
                + self.weights[1] * SENSOR_POS_NORM[1]
                + self.weights[2] * SENSOR_POS_NORM[2]
                + self.weights[3] * SENSOR_POS_NORM[3];
            // CRITICAL: Regularized denominator `sum_w.max(SENSOR_DENOM_FLOOR)` eliminates the
            // discontinuous -0.56 <-> +0.56 relay jump across the 22.5 mm S2-S3 center gap!
            let mut pos = weighted_sum / sum_w.max(SENSOR_DENOM_FLOOR);

            // Sub-edge extrapolation when ONLY outer-left (S1) or outer-right (S4) sees fading edge
            let only_left_outer = self.weights[0] > 0.05
                && self.weights[1] < 0.08
                && self.weights[2] < 0.06
                && self.weights[3] < 0.06;
            let only_right_outer = self.weights[3] > 0.05
                && self.weights[2] < 0.08
                && self.weights[1] < 0.06
                && self.weights[0] < 0.06;

            if only_left_outer && self.last_valid_pos < -0.65 {
                let fade = (1.0 - self.weights[0]).clamp(0.0, 1.0);
                pos = -1.0 - OUTER_EDGE_EXTRAP * fade;
            } else if only_right_outer && self.last_valid_pos > 0.65 {
                let fade = (1.0 - self.weights[3]).clamp(0.0, 1.0);
                pos = 1.0 + OUTER_EDGE_EXTRAP * fade;
            }

            self.line_width_ema = 0.96 * self.line_width_ema + 0.04 * sum_w;
            self.last_valid_pos = pos;
            if pos > 0.25 {
                self.last_side = 1.0;
            } else if pos < -0.25 {
                self.last_side = -1.0;
            }

            return LineState::Tracking {
                pos,
                strength: sum_w,
            };
        }

        // 6. All-White: Thin-Line Center Straddle vs. True Hairpin Exit
        if libm::fabsf(self.last_valid_pos) < STRADDLE_POS_LIMIT
            && self.straddle_ticks * tick_ms < MAX_STRADDLE_MS
        {
            self.straddle_ticks = self.straddle_ticks.saturating_add(1);
            let pos_hold = self.last_valid_pos * 0.30;
            return LineState::CenterStraddle {
                pos_hold,
                elapsed_ms: self.straddle_ticks * tick_ms,
            };
        }

        // 7. True Line Loss (Sharp Curve Exit)
        self.lost_ticks = self.lost_ticks.saturating_add(1);
        let dir = if libm::fabsf(self.last_valid_pos) >= STRADDLE_POS_LIMIT {
            if self.last_valid_pos >= 0.0 {
                1.0
            } else {
                -1.0
            }
        } else if libm::fabsf(eso_vel_hint) > 0.25 {
            if eso_vel_hint >= 0.0 { 1.0 } else { -1.0 }
        } else {
            self.last_side
        };

        LineState::Lost {
            dir,
            elapsed_ms: self.lost_ticks * tick_ms,
        }
    }

    pub fn weights(&self) -> [f32; 4] {
        self.weights
    }

    pub fn last_raw(&self) -> [u16; 4] {
        self.last_raw
    }

    pub fn white_ref(&self) -> [f32; 4] {
        self.white_ref
    }

    pub fn black_ref(&self) -> [f32; 4] {
        self.black_ref
    }

    pub fn line_width_ema(&self) -> f32 {
        self.line_width_ema
    }
}

impl Default for SensorProcessor {
    fn default() -> Self {
        Self::new()
    }
}
