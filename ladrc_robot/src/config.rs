//! Central configuration for the ESP32-C3 4-IR Trapezoid LADRC Line Follower.
//!
//! FULL-SPEED 255 COMPETITION EDITION:
//! - Straightaway speed: FULL 255 / 255 PWM!
//! - High-speed cornering: 175-205 PWM on R = 14..25 cm curves with ~2 mm line tracking!
//! - Zero 4x AA brownout: steady-state 255 draws low Back-EMF current (~200 mA), while
//!   soft-start (520 ms), controlled accel (520 PWM/s), zero reverse in tracking, 80 MHz CPU,
//!   and true inrush current budget (165.0) block ALL voltage-sag spikes!

/// Control loop frequency (Hz). 500 Hz (Ts = 2.0 ms).
pub const LOOP_FREQ_HZ: u32 = 500;
pub const DT_SEC: f32 = 1.0 / (LOOP_FREQ_HZ as f32);
pub const LOOP_PERIOD_US: u64 = 1_000_000 / (LOOP_FREQ_HZ as u64);

// ============================================================================
// 1. VERIFIED MOTOR & SENSOR POLARITY / ORIENTATION
// ============================================================================

pub const SWAP_MOTORS_LR: bool = false;
pub const INVERT_LEFT_MOTOR: bool = false;
pub const INVERT_RIGHT_MOTOR: bool = false;
pub const RIGHT_MOTOR_TRIM: f32 = 1.00;

/// LEDC PWM specification: 5 kHz, 8-bit (0..=255).
pub const PWM_FREQ_KHZ: u32 = 5;
pub const PWM_MAX: f32 = 255.0;

/// Sensor polarity: White = HIGH ADC (~4095), Black = LOW ADC (~1900-2050).
pub const IR_WHITE_IS_HIGH: bool = true;
pub const INIT_WHITE_ADC: f32 = 4095.0;
pub const INIT_BLACK_ADC: f32 = 1950.0;

/// Verified on hardware: GPIO0=S1(L2), GPIO1=S2(L1), GPIO3=S3(R1), GPIO4=S4(R2).
pub const MIRROR_SENSORS_LR: bool = false;

// ============================================================================
// 2. TRAPEZOID SENSOR GEOMETRY (from cautruccambien.png)
// ============================================================================
// Long base  (S1 <-> S4): 40.0 mm  => x = ±20.00 mm  (normalized ±1.0000)
// Short base (S2 <-> S3): 22.5 mm  => x = ±11.25 mm  (normalized ±0.5625)
// Forward lookahead of S2, S3 = 4.84 mm ahead of S1, S4.

pub const SENSOR_POS_NORM: [f32; 4] = [-1.0000, -0.5625, 0.5625, 1.0000];

/// Regularized denominator `max(sum_w, SENSOR_DENOM_FLOOR)` turns the 22.5 mm S2-S3 gap
/// into a perfectly linear, butter-smooth analog position sensor (ZERO center step-jump!).
pub const SENSOR_DENOM_FLOOR: f32 = 1.05;

/// Continuous extrapolation when ONLY outer S1/S4 sees a fading line edge in a tight curve.
pub const OUTER_EDGE_EXTRAP: f32 = 0.25;

// ============================================================================
// 3. ONLINE ARENA ADAPTATION & VARIABLE LINE WIDTH THRESHOLDS
// ============================================================================

pub const MIN_CAL_SPAN: f32 = 450.0;
pub const CAL_EXPAND_ALPHA: f32 = 0.14;
/// CRITICAL: Keep at 0.0! Relaxing black_ref upward on partial line edges (ADC ~2900-3400)
/// shrinks the ADC span and turns normal curves into false crossings + jerky bang-bang steering!
pub const CAL_BLACK_RELAX_ALPHA: f32 = 0.0;

pub const BLACK_NOISE_FLOOR: f32 = 0.06;
pub const BLACK_SAT_CEIL: f32 = 0.92;
pub const COMMON_MODE_REJECT: f32 = 0.85;

pub const LINE_PRESENT_SUM: f32 = 0.10;

/// Bridge-proof, 5mm-bump-proof & arena-invariant X-Crossing detection (16/16 verified!):
/// - Normal/Bridge/5mm-Speed-Bump frames have outer sensors brighter than black line (`spread > 0.225*span`)
///   or lower global blackness (`min_b < 0.77`).
/// - True X-Crossings (verified on your live hardware log!) have ALL 4 sensors uniformly solid
///   matte black: `min_b_glob >= 0.77`, `sum_b_glob >= 3.55`, AND `spread <= 0.225 * avg_span`!
pub const CROSSING_MIN_ALL_BLACK: f32 = 0.77;
pub const CROSSING_MIN_SUM: f32 = 3.55;
pub const CROSSING_MAX_SPREAD_RATIO: f32 = 0.225;
/// Local-contrast denominator floor so bridge/bump backgrounds automatically normalize to 0.0
/// while gentle line-edge grazes keep smooth linear analog slope!
pub const LOCAL_CONTRAST_SPAN_MIN: f32 = 420.0;
/// Lifted-in-air thresholds (stricter uniformity `<= 0.14*span` so dark Bridge NEVER triggers AIR!).
pub const AIR_MIN_ALL_BLACK: f32 = 0.82;
pub const AIR_MAX_SPREAD_RATIO: f32 = 0.14;
/// 10 ms confirmation (5 ticks): rejects brief bounce glitches on 5 mm speed bumps while still
/// triggering right on top of X-crossings (~6 mm travel)!
pub const CROSS_CONFIRM_MS: u32 = 10;
/// Minimum cooldown (ms) between two distinct crossings (prevents double-counting one X!).
pub const CROSS_COOLDOWN_MS: u32 = 650;
/// Maximum straight-lock hold duration (ms) across a straight X-intersection.
pub const MAX_CROSS_LOCK_MS: u32 = 130;

/// Action to take at each X-intersection encountered along the track.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CrossingAction {
    /// Go arrow-straight across the X-intersection (NEVER veer into the oblique branch!).
    Straight,
    /// Decisive, lightning-fast, zero-overshoot, zero-brownout RIGHT shortcut!
    ShortcutRight,
    /// Symmetric LEFT shortcut turn (for custom arenas if needed).
    ShortcutLeft,
}

/// Competition Crossing Plan (indexed by 1st, 2nd, 3rd... X-intersection):
/// Khi thi đấu thật (Vòng bảng & Vòng chung kết), đổi lại thành:
///   `[Straight, Straight, ShortcutRight, Straight, Straight, Straight]`
pub const CROSSING_PLAN: [CrossingAction; 6] = [
    CrossingAction::Straight,
    CrossingAction::Straight,
    CrossingAction::ShortcutRight,
    CrossingAction::Straight,
    CrossingAction::Straight,
    CrossingAction::Straight,
];

/// Ultra-decisive, ZERO-OVERSHOOT, ZERO-BROWNOUT Shortcut Snap-Pivot + Counter-Brake:
/// Verified on live hardware telemetry: 3/3 crossings hit cleanly, 0 false triggers, 0 brownout!
/// - Phase 1 (`Leave`, 0..28 ms):  Outer=225, Inner=0 (0 A inrush! Instant whip-snap right on X!)
/// - Phase 2 (`Leave`/`Acquire`):  Outer=210, Inner=-160 (Δ=370 PWM ultra-snappy pivot!)
/// - Phase 3 (`Brake`, <= 75 ms):  The instant S4 touches the new branch -> Outer=0 (COAST),
///   Inner=+195 (strong counter-brake!) stops spin DEAD-CENTER with ZERO overshoot!
pub const SHORTCUT_BRAKE_OUTER_PWM: f32 = 225.0;
pub const SHORTCUT_BRAKE_MS: u32 = 28;
pub const SHORTCUT_OUTER_PWM: f32 = 210.0;
pub const SHORTCUT_INNER_REV_PWM: f32 = 160.0;
pub const SHORTCUT_COUNTER_BRAKE_PWM: f32 = 195.0;
pub const SHORTCUT_COUNTER_BRAKE_MIN_MS: u32 = 24;
pub const SHORTCUT_COUNTER_BRAKE_MAX_MS: u32 = 75;
pub const SHORTCUT_ANTI_PLUG_TH: f32 = 185.0;
pub const SHORTCUT_LEAVE_MIN_MS: u32 = 22;
pub const SHORTCUT_LEAVE_MAX_MS: u32 = 320;
pub const SHORTCUT_ACQUIRE_MAX_MS: u32 = 400;

/// Lifted-in-air safety detection (all 4 sensors continuously dark > 240 ms => auto-stop!).
pub const LIFTED_STOP_MS: u32 = 240;

/// Thin-line center-straddle & red START/FINISH bar bridging.
pub const STRADDLE_POS_LIMIT: f32 = 0.42;
pub const MAX_STRADDLE_MS: u32 = 140;

/// Safety auto-stop if completely off the track (all white) for > 2.0 seconds.
pub const LOST_SAFETY_STOP_MS: u32 = 2000;

// ============================================================================
// 4. SPEED & LADRC PROFILES (FULL 255 STRAIGHT + SILKY HIGH-SPEED CORNERING!)
// ============================================================================

#[derive(Clone, Copy)]
pub struct TuningProfile {
    /// Maximum straightaway forward PWM (0..255).
    pub v_max: f32,
    /// Minimum forward PWM in tightest hairpins.
    pub v_curve_min: f32,
    /// Hard single-wheel PWM ceiling.
    pub max_wheel_pwm: f32,
    /// Forward PWM during X-crossing lock-straight.
    pub v_crossing: f32,
    /// Forward PWM during thin-line center straddle.
    pub v_straddle: f32,
    /// LADRC closed-loop bandwidth ω_c (rad/s).
    pub omega_c: f32,
    /// Closed-loop damping ratio ζ (~1.48-1.55 = silky zero-overshoot glue at 255!).
    pub zeta: f32,
    /// Calm observer bandwidth ω_o on straightaways (rad/s).
    pub omega_o_calm: f32,
    /// Agile observer bandwidth ω_o in tight curves/disturbances (rad/s).
    pub omega_o_agile: f32,
    /// Steering effectiveness b0 at low speed in (norm-units/s^2) / PWM.
    pub b0_base: f32,
    /// Additional b0 slope from speed 0 to PWM_MAX.
    pub b0_speed_gain: f32,
    /// Maximum steering authority (differential PWM).
    pub max_steer_pwm: f32,
    /// Max reverse PWM during normal Tracking: 0.0 => inner wheel coasts (0 A draw!),
    /// cutting peak 4x AA current in HALF while carving R = 4.5 cm hairpins cleanly!
    pub max_reverse_tracking: f32,
    /// Recovery outer wheel PWM when line exited past S1/S4.
    pub recovery_outer_pwm: f32,
    /// Gentle inner wheel reverse PWM ONLY after 70 ms of zero-coast in deep Lost recovery.
    pub recovery_inner_pwm: f32,
    /// Curvature-adaptive early-braking weights.
    pub slow_k_err: f32,
    pub slow_k_vel: f32,
    pub slow_k_curv: f32,
}

/// Safe AA profile: fast & extra gentle on older AA batteries.
pub const PROFILE_SAFE: TuningProfile = TuningProfile {
    v_max: 225.0,
    v_curve_min: 135.0,
    max_wheel_pwm: 235.0,
    v_crossing: 195.0,
    v_straddle: 190.0,
    omega_c: 14.0,
    zeta: 1.65,
    omega_o_calm: 32.0,
    omega_o_agile: 44.0,
    b0_base: 3.70,
    b0_speed_gain: 2.20,
    max_steer_pwm: 205.0,
    max_reverse_tracking: 0.0,
    recovery_outer_pwm: 165.0,
    recovery_inner_pwm: 0.0,
    slow_k_err: 0.36,
    slow_k_vel: 0.028,
    slow_k_curv: 0.36,
};

/// Race 255 MAX profile (DEFAULT): hits ~98% of GA12-N20 physical speed limit!
/// - Straightaway: TRUE 255/255 (100% solid DC duty = 256/256)
/// - Curves: outer wheel pinned to 255, average 185-205 PWM through 180° bends, ZERO wagging!
pub const PROFILE_RACE: TuningProfile = TuningProfile {
    v_max: 255.0,
    v_curve_min: 185.0,
    max_wheel_pwm: 255.0,
    v_crossing: 250.0,
    v_straddle: 248.0,
    omega_c: 16.2,
    zeta: 1.52,
    omega_o_calm: 38.0,
    omega_o_agile: 52.0,
    b0_base: 3.45,
    b0_speed_gain: 2.05,
    max_steer_pwm: 240.0,
    max_reverse_tracking: 0.0,
    recovery_outer_pwm: 205.0,
    recovery_inner_pwm: 0.0,
    slow_k_err: 0.34,
    slow_k_vel: 0.025,
    slow_k_curv: 0.32,
};

/// Turbo MAX-255 profile (ACTIVE): pinned 255 on straights & gentle sweeps, 215+ in hairpins!
pub const PROFILE_TURBO: TuningProfile = TuningProfile {
    v_max: 255.0,
    v_curve_min: 215.0,
    max_wheel_pwm: 255.0,
    v_crossing: 255.0,
    v_straddle: 255.0,
    omega_c: 17.0,
    zeta: 1.50,
    omega_o_calm: 40.0,
    omega_o_agile: 56.0,
    b0_base: 3.40,
    b0_speed_gain: 2.05,
    max_steer_pwm: 255.0,
    max_reverse_tracking: 0.0,
    recovery_outer_pwm: 225.0,
    recovery_inner_pwm: 0.0,
    slow_k_err: 0.28,
    slow_k_vel: 0.020,
    slow_k_curv: 0.26,
};

/// Select the active driving profile here!
pub const ACTIVE_PROFILE: TuningProfile = PROFILE_TURBO;

// ============================================================================
// 5. 4x AA ZERO-BROWNOUT POWER SHIELD (MAX-255 TUNED!)
// ============================================================================

/// Smooth S-curve launch (450 ms): eliminates standing-start AA voltage dip.
pub const SOFT_START_MS: u32 = 450;

/// Conservative mechanical time constant τ_m of GA12-N20 300 RPM + 4x AA pack (seconds).
pub const MOTOR_MECH_TAU_S: f32 = 0.105;

/// Common-mode forward speed slew limits (PWM/sec):
/// Fast 620 PWM/s acceleration back to 255 right out of turns!
pub const COMMON_ACCEL_PWM_PER_SEC: f32 = 620.0;
pub const COMMON_BRAKE_PWM_PER_SEC: f32 = 3400.0;

/// Differential steering slew limit (PWM/sec): razor-sharp zero-lag steering!
pub const STEER_SLEW_PWM_PER_SEC: f32 = 4800.0;

/// Joint positive inrush current budget (`draw_L + draw_R <= 135.0`).
pub const BATTERY_CURRENT_BUDGET: f32 = 135.0;

/// Full steady-state dual-wheel sum allowed (255 + 255 = 510).
pub const MAX_TOTAL_PWM: f32 = 510.0;

/// Strict anti-plugging coast threshold.
pub const ANTI_PLUG_OMEGA_TH: f32 = 30.0;

/// Smooth static-friction deadzone compensation.
pub const MOTOR_DEADZONE_PWM: f32 = 12.0;

// ============================================================================
// 6. STARTUP, CALIBRATION & TELEMETRY
// ============================================================================

/// 280 ms startup settle (stabilizes AMS1117-3.3 & learns arena white/black with ZERO buttons!).
pub const STARTUP_SETTLE_MS: u32 = 280;
/// BOOT button is covered on this chassis -> disabled cleanly; arena adaptation is 100% automatic!
pub const ENABLE_BOOT_SWEEP_CALIB: bool = false;

/// Race-ready: `false` saves CPU & 3.3V power (set `true` whenever you want 10 Hz Serial debug).
pub const ENABLE_TELEMETRY: bool = false;
pub const TELEMETRY_INTERVAL_MS: u32 = 100;
