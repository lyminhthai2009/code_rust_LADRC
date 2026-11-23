#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::ledc::channel::{self, ChannelIFace};
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::main;
use esp_hal::time::{Duration, Instant, Rate};
use esp_println::println;

use ladrc_robot::config::*;
use ladrc_robot::ladrc::LadrcController;
use ladrc_robot::planner::MotionPlanner;
use ladrc_robot::power_motor::MotorPowerController;
use ladrc_robot::sensor::{LineState, SensorProcessor};

esp_bootloader_esp_idf::esp_app_desc!();

/// Read all 4 IR ADC1 channels with median-of-3 spike rejection.
/// Order: [GPIO0 (S1), GPIO1 (S2), GPIO3 (S3), GPIO4 (S4)].
macro_rules! read_ir_median3 {
    ($adc:expr, $p0:expr, $p1:expr, $p2:expr, $p3:expr) => {{
        let mut s = [[0u16; 3]; 4];
        for k in 0..3 {
            s[0][k] = nb::block!($adc.read_oneshot(&mut $p0)).unwrap_or(INIT_WHITE_ADC as u16);
            s[1][k] = nb::block!($adc.read_oneshot(&mut $p1)).unwrap_or(INIT_WHITE_ADC as u16);
            s[2][k] = nb::block!($adc.read_oneshot(&mut $p2)).unwrap_or(INIT_WHITE_ADC as u16);
            s[3][k] = nb::block!($adc.read_oneshot(&mut $p3)).unwrap_or(INIT_WHITE_ADC as u16);
        }
        [
            SensorProcessor::median3(s[0][0], s[0][1], s[0][2]),
            SensorProcessor::median3(s[1][0], s[1][1], s[1][2]),
            SensorProcessor::median3(s[2][0], s[2][1], s[2][2]),
            SensorProcessor::median3(s[3][0], s[3][1], s[3][2]),
        ]
    }};
}

#[allow(
    clippy::large_stack_frames,
    reason = "Hardware initialization structs live on main stack"
)]
#[main]
fn main() -> ! {
    // 1. Initialize ESP32-C3 at 80 MHz (NO Wi-Fi/BLE => ~40% lower 3.3V draw than 160 MHz,
    //    maximizes AMS1117-3.3 brownout margin on 4x AA while running 500 Hz effortlessly!)
    let hal_cfg = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let peripherals = esp_hal::init(hal_cfg);

    // 2. Turn ON 4x IR emitter LEDs (GPIO 21 = HIGH)
    let _ir_led_on = Output::new(peripherals.GPIO21, Level::High, OutputConfig::default());

    // 3. Optional BOOT button on GPIO 9 (internal pull-up, active LOW)
    let boot_btn = Input::new(
        peripherals.GPIO9,
        InputConfig::default().with_pull(Pull::Up),
    );

    // 4. Configure ADC1 (12-bit, 11 dB attenuation for full ~0..3.1V IR sensor range)
    //    S1 = GPIO0, S2 = GPIO1, S3 = GPIO3, S4 = GPIO4
    let mut adc1_cfg = AdcConfig::new();
    let mut pin_s1 = adc1_cfg.enable_pin(peripherals.GPIO0, Attenuation::_11dB);
    let mut pin_s2 = adc1_cfg.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
    let mut pin_s3 = adc1_cfg.enable_pin(peripherals.GPIO3, Attenuation::_11dB);
    let mut pin_s4 = adc1_cfg.enable_pin(peripherals.GPIO4, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_cfg);

    // 5. Configure LEDC PWM: 5 kHz, 8-bit (0..=255), LowSpeed Timer0
    //    Left Motor:  GPIO2  (Ch0 = A), GPIO10 (Ch1 = B)
    //    Right Motor: GPIO6  (Ch2 = A), GPIO5  (Ch3 = B)
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(PWM_FREQ_KHZ),
        })
        .expect("LEDC timer0 config failed");

    let mut ch_la = ledc.channel(channel::Number::Channel0, peripherals.GPIO2);
    ch_la
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .expect("LEDC Ch0 config failed");

    let mut ch_lb = ledc.channel(channel::Number::Channel1, peripherals.GPIO10);
    ch_lb
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .expect("LEDC Ch1 config failed");

    let mut ch_ra = ledc.channel(channel::Number::Channel2, peripherals.GPIO6);
    ch_ra
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .expect("LEDC Ch2 config failed");

    let mut ch_rb = ledc.channel(channel::Number::Channel3, peripherals.GPIO5);
    ch_rb
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .expect("LEDC Ch3 config failed");

    // 6. Instantiate Control Subsystems
    let mut motors = MotorPowerController::new(ch_la, ch_lb, ch_ra, ch_rb);
    let mut sensors = SensorProcessor::new();
    let mut ladrc = LadrcController::new(ACTIVE_PROFILE);
    let mut planner = MotionPlanner::new(ACTIVE_PROFILE);

    println!("=== LADRC Line Follower (ESP32-C3 / 500Hz / 4xAA Safe) ===");
    println!(
        "Profile: v_max={}, v_curve={}, wc={}, wo={}..{}",
        ACTIVE_PROFILE.v_max as i32,
        ACTIVE_PROFILE.v_curve_min as i32,
        ACTIVE_PROFILE.omega_c as i32,
        ACTIVE_PROFILE.omega_o_calm as i32,
        ACTIVE_PROFILE.omega_o_agile as i32
    );

    // 7. Startup Settle Window (350 ms): stabilize AMS1117-3.3 & IR LEDs, seed arena baseline,
    //    and check if user holds BOOT (GPIO9) for optional sweep calibration.
    let settle_ticks = (STARTUP_SETTLE_MS * LOOP_FREQ_HZ) / 1000;
    let mut boot_low_count = 0u32;
    for _ in 0..settle_ticks {
        let tick_start = Instant::now();
        let raw = read_ir_median3!(adc1, pin_s1, pin_s2, pin_s3, pin_s4);
        sensors.seed_startup_sample(raw);
        if ENABLE_BOOT_SWEEP_CALIB && boot_btn.is_low() {
            boot_low_count += 1;
        }
        while tick_start.elapsed() < Duration::from_micros(LOOP_PERIOD_US) {}
    }

    // 8. Optional Gentle In-Place Sweep Calibration (triggered ONLY if BOOT held during settle)
    if ENABLE_BOOT_SWEEP_CALIB && boot_low_count > (settle_ticks / 3) {
        println!("[CALIB] BOOT detected -> Starting 2.4s gentle sweep calibration...");
        sensors.sweep_calibrate_reset();

        // Wait briefly for finger to release BOOT
        let wait_release = Instant::now();
        while boot_btn.is_low() && wait_release.elapsed() < Duration::from_millis(800) {}

        // 4 gentle sweeps (Left -> Right -> Left -> Center) at safe low PWM (±75)
        let sweep_ticks = (2400 * LOOP_FREQ_HZ) / 1000;
        for t in 0..sweep_ticks {
            let tick_start = Instant::now();
            let raw = read_ir_median3!(adc1, pin_s1, pin_s2, pin_s3, pin_s4);
            sensors.sweep_calibrate_sample(raw);

            let phase = (t * 4) / sweep_ticks;
            let (l_cmd, r_cmd) = match phase {
                0 => (-72.0, 72.0),  // gently turn left
                1 => (72.0, -72.0),  // gently turn right
                2 => (72.0, -72.0),  // continue right
                _ => (-72.0, 72.0),  // return toward center
            };
            motors.drive_calibration(l_cmd, r_cmd);
            while tick_start.elapsed() < Duration::from_micros(LOOP_PERIOD_US) {}
        }

        motors.coast_stop();
        sensors.sweep_calibrate_finish();
        let w = sensors.white_ref();
        let b = sensors.black_ref();
        println!(
            "[CALIB] Done! White=[{},{},{},{}] Black=[{},{},{},{}]",
            w[0] as i32,
            w[1] as i32,
            w[2] as i32,
            w[3] as i32,
            b[0] as i32,
            b[1] as i32,
            b[2] as i32,
            b[3] as i32
        );

        // Short pause before race launch (or tap BOOT to launch immediately)
        let pause_start = Instant::now();
        while pause_start.elapsed() < Duration::from_millis(1200) {
            if boot_btn.is_low() {
                break;
            }
        }
        motors.reset();
    }

    println!("[RUN] Soft-start launch!");
    ladrc.reset();
    planner.reset();

    // 9. Deterministic 500 Hz Real-Time Control Loop
    let telemetry_ticks = ((TELEMETRY_INTERVAL_MS * LOOP_FREQ_HZ) / 1000).max(1);
    let mut tick_counter: u32 = 0;
    let mut est_forward_speed: f32 = 0.0;

    loop {
        let tick_start = Instant::now();

        // A. Read 4 IR sensors (Median-of-3 spike rejection)
        let raw = read_ir_median3!(adc1, pin_s1, pin_s2, pin_s3, pin_s4);

        // B. Update online auto-calibration + trapezoid position & obstacle classifier
        let line_state = sensors.update(raw, ladrc.velocity_hint());

        // C. Execute LADRC + Curvature-Adaptive Velocity Planner & 80°-120° Crossing Shortcut
        let drive_cmd = planner.compute(
            line_state,
            sensors.weights(),
            &mut ladrc,
            est_forward_speed,
        );

        // D. Apply Zero-Lag Steering + Battery-Safe Motor Control
        let applied = if drive_cmd.safety_stopped {
            if matches!(line_state, LineState::Lifted) {
                // Full reset on Lifted => smooth S-curve soft-start when placed down at START!
                motors.reset();
            } else {
                motors.coast_stop();
            }
            ladrc.feedback_applied_steer(0.0);
            est_forward_speed = 0.0;
            None
        } else {
            let app = motors.apply(&drive_cmd);
            // Feed ACTUAL post-limiter steering back into LESO => exact bumpless anti-windup!
            ladrc.feedback_applied_steer(app.applied_steer);
            est_forward_speed = app.est_forward_speed;
            Some(app)
        };

        // E. Optional Throttled Serial Telemetry (10 Hz)
        tick_counter = tick_counter.wrapping_add(1);
        if ENABLE_TELEMETRY && tick_counter.is_multiple_of(telemetry_ticks) {
            let r = sensors.last_raw();
            let w = sensors.weights();
            let state_tag = if planner.is_in_shortcut() {
                "SHRT"
            } else {
                match line_state {
                    LineState::Tracking { .. } => "TRK",
                    LineState::Crossing { .. } => "CRS",
                    LineState::CenterStraddle { .. } => "STR",
                    LineState::Lost { .. } => "LST",
                    LineState::Lifted => "AIR",
                }
            };
            let (al, ar) = match applied {
                Some(a) => (a.left_pwm as i32, a.right_pwm as i32),
                None => (0, 0),
            };
            println!(
                "[{}#{}] raw=[{},{},{},{}] w=[{},{},{},{}] L={} R={} v={}",
                state_tag,
                planner.cross_count(),
                r[0],
                r[1],
                r[2],
                r[3],
                (w[0] * 100.0) as i32,
                (w[1] * 100.0) as i32,
                (w[2] * 100.0) as i32,
                (w[3] * 100.0) as i32,
                al,
                ar,
                est_forward_speed as i32
            );
        }

        // F. Exact Fixed-Timestep Pace (2000 us = 500 Hz)
        while tick_start.elapsed() < Duration::from_micros(LOOP_PERIOD_US) {}
    }
}
