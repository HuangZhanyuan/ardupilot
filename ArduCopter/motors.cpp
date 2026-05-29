#include "Copter.h"
#include <AP_HAL/GPIO.h>

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint32_t auto_disarm_begin;

bool Copter::read_s11_gpio(bool &high)
{
    AP_HAL::GPIO *gpio = hal.gpio;
    if (gpio == nullptr) {
        return false;
    }

    if (!s11_auto.gpio_initialised) {
        gpio->pinMode(s11_input_pin, HAL_GPIO_INPUT);
        s11_auto.gpio_initialised = true;
    }

    high = gpio->read(s11_input_pin);
    return true;
}

void Copter::update_s11_auto()
{
    bool s11_high;
    if (!read_s11_gpio(s11_high)) {
        return;
    }

    const uint32_t now_ms = millis();

    if (!s11_auto.last_level_valid) {
        s11_auto.last_level_valid = true;
        s11_auto.last_level_high = s11_high;
        return;
    }

    if (s11_high != s11_auto.last_level_high) {
        s11_auto.last_level_high = s11_high;

        if (s11_high) {
            s11_auto.pending_rise = true;
            s11_auto.rise_start_ms = now_ms;
            s10_pitch_track.high_transition_valid = true;
            s10_pitch_track.high_transition_ms = now_ms;
        } else {
            s11_auto.pending_rise = false;
            s11_auto.arm_ready = false;
            s11_auto.servo_output_valid = true;
            s11_auto.servo_output_high = false;

            if (motors->armed() && !arming.disarm(AP_Arming::Method::MAVLINK)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "S11 low: disarm denied");
            }
        }
    }

    if (s11_auto.pending_rise && s11_auto.last_level_high) {
        const float delay_s = MAX(g2.s11_delay.get(), 0.0f);
        const uint32_t delay_ms = uint32_t(delay_s * 1000.0f);
        if (now_ms - s11_auto.rise_start_ms >= delay_ms) {
            s11_auto.pending_rise = false;
            s11_auto.arm_ready = true;
            s11_auto.servo_output_valid = true;
            s11_auto.servo_output_high = true;

            if (!motors->armed() && !arming.arm(AP_Arming::Method::AUXSWITCH)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "S11 high: arm failed");
            }
        }
    }
}

void Copter::apply_s11_servo_outputs()
{
    if (!s11_auto.servo_output_valid) {
        return;
    }

    const uint8_t channels[] = {
        s7_output_channel,
        s8_output_channel,
        s9_output_channel
    };
    AP_Int16 *const hi_params[] = {
        &g2.s7_hi_pwm,
        &g2.s8_hi_pwm,
        &g2.s9_hi_pwm
    };
    AP_Int16 *const lo_params[] = {
        &g2.s7_lo_pwm,
        &g2.s8_lo_pwm,
        &g2.s9_lo_pwm
    };

    for (uint8_t i = 0; i < ARRAY_SIZE(channels); i++) {
        const AP_Int16 *const pwm_param = s11_auto.servo_output_high ? hi_params[i] : lo_params[i];
        const uint16_t pwm = uint16_t(constrain_int16(pwm_param->get(), 800, 2200));
        hal.rcout->enable_ch(channels[i]);
        hal.rcout->write(channels[i], pwm);
    }
}

bool Copter::s10_pitch_tracking_active(uint32_t now_ms) const
{
    if (!s11_auto.last_level_valid) {
        return false;
    }

    if (!s11_auto.last_level_high) {
        return true;
    }

    if (!s10_pitch_track.high_transition_valid) {
        return false;
    }

    const float hold_time_s = MAX(g2.s10_hold_time_s.get(), 0.0f);
    const uint32_t hold_time_ms = uint32_t(hold_time_s * 1000.0f);
    return now_ms - s10_pitch_track.high_transition_ms < hold_time_ms;
}

void Copter::reset_s10_pitch_tracking_controller()
{
    s10_pitch_track.integrator_pwm = 0.0f;
    s10_pitch_track.last_error_deg = 0.0f;
    s10_pitch_track.last_error_valid = false;
}

float Copter::get_s10_wrapped_pitch_deg() const
{
    const Matrix3f &rot = ahrs.get_rotation_body_to_ned();
    // Use the 3-1-2 decomposition pitch term so the commanded pitch can
    // continue past 90 degrees without the standard Euler pitch folding back.
    return wrap_360(degrees(rot.to_euler312().y));
}

void Copter::apply_s10_pitch_tracking_output()
{
    SRV_Channel *const s10_channel = SRV_Channels::srv_channel(s10_output_channel);
    if (s10_channel == nullptr) {
        return;
    }

    const uint16_t trim_pwm = s10_channel->get_trim();
    const uint16_t min_pwm = s10_channel->get_output_min();
    const uint16_t max_pwm = s10_channel->get_output_max();
    const uint32_t now_ms = millis();
    const bool controller_active = s10_pitch_tracking_active(now_ms);

    if (controller_active != s10_pitch_track.controller_was_active) {
        reset_s10_pitch_tracking_controller();
        s10_pitch_track.controller_was_active = controller_active;
    }

    uint16_t pwm = trim_pwm;
    const float dt = constrain_float(G_Dt, 0.0f, 0.5f);
    const float pitch_wrapped_deg = get_s10_wrapped_pitch_deg();

    if (controller_active) {
        const float target_wrapped_deg = wrap_360(g2.s10_pitch_target_deg.get());
        float error_deg = pitch_wrapped_deg - target_wrapped_deg;
        if (error_deg > 180.0f) {
            error_deg -= 360.0f;
        } else if (error_deg < -180.0f) {
            error_deg += 360.0f;
        }
        const float p_out = g2.s10_pitch_kp.get() * error_deg;
        const float positive_headroom = float(max_pwm) - float(trim_pwm);
        const float negative_headroom = float(trim_pwm) - float(min_pwm);

        if (dt > 0.0f) {
            s10_pitch_track.integrator_pwm += g2.s10_pitch_ki.get() * error_deg * dt;
            s10_pitch_track.integrator_pwm = constrain_float(s10_pitch_track.integrator_pwm,
                                                             -negative_headroom,
                                                             positive_headroom);
        }

        float d_out = 0.0f;
        if (s10_pitch_track.last_error_valid && dt > 0.0f) {
            d_out = g2.s10_pitch_kd.get() * (error_deg - s10_pitch_track.last_error_deg) / dt;
        }

        s10_pitch_track.last_error_deg = error_deg;
        s10_pitch_track.last_error_valid = true;

        const float output_pwm = float(trim_pwm) + p_out + s10_pitch_track.integrator_pwm + d_out;
        pwm = uint16_t(constrain_float(output_pwm, min_pwm, max_pwm));
    }

    hal.rcout->enable_ch(s10_output_channel);
    hal.rcout->write(s10_output_channel, pwm);
}

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
void Copter::arm_motors_check()
{
    static int16_t arming_counter;

    // check if arming/disarm using rudder is allowed
    AP_Arming::RudderArming arming_rudder = arming.get_rudder_arming_type();
    if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
        arming_counter = 0;
        return;
    }

#if TOY_MODE_ENABLED
    if (g2.toy_mode.enabled()) {
        // not armed with sticks in toy mode
        return;
    }
#endif

    // ensure throttle is down
    if (channel_throttle->get_control_in() > 0) {
        arming_counter = 0;
        return;
    }

    int16_t yaw_in = channel_yaw->get_control_in();

    // full right
    if (yaw_in > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if (arming_counter <= AUTO_TRIM_DELAY) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // reset arming counter if arming fail
            if (!arming.arm(AP_Arming::Method::RUDDER)) {
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors->armed() && flightmode->mode_number() == Mode::Number::STABILIZE) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim start");
            auto_trim_counter = 250;
            auto_trim_started = false;
            // ensure auto-disarm doesn't trigger immediately
            auto_disarm_begin = millis();
        }

    // full left and rudder disarming is enabled
    } else if ((yaw_in < -4000) && (arming_rudder == AP_Arming::RudderArming::ARMDISARM)) {
        if (!flightmode->has_manual_throttle() && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if (arming_counter <= DISARM_DELAY) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            arming.disarm(AP_Arming::Method::RUDDER);
        }

    // Yaw is centered so reset arming counter
    } else {
        arming_counter = 0;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
void Copter::auto_disarm_check()
{
    uint32_t tnow_ms = millis();
    uint32_t disarm_delay_ms = 1000*constrain_int16(g.disarm_delay, 0, 127);

    // exit immediately if we are already disarmed, or if auto
    // disarming is disabled
    if (!motors->armed() || disarm_delay_ms == 0 || flightmode->mode_number() == Mode::Number::THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // if the rotor is still spinning, don't initiate auto disarm
    if (motors->get_spool_state() > AP_Motors::SpoolState::GROUND_IDLE) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors->get_interlock()) || SRV_Channels::get_emergency_stop()) {
#if FRAME_CONFIG != HELI_FRAME
        // use a shorter delay if using throttle interlock switch or Emergency Stop, because it is less
        // obvious the copter is armed as the motors will not be spinning
        disarm_delay_ms /= 2;
#endif
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (flightmode->has_manual_throttle() || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = get_throttle_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        if (!thr_low || !ap.land_complete) {
            // reset timer
            auto_disarm_begin = tnow_ms;
        }
    }

    // disarm once timer expires
    if ((tnow_ms-auto_disarm_begin) >= disarm_delay_ms) {
        arming.disarm(AP_Arming::Method::DISARMDELAY);
        auto_disarm_begin = tnow_ms;
    }
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Copter::motors_output()
{
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // this is to allow the failsafe module to deliberately crash
    // the vehicle. Only used in extreme circumstances to meet the
    // OBC rules
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();
        if (!g2.afs.terminating_vehicle_via_landing()) {
            return;
        }
        // landing must continue to run the motors output
    }
#endif

    // Update arming delay state
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || flightmode->mode_number() == Mode::Number::THROW)) {
        ap.in_arming_delay = false;
    }

    update_s11_auto();

    // output any servo channels
    SRV_Channels::calc_pwm();

    auto &srv = AP::srv();

    // cork now, so that all channel outputs happen at once
    srv.cork();

    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();

    // update motors interlock state
    bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !SRV_Channels::get_emergency_stop();
    if (!motors->get_interlock() && interlock) {
        motors->set_interlock(true);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_ENABLED);
    } else if (motors->get_interlock() && !interlock) {
        motors->set_interlock(false);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_DISABLED);
    }

    if (ap.motor_test) {
        // check if we are performing the motor test
        motor_test_output();
    } else {
        // send output signals to motors
        flightmode->output_to_motors();
    }

    apply_s11_servo_outputs();
    apply_s10_pitch_tracking_output();

    // push all channels
    srv.push();
}

// check for pilot stick input to trigger lost vehicle alarm
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND)) {
        return;
    }

    // ensure throttle is down, motors not armed, pitch and roll rc at max. Note: rc1=roll rc2=pitch
    if (ap.throttle_zero && !motors->armed() && (channel_roll->get_control_in() > 4000) && (channel_pitch->get_control_in() > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs().send_text(MAV_SEVERITY_NOTICE,"Locate Copter alarm");
            }
        } else {
            soundalarm_counter++;
        }
    } else {
        soundalarm_counter = 0;
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }
    }
}
