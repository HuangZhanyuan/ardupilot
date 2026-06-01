#include "mode.h"
#include "Plane.h"

bool ModeRollPID::_enter()
{
    reset_pid();
    reset_controllers();
    return true;
}

void ModeRollPID::update()
{
    const float target_deg = constrain_float(plane.g2.roll_tgt.get(), -180.0f, 180.0f);
    plane.nav_roll_cd = int32_t(target_deg * 100.0f);
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

void ModeRollPID::run()
{
    update_pid();

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 0);
    output_rudder_and_steering(0);
    output_pilot_throttle();
}

bool ModeRollPID::use_throttle_limits() const
{
    return !plane.g.throttle_passthru_stabilize;
}

void ModeRollPID::apply_servo_outputs() const
{
    SRV_Channels::set_output_pwm_chan(0, surface1_pwm);
    SRV_Channels::set_output_pwm_chan(1, surface2_pwm);
}

void ModeRollPID::reset_pid()
{
    integrator_deg_s = 0;
    last_error_deg = 0;
    have_last_error = false;
    surface1_pwm = uint16_t(constrain_float(plane.g2.surf1_mid.get(), 800.0f, 2200.0f));
    surface2_pwm = uint16_t(constrain_float(plane.g2.surf2_mid.get(), 800.0f, 2200.0f));
}

void ModeRollPID::update_pid()
{
    float dt = plane.G_Dt;
    if (!is_positive(dt) || dt > 0.5f) {
        dt = 0.02f;
    }

    const float target_deg = constrain_float(plane.g2.roll_tgt.get(), -180.0f, 180.0f);
    const float actual_deg = ahrs.roll_sensor * 0.01f;
    const float error_deg = wrap_180(target_deg - actual_deg);

    if (!have_last_error) {
        last_error_deg = error_deg;
        have_last_error = true;
    }

    const float kp = plane.g2.roll_p.get();
    const float ki = plane.g2.roll_i.get();
    const float kd = plane.g2.roll_d.get();

    integrator_deg_s += error_deg * dt;

    float i_term = 0;
    if (is_zero(ki)) {
        integrator_deg_s = 0;
    } else {
        i_term = constrain_float(ki * integrator_deg_s, -500.0f, 500.0f);
        integrator_deg_s = i_term / ki;
    }

    const float derivative_deg_s = (error_deg - last_error_deg) / dt;
    last_error_deg = error_deg;

    const float correction_pwm = constrain_float((kp * error_deg) + i_term + (kd * derivative_deg_s),
                                                 -500.0f,
                                                  500.0f);

    const float mid1 = constrain_float(plane.g2.surf1_mid.get(), 800.0f, 2200.0f);
    const float mid2 = constrain_float(plane.g2.surf2_mid.get(), 800.0f, 2200.0f);

    surface1_pwm = uint16_t(constrain_float(mid1 + correction_pwm, 800.0f, 2200.0f) + 0.5f);
    surface2_pwm = uint16_t(constrain_float(mid2 - correction_pwm, 800.0f, 2200.0f) + 0.5f);
}
