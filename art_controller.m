function [delta_r_cmd, state_out] = art_controller(ay_des, ay_act, Vx, delta_f, dt, state_in, flags, veh)
% ART_CONTROLLER Robust Feedforward-Augmented PI Controller with Saturation Fading.

    %% 1. Speed Grid & Scheduled Gain Tables
    Vx_grid  = [10, 20, 30, 40];
    Kp_table = [0.015, 0.008, 0.004, 0.002];
    Ki_table = [0.050, 0.025, 0.012, 0.006];
    Kb       = 1.0;

    if flags.GS_on
        Kp = interp1(Vx_grid, Kp_table, max(Vx, 5), 'linear', 'extrap');
        Ki = interp1(Vx_grid, Ki_table, max(Vx, 5), 'linear', 'extrap');
    else
        Kp = Kp_table(2);
        Ki = Ki_table(2);
    end

    %% 2. Reference Filtering & Error Clamping
    if ~isfield(state_in, 'ay_des_filt') || isempty(state_in.ay_des_filt)
        state_in.ay_des_filt = ay_des;
    end

    % Low-pass filter reference target to match chassis yaw inertia lag
    tau_ref = 0.10; % [s]
    ay_des_smooth = state_in.ay_des_filt + (dt / (tau_ref + dt)) * (ay_des - state_in.ay_des_filt);
    
    e_raw = ay_des_smooth - ay_act;
    
    % Limit maximum error magnitude during rapid steer reversals (prevents spikes)
    max_e = 3.0; % [m/s^2]
    e = max(min(e_raw, max_e), -max_e);

    %% 3. Inverse-Bicycle Feedforward Term
    if flags.FF_on
        Kus_base = (veh.m / veh.L) * ((veh.b / 80000) - (veh.a / 80000));
        Vx_safe  = max(Vx, 3.0);
        delta_r_ff = delta_f - ay_des_smooth * ((veh.L + Kus_base * (Vx_safe^2 / veh.g)) / (Vx_safe^2));
    else
        delta_r_ff = 0;
    end

    %% 4. Friction Boundary Intervention Fading
    % Fade out active rear steer when approaching tire grip limits (ay >= 7.5 m/s^2)
    mu_peak = 1.0;
    ay_limit_threshold = 0.75 * mu_peak * veh.g; % ~7.35 m/s^2
    fade_factor = max(0, min(1, (mu_peak * veh.g - abs(ay_act)) / (mu_peak * veh.g - ay_limit_threshold)));

    u_pi = (Kp * e + state_in.xi) * fade_factor;
    delta_r_unsat = (delta_r_ff * fade_factor) - u_pi;

    % Suppress steering when straight-line coasting
    if abs(delta_f) < deg2rad(0.1) && abs(ay_des) < 0.05
        delta_r_unsat = 0;
        state_in.xi = 0;
    end

    pos_lim = deg2rad(5);
    delta_r_sat = max(min(delta_r_unsat, pos_lim), -pos_lim);

    %% 5. Anti-Windup Integrator Update
    delta_u = delta_r_sat - delta_r_unsat;

    if flags.AW_on
        dxi_dt = Ki * e - Kb * delta_u;
    else
        dxi_dt = Ki * e;
    end

    % Persistent State Updates
    state_out.xi          = state_in.xi + dxi_dt * dt;
    state_out.e_prev      = e;
    state_out.ay_des_filt = ay_des_smooth;

    delta_r_cmd = delta_r_sat;
end
% ART_CONTROLLER Feedforward-Augmented Gain-Scheduled PID with Anti-Windup.
%
% Inputs:
%   ay_des   : Target lateral acceleration [m/s^2]
%   ay_act   : Measured lateral acceleration [m/s^2]
%   Vx       : Current longitudinal vehicle speed [m/s]
%   delta_f  : Current front steer angle [rad]
%   dt       : Simulation time step [s]
%   state_in : Struct holding internal states: state_in.xi (integrator), state_in.e_prev
%   flags    : Struct holding ablation toggles: flags.FF_on, flags.GS_on, flags.AW_on
%   veh      : Struct holding vehicle physical parameters
%
% Outputs:
%   delta_r_cmd : Total commanded central rear steer angle [rad]
%   state_out   : Updated internal controller state struct

