function [delta_r_out, ddelta_r_out] = simulate_actuator(t_vec, cmd_vec, act, reset_flag)
% SIMULATE_ACTUATOR Numerically integrates 2nd-order actuator dynamics in MATLAB.
%
% Supports:
%   1. Batch Mode      : Pass full time vectors t_vec (1xN) and cmd_vec (1xN).
%   2. Single-Step Mode: Pass t_vec = [0 dt] and cmd_vec = [cmd cmd].
%                        State is preserved using persistent internal variables.
%   3. State Reset     : Call simulate_actuator([], [], [], true) at test start.

    persistent x1 x2 cmd_buffer

    % Explicit state reset check
    if nargin >= 4 && islogical(reset_flag) && reset_flag
        x1 = []; x2 = []; cmd_buffer = [];
        delta_r_out = 0; ddelta_r_out = 0;
        return;
    end

    dt = t_vec(2) - t_vec(1);
    N  = length(t_vec);

    % --- BATCH MODE (Full vectors passed) ---
    if N > 2
        delta_r_out  = zeros(1, N);
        ddelta_r_out = zeros(1, N);
        
        delay_steps = max(1, round(act.tau / dt));
        buf = zeros(1, delay_steps);
        
        state_x1 = 0;
        state_x2 = 0;
        
        for i = 1:N
            buf = [cmd_vec(i), buf(1:end-1)];
            delayed_cmd = buf(end);
            
            x2_dot = act.wn^2 * (delayed_cmd - state_x1) - 2 * act.zeta * act.wn * state_x2;
            state_x2 = state_x2 + x2_dot * dt;
            state_x2 = max(min(state_x2, act.rate_lim), -act.rate_lim);
            
            state_x1 = state_x1 + state_x2 * dt;
            if state_x1 >= act.pos_lim
                state_x1 = act.pos_lim;
                state_x2 = min(state_x2, 0);
            elseif state_x1 <= -act.pos_lim
                state_x1 = -act.pos_lim;
                state_x2 = max(state_x2, 0);
            end
            
            delta_r_out(i)  = state_x1;
            ddelta_r_out(i) = state_x2;
        end
        return;
    end

    % --- SINGLE-STEP MODE (Step-by-step loop execution) ---
    if isempty(x1)
        x1 = 0;
        x2 = 0;
        delay_steps = max(1, round(act.tau / dt));
        cmd_buffer  = zeros(1, delay_steps);
    end

    cmd = cmd_vec(end);

    % Shift buffer for transport delay
    cmd_buffer  = [cmd, cmd_buffer(1:end-1)];
    delayed_cmd = cmd_buffer(end);

    % 2nd-Order Differential Equation
    x2_dot = act.wn^2 * (delayed_cmd - x1) - 2 * act.zeta * act.wn * x2;

    % Euler integration
    x2 = x2 + x2_dot * dt;
    x2 = max(min(x2, act.rate_lim), -act.rate_lim); % Rate limiting

    x1 = x1 + x2 * dt;
    if x1 >= act.pos_lim % Position saturation
        x1 = act.pos_lim;
        x2 = min(x2, 0);
    elseif x1 <= -act.pos_lim
        x1 = -act.pos_lim;
        x2 = max(x2, 0);
    end

    delta_r_out  = x1;
    ddelta_r_out = x2;
end