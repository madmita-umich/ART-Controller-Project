function [delta_rl, delta_rr] = ackermann_rear_splitter(delta_f, delta_r, L, tr)
% ACKERMANN_REAR_SPLITTER Computes individual left/right rear wheel steer angles.
% Inputs:
%   delta_f : Front steer angle [rad]
%   delta_r : Central rear steer angle command [rad]
%   L       : Wheelbase [m]
%   tr      : Rear track width [m]
% Outputs:
%   delta_rl: Rear-left wheel steer angle [rad]
%   delta_rr: Rear-right wheel steer angle [rad]

    % Threshold for straight-line driving / zero steer to avoid division by zero
    eps_steer = 1e-5; 

    if abs(delta_f) < eps_steer && abs(delta_r) < eps_steer
        delta_rl = 0;
        delta_rr = 0;
        return;
    elseif abs(delta_f) < eps_steer
        % Parallel rear steering when front steer is zero
        delta_rl = delta_r;
        delta_rr = delta_r;
        return;
    end

    % Determine steering sign convention (Kaufman 2017)
    if delta_f > 0
        sgn = -1;
    else
        sgn = 1;
    end

    % Kinematic turning center calculations
    tan_r = tan(sgn * delta_r);
    tan_f = tan(sgn * delta_f);
    
    denom = tan_r - tan_f;
    if abs(denom) < eps_steer
        % Parallel condition
        delta_rl = delta_r;
        delta_rr = delta_r;
        return;
    end

    X = L / denom;
    Y = tan_r * X;

    % Individual rear wheel steer angles
    delta_rl = -atan(Y / (X - sgn * (tr / 2)));
    delta_rr = -atan(Y / (X + sgn * (tr / 2)));
end