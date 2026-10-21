
% PURE_PURSUIT_DRIVER Closed-loop geometric path tracking controller.
%
% Inputs:
%   X, Y    : Current vehicle CG global coordinates [m]
%   psi     : Current vehicle heading angle [rad]
%   Vx      : Current longitudinal velocity [m/s]
%   path_X  : Vector of reference path X-coordinates [m]
%   path_Y  : Vector of reference path Y-coordinates [m]
%   veh     : Struct holding vehicle physical parameters
%
% Outputs:
%   delta_f      : Commanded front steering angle [rad]
%   lookahead_pt : [X_target, Y_target] coordinates of selected lookahead point

function [delta_f, lookahead_pt] = pure_pursuit_driver(X, Y, psi, Vx, path_X, path_Y, veh)
% PURE_PURSUIT_DRIVER Tuned closed-loop path tracking for ISO 3888-2 DLC.

    %% 1. Dynamic Lookahead Distance
    klh     = 0.40;      % Lookahead gain (~8.8 m lookahead at 22.2 m/s)
    Llh_min = 6.0;       % Minimum lookahead distance [m]
    Llh_max = 18.0;      % Maximum lookahead distance [m]
    
    Llh = max(Llh_min, min(Llh_max, klh * max(Vx, 1.0)));

    %% 2. Target Point Selection
    dx = path_X - X;
    dy = path_Y - Y;
    dist_sq = dx.^2 + dy.^2;

    [~, min_idx] = min(dist_sq);

    target_idx = min_idx;
    for k = min_idx:length(path_X)
        d_val = sqrt((path_X(k) - X)^2 + (path_Y(k) - Y)^2);
        if d_val >= Llh
            target_idx = k;
            break;
        end
    end

    X_target = path_X(target_idx);
    Y_target = path_Y(target_idx);
    lookahead_pt = [X_target, Y_target];

    %% 3. Steering Command Calculation
    dx_global = X_target - X;
    dy_global = Y_target - Y;

    x_local =  cos(psi) * dx_global + sin(psi) * dy_global;
    y_local = -sin(psi) * dx_global + cos(psi) * dy_global;

    d_actual = max(sqrt(x_local^2 + y_local^2), 1.0);
    gamma = (2 * y_local) / (d_actual^2);

    delta_f_raw = atan(gamma * veh.L);

    steer_lim = deg2rad(25); % Maximum front steer angle
    delta_f = max(min(delta_f_raw, steer_lim), -steer_lim);
end