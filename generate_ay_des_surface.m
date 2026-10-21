function [Vx_grid, delta_f_grid, AY_des] = generate_ay_des_surface()
% GENERATE_AY_DES_SURFACE Builds target lateral acceleration lookup grid.

    L = 2.70;                     % Wheelbase [m]
    g = 9.81;                     % Gravity [m/s^2]
    mu_max = 1.0;                 % Road friction cap
    Kus_des_deg = 2.5;            % Desired understeer gradient [deg/g]
    Kus_des = deg2rad(Kus_des_deg) / g;

    Vx_grid = 2:2:40;             % Velocity grid [m/s]
    delta_f_grid = deg2rad(0:0.5:15); % Steering grid [rad]

    [VX, DELTA_F] = meshgrid(Vx_grid, delta_f_grid);

    % Linear handling formulation
    AY_raw = (VX.^2 ./ (L + Kus_des .* VX.^2)) .* DELTA_F;

    % Physical friction limit cap (|Ay| <= mu*g)
    AY_des = sign(AY_raw) .* min(abs(AY_raw), mu_max * g);
end