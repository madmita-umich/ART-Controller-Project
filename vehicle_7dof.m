function [dx, out] = vehicle_7dof(t, x, u, veh, tire)
%VEHICLE_7DOF  Rigid-body 7-DOF vehicle model with Pacejka tires.
%
%
%   [dx, out] = vehicle_7dof(t, x, u, veh, tire)
%
%   State x (10x1):
%     x(1)    Vx      longitudinal velocity, body frame [m/s]
%     x(2)    Vy      lateral velocity, body frame [m/s] (+ right, SAE)
%     x(3)    r       yaw rate [rad/s] (+ clockwise seen from above, SAE)
%     x(4:7)  omega   wheel spin rates [FL FR RL RR] [rad/s]
%     x(8)    X       global position [m]
%     x(9)    Y       global position [m]
%     x(10)   psi     heading [rad]
%   States 1-7 are the 7 degrees of freedom; 8-10 are kinematic only.
%
%   Input u (8x1), or a function handle u(t, x) returning it:
%     u(1:4)  road-wheel steer angles [FL FR RL RR] [rad]
%             (rear entries are the active rear toe commands)
%     u(5:8)  drive(+)/brake(-) torque at each wheel [N m]
%
%   Output struct out: Fz, alpha, kappa, Fx/Fy (tire frame), Fx_b/Fy_b
%   (body frame), ax, ay (accelerometer, body frame), beta, Mz.
%
%   Usage with ode45:
%     load vehicle_params.mat; load tire_params.mat
%     V0 = 20; x0 = [V0; 0; 0; V0/veh.Rw*ones(4,1); 0; 0; 0];
%     ufun = @(t,x) [deg2rad(2)*[1;1]; 0; 0; zeros(4,1)];
%     [T, X] = ode45(@(t,x) vehicle_7dof(t, x, ufun, veh, tire), [0 5], x0);

    if isa(u, 'function_handle'), u = u(t, x); end
    u = u(:);

    Vx  = x(1);  Vy = x(2);  r = x(3);
    w   = x(4:7);
    psi = x(10);
    delta = u(1:4);
    Tw    = u(5:8);

    m = veh.m;  g = veh.g;  a = veh.a;  b = veh.b;  L = veh.L;
    H = veh.h;  T = veh.t;  d = T/2;

    % ---------------- Wheel positions relative to CG (SAE: y right) --------
    xw = [ a;  a; -b; -b];
    yw = [-d;  d; -d;  d];

    % ---------------- Aerodynamics (Eqs. 15-16) ----------------
    Fdrag = 0.5*veh.rho*veh.Af*veh.Cd*Vx*abs(Vx);
    Flift = 0.5*veh.rho*veh.Af*veh.Cl*Vx^2;

    % ---------------- Per-wheel velocities (Eq. 17) ----------------
    Vxw = Vx - r.*yw;
    Vyw = Vy + r.*xw;
    % rotate into each tire frame
    v_long =  Vxw.*cos(delta) + Vyw.*sin(delta);
    v_lat  = -Vxw.*sin(delta) + Vyw.*cos(delta);

    % ---------------- Slip angle and slip ratio (Eqs. 18-19) --------------
    alpha = -atan2(v_lat, max(abs(v_long), 0.1));
    vref  = max(abs(v_long), 1.0);        % low-speed regularisation
    kappa = (veh.Rw.*w - v_long)./vref;

    % ---------------- Normal loads (Eqs. 11-14) ----------------
    % Load transfer depends on Ax, Ay, which depend on the tire forces.
    % The thesis resolves this with a one-step delay in Simulink; here a
    % short fixed-point iteration is used instead.
    Ax = 0;  Ay = 0;
    for k = 1:3
        lon = [b/L - (H/L)*Ax/g; b/L - (H/L)*Ax/g; a/L + (H/L)*Ax/g; a/L + (H/L)*Ax/g];
        lat = [d/T + (H/T)*Ay/g; d/T - (H/T)*Ay/g; d/T + (H/T)*Ay/g; d/T - (H/T)*Ay/g];
        Fz  = lon.*lat.*m*g;
        Fz  = Fz - Flift.*[b; b; a; a]/(2*L);     % lift split by axle
        Fz  = max(Fz, 0);

        [Fx, Fy] = pacejka_tire(kappa, alpha, Fz, tire);

        % ---------------- Tire frame to body frame (Eq. 24) ----------------
        Fx_b = Fx.*cos(delta) - Fy.*sin(delta);
        Fy_b = Fx.*sin(delta) + Fy.*cos(delta);

        Ax = (sum(Fx_b) - Fdrag)/m;     % accelerometer reading, body x
        Ay =  sum(Fy_b)/m;              % accelerometer reading, body y
    end

    % ---------------- Body equations of motion (Eq. 25) ----------------
    Mz   = sum(xw.*Fy_b - yw.*Fx_b);
    dVx  = Ax + Vy*r;
    dVy  = Ay - Vx*r;
    dr   = Mz/veh.Iz;

    % ---------------- Wheel spin dynamics (Eq. 23 + torque balance) -------
    Trr   = Fz.*veh.mu_rr.*veh.Rw.*tanh(w/0.5);   % smooth sign(omega)
    Tloss = veh.c_brg.*w;                         % bearing / half-shaft loss
    dw    = (Tw - veh.Rw.*Fx - Trr - Tloss)./veh.Iw;

    % ---------------- Global position ----------------
    dX   = Vx*cos(psi) - Vy*sin(psi);
    dY   = Vx*sin(psi) + Vy*cos(psi);
    dpsi = r;

    dx = [dVx; dVy; dr; dw; dX; dY; dpsi];

    if nargout > 1
        out = struct('Fz', Fz, 'alpha', alpha, 'kappa', kappa, ...
                     'Fx', Fx, 'Fy', Fy, 'Fx_b', Fx_b, 'Fy_b', Fy_b, ...
                     'ax', Ax, 'ay', Ay, 'Mz', Mz, ...
                     'beta', atan2(Vy, max(abs(Vx), 0.1)));
    end
end
