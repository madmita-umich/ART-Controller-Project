function [Fx, Fy, info] = pacejka_tire(kappa, alpha, Fz, tire)
%PACEJKA_TIRE  Magic Formula tire forces with load dependence and combined slip.
%
%   [Fx, Fy] = pacejka_tire(kappa, alpha, Fz, tire)
%
%   Inputs (scalars or equal-size arrays, one entry per tire):
%     kappa  longitudinal slip ratio [-]   (positive = driving)
%     alpha  slip angle [rad]              (positive gives positive Fy)
%     Fz     normal load [N]               (forces are zero for Fz <= 0)
%     tire   parameter struct (load from tire_params.mat)
%
%   Outputs, in the tire frame:
%     Fx     longitudinal force [N]
%     Fy     lateral force [N]
%     info   struct with pure-slip forces and the MF coefficients B, C, D
%
%   Pure slip:  F0 = D*sin(C*atan(B*x - E*(B*x - atan(B*x))))
%     Lateral:      D = mu_y(Fz)*Fz,  K_y = a3*sin(2*atan(Fz/a4)),  B = K_y/(C*D)
%     Longitudinal: D = mu_x(Fz)*Fz,  K_x = CKx*Fz,                 B = K_x/(C*D)
%     Friction load sensitivity: mu = mu0*(1 + pD2*(Fz - Fz0)/Fz0)
%
%   Combined slip (cosine weighting):
%     Fx = Fx0*cos(Cxa*atan(Bxa*alpha)),   Fy = Fy0*cos(Cyk*atan(Byk*kappa))
%
%   Reference: H. B. Pacejka, Tire and Vehicle Dynamics, 3rd ed., 2012.

    Fz  = max(Fz, 0);                 % a lifted wheel carries no load
    Fzs = max(Fz, 1);                 % avoids division by zero in B
    dfz = (Fzs - tire.Fz0) ./ tire.Fz0;

    % ---------------- Lateral, pure slip ----------------
    Dy = tire.muy0 .* (1 + tire.pDy2 .* dfz) .* Fzs;
    Ky = tire.a3 .* sin(2 .* atan(Fzs ./ tire.a4));
    By = Ky ./ (tire.Cy .* Dy);
    xy = By .* alpha;
    Fy0 = Dy .* sin(tire.Cy .* atan(xy - tire.Ey .* (xy - atan(xy))));

    % ---------------- Longitudinal, pure slip ----------------
    Dx = tire.mux0 .* (1 + tire.pDx2 .* dfz) .* Fzs;
    Kx = tire.CKx .* Fzs;
    Bx = Kx ./ (tire.Cx .* Dx);
    xx = Bx .* kappa;
    Fx0 = Dx .* sin(tire.Cx .* atan(xx - tire.Ex .* (xx - atan(xx))));

    % ---------------- Combined slip ----------------
    Gxa = cos(tire.Cxa .* atan(tire.Bxa .* alpha));
    Gyk = cos(tire.Cyk .* atan(tire.Byk .* kappa));
    Fx = Fx0 .* Gxa;
    Fy = Fy0 .* Gyk;

    % zero force on unloaded tires
    Fx(Fz <= 0) = 0;
    Fy(Fz <= 0) = 0;

    if nargout > 2
        info = struct('Fx0', Fx0, 'Fy0', Fy0, 'Bx', Bx, 'Dx', Dx, ...
                      'By', By, 'Dy', Dy, 'Ky', Ky, 'Kx', Kx);
    end
end
