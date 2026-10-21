function [Fx, Fy] = pacejka96_combined_slip(alpha, kappa, Fz, mu)
% PACEJKA96_COMBINED_SLIP Pacejka '96 combined-slip tire formulation.

    if Fz <= 0
        Fx = 0; Fy = 0;
        return;
    end

    Fz0 = 4000;
    dfz = (Fz - Fz0) / Fz0;

    % Pure Lateral Force (Fy0)
    Cy = 1.30;
    Dy = mu * Fz;
    By = 10.0 / (mu * (1 + dfz));
    Ey = -1.0;
    Fy0 = Dy * sin(Cy * atan(By * alpha - Ey * (By * alpha - atan(By * alpha))));

    % Pure Longitudinal Force (Fx0)
    Cx = 1.65;
    Dx = mu * Fz;
    Bx = 12.0 / (mu * (1 + dfz));
    Ex = -0.5;
    Fx0 = Dx * sin(Cx * atan(Bx * kappa - Ex * (Bx * kappa - atan(Bx * kappa))));

    % Combined Weighting Functions (Gyk & Gxa)
    Cyk = 1.0; Byk = 8.0; Eyk = 0.0;
    Gyk = cos(Cyk * atan(Byk * kappa - Eyk * (Byk * kappa - atan(Byk * kappa))));

    Cxa = 1.0; Bxa = 8.0; Exa = 0.0;
    Gxa = cos(Cxa * atan(Bxa * alpha - Exa * (Bxa * alpha - atan(Bxa * alpha))));

    % Combined Outputs
    Fy = Fy0 * Gyk;
    Fx = Fx0 * Gxa;
end