function [path_X, path_Y] = generate_iso3888_2_path(dx_resolution)
% GENERATE_ISO3888_2_PATH Standard ISO 3888-2 Double Lane Change reference profile.
%
% Inputs:
%   dx_resolution : Spatial grid resolution [m] (default = 0.1m)
%
% Outputs:
%   path_X : Vector of reference X-coordinates along test track [m]
%   path_Y : Vector of reference Y-coordinates (lateral displacement) [m]

    if nargin < 1
        dx_resolution = 0.1;
    end

    % ISO 3888-2 Section Boundaries [m]
    x_total = 125.0; % Total path length
    path_X  = 0:dx_resolution:x_total;

    % Section geometry parameters
    x1 = 12.0;               % Entry lane length
    x2 = x1 + 13.5;          % First lane change section
    x3 = x2 + 11.0;          % Offset lane length
    x4 = x3 + 12.5;          % Second lane change section
    
    y_offset = 3.5;          % Lateral offset [m]
    
    % Smooth logistic transition widths
    sigma1 = 1.8;
    sigma2 = 1.8;

    % Sigmoidal double lane change profile formulation
    term1 = 0.5 * (1 + tanh((path_X - x1) / sigma1));
    term2 = 0.5 * (1 + tanh((path_X - x3) / sigma2));

    path_Y = y_offset * (term1 - term2);
end