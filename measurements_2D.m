function [r, l] = measurements_2D(p, R, env, D_B)
% Measurements of Obstacles
%
% This function calculates measurements related to obstacles in the
% environment based on the position of a mobile robot and the orientation
% of its coordinate frame.
%
% Input:
%   p     - A vector [x, y] ∈ ℝ² representing the position of the mobile
%           robot in the world coordinate frame.
%   R     - A 2x2 rotation matrix ℝ^{2x2} that transforms coordinates from
%           the body-fixed frame to the world frame.
%   env   - A structure containing:
%       obs  - A 2D logical matrix (h,w) indicating obstacle presence:
%              obs(h,w) == true  indicates an obstacle at the position
%              (-h, w)/N + p_e (in the world frame).
%              obs(h,w) == false indicates no obstacle at that position.
%       p_e  - A vector [x_e, y_e] ∈ ℝ² representing the origin of obs in
%              the world coordinate frame.
%       N    - A scalar that represents the ratio of pixels to meters
%              (N pixels/m).
%
% Coordinate System Illustration:
%
%        ↑ [p]_2 (m)
%        |
%        |        The pixel coordinate frame
% [p_e]_2|————————+———————————+———————————+——→ w (pixel)
%   (m)  |        |           |           |
%        |        |           |           |
%        |        |           |           |
%        |        +———————————·obs(h,w)   | N pixels/m
%        |        |                       |
%        |        |                       |
%        |        +———————————————————————+---
%        |        ↓ h (pixel)
%        |        |
%        +————————————————————————————————→ [p]_1 (m)
%               [p_e]_1 (m)
%               The world coordinate frame
%
% Output:
%   r     - A row vector r ∈ ℝ_+^{1xN} representing the measurement distances to
%           detected obstacles.
%   l     - A 2D row vector l ∈ ℝ^{2xN} indicating the measurement directions
%           towards detected obstacles.

persistent N theta l_;

if isempty(l_)
    % Number of sensor lines
    N     = 360;
    % Generate angles for sensor directions (0 to 2π), N directions total
    theta = linspace(0, 2*pi, N+1);
    theta = theta(1:end-1); % Remove duplicate endpoint

    % Measurement directions in the body-fixed coordinate frame
    l_ = [cos(theta); sin(theta)];
end

l = l_;
yaw = atan2(R(2, 1), R(1, 1)); % Robot's orientation (yaw angle)

% Calculate measurement directions in world coordinates
l_w = [cos(theta + yaw); sin(theta + yaw)];

% Initialize all measurement distances to maximum range (D_B)
r = D_B * ones(1, N);

% Convert robot position to pixel coordinates
p_hw = ([0, -1; 1, 0] * (p - env.p_e)) * env.N;

[h_obs, w_obs] = size(env.obs); % Get dimensions of obstacle matrix

% Check if the robot's pixel position is within obstacle matrix bounds
is_p_hw_in_obs = is_in_rec(p_hw, [1; 1], [h_obs; w_obs]);

for i = 1:N
    % Measurement direction in pixel coordinates
    l_pix = [-l_w(2, i); l_w(1, i)];

    % Find the indices of points along the direction vector in the environment
    p_check = check_range(p_hw, l_pix, [h_obs; w_obs], is_p_hw_in_obs, D_B*env.N);

    % If no points are found along the direction, set max range
    if isempty(p_check)
        r(i) = D_B;
        continue;
    else
        % Check if any of these points are obstacles
        index = find(env.obs(p_check) == true);
        if isempty(index)
            r(i) = D_B; % No obstacle found, set to max range
        else
            % Convert obstacle indices to row, column and compute distance
            [row, col] = ind2sub([h_obs, w_obs], p_check(index(1)));
            r(i) = min(norm(([row; col] - p_hw)) / env.N, D_B);
        end
    end
end

end

function flag = is_in_rec(p, range_min, range_max)
% Determines if a point is within a specified rectangular range
flag = all(p >= range_min & p <= range_max);
end

function p_check = check_range(p_hw, l_pix, size_hw, is_p_hw_in_obs, max_pixels_len)
% Finds the pixel points along the specified direction vector l_pix,
% starting from p_hw and continuing within the boundaries defined by size_hw.
%
% Inputs:
%   p_hw           - Start position in pixel coordinates
%   l_pix          - Direction vector in pixel coordinates
%   size_hw        - Size of the environment in pixels
%   is_p_hw_in_obs - Boolean, indicating if p_hw is within obstacle matrix
%
% Output:
%   p_check - Array of linear indices of points along the direction vector

A = [1, 0; 1, 0; 0, 1; 0, 1];
b = [1; size_hw(1); 1; size_hw(2)];

% Calculate parameter t for each boundary
t = ((b - A * p_hw) ./ round(A * l_pix*10e10) * 10e10)';
% Calculate points along the direction vector
p = round(p_hw + l_pix .* t);

% Filter points that are within the environment and in the forward direction
mask = is_in_rec(p, [1; 1], size_hw);
mask = mask & t >= 0 & isfinite(t);
p    = p(:, mask);

% Sort points by distance to origin
[~, idx] = sort(vecnorm(p - p_hw));

% Define the line segment based on obstacle location
if is_p_hw_in_obs
    start_hw = p_hw;
    end_hw   = p(:, idx(1));
elseif ~isempty(p)
    start_hw = p(:, idx(1));
    end_hw   = p(:, idx(end));
else
    p_check = zeros(1, 0);
    return;
end

if(norm(start_hw-p_hw) > max_pixels_len)
    p_check = zeros(1, 0);
    return;
end

% Generate linearly spaced points between start and end
start_hw = p_hw + (start_hw-p_hw) * min(1, max_pixels_len/norm(start_hw-p_hw));
end_hw   = p_hw + (  end_hw-p_hw) * min(1, max_pixels_len/norm(  end_hw-p_hw));
len = ceil(norm(end_hw - start_hw));
h = round(linspace(start_hw(1), end_hw(1), len));
w = round(linspace(start_hw(2), end_hw(2), len));

% improfile

% Convert (h, w) points to linear indices in obstacle matrix
p_check = sub2ind(size_hw, h, w);
end
