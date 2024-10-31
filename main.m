clear all; clc; close all;

% The maximum ranging radius, meter
D_B   = 2.0;


% The environments
env.obs  = ~imread('map01.bmp');
env.N    = 100;
env.p_e  = [-5; -5] + [0; size(env.obs, 1) / env.N];

% Graphics
figure(1); 
hold on; axis equal; box on; grid on; % axis off;
range = [env.p_e, env.p_e + [0, 1; -1, 0] * size(env.obs)'/env.N];
axis([min(range(1, :))-1, max(range(1, :))+1, min(range(2, :))-1, max(range(2, :))+1]);

Handle_obs_map = patch_img(env.obs, env.p_e, env.N);
Handle_p = animatedline('Color','k');

N = 100;
for i = 1:N
    

    % states
    p = 2*[cos(2*pi*i/N); sin(2*pi*i/N)];
    theta = 4*pi*i/N;
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

    % Feedback
    [r, l] = measurements_2D(p, R, env, D_B);
    p_obs  = p + R*l.*r;
    
    % Update the graphic
    if(exist('Handle_obs', 'var') == 0)
        Handle_obs     = patch(p_obs(1,:), p_obs(2,:), -ones(1, size(p_obs,2)), 'b', 'facealpha', 0.1);
    else
        Handle_obs.XData = p_obs(1, :);
        Handle_obs.YData = p_obs(2, :);
    end
    addpoints(Handle_p, p(1), p(2));

    pause(0.05);
end

%{
    Draw the map
    pic         -- map
    p_m         -- the origin of the map
    resolution  -- specific pix per meter
%}
function h = patch_img(obs, p_m, resolution)
x =  (1:size(obs, 2))/resolution + p_m(1);  %  w/N + p_e(1)
y = -(1:size(obs, 1))/resolution + p_m(2);  % -h/N + p_e(2)


[X, Y] = meshgrid(x, y);

Z       = double(obs);
Z(Z<=0) = nan;
Z(Z> 0) =  -1;  % true means obstacle

h = surf('XData',X,'YData',Y,'ZData',Z, 'EdgeColor', 'none', 'displayname', 'obstacle', 'HandleVisibility', 'off');
colorres = [ones(1,3); 0,0,0];
colormap(colorres);

end