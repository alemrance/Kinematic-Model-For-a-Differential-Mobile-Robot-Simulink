%% plot_results.m
% Plots and saves the result figures for the README.
% Requires To Workspace blocks (Timeseries format) named:
%   robotState  -> Diff Drive "state" output        (ground truth)
%   odomPose    -> encoder-only odometry Integrator (estimate 1)
%   gyroPose    -> odom_gyro Integrator             (estimate 2)

if ~exist('out', 'var')
    out = sim('mobile_robot_model');
end
if ~isprop(out, 'gyroPose') && ~any(strcmp(out.who, 'gyroPose'))
    error('gyroPose not found: add a To Workspace block (Timeseries) named gyroPose after the odom_gyro Integrator.');
end

t = out.robotState.Time;
S = toN3(out.robotState);                                   % ground truth [x y theta]
P = interp1(out.odomPose.Time, toN3(out.odomPose), t);      % encoder odometry
G = interp1(out.gyroPose.Time, toN3(out.gyroPose), t);      % encoder + gyro

E_odom = S - P;      % error = truth - estimate
E_gyro = S - G;

if ~exist('images', 'dir'), mkdir('images'); end

%% Figure 1 - Paths in the XY plane
f1 = figure('Name', 'XY paths');
plot(S(:,1), S(:,2), 'b', 'LineWidth', 1.5); hold on
plot(P(:,1), P(:,2), 'r--', 'LineWidth', 1.2)
plot(G(:,1), G(:,2), 'g-.', 'LineWidth', 1.2)
plot(S(1,1), S(1,2), 'ko', 'MarkerFaceColor', 'k')
axis equal; grid on
xlabel('x [m]'); ylabel('y [m]')
title('Robot path: ground truth vs. estimates')
legend('Ground truth', 'Odometry (encoders only)', 'Odometry + gyro', 'Start', 'Location', 'best')
exportgraphics(f1, 'images/xy_paths.png', 'Resolution', 150)

%% Figure 2 - Pose error, encoders only
f2 = plotError(t, E_odom, 'Pose error - odometry with noisy encoders (no gyro)');
exportgraphics(f2, 'images/error_odometry.png', 'Resolution', 150)

%% Figure 3 - Pose error, encoders + gyro
f3 = plotError(t, E_gyro, 'Pose error - odometry with gyro heading');
exportgraphics(f3, 'images/error_gyro.png', 'Resolution', 150)

%% Figure 4 - Heading error comparison
f4 = figure('Name', 'Heading error comparison');
plot(t, E_odom(:,3), 'r', t, E_gyro(:,3), 'g', 'LineWidth', 1.2); grid on
xlabel('Time [s]'); ylabel('\theta_{true} - \theta_{est} [rad]')
title('Heading error: encoders only vs. gyro')
legend('Encoders only (random walk)', 'Gyro (bias drift)', 'Location', 'best')
exportgraphics(f4, 'images/heading_error_comparison.png', 'Resolution', 150)

fprintf('Saved 4 figures to the images/ folder.\n');

%% Local functions
function M = toN3(ts)
% Returns timeseries data as an N-by-3 matrix [x y theta]
M = squeeze(ts.Data);
if size(M,1) == 3 && size(M,2) ~= 3
    M = M.';
end
end

function f = plotError(t, E, ttl)
f = figure('Name', ttl);
labels = {'x error [m]', 'y error [m]', '\theta error [rad]'};
for k = 1:3
    subplot(3,1,k)
    plot(t, E(:,k), 'LineWidth', 1.2); grid on
    ylabel(labels{k})
    if k == 1, title(ttl); end
end
xlabel('Time [s]')
end
