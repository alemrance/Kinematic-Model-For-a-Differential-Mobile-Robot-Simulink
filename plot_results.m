%% plot_results.m
% Plots and saves the result figures for the README.
% Requires To Workspace blocks (Timeseries format) named:
%   robotState -> Diff Drive "state" output         (ground truth)
%   odomPose   -> encoder-only odometry Integrator  (estimate 1)
%   gyroPose   -> odom_gyro Integrator              (estimate 2)
%   posMeas    -> absolute position sensor output   (measurements)
%   ekfPose    -> EKF "pose" output                 (estimate 3)
%   ekfP       -> EKF "Pdiag" output                (EKF variances)
%
% Usage:  out = sim('mobile_robot_model');  plot_results

if ~exist('out', 'var')
    out = sim('mobile_robot_model');
end

%% Data (everything resampled to the ground-truth time vector)
t   = out.robotState.Time;
S   = toNk(out.robotState, 3);                                         % truth [x y th]
P   = interp1(out.odomPose.Time, toNk(out.odomPose, 3), t);            % encoders only
G   = interp1(out.gyroPose.Time, toNk(out.gyroPose, 3), t);            % encoders + gyro
K   = interp1(out.ekfPose.Time,  toNk(out.ekfPose, 3),  t, 'previous', 'extrap');  % EKF
Pk  = interp1(out.ekfP.Time,     toNk(out.ekfP, 3),     t, 'previous', 'extrap');  % EKF variances
tm  = out.posMeas.Time;
M   = toNk(out.posMeas, 2);                                            % sensor [x y]

E_odom = S - P;      % error = truth - estimate
E_gyro = S - G;
E_ekf  = S - K;
sig3   = 3*sqrt(Pk); % EKF 3-sigma bounds

imgDir = fullfile(pwd, 'images');
if ~isfolder(imgDir), mkdir(imgDir); end

%% Figure 1 - Paths in the XY plane
f1 = figure('Name', 'XY paths');
plot(S(:,1), S(:,2), 'b',   'LineWidth', 1.5); hold on
plot(P(:,1), P(:,2), 'r--', 'LineWidth', 1.0)
plot(G(:,1), G(:,2), 'm-.', 'LineWidth', 1.0)
plot(K(:,1), K(:,2), 'g',   'LineWidth', 1.2)
plot(M(:,1), M(:,2), 'k.',  'MarkerSize', 12)
plot(S(1,1), S(1,2), 'ko',  'MarkerFaceColor', 'k'); hold off
axis equal; grid on
xlabel('x [m]'); ylabel('y [m]')
title('Robot path: ground truth vs. estimates')
legend('Ground truth', 'Odometry (encoders)', 'Odometry + gyro', 'EKF', ...
       'Position sensor', 'Start', 'Location', 'bestoutside')
exportgraphics(f1, fullfile(imgDir, 'xy_paths.png'), 'Resolution', 150)

%% Figure 2 - Pose error, encoders only
f2 = plotError(t, E_odom, [], 'Pose error - odometry with noisy encoders');
exportgraphics(f2, fullfile(imgDir, 'error_odometry.png'), 'Resolution', 150)

%% Figure 3 - Pose error, encoders + gyro
f3 = plotError(t, E_gyro, [], 'Pose error - odometry with gyro heading');
exportgraphics(f3, fullfile(imgDir, 'error_gyro.png'), 'Resolution', 150)

%% Figure 4 - Pose error, EKF with 3-sigma bounds
f4 = plotError(t, E_ekf, sig3, 'Pose error - EKF (encoders + gyro + position sensor)');
exportgraphics(f4, fullfile(imgDir, 'error_ekf.png'), 'Resolution', 150)

%% Figure 5 - Heading error comparison
f5 = figure('Name', 'Heading error comparison');
plot(t, E_odom(:,3), 'r', t, E_gyro(:,3), 'm', t, E_ekf(:,3), 'g', 'LineWidth', 1.2); grid on
xlabel('Time [s]'); ylabel('\theta_{true} - \theta_{est} [rad]')
title('Heading error comparison')
legend('Encoders only', 'Encoders + gyro', 'EKF', 'Location', 'best')
exportgraphics(f5, fullfile(imgDir, 'heading_error_comparison.png'), 'Resolution', 150)

%% Summary table (t > 5 s)
k   = t > 5;
rmsPos = @(E) sqrt(mean(sum(E(k,1:2).^2, 2)));
rmsTh  = @(E) sqrt(mean(E(k,3).^2));
fprintf('\n%-20s %14s %14s %14s\n', 'Method', 'RMS pos [m]', 'RMS th [rad]', 'final pos [m]');
names = {'Encoders only', 'Encoders + gyro', 'EKF'};
Es    = {E_odom, E_gyro, E_ekf};
for i = 1:3
    fprintf('%-20s %14.3f %14.3f %14.3f\n', names{i}, rmsPos(Es{i}), rmsTh(Es{i}), norm(Es{i}(end,1:2)));
end
inside = mean(all(abs(E_ekf(k,:)) <= sig3(k,:), 2)) * 100;
fprintf('EKF error inside 3-sigma bounds: %.1f %% of the time\n', inside);
fprintf('Saved 5 figures to the images/ folder.\n');

%% Local functions
function A = toNk(ts, n)
% Returns timeseries data as an N-by-n matrix
A = squeeze(ts.Data);
if size(A,1) == n && size(A,2) ~= n
    A = A.';
end
end

function f = plotError(t, E, bounds, ttl)
f = figure('Name', ttl);
labels = {'x error [m]', 'y error [m]', '\theta error [rad]'};
for i = 1:3
    subplot(3,1,i)
    plot(t, E(:,i), 'LineWidth', 1.2); grid on; hold on
    if ~isempty(bounds)
        plot(t,  bounds(:,i), 'k--', t, -bounds(:,i), 'k--')
    end
    hold off
    ylabel(labels{i})
    if i == 1
        title(ttl)
        if ~isempty(bounds), legend('Error', '\pm3\sigma (EKF)', 'Location', 'best'); end
    end
end
xlabel('Time [s]')
end
