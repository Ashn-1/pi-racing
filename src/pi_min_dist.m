%% Initialization of the program

% @author Ahmad Haidari
% @date February 13, 2022

% Cleanup
clear
%close all

% Initialize paths
addpath("/usr/local/gtsam_toolbox/")
addpath("res")
addpath("src")
addpath("src/matlab")

% Load libraries
import gtsam.*
import pi_racing.*

% Settings
plot_loaded_dataset = true;
is_flying_start = true;
plot_zoomed = true;
safety_dist = 1.0; % in meters
boundary_cost_min = 2.0; % in meters
boundary_cost_max = 3.0; % in meters
bound_cost_max_perc = 0.99; 
bound_cost_min_meter = 1.5;
line_sample_rate = 1 / 0.5;

%% Load dataset
t_total = tic;
t_loading = tic;

% Track
track_file = "berlin_2018.csv";
%track_file = "modena_2019.csv";
%track_file = "simple_racetrack.csv";
%track_file = "handling_track.csv";
%track_file = "rounded_rectangle.csv";

% Load track
[centerline, og_bounds_right, og_bounds_left] = load_dataset(track_file, line_sample_rate);

% Compute angles between points
angles = zeros(length(centerline), 1);
for i = 1:length(centerline)
    next = mod(i, length(angles))+1;
    after_next = mod(i+1, length(angles))+1;
    angles(i) = get_angle(centerline(i,:), centerline(next,:), centerline(after_next,:));
end

% Compute dynamic cost boundary depending on angle
angles_n = rescale(angles, 0, bound_cost_max_perc);
dyn_cost_boundary = vecnorm((og_bounds_right - og_bounds_left)')' * 0.5 .* angles_n;
dyn_cost_boundary = max(bound_cost_min_meter, dyn_cost_boundary);

% Get track boundaries
[bound_cost_start_left, bound_cost_start_right] = eps_dist_boundaries(og_bounds_left, og_bounds_right, dyn_cost_boundary);
[bound_safety_left, bound_safety_right] = eps_dist_boundaries(og_bounds_left, og_bounds_right, safety_dist);

if plot_loaded_dataset
    figure(1);
    clf;
    if plot_zoomed
        axis([-60, 140, -230, -60]);
    end
    axis equal;
    hold on;
    plot(centerline(:,1), centerline(:,2), "bx-", "LineWidth", 2.0);
    plot(og_bounds_right(:,1), og_bounds_right(:,2), "k-", "LineWidth", 2.0);
    plot(bound_cost_start_right(:,1), bound_cost_start_right(:,2), "g-", "LineWidth", 2.0);
    plot(og_bounds_left(:,1), og_bounds_left(:,2), "k-", "LineWidth", 2.0);
    plot(bound_cost_start_left(:,1), bound_cost_start_left(:,2), "g-", "LineWidth", 2.0);
    legend("Centerline", "Track Boundary", "Cost Boundaries");
    title("Racetrack with Cost Boundaries");
    xlabel("x [m]");
    ylabel("y [m]");
end

disp("Finished loading racetrack");

%% Factor Graph Settings

% Variable nodes
max_step = length(centerline);

% Bounding factor parameters 1
bounding_sigma = 1;
bounding_model = noiseModel.Gaussian.Covariance(eye(2) * bounding_sigma);

% Minimum angle factor parameters 
% - 8e-2 for 1/4 sample rate
% - 6e-3 for 1/2 sample rate
% - 1.6e-3 for 1/1 sample rate
min_dist_sigma = 5e-1;
min_dist_diff_model = noiseModel.Gaussian.Covariance(eye(2) * min_dist_sigma);

% Prior to start/goal
fixed_point_model = noiseModel.Gaussian.Covariance(eye(2) * 0.0001);

disp("Finished applying settings");

t_loading = toc(t_loading);

%% Init trajectory computation

t_init_graph = tic;

[init_values, graph] = build_graph_min_dist(...
    centerline,...
    bound_cost_start_left,...
    bound_cost_start_right,...
    is_flying_start,...
    fixed_point_model,...
    bounding_model,...
    min_dist_diff_model...
);

t_init_graph = toc(t_init_graph);

disp("Finished initial trajectory computation");

%% Optimization

t_optimization = tic;

parameters = gtsam.LevenbergMarquardtParams;
parameters.setLinearSolverType('SEQUENTIAL_CHOLESKY')
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values, parameters);
optimizer.optimize();

t_optimization = toc(t_optimization);

disp("Finished trajectory optimization");

%% Check Trajectory

t_checking = tic;

results = optimizer.values();

result_pos = zeros(max_step, 2);

for step = 1:max_step
    position = results.atPoint2(gtsam.symbol('x', step));
    result_pos(step, :) = [position.x(), position.y()];
end

[~, states_outbounds] = post_process(result_pos, bound_safety_left, bound_safety_right, true);

t_checking = toc(t_checking);

disp("Finished post processing");

%% Get results

t_plotting = tic;

% Load results from MinCurv paper
result_other = readtable("res/min_curv_paper_results.csv");
result_pos_other = [result_other{:, "x_m"}, result_other{:, "y_m"}];
%indices = round(linspace(1, length(other_results_traj), length(centerline)));
%other_results_traj = other_results_traj(indices, :);

% Upsample the resulting trajectory to match the reference trajectory
%sample_rate = length(other_results_traj) / length(result_pos);
%result_pos = resample(result_pos, length(other_results_traj), length(result_pos));

% Save results to file
disp("Exporting results");
writematrix(result_pos, "output/pi_results.csv");
writematrix(centerline, "output/centerline.csv");
writematrix(result_pos_other, "output/optimization_based.csv");

% Compute curvature of both trajectories
[total_curvature, curvatures] = compute_curvature(result_pos, true);
[total_curvature_other, curvatures_other] = compute_curvature(result_pos_other, true);

% Compute the total distance travelled of both trajectories
total_distance = compute_distance_travelled(result_pos);
total_distance_other = compute_distance_travelled(result_pos_other);

%% Plot results

figure(8);
clf;
if plot_zoomed
    axis([-60, 140, -230, -60]);
end
axis equal;
hold on;
title("Optimized results - Minimum Curvature");
xlabel("x [m]");
ylabel("y [m]");
plot(centerline(:, 1), centerline(:, 2), "b-", "LineWidth", 2);
plot(result_pos_other(:, 1), result_pos_other(:, 2), "g-", "LineWidth", 2);
plot(result_pos(:, 1), result_pos(:, 2), "r-", "LineWidth", 2);
plot(og_bounds_right(:, 1), og_bounds_right(:, 2), "k-", "LineWidth", 2);
plot(result_pos(states_outbounds, 1), result_pos(states_outbounds, 2), "mo", "LineWidth", 2);
plot(og_bounds_left(:, 1), og_bounds_left(:, 2), "k-", "LineWidth", 2);
legend("Centerline", "Comparison Trajectory", "Optimized Trajectory", "Track Boundaries");

t_plotting = toc(t_plotting);
t_total = toc(t_total);

%% Print Timings

disp("===== PRINTING RESULTS =====");

fprintf("Number of iterations: %d\n", optimizer.iterations())

fprintf("Out of bounds states: %d\n", sum(states_outbounds));

fprintf("Timings:\n- %.3fs - Loading Time\n- %.3fs - Init Graph Time\n- %.3fs - Optimization Time\n- %.3fs - Checking Trajectory Time\n- %.3fs - Plotting Time\n- %.3fs - Total Time\n",...
    t_loading, t_init_graph, t_optimization, t_checking, t_plotting, t_total);

fprintf("Error: %.3f (Centerline) --> %.3f (Optimized)\n", graph.error(init_values), graph.error(results));

fprintf("Sum of absolute curvature values: %.3f (vs %.3f in optimization-based approach) \n", total_curvature, total_curvature_other);

fprintf("Distance travelled: %.3fm (vs %.3fm in optimization-based approach\n", total_distance, total_distance_other);

%% Extra plots and data from Python scripts

fprintf("\n")

system("python3.7 eval/compute_stats.py");

profiles = readtable("output/pi_results_vel_profile.csv");
vel_profile = [profiles{:, 1}];
profiles = readtable("output/pi_results_acc_profile.csv");
acc_profile = [profiles{:, 1}];
figure(9);
clf;
plot(1:length(vel_profile), vel_profile, "b-", "LineWidth", 2.0);
hold on;
plot(1:length(acc_profile), acc_profile, "r-", "LineWidth", 2.0);
legend("Velocity [m/s]", "Acceleration [m/s^2]");
xlabel("State")
ylabel("v [m/s] | a [m/s^2]")
title("Velocity and Acceleration Profiles")
