%% Initialization of the program

% @author Ahmad Haidari
% @date November 08, 2021

% Cleanup
clear
%close all

% Initialize paths
addpath("/usr/local/gtsam_toolbox/")
addpath("res")
addpath("matlab")

% Load libraries
import gtsam.*
import pi_racing.*

t_total = tic;

% Settings
plot_loaded_dataset = true;
is_flying_start = true;
plot_zoomed = true;
safety_dist = 1.0; % in meters
bound_cost_max_perc = 0.99;
bound_cost_min_meter = 1.5;
line_sample_rate = 1 / 0.5;

%% Load datase
t_loading = tic;

% Track
track_file = "berlin_2018.csv";
%track_file = "modena_2019.csv";
%track_file = "simple_racetrack.csv";
%track_file = "handling_track.csv";
%track_file = "rounded_rectangle.csv";

compare_results = false;
if track_file == "berlin_2018.csv"
    comparison_file = "res/min_curv_iqp_results_berlin.csv";
    compare_results = true;
elseif track_file == "modena_2019.csv"
    comparison_file = "res/min_curv_iqp_results_modena.csv";
    compare_results = true;
end

% Load track
[centerline, og_bounds_right, og_bounds_left] = load_dataset(track_file, line_sample_rate);

t_loading = toc(t_loading);

disp("Finished loading racetrack");

%% Pre-processing

t_pre_processing = tic;

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

t_pre_processing = toc(t_pre_processing);

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

disp("Finished Pre-Processing")

%% Factor Graph 

t_init_graph = tic;

% Variable nodes
max_step = length(centerline);

% Bounding factor parameters 1
bounding_sigma = 1;
bounding_model = noiseModel.Gaussian.Covariance(eye(2) * bounding_sigma);

% Minimum angle factor parameters 
% - 6e-3 for Berlin
% - 2e-3 for Modena
min_angle_diff_sigma = 6e-3;
min_angle_diff_model = noiseModel.Gaussian.Covariance(eye(2) * min_angle_diff_sigma);

% Prior to start/goal
fixed_point_model = noiseModel.Gaussian.Covariance(eye(2) * 0.0001);

disp("Finished applying settings");

[init_values, graph] = build_graph(...
    centerline,...
    bound_cost_start_left,...
    bound_cost_start_right,...
    is_flying_start,...
    fixed_point_model,...
    bounding_model,...
    min_angle_diff_model...
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
if compare_results
    result_other = readtable(comparison_file);
    result_pos_other = [result_other{:, "x_m"}, result_other{:, "y_m"}];
    writematrix(result_pos_other, "output/optimization_based.csv");
end

% Save results to file
disp("Exporting results");
writematrix(result_pos, "output/pi_results.csv");
writematrix(centerline, "output/centerline.csv");

% Compute curvature of both trajectories
[total_curvature, curvatures] = compute_curvature(result_pos, true);
if compare_results
    [total_curvature_other, curvatures_other] = compute_curvature(result_pos_other, true);
else
    total_curvature_other = -1;
    curvatures_other = -1;
end

% Compute the total distance travelled of both trajectories
total_distance = compute_distance_travelled(result_pos, true);
if compare_results
    total_distance_other = compute_distance_travelled(result_pos_other, true);
else
    total_distance_other = -1;
end

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
if compare_results
    plot(result_pos_other(:, 1), result_pos_other(:, 2), "g-", "LineWidth", 2);
end
plot(result_pos(:, 1), result_pos(:, 2), "r-", "LineWidth", 2);
plot(og_bounds_right(:, 1), og_bounds_right(:, 2), "k-", "LineWidth", 2);
plot(result_pos(states_outbounds, 1), result_pos(states_outbounds, 2), "mo", "LineWidth", 2);
plot(og_bounds_left(:, 1), og_bounds_left(:, 2), "k-", "LineWidth", 2);
if compare_results
    legend("Centerline", "Comparison Trajectory", "Optimized Trajectory", "Track Boundaries");
else
    legend("Centerline", "Optimized Trajectory", "Track Boundaries");
end

t_plotting = toc(t_plotting);
t_total = toc(t_total);

%% Print Timings

disp("===== PRINTING RESULTS =====");

fprintf("Number of iterations: %d\n", optimizer.iterations())

fprintf("Out of bounds states: %d\n", sum(states_outbounds));

fprintf("Timings:\n- %.3fs - Loading Time\n- %.3fs - Pre-Processing Time\n- %.3fs - Init Graph Time\n- %.3fs - Optimization Time\n- %.3fs - Checking Trajectory Time\n- %.3fs - Plotting Time\n- %.3fs - Total Time\n",...
    t_loading, t_pre_processing,t_init_graph, t_optimization, t_checking, t_plotting, t_total);

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
