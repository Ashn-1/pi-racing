function [centerline, bounds_left, bounds_right] = load_dataset(filename, sample_rate)
%LOAD_DATASET Loads the given track file.
%   The track file must be a CSV file containing the reference line and the
%   track widths in both directions from the reference line. The
%   centerline and the track boundaries can be sampled if needed.
%   
%   Returns the centerline, track boundary left and track boundary
%   right

if nargin < 1
    error("load_dataset: no filename given");
end

% Get data from CSV
dataset = readtable(filename);
centerline = [dataset{:, "x_m"}, dataset{:, "y_m"}];
width_right = dataset{:, "w_tr_right_m"};
width_left = dataset{:, "w_tr_left_m"};

% Calculate the normals
tmp = centerline(1:end-1,:) - centerline(2:end,:);
tmp = [tmp; centerline(end,:) - centerline(1,:)];
normals = [tmp(:,2), -tmp(:,1)];

% Calculate the track boundaries
bounds_right = centerline - normals .* width_right;
bounds_left = centerline + normals .* width_left;

% Compute the centerline
centerline = bounds_right + (bounds_left - bounds_right) * 0.5;

% Downsample the lines if a sample rate is given
if nargin < 2
    % No sampling needed
else
    centerline = centerline(1:sample_rate:end, :);
    bounds_right = bounds_right(1:sample_rate:end, :);
    bounds_left = bounds_left(1:sample_rate:end, :);
end

%centerline = centerline(1:end-1, :);

%bounds_right = [bounds_right; bounds_right(1,1), bounds_right(1,2)];
%bounds_left = [bounds_left; bounds_left(1,1), bounds_left(1,2)];

end
