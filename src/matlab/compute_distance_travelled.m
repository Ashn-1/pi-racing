function distance = compute_distance_travelled(path, is_looping)
%COMPUTE_DISTANCE_TRAVELLED Computes the total distance of a given 2D path.
%If the is_looping flag is given, the first point will also be used as the
%last point.

if nargin < 2
    is_looping = false;
end

if is_looping
    path(end+1,:) = path(1,:);

differences = diff(path);
distances = vecnorm(differences');
distance = sum(distances);

end

