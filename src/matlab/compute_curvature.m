function [total_curv, curvatures] = compute_curvature(trajectory, absolute)
% Computes the curvature of the given trajectory in each point and in
% total. If the optional argument 'absolute' is set to true, the sum of the
% absolute curvature values is summed up (defaults to false).

    if nargin < 2
        absolute = false;
    end

    max_step = length(trajectory);

    curvatures = zeros(max_step, 1);
    for step = 1:max_step
        next_step = mod(step, max_step) + 1;
        after_next_step = mod(step+1, max_step) +1;

        current = trajectory(step, :);
        next = trajectory(next_step, :);
        after_next = trajectory(after_next_step, :);

        angle = get_angle(current, next, after_next);
        distance = norm(next - current);
        curvatures(step) = angle / distance;
    end
    
    if absolute
        total_curv = sum(abs(curvatures));
    else
        total_curv = sum(curvatures);
    end
end

