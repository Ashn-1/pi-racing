function [init_values, graph] = build_graph(...
    ref_line,...
    bounds_left,...
    bounds_right,...
    is_flying_start,...
    fixed_point_model,...
    bounding_model,...
    min_angle_diff_model...
)
%BUILD_GRAPH Builds the factor graph for raceline optimization. To disable
%a factor, the corresponding covariance has to be set to 0.

graph = gtsam.NonlinearFactorGraph;
init_values = gtsam.Values;

key_pos_2 = gtsam.symbol('x', 1);
key_pos_3 = gtsam.symbol('x', 2);

max_step = length(ref_line);

for step = 1 : max_step
    % Next steps might wrap around
    next_step = mod(step + 1, max_step);
    if next_step == 0
        next_step = max_step;
    end
    after_next_step = mod(next_step + 1, max_step);
    if after_next_step == 0
        after_next_step = max_step;
    end
    
    % Keys for the current step
    key_pos_1 = key_pos_2;
    key_pos_2 = key_pos_3;
    key_pos_3 = gtsam.symbol('x', after_next_step);
    
    % Position for the current step
    position = gtsam.Point2(ref_line(step, 1), ref_line(step, 2));
    
    % Add position to initial guess
    init_values.insert(key_pos_1, position);
        
    % Minimum anlge difference factors
    graph.add(pi_racing.MinimumSteeringAngleFactor(min_angle_diff_model, key_pos_1, key_pos_2, key_pos_3));
        
    % Start and end prior
    if ~is_flying_start && step == 0
        graph.add(gtsam.PriorFactorPoint2(key_pos_1, position, fixed_point_model));
    elseif ~is_flying_start && step == max_step
        graph.add(gtsam.PriorFactorPoint2(key_pos_1, position, fixed_point_model));
    else
        graph.add(...
            pi_racing.RacetrackBoundingFactor(...
                bounding_model, key_pos_1,...
                bounds_left(step, 1), bounds_left(step, 2),...
                bounds_right(step, 1), bounds_right(step, 2)...
            )...
        );
    end
end

end

