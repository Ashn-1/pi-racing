function query_results = check_trajectory(trajectory, bounds_left, bounds_right)
%CHECK_TRAJECTORY Checks if each trajectory point is inbounds. Returns a
%vector with values for each point. True for out of bounds point, false if
%inbounds.

bounds_polygon = [bounds_right; NaN, NaN; flipud(bounds_left)];
[query_results, on_edge] = inpolygon(trajectory(:, 1), trajectory(:, 2), bounds_polygon(:, 1), bounds_polygon(:, 2));
query_results = ~(xor(query_results, on_edge));

end

