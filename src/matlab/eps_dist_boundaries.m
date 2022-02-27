function [left_new, right_new] = eps_dist_boundaries(left, right, eps_dist)
%EPS_DIST_BOUNDARIES Computes boundaries that have a certain distance to
%the actual track boundaries. 

vec = left - right;
vec_length = sqrt(vec(:,1).^2 + vec(:,2).^2);
vec = vec ./ vec_length;

if isscalar(eps_dist)
    right_new = right + vec * eps_dist;
    left_new = left - vec * eps_dist;
else
    %eps_dist = [eps_dist, eps_dist];
    right_new = right + vec .* eps_dist;
    left_new = left - vec .* eps_dist;
end

end
