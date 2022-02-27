function [trajectory, changed_states] = post_process(trajectory, bounds_left, bounds_right, smoothen_traj)

max_steps = length(trajectory);

% Keep track of all changed states
changed_states = zeros(length(trajectory), 1);

% Vector pointing from left to right boundary and its length
heading = bounds_right - bounds_left;
max_length = sqrt(heading(:, 1).^2 + heading(:, 2).^2);
for i = 1:max_steps % Normalize headings
    heading(i, :) = heading(i, :) / max_length(i);
end

% Bound the trajectory points to the left and right boundary
for i = 1:max_steps
    len = dot(trajectory(i, :) - bounds_left(i, :), heading(i, :));
    
    if len < 0.0 % Out of bounds left
        trajectory(i, :) = trajectory(i, :) - len * heading(i, :);
        changed_states(i) = 1;
    elseif len > max_length(i) % Out of bounds right
        trajectory(i, :) = trajectory(i, :) + (max_length(i) - len) * heading(i, :);
        changed_states(i) = -1;
    end
end

changed_dir = changed_states;
changed_states = changed_states & 1;

% %Smooth out the trajectory after shifting states
% if smoothen_traj
%     for i = 1:max_steps
%         i_next = i + 1;
%         if i_next > max_steps
%             i_next = 1;
%         end
%         i_after_next = i_next + 1;
%         if i_after_next > max_steps
%             i_after_next = 1;
%         end
%         
%         if (changed_states(i) && changed_states(i_next) && ~changed_states(i_after_next)) ||...
%                 (~changed_states(i) && ~changed_states(i_next) && changed_states(i_after_next))
%             i_to_in = trajectory(i_next, :) - trajectory(i, :);
%             i_to_ian = trajectory(i_after_next, :) - trajectory(i, :);
%             i_to_ian = i_to_ian / norm(i_to_ian);
%             
%             scalar_projection = dot(i_to_in, i_to_ian);
%             trajectory(i_next, :) = trajectory(i, :) + i_to_ian * scalar_projection;
%         end
%     end
% end

% if smoothen_traj
%     for i = 1:max_steps
%         i_next = i + 1;
%         if i_next > max_steps
%             i_next = 1;
%         end
%         
%         if xor(changed_states(i), changed_states(i_next))
%             i_to_in = trajectory(i_next, :) - trajectory(i, :);
%             
%             scalar_projection = dot(i_to_in, heading(i, :));
%             trajectory(i, :) = trajectory(i, :) - heading(i, :) * scalar_projection * 0.25 * changed_dir(i);
%             trajectory(i_next, :) = trajectory(i_next, :) + heading(i, :) * scalar_projection * 0.25 * changed_dir(i);
%         end
%     end
% end

end

