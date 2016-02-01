function plot_last_step( arena )
% PLOT_PATH_STEP Plots only the last step taken by the rat in the given
% arena. This function is used for real-time plotting of movement.
    
    % Extract two last steps taken by the rat to connect them with a line
    % segment.
    from = arena.movement_path(end-1, :);
    to = arena.movement_path(end, :);
    is_random = arena.path_step_random(end-1:end);
    
    % Plot the step.
    plot_step(from, to, is_random);
    
    title(sprintf('Steps: %d', arena.num_steps));

end

