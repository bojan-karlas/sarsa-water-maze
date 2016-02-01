function plot_path( arena )
% PLOT_PATH Plots a path made up of all steps the rat has taken in the
% given arena.

    % We plot the path as a collection of 2 point line segments.
    for i = 2:length(arena.movement_path)
        from = arena.movement_path(i-1, :);
        to = arena.movement_path(i, :);
        is_random = arena.path_step_random(i-1:i);

        plot_step(from, to, is_random);
    end
    
    title(sprintf('Steps taken: %d; Total reward: %d', arena.num_steps, arena.total_reward));
    
end

