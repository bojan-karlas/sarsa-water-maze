function [h_arena, h_goal] = plot_arena( arena, arena_color, goal_color )
% PLOT_ARENA Draws an arena along with the goal area in the preferred 
% color.
%
% PLOT_ARENA( arena, arena_color, goal_color ) plots both the arena and the
% goal area with specified colors.
%
% PLOT_ARENA( arena ) plots both the arena and the goal area with default
% colors (black).
%
% PLOT_ARENA( arena, [], goal_color ) plots only the goal in the specified
% color and doesn't plot the arena at all.

    if nargin < 3, goal_color = [0 0 0]; end
    if nargin < 2, arena_color = [0 0 0]; end
    
    h_arena = 0; h_goal = 0;

    % Draw arena.
    if length(arena_color) == 3
        h_arena = rectangle('Position', [0 0 1 1], 'LineWidth', 2, 'EdgeColor', arena_color);
    end
    
    axis([-0.1 1.1 -0.1 1.1]);
    axis square;
    
    % Draw goal area.
    if length(goal_color) == 3
        goal_pos = [arena.goal_coord - arena.goal_radius, arena.goal_radius*2, arena.goal_radius*2];
        h_goal = rectangle('Position', goal_pos, 'Curvature', [1 1], 'LineStyle', '--', 'EdgeColor', goal_color);
    end
    
    set(gca, 'XTick', []);
    set(gca, 'YTick', []);
    set(gca, 'XColor', [1 1 1]);
    set(gca, 'YColor', [1 1 1]);
    
end

