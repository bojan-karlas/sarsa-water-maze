function plot_wall_crossings( arena )
% PLOT_WALL_CROSSINGS Draws all the points where the rat has crossed the
% boundary of the arena and marks them with a red X.
    
    hold on;
    plot(arena.wall_crossings(:,1), arena.wall_crossings(:,2), 'x', ...
        'Color', [192 46 29] / 255, 'LineWidth', 2, 'MarkerSize', 8);
    hold off;

end

