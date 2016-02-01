function plot_step( from, to, is_random )
% PLOT_STEP Plots a single step of a path given two 2D points: 'from' and
% 'to'. The function also takes a 2D vector 'is_random' which specifies for
% both points whether they were made as an exploration or an exploitation
% step.
    
    % Set the colors of both points depending on their nature (i.e.
    % exploration / exploitation).
    if is_random(1), color_from = [162 184 108] / 255;
    else color_from = [19 149 186] / 255; end
    
    if is_random(2), color_to = [162 184 108] / 255;
    else color_to = [19 149 186] / 255; end

    % Draw the two points, adequately colored and connected by a line
    % segment.
    
    hold on;
    plot([from(1), to(1)], [from(2) to(2)], '-','Color', color_to, 'LineWidth', 1.5);
    plot(from(1), from(2), 'o' ,'Color', color_from, 'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', color_from);
    plot(to(1), to(2), 'o' ,'Color', color_to, 'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', color_to);
    hold off;

end

