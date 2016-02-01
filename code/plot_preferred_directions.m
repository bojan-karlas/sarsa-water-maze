function plot_preferred_directions( arena )
% PLOT_PREFERRED_DIRECTIONS Draws a graph containing, for each neuron in
% the network, the preferred direction of movement for that neuron. Also,
% the intensity of expected reward (the Q value) of the strongest direction
% is shown.
    
    DX = zeros(arena.n);   % Arrow direction projected onto X.
    DY = zeros(arena.n);   % Arrow direction projected onto Y.
    Q = zeros(arena.n);    % Q-values of strongest directions.
    
    for i = 1:arena.n
        for j = 1:arena.n
            pos = [arena.neuron_pos_x(i, j) arena.neuron_pos_y(i, j)];
            q = arena.get_Q(pos);
            [Q(i,j), direction] = max(q);
            
            angle = 2 * pi * direction / arena.n_directions;
            DX(i, j) = cos(angle);
            DY(i, j) = sin(angle);
        end
    end
    
    hold on;
    % Plot points with size proportional to intensity.
    X = arena.neuron_pos_x;
    Y = arena.neuron_pos_y;
    
    %Q = Q ./ max(max(Q));
    fprintf('Max Q value: %f\n', max(max(Q)));
    Q = Q - min(min(Q)) + 0.01;
    Q = Q ./ 10;
    Q = Q .* 80;
    
    % base_col = [19 149 186] / 255;
    % C = zeros(arena.n, arena.n, 3);
    
    % for i = 1:3, C(:,:,i) = Q .* base_col(i); end
    % C = max(C, 0);
    % C = min(C, 1);
    
    scatter(reshape(X, [], 1), reshape(Y, [], 1), reshape(Q, [], 1), [236 170  56] / 255, 'filled');
    %image([0 1], [0 1], C);
    
    % Adjust arrow positions.
    X = X - DX .* 0.5 .* (1/(arena.n * 1));
    Y = Y - DY .* 0.5 .* (1/(arena.n * 1));
    
    hold on;
    q = quiver(X, Y, DX, DY, 0.5, 'Color', [ 13, 60, 85] / 255, 'MarkerSize', 5);
    set(q, 'AutoScaleFactor', 0.7);
    set(q, 'LineWidth', 0.75);
    hold off;
    
    [~, h_goal] = plot_arena(arena, [], [ 13, 60, 85] / 255);
    set(h_goal, 'LineWidth', 3);
    set(h_goal, 'LineStyle', ':');

end

