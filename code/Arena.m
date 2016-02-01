classdef Arena < matlab.mixin.Copyable
    % ARENA Class which implements the SARSA algorithm for reinforcement
    % learning extended for continuous state spaces. It is applied at a
    % pathfinding problem inspired by the Morris water-maze experimental
    % procedure.
    %
    % A rat is placed on a fixed starting position inside a water-filled
    % arena that has a hidden platform which the rat is supposed to find.
    % When the rat finds the platform, the experiment is finished and
    % restarted. At first, the rat swims around randomly, but after it
    % begins finding the platform, it is supposed to learn its whearabouts
    % and be able to find it faster in subsequent trials.
    %
    % The rat's sensory system is approximated with a 2D N by N lattice of
    % neurons which are excited if the rat is near (the Gaussian radial
    % basis function is used as a measure of distance). The sensory neurons
    % make up the input layer of the neural network. There are D output
    % neurons (proparty value: n_directions) whose excitations correspond
    % to action-value functions for each possible direction of travel.
    %
    % In each time step, the rat makes a move for a fixed distance. The
    % direction is chosen either randomly (exploration step) or by looking
    % at the action-value estimates and taking the direction which
    % corresponds to the maximum action-value. When the rat reaches a
    % circular goal area, it is rewarded and the trial is stopped. If a rat
    % crosses the boundary of the 1x1 square arena, it is penalized and put
    % back in.
    %
    % The most basic way to use this class is the following:
    % a = Arena();         % Initialize with default values.
    % a.run_trial(10000);  % Run one trial that finishes either when the
    %                      % rat finds the platform (goal area) or until
    %                      % the number of time steps reaches the secified
    %                      % maximum value (10000).
    
    properties (SetAccess = immutable)
        % Number of neurons in any row or column of the arena.
        n;
        
        % Number of directions a rat can move in.
        n_directions;
        
        % Matrices of size (n x n) that contain x- and y-coordinates of all
        % neurons of the network.
        neuron_pos_x;
        neuron_pos_y;
    end
    
    properties
        
        % Current position of the rat.
        rat_pos = [0.1 0.1];
        
        % Distance which a rat travels in each time step.
        move_distance = 0.03
        
        % Parameter that adjusts how sensitive the neurons are to the rat.
        sigma = 0.05
        
        % Probability of doing an exploration step instead of exploiting
        % the learnt knowledge.
        epsilon = 0.5
        
        % Parameter with which we multiply the epsilon value before each
        % trial. Controls the decay rate of the exploration probability.
        epsilon_decay_rate = 1
        
        % Parameters used by the SARSA algorithm.
        learning_rate = 0.005
        discount_rate = 0.95
        trace_decay = 0.95
        
        % Coordinates and radius of the circular goal area.
        goal_coord = [0.8 0.8]
        goal_radius = 0.1
        
        % Rewards received when reaching the goal and when hitting a wall.
        reward_at_goal = 10
        reward_at_wall = -2
        
    end
    
    properties (SetAccess = private)
        
        % The direction the rat will take in the next iteration.
        next_direction = NaN;
        
        % True if the next direction was chosen as an exploration step.
        next_direction_ranom;
        
        % True if the rat has reached the goal area.
        rat_arrived = false;
        
        % Number of steps taken by the rat so far.
        num_steps = 0;
        
        % Number of trials performed so far.
        num_trials = 0;
        
        % Total reward collected by the rat during the trial.
        total_reward = 0;
        
        % Path the rat has travelled so far.
        movement_path = zeros(0,2);
        
        % Array of booleans which specify for each step whether it was an
        % exploration step (true) or an exploitation step (false).
        path_step_random = [];
        
        % List of all positions where the rat has crossed the wall.
        wall_crossings = zeros(0,2);
    end
    
    % Should become private
    properties
        
        % The x and y coordinates of each neuron, all stored vertically.
        neuron_pos;
        
        % n_directions x n matrix with weights.
        weights;
        
        % n_directions x n matrix with eligibility traces.
        eligibility;
    end
    
    methods
        
        % CONSTUCTOR
        
        function obj = Arena(n, n_directions)
            % ARENA Construct a new rat arena.
            %
            % a = ARENA(n, n_directions) generates a new (n x n) arena with
            % n_directions as the number of directions a mouse can move in.
            % Both parameters can be ommited. The default value for n is 20
            % and the default number of directions is 8.
            
            if nargin < 2, n_directions = 8; end
            if nargin < 1, n = 20; end
            
            obj.n = n;
            obj.n_directions = n_directions;
            
            obj.neuron_pos_x = repmat(linspace(0, 1, n), n, 1);
            obj.neuron_pos_y = repmat(linspace(0, 1, n)', 1, n);
            
            obj.neuron_pos = [reshape(obj.neuron_pos_x, [], 1) reshape(obj.neuron_pos_y, [], 1)];
            obj.weights = unifrnd(0, 1, n_directions, n^2);
            obj.eligibility = zeros(n_directions, n^2);
        end
    end
    
    methods
        
        function reset_weights(obj)
            % RESET_WEIGHTS Clears all the weights and initializes them to
            % random values uniformly sampled from the [0, 1] interval.
            % This makes the agent forget all the previously learned
            % knowledge.
            
            obj.weights = unifrnd(0, 1, obj.n_directions, obj.n^2);
        end
        
        function r = compute_activations(obj, pos)
            % COMPUTE_ACTIVATIONS Compute activations of position neurons
            % given the position of the rat.
            % 
            % r = arena.COMPUTE_ACTIVATIONS(pos) Compute activations for
            % position pos which is a 2D vector.
            %
            % r = arena.COMPUTE_ACTIVATIONS() Compute activations for the
            % current rat position.
            
            if nargin < 2, pos = obj.rat_pos; end
            
            s = repmat([pos(1) pos(2)], obj.n^2, 1);
            d = (obj.neuron_pos - s).^2;
            r = exp(- (d(:,1) + d(:,2)) ./ (ones(obj.n^2, 1) * 2 * obj.sigma^2));
        end
        
        function [q, r] = get_Q(obj, pos, directions)
            % GET_Q Compute Q values for the current state. Also gives the
            % input neuron activations that were used.
            %
            % [q, r] = arena.GET_Q(pos) Gives the Q values and activations
            % for all actions in a specific position. 
            %
            % q = arena.GET_Q(pos, actions) Gives the Q values and
            % activations for the specified subset of directions.
            
            if nargin < 3, directions = 1:obj.n_directions; end
            
            if nargin < 2, pos = obj.rat_pos; end
            
            r = obj.compute_activations(pos);
            q = obj.weights(directions, :) * r;
        end
        
        function [direction, is_random] = get_next_dir(obj, q)
            % GET_NEXT_DIR Compute the next direction of the rat by using
            % the epsilon-greedy strategy and the Q values.
            %
            % pos = arena.GET_NEXT_POS(q) Gets the next direction of the
            % rat and uses the provided Q values.
            %
            % pos = arena.GET_NEXT_POS() Gets the next direction of the rat
            % by using the current Q values.
            
            if unifrnd(0,1) <= obj.epsilon
                % Pick a random direction.
                direction = randi(obj.n_directions);
                is_random = true;
            else
                % Pick a direction based on the activations.
                if nargin < 2, q = obj.get_Q(); end
                [~,direction] = max(q);
                is_random = false;
            end
            
        end
        
        function pos = get_next_pos(obj, direction)
            % GET_NEXT_POS Compute the next position of the rat by moving
            % it in the given direction.
            %
            % pos = arena.GET_NEXT_POS(direction) Get next position of the
            % rat by moving it in the given direction.
            
            angle = 2 * pi * direction / obj.n_directions;
            dx = obj.move_distance * cos(angle);
            dy = obj.move_distance * sin(angle);
            
            pos = obj.rat_pos + [dx dy];
        end
        
        function [reward, pos] = get_reward(obj, pos)
            % GET_REWARD Compute the reward for the given position and the
            % adjusted position. If the given position is outside of the
            % arena, the adjusted position is moved inside.
            
            reward = 0;
            
            if pos(1) < 0 || pos(1) > 1
                reward = reward + obj.reward_at_wall;
                pos(1) = max(0, min(1, pos(1)));
            end
            
            if pos(2) < 0 || pos(2) > 1
                reward = reward + obj.reward_at_wall;
                pos(2) = max(0, min(1, pos(2)));
            end
            
            if norm(pos - obj.goal_coord) <= obj.goal_radius
                reward = reward + obj.reward_at_goal;
                obj.rat_arrived = true;
            end;
            
        end
        
        function reset(obj, pos)
            % RESET Resets the arena by moving the rat in the given
            % starting position. If the position is ommited, the default
            % value of [0.1 0.1] is used. Note that all the weights and
            % eligibility traces are retained. This should be called
            % between consecutive trials.
            
            if nargin < 2, pos = [0.1 0.1]; end
            
            obj.rat_pos = pos;
            obj.next_direction = NaN;
            obj.rat_arrived = false;
            obj.num_steps = 0;
            obj.total_reward = 0;
            obj.movement_path = [pos];
            obj.wall_crossings = zeros(0,2);
            obj.path_step_random = [false];
        end
        
        function arrived = step(obj)
            % STEP Performs a single step of the SARSA learning algorithm
            % on the given arena.
            %
            % arrived = arena.STEP() Performs a single step and returns
            % true if the rat has reached the target. Otherwise returns
            % false.
            
            if obj.rat_arrived
                % We are already at the target.
                arrived = true;
                return
            end
            
            if not(obj.next_direction > 0)
                % This happens only if we are in the first step.
                [obj.next_direction, obj.next_direction_ranom] = obj.get_next_dir();
            end
            
            % Our current direction is the direction calculated in the
            % pervious step.
            direction = obj.next_direction;
            direction_random = obj.next_direction_ranom;
            
            % Recover the Q values and the activations for the current
            % position.
            [q_curr, r_curr] = obj.get_Q();
            
            % Taking a step in the current direction, we observe the new
            % position and the reward. If the rat has reached the goal, the
            % get_reward() method will update the rat_arrived state also.
            pos = obj.get_next_pos(direction);
            [reward, pos_adj] = obj.get_reward(pos);
            arrived = obj.rat_arrived;
            obj.total_reward = obj.total_reward + reward;
            
            % Get the Q values for the next position and use them to
            % compute the next direction (which the rat will take in the
            % next iteration).
            q_next = obj.get_Q(pos_adj);
            [obj.next_direction, obj.next_direction_ranom] = obj.get_next_dir(q_next);
            
            % Update the eligibility traces.
            obj.eligibility = obj.eligibility .* (obj.discount_rate * obj.trace_decay);
            obj.eligibility(direction, :) = obj.eligibility(direction, :) + r_curr';
            
            % Calculate the TD error.
            if arrived
                % To prevent inflation of Q-values, if we are at the end of
                % an episode, we should take Q_(t+1) to be zero.
                delta = reward - q_curr(direction); 
            else
                delta = reward + obj.discount_rate * q_next(obj.next_direction) - q_curr(direction);
            end
            
            % Apply the learning rule.
            obj.weights = obj.weights + obj.eligibility .* (obj.learning_rate * delta);
            
            % Update position.
            obj.rat_pos = pos_adj;
            
            % Update stats.
            obj.num_steps = obj.num_steps + 1;
            obj.movement_path = [obj.movement_path; obj.rat_pos];
            obj.path_step_random = [obj.path_step_random direction_random];
            if norm(pos - pos_adj) > 0, obj.wall_crossings = [obj.wall_crossings; pos]; end
            
        end
        
        function run_trial(obj, max_steps)
            % RUN_TRIAL Performs a single trial until the rat reaches the
            % goal or until the maximum number of steps has been reached.
            
            if (obj.num_trials > 0)
                obj.epsilon = obj.epsilon * obj.epsilon_decay_rate;
            end
            
            while obj.num_steps < max_steps && not(obj.step())
            end
            
            obj.num_trials = obj.num_trials + 1;
        end
    end
end

