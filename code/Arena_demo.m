% This file demonstrates a simple usage of the Arena class. We perform a
% single trial which can be stopped at any time (as it is mostly random and
% usually takes a long time). Afterwards, we run several trials in order to
% train the agent so that we can see how it behaves once it has learned
% something about the arena.

%% Specify parameters.

% Number of neurons in one dimension of the NxN lattice.
N = 10;

% Number of allowed directions of travel.
DOF = 8;

% Maximum allowed steps per trial.
max_steps = 10000;

%% Initialize a new arena.

clear a;
a = Arena(N, DOF);

%% Do the first run.
disp('Running first trial. At this point the agent doesn''t know anything about the arena.');

a.reset();
run_until_cancel(a);
a.run_trial(max_steps);

disp('Trial finished.');
drawnow;

%% Plot the trial results.

figure(2);
subplot(121);
plot_arena(a);
plot_wall_crossings(a);
plot_path(a);

subplot(122);
plot_preferred_directions(a);
title('Learned directions of travel');
plot_arena(a);
drawnow;

%% Run 100 more trials.

num_trials = 100;
disp('Running 100 trials to train the agent.');

for i = 1:num_trials
    a.reset();
    a.run_trial(max_steps);
    fprintf('.');
end
fprintf('\n');

%% Do another run after the agent has gained some knowledge about the arena.
disp('Running another trial, after many trials where the agent has gained knowledge about the arena.');

a.reset();
run_until_cancel(a);
a.run_trial(max_steps);

disp('Trial finished.');

%% Plot the trial results.

figure(3);
subplot(121);
plot_arena(a);
plot_wall_crossings(a);
plot_path(a);

subplot(122);
plot_preferred_directions(a);
title('Learned directions of travel');
plot_arena(a);