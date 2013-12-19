% first test with the person

clear all
close all

% create simulator object
qrsim = QRSim();

% load task parameters and do housekeeping
state = qrsim.init('TaskPersonWalkToWaypoints');

% number of steps we run the simulation for
N = 3000;

% Set up a random set of waypoints
wp = [0 200 200 0;0 0 200 200];

tstart = tic;

for i=1:N,
    tloop=tic;
    qrsim.step([]);
    % wait so to run in real time
    wait = max(0,state.task.dt-toc(tloop));
    pause(wait);
end

% get reward
% qrsim.reward();

elapsed = toc(tstart);

fprintf('running %d times real time\n',(N*state.DT)/elapsed);
