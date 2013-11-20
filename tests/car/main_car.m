% first test with the car

clear all
close all

% create simulator object
qrsim = QRSim();

% load task parameters and do housekeeping
state = qrsim.init('TaskCarDriveInCircle');

% number of steps we run the simulation for
N = 3000;

wp = state.platforms{1}.getX(1:3)

% Null controller - keeps turning on a circle
U{1}=[1.5; 3*pi/180];

tstart = tic;

for i=1:N,
    tloop=tic;
    % one should alway make sure that the uav is valid
    % i.e. no collision or out of area event happened
    if(state.platforms{1}.isValid())
        % compute controls
        % step simulator
        qrsim.step(U);
    end
    % wait so to run in real time
    wait = max(0,state.task.dt-toc(tloop));
    pause(wait);
end

% get reward
% qrsim.reward();

elapsed = toc(tstart);

fprintf('running %d times real time\n',(N*state.DT)/elapsed);
