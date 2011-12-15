function init(configFile)
% INIT
% Initializes the simulator state given a configuration file.

global state;

% add the configuration files
addpath(genpath('configs'));


% load the required configuration
eval(configFile);

% simulation time
state.t = 0;

% simulation timestep
state.DT = params.DT;

% random number generator stream
if(params.seed~=0)
    state.rStream = RandStream('mt19937ar','Seed',params.seed);
else
    state.rStream = RandStream('mt19937ar','Seed',sum(100*clock));
end    

%%% instantiates the objects that are part of the environment

% space segment of GPS
state.environment.gpsspacesegment = feval(params.environment.gpsspacesegment.type,...
    params.environment.gpsspacesegment);

% common part of Wind
state.environment.wind = feval(params.environment.wind.type, params.environment.wind);

% 3D visualization
if params.display3d.on == 1
    state.display3d.figure = figure('Name','3D Window','NumberTitle','off','Position',...
        [20,20,params.display3d.width,params.display3d.height]);
    state.display3d.area = feval(params.environment.area.type, params.environment.area);
end


%%% instantiates the platform objects

for i=1:length(params.platforms)
    p = loadPlatformConfig(params.platforms(i).configfile, params);
    p.X = params.platforms(i).X;
    state.platforms(i)=feval(p.type,p);
end


% cleanup the structure generated by the configuration files
clear params;

end

