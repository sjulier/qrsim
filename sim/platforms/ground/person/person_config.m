% This script defines all the typical parameters for a person
% 
% These paramters must be loaded using the function loadConfig and are then
% passed to the platform constructor, which will take care of propagating the correct 
% parameters to each of the objects part of a platform.  
%
% Example of use:
%
%  entity  = loadConfig('person_config');
%  n = Person(entity);
% 
% note: generally the loading is performed automatically by qrsim
%
%
% GENERAL NOTES:
% - if the on flag is zero, a NOISELESS version of the object is loaded instead
% - the parameter dt MUST be always specified even if on=0
%
    
if(~exist('params','var'))
    error('The platform parameters must be loaded after the global parameters');
end

% platforms %
c.dt = 0.02;
c.on = 1;
c.type = 'Person';

% max and min limits for each of the state variables, exceeding this limits
% makes the state invalid (i.e. 19x1 nan)
c.stateLimits =[params.environment.area.limits(1:2);params.environment.area.limits(3:4);...
    params.environment.area.limits(5:6);... % position limits defined by the area
    -pi,pi;-pi,pi;-10*pi,10*pi;... % attitude limits
    -15,15;-15,15;-15,15;... % linear velocity limits
    -3,3;-3,3;-3,3]; %rotational velocity limits

c.collisionDistance = 2; % two platforms closer than this distance are deemed in collision 
c.dynNoise = [0.2;0.2;0.2;0.2;0.2;0.2];

% Graphics
c.graphics.type = 'PersonGraphics';
c.graphics.trajectory = 1; % plot trajectory
c.graphics.BT = 1;      % body thickness m
c.graphics.BH = 2;      % body height m
c.graphics.trjLength = 100000;
