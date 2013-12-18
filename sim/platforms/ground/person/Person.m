classdef Person<Entity
    % Class that implementatioms dynamic and sensors of a generic person
    %
    % Pelican Properties:
    % X   - state = [px;py;pz;phi;theta;psi]
    %       px,py,pz         [m]     position (NED coordinates)
    %       phi,theta,psi    [rad]   attitude in Euler angles right-hand ZYX convention
    %       u,v,w            [m/s]   velocity in body coordinates
    %       p,q,r            [rad/s] rotational velocity  in body coordinates
    %
    % eX  - estimated state = [~px;~py;~pz;~theta]
    %       ~px,~py,~pz      [m]     position estimated by GPS (NED coordinates)
    %       ~theta           [rad]   estimated attitude in Euler angles right-hand ZYX convention
    %       0,0,0                    placeholder (the uav does not provide velocity estimation)
    %       ~p,~q,~r         [rad/s] measured rotational velocity in body coordinates
    %       0                        placeholder (the uav does not provide thrust estimation)
    %       ~ax,~ay,~az      [m/s^2] measured acceleration in body coordinates
    %       ~h               [m]     estimated altitude from altimeter NED, POSITIVE UP!
    %       ~pxdot           [m/s]   x velocity from GPS (NED coordinates)
    %       ~pydot           [m/s]   y velocity from GPS (NED coordinates)
    %       ~hdot            [m/s]   altitude rate from altimeter (NED coordinates)
    %
    % U   - controls  = [pt,rl,th,ya,bat]
    %       pt  [-0.89..0.89]  [rad]   commanded pitch
    %       rl  [-0.89..0.89]  [rad]   commanded roll
    %       th  [0..1]         unitless commanded throttle
    %       ya  [-4.4,4.4]     [rad/s] commanded yaw velocity
    %       bat [9..12]        [Volts] battery voltage
    %
    % Pelican Methods:
    %    Car(objparams)     - constructs object
    %    reset()            - resets all the platform subcomponents
    %    setX(X)            - reinitialise the current state and noise
    %    isValid()          - true if the state is valid
    %    getX()             - returns the state (noiseless)
    %    getEX()            - returns the estimated state (noisy)
    %    getEXasX()         - returns the estimated state (noisy) formatted as the noiseless state    
    %
    properties (Constant)
        psize = 1;
    end
    
    properties (Access = protected)
        bb;       % bounding box defined columnwise in, tl,tr,bl,br
    end
    
    methods (Access = public)
        function obj = Person(objparams)
            % constructs the platform object and initialises its subcomponent
            % The configuration of the type and parameters of the subcomponents are read
            % from the platform config file e.g. car_config.m
            %
            % Example:
            %
            %   obj=Person(objparams);
            %                objparams.dt - timestep of this object
            %                objparams.on - 1 if the object is active
            %                objparams.sensors.ahars - ahrs parameters
            %                objparams.sensors.gpsreceiver - gps receiver parameters
            %                objparams.graphics - graphics parameters
            %                objparams.stateLimits - FIX FOR WHEEL SPEED AND STEER ANGLE 13 by 2 vector of allowed values of the state
            %                objparams.collisionDistance - distance from any other object that defines a collision
            %                objparams.dynNoise -  standard deviation of the noise dynamics
            %                objparams.state - handle to simulator state
            %
            
            obj=obj@Entity(objparams);
            
            obj.prngIds = [1;2;3;4;5;6] + obj.simState.numRStreams;
            obj.simState.numRStreams = obj.simState.numRStreams + 6;
            
            assert(isfield(objparams,'stateLimits'),'car:nostatelimits',...
                'the platform config file must define the stateLimits parameter');
            obj.stateLimits = objparams.stateLimits;
            
            assert(isfield(objparams,'collisionDistance'),'car:nocollisiondistance',...
                'the platform config file must define the collisionDistance parameter');
            obj.collisionD = objparams.collisionDistance;
            
            assert(isfield(objparams,'dynNoise'),'car:nodynnoise',...
                'the platform config file must define the dynNoise parameter');
            obj.dynNoise = objparams.dynNoise;
            
            if(isfield(objparams,'behaviourIfStateNotValid'))
                obj.behaviourIfStateNotValid = objparams.behaviourIfStateNotValid;
            end
            
            %instantiation of sensor objects, with some "manual" type checking            
                        
            % GRAPHICS
            assert(isfield(objparams,'graphics')&&isfield(objparams.graphics,'on'),'person:nographics',...
                'the platform config file must define a graphics parameter if not needed set graphics.on = 0');
            objparams.graphics.DT = objparams.DT;
            objparams.graphics.state = objparams.state;
            if(objparams.graphics.on)
                obj.graphicsOn = 1;
                assert(isfield(objparams.graphics,'type'),'person:nographicstype',...
                    'the platform config file must define a graphics.type');
                obj.graphics=feval(objparams.graphics.type,objparams.graphics);
            else
                obj.graphicsOn = 0;
            end
        end
        
        function eX = getEX(obj,varargin)
            % returns the estimated state (noisy)
            % eX = [~px;~py;~pz;~phi;~theta;~psi;0;0;0;~p;~q;~r;0;~ax;~ay;~az;~h;~pxdot;~pydot;~hdot]
            %
            % Examples
            %    allX = obj.getEX(); returns the whole estimated state vector
            %  shortX = obj.getEX(1:3); returns only the first three elements of the estimated state vector
            %
            if(isempty(varargin))
                eX = obj.eX;
            else
                eX = obj.eX(varargin{1});
            end
        end
        
        function obj = setX(obj,X)
            % reinitialise the current state and noise
            %
            % Example:
            %
            %   obj.setState(X)
            %       X - platform new state vector [px,py,pz,phi,theta,psi,u,v,w,p,q,r,thrust]
            %           if the length of the X vector is 12, thrust is initialized automatically
            %           if the length of the X vector is 6, all the velocities are set to zero
            %
            
            obj = setX@Entity(obj,X);
            
            obj.valid = 1;
            
            % clean the trajectory plot if any
            if(obj.graphicsOn)
                obj.graphics.reset();
            end
            
            obj.bootstrapped = 1;
        end        
    end
    methods (Access=protected)
        
        function obj = update(obj,U)
            % updates the state of the platform and of its components
            %
            % In turns this:
            %  updates turbulence model
            %  updates the state of the platform applying controls
            %  updates local part of gps model
            %  updates ahars noise model
            %  updates the graphics
            %
            % Note:
            %  this method is called automatically by the step() of the Steppable parent
            %  class and should not be called directly.
            %
            
            if(obj.valid)
                U=U{1};
                if (length(U)~=2)
                    error('a 2 element column vector [wheel_speed, steer_angle] is expected as input ');
                end
                                
                % dynamics
                %[obj.X obj.a] = ruku2('pelicanODE', obj.X, [US;meanWind + turbWind; obj.MASS; accNoise], obj.dt);
                
                S = U(1) * obj.dt;
                mu = obj.X(6) + U(2);
                obj.X(1) = obj.X(1) + S * cos(mu);
                obj.X(2) = obj.X(2) + S * sin(mu);
                obj.X(6) = obj.X(6) + S * sin(U(2)) / obj.WHEEL_BASE;
                
                if(isreal(obj.X)&& obj.thisStateIsWithinLimits(obj.X) && ~obj.inCollision())
                    
                    % AHARS
                    if (false)
                    obj.ahars.step([obj.X;obj.a]);
                    
                    estimatedAHA = obj.ahars.getMeasurement([obj.X;obj.a]);
                    
                    % GPS
                    obj.gpsreceiver.step(obj.X);
                    
                    estimatedPosNED = obj.gpsreceiver.getMeasurement(obj.X);
                    
                    %return values
                    obj.eX = [estimatedPosNED(1:3);estimatedAHA(1:3);zeros(3,1);...
                        estimatedAHA(4:6);0;estimatedAHA(7:10);estimatedPosNED(4:5);estimatedAHA(11)];
                    end
                    obj.updateAdditional(U);
                    
                    % graphics      
                    if(obj.graphicsOn)
                        obj.graphics.update(obj.X);
                        obj.updateAdditionalGraphics(obj.X);
                    end
                    
                    obj.valid = 1;
                else
                    obj.eX = nan(20,1);
                    obj.valid=0;
                    
                    obj.printStateNotValidError();
                end                
            end
        end        
    end
end
