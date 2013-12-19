classdef Car<Platform
    % Class that implementatios dynamic and sensors of a generic car
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
        WHEEL_BASE = 2; % the wheelbase between the front and rear wheels
        CONTROL_LIMITS = [-5,25; -1,1]; %limits of the control inputs
        labels = {'px','py','pz','phi','theta','psi','u','v','w','p','q','r','thrust'};
    end
    
    properties (Access = protected)
        dynNoise;    % standard deviation of the noise dynamics
        eX;          % estimated state  [~px;~py;~pz;~phi;~theta;~psi]
    end
    
    methods (Access = public)
        function obj = Car(objparams)
            % constructs the platform object and initialises its subcomponent
            % The configuration of the type and parameters of the subcomponents are read
            % from the platform config file e.g. car_config.m
            %
            % Example:
            %
            %   obj=Car(objparams);
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
            
            obj=obj@Platform(objparams);
            
            obj.prngIds = [1;2;3;4;5;6] + obj.simState.numRStreams;
            obj.simState.numRStreams = obj.simState.numRStreams + 6;
            
            assert(isfield(objparams,'dynNoise'),'car:nodynnoise',...
                'the platform config file must define the dynNoise parameter');
            obj.dynNoise = objparams.dynNoise;
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
                
        function X = getEXasX(obj,varargin)
            % returns the estimated state (noisy) formatted as the noiseless state
            % eX = [~px;~py;-~h;~phi;~theta;~psi;~u;~v;~w;~p;~q;~r]
            %
            % Examples
            %    allX = obj.getEXasX(); returns the whole 12 elements state vector
            %  shortX = obj.getEXasX(1:3); returns only the first three elements of the state vector
            %
            uvw = dcm(obj.X)*[obj.eX(18:19);-obj.eX(20)];
            X = [obj.eX(1:2);-obj.eX(17);obj.eX(4:6);uvw;obj.eX(10:12)];
            if(~isempty(varargin))
                X = X(varargin{1});
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
            
            % set things
            if (false)
            obj.gpsreceiver.setState(X);
            obj.ahars.setState(X);  
            
            %obj.a  = zeros(3,1);
            
            
            % now rest to make sure components are initialised correctly
            obj.gpsreceiver.reset();
            obj.ahars.reset();
            obj.resetAdditional();            
            
            % get measurements
            estimatedAHA = obj.ahars.getMeasurement([obj.X;obj.a]);
            estimatedPosNED = obj.gpsreceiver.getMeasurement(obj.X);
            
            obj.eX = [estimatedPosNED(1:3);estimatedAHA(1:3);zeros(3,1);...
                estimatedAHA(4:6);0;estimatedAHA(7:10);estimatedPosNED(4:5);estimatedAHA(11)];
            
            end
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
                if (length(U)~=2)
                    error('a 2 element column vector [wheel_speed, steer_angle] is expected as input ');
                end
                                
                % dynamics
                
                S = U(1) * obj.dt;
                mu = obj.X(6) + U(2);
                obj.X(1) = obj.X(1) + S * cos(mu);
                obj.X(2) = obj.X(2) + S * sin(mu);
                obj.X(6) = obj.X(6) + S * sin(U(2)) / obj.WHEEL_BASE;
                
                % Assumes coordinate system is over front wheel
                obj.X(7) = U(1) * cos(U(2));
                obj.X(8) = U(1) * sin(U(2));
                
                obj.X(12) = U(1) * sin(U(2)) / obj.WHEEL_BASE;
                
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
