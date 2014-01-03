classdef Pelican<Platform
    % Class that implementatios dynamic and sensors of an AscTec Pelican quadrotor
    % The parameters are derived from the system identification of one of
    % the UCL quadrotors
    %
    % Pelican Properties:
    % X   - state = [px;py;pz;phi;theta;psi;u;v;w;p;q;r;thrust]
    %       px,py,pz         [m]     position (NED coordinates)
    %       phi,theta,psi    [rad]   attitude in Euler angles right-hand ZYX convention
    %       u,v,w            [m/s]   velocity in body coordinates
    %       p,q,r            [rad/s] rotational velocity  in body coordinates
    %       thrust           [N]     rotors thrust
    %
    % eX  - estimated state = [~px;~py;~pz;~phi;~theta;~psi;0;0;0;~p;~q;~r;0;~ax;~ay;~az;
    %                          ~h;~pxdot;~pydot;~hdot]
    %       ~px,~py,~pz      [m]     position estimated by GPS (NED coordinates)
    %       ~phi,~theta,~psi [rad]   estimated attitude in Euler angles right-hand ZYX convention
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
    %    Pelican(objparams) - constructs object
    %    reset()            - resets all the platform subcomponents
    %    setX(X)            - reinitialise the current state and noise
    %    isValid()          - true if the state is valid
    %    getX()             - returns the state (noiseless)
    %    getEX()            - returns the estimated state (noisy)
    %    getEXasX()         - returns the estimated state (noisy) formatted as the noiseless state    
    %
    properties (Constant)
        CONTROL_LIMITS = [-0.9,0.9; -0.9,0.9; 0,1; -4.5,4.5; 9,12]; %limits of the control inputs
        SI_2_UAVCTRL = [-1/degsToRads(0.025);-1/degsToRads(0.025);4097;-1/degsToRads(254.760/2047);1]; % conversion factors
        BATTERY_RANGE = [9,12]; % range of valid battery values volts
        % The parameters of the system dynamics are defined in the
        % pelicanODE function
        G = 9.81;    %  gravity m/s^2
        MASS = 1.68; %  mass of the platform Kg
        labels = {'px','py','pz','phi','theta','psi','u','v','w','p','q','r','thrust'};
    end
    
    properties (Access = protected)
        aerodynamicTurbulence;  % handle to the aerodynamic turbulence
    end
    
    methods (Access = public)
        function obj = Pelican(objparams)
            % constructs the platform object and initialises its subcomponent
            % The configuration of the type and parameters of the subcomponents are read
            % from the platform config file e.g. pelican_config.m
            %
            % Example:
            %
            %   obj=Pelican(objparams);
            %                objparams.dt - timestep of this object
            %                objparams.on - 1 if the object is active
            %                objparams.aerodynamicturbulence - aerodynamicturbulence parameters
            %                objparams.sensors.ahars - ahrs parameters
            %                objparams.sensors.gpsreceiver - gps receiver parameters
            %                objparams.graphics - graphics parameters
            %                objparams.stateLimits - 13 by 2 vector of allowed values of the state
            %                objparams.collisionDistance - distance from any other object that defines a collision
            %                objparams.dynNoise -  standard deviation of the noise dynamics
            %                objparams.state - handle to simulator state
            %
            
            % Check that the AHARS and the GPS have been set up; these are
            % mandatory for the Pelican
            assert(isfield(objparams.sensors,'ahars')&&isfield(objparams.sensors.ahars,'on'),'pelican:noahars',...
                'the platform config file must define an ahars');

            assert(isfield(objparams.sensors,'gpsreceiver')&&isfield(objparams.sensors.gpsreceiver,'on'),'pelican:nogpsreceiver',...
                'the platform config file must define a gps receiver if not needed set gpsreceiver.on = 0');

            obj=obj@Platform(objparams);

            obj.prngIds = [1;2;3;4;5;6] + obj.simState.numRStreams;
            obj.simState.numRStreams = obj.simState.numRStreams + 6;
            
            assert(isfield(objparams,'dynNoise'),'pelican:nodynnoise',...
                'the platform config file must define the dynNoise parameter');
            obj.dynNoise = objparams.dynNoise;
            
            %instantiation of sensor and wind objects, with some "manual" type checking
            
            % TURBULENCE
            objparams.aerodynamicturbulence.DT = objparams.DT;
            objparams.aerodynamicturbulence.dt = objparams.dt;
            objparams.aerodynamicturbulence.state = objparams.state;
            if(objparams.aerodynamicturbulence.on)
                
                assert(isfield(objparams.aerodynamicturbulence,'type'),'pelican:noaerodynamicturbulencetype',...
                    'the platform config file must define an aerodynamicturbulence.type ');
                
                limits = obj.simState.environment.area.getLimits();
                objparams.aerodynamicturbulence.zOrigin = limits(6);
                
                tmp = feval(objparams.aerodynamicturbulence.type, objparams.aerodynamicturbulence);
                if(isa(tmp,'AerodynamicTurbulence'))
                    obj.aerodynamicTurbulence = tmp;
                else
                    error('c.aerodynamicturbulence.type has to extend the class AerodynamicTurbulence');
                end
            else
                obj.aerodynamicTurbulence = feval('AerodynamicTurbulence', objparams.aerodynamicturbulence);
            end
            
        end
        
        function X = getX(obj,varargin)
            % returns the state (noiseless)
            % X = [px;py;pz;phi;theta;psi;u;v;w;p;q;r;thrust]
            %
            % Examples
            %    allX = obj.getX(); returns the whole state vector
            %  shortX = obj.getX(1:3); returns only the first three elements of teh state vector
            %
            if(isempty(varargin))
                X = obj.X;
            else
                X = obj.X(varargin{1});
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
        
        function iv = isValid(obj)
            % true if the state is valid
            iv = obj.valid;
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
            
            assert((size(X,1)==6)||(size(X,1)==12)||(size(X,1)==13),'pelican:wrongsetstate',...
                'setState() on a pelican object requires an input of length 6, 12 or 13 instead we have %d',size(X,1));
            
            assert(obj.thisStateIsWithinLimits(X),'pelican:settingoobstate',...
                'the state passed through setState() is not valid (i.e. out of limits)');
            
            if(size(X,1)==6)
                X = [X;zeros(6,1)];
            end
            
            if(size(X,1)==12)
                X = [X;abs(obj.MASS*obj.G)];
            end
            
            obj.X = X;
            
            % set things
            obj.gpsreceiver.setState(X);
            obj.ahars.setState(X);            
            obj.aerodynamicTurbulence.setState(obj.X);
            
            obj.a  = zeros(3,1);
            
            
            % now rest to make sure components are initialised correctly
            obj.gpsreceiver.reset();
            obj.aerodynamicTurbulence.reset();
            obj.ahars.reset();
            obj.resetAdditional();            
            
            % get measurements
            estimatedAHA = obj.ahars.getMeasurement([obj.X;obj.a]);
            estimatedPosNED = obj.gpsreceiver.getMeasurement(obj.X);
            
            obj.eX = [estimatedPosNED(1:3);estimatedAHA(1:3);zeros(3,1);...
                estimatedAHA(4:6);0;estimatedAHA(7:10);estimatedPosNED(4:5);estimatedAHA(11)];
            
            obj.valid = 1;
            
            % clean the trajectory plot if any
            if(obj.graphicsOn)
                obj.graphics.reset();
            end
            
            obj.bootstrapped = 1;
        end
    end
    
    methods (Sealed,Access=protected)
        
        function US = scaleControls(obj,U)
            % scales the controls from SI units to what required by the ODE model
            % The dynamic equations (and the real model) require the following input ranges
            % pt  [-2048..2048] 1=4.36332313e-4 rad = 0.25 deg commanded pitch
            % rl  [-2048..2048] 1=4.36332313e-4 rad = 0.25 deg commanded roll
            % th  [0..4096] 1=4.36332313e-4 rad = 0.25 deg commanded throttle
            % ya  [-2048..2048] 1=2.17109414e-3 rad/s = 0.124394531 deg/s commanded yaw velocity
            % bat [9..12] Volts battery voltage
            %
             assert(size(U,1)==5,'pelican:inputoob','wrong size of control inputs should be 5xN\n\tU = [pt;rl;th;ya;bat] \n');
             
             for i = 1:size(U,2),
             assert(~(any(U(:,i)<obj.CONTROL_LIMITS(:,1)) || any(U(:,i)>obj.CONTROL_LIMITS(:,2))),...
                'pelican:inputoob',['control inputs values not within limits \n',...
                '\tU = [pt;rl;th;ya;bat] \n\n\tpt  [-0.9..0.9] rad commanded pitch \n\trl  [-0.9..0.9] rad commanded roll \n',...
                '\tth  [0..1] unitless commanded throttle \n\tya  [-4.5..4.5] rad/s commanded yaw velocity \n\tbat [9..12] Volts battery voltage \n']);
            end
            
            US = U.*obj.SI_2_UAVCTRL;
        end
   end
    
    methods (Access=protected)
                
           function obj = handleStateNotValid(obj)
               obj.eX = nan(20,1);

           end
      
        function obj = updateEntityState(obj,US)
            % updates the state of the platform and of its components
            %
            % In turns this:
            %  updates turbulence model
            %  updates the state of the platform applying controls
            
            %wind and turbulence this closely mimic the Simulink example "Lightweight Airplane Design"
            % asbSkyHogg/Environment/WindModels
            
            if (size(US,1)~=5)
                error('a 5 element column vector [-2048..2048;-2048..2048;0..4096;-2048..2048;9..12] is expected as input ');
            end
            
            meanWind = obj.simState.environment.wind.getLinear(obj.X);

            obj.aerodynamicTurbulence.step(obj.X);
            turbWind = obj.aerodynamicTurbulence.getLinear(obj.X);

            accNoise = obj.dynNoise.*[randn(obj.simState.rStreams{obj.prngIds(1)},1,1);
                                      randn(obj.simState.rStreams{obj.prngIds(2)},1,1);
                                      randn(obj.simState.rStreams{obj.prngIds(3)},1,1);
                                      randn(obj.simState.rStreams{obj.prngIds(4)},1,1);
                                      randn(obj.simState.rStreams{obj.prngIds(5)},1,1);
                                      randn(obj.simState.rStreams{obj.prngIds(6)},1,1)];

            % dynamics
            [obj.X, obj.a] = ruku2('pelicanODE', obj.X, [US;meanWind + turbWind; obj.MASS; accNoise], obj.dt);
        end        
    end
end

