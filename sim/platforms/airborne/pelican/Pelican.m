classdef Pelican<AirbornePlatform
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

            obj=obj@AirbornePlatform(objparams);

            obj.prngIds = [1;2;3;4;5;6] + obj.simState.numRStreams;
            obj.simState.numRStreams = obj.simState.numRStreams + 6;
        end
                
        function iv = isValid(obj)
            % true if the state is valid
            iv = obj.valid;
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

