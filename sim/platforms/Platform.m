classdef Platform<Entity
    % Class that implements a platform. A platform extends an entity by
    % providing a potentially instrumented set of sensors.
    %
    
    properties (Access = protected)
        gpsreceiver; % handle to the gps receiver
        ahars ;      % handle to the attitude heading altitude reference system
        a;           % linear accelerations in body coordinates [ax;ay;az]
        eX;          % estimated state  [~px;~py;~pz;~phi;~theta;~psi;0;0;0;~p;~q;~r;0;~ax;~ay;~az;~h;~pxdot;~pydot;~hdot]
    end
    
    methods (Access = public)
        function obj = Platform(objparams)
            % constructs the platform object and initialises its subcomponent
            % The configuration of the type and parameters of the subcomponents are read
            % from the platform config file e.g. pelican_config.m
            %
            % Example:
            %
            %   obj=Platform(objparams);
            
            obj=obj@Entity(objparams);
            
            % AHARS
            if (isfield(objparams.sensors,'ahars')&&isfield(objparams.sensors.ahars,'on'))
                objparams.sensors.ahars.DT = objparams.DT;
                assert(isfield(objparams.sensors.ahars,'type'),'platform:noaharstype',...
                'the platform config file must define an ahars.type');
                objparams.sensors.ahars.state = objparams.state;
                tmp = feval(objparams.sensors.ahars.type,objparams.sensors.ahars);
                if(isa(tmp,'AHARS'))
                    obj.ahars = tmp;
                else
                    error('c.sensors.ahars.type has to extend the class AHRS');
                end
            end
            
            % GPS
            if (isfield(objparams.sensors,'gpsreceiver')&&isfield(objparams.sensors.gpsreceiver,'on'))
                objparams.sensors.gpsreceiver.DT = objparams.DT;
                objparams.sensors.gpsreceiver.state = objparams.state;
                if(objparams.sensors.gpsreceiver.on)                
                    assert(~isa(class(obj.simState.environment.gpsspacesegment),'GPSSpaceSegment'),...
                        'platform:nogpsspacesegment','the task config file must define a gpsspacesegment if a gps receiver is in use');

                    assert(isfield(objparams.sensors.gpsreceiver,'type'),'platform:nogpsreceivertype',...
                        'the platform config file must define a gpsreceiver.type');
                    tmp = feval(objparams.sensors.gpsreceiver.type,objparams.sensors.gpsreceiver);
                    if(isa(tmp,'GPSReceiver'))
                        obj.gpsreceiver = tmp;
                    else
                        error('c.sensors.gpsreceiver.type has to extend the class GPSReceiver');
                    end
                else
                    obj.gpsreceiver = feval('GPSReceiver',objparams.sensors.gpsreceiver);
                end                        
            end
        end
        
        function obj = setX(obj,X)
            
            % Call base function
            obj = setX@Entity(obj,X);
            
            % set things
            if (~isempty(obj.gpsreceiver))
                obj.gpsreceiver.setState(obj.X);
                obj.gpsreceiver.reset();
            end
            
            if (~isempty(obj.ahars))
                obj.ahars.setState(obj.X);
                obj.ahars.reset();

            end
            
            obj.a  = zeros(3,1);
                        
            % now rest to make sure components are initialised correctly
            obj.resetAdditional();            
            
            obj.eX = zeros(20, 1);
            
            % get measurements
            if (~isempty(obj.ahars))
                estimatedAHA = obj.ahars.getMeasurement([obj.X;obj.a]);
                obj.eX(4:6) = estimatedAHA(1:3);
                obj.eX(10:12) = estimatedAHA(4:6);
                obj.eX(14:17) = estimatedAHA(7:10);
                obj.eX(20) = estimatedAHA(11);
            end

            % GPS
            if (~isempty(obj.gpsreceiver))
                estimatedPosNED = obj.gpsreceiver.getMeasurement(obj.X);
                obj.eX(1:3) = estimatedPosNED(1:3);
                obj.eX(18:19) = estimatedPosNED(4:5);
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
    end
    
    methods (Access = protected)
        
        function US = scaleControls(~, U)
            US = U;
        end
        
        function obj = handleStateNotValid(~)
        end
        
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
                
                disp('Going in...')
                obj.X
                
                % do scaling of inputs
                US = obj.scaleControls(U);
                
                obj.updateEntityState(US);
                                
                if(isreal(obj.X)&& obj.thisStateIsWithinLimits(obj.X) && ~obj.inCollision())
                    
                    obj.updateSensors(U);
                    
                    obj.updateAdditional(U);
                    
                    obj.updateGraphics(U);

                    obj.valid = 1;
                else
                    obj.handleStateNotValid();
                    
                    obj.valid=0;
                    
                    obj.printStateNotValidError();
                end                
            end
        end
        
       function obj = updateSensors(obj, ~)
            obj.eX = zeros(19, 1);
           
            % AHARS
            if (isfield(obj,'ahars'))
                obj.ahars.step([obj.X;obj.a]);
                estimatedAHA = obj.ahars.getMeasurement([obj.X;obj.a]);
                obj.eX(4:6) = estimatedAHA(1:3);
                obj.eX(10:12) = estimatedAHA(4:6);
                obj.eX(14:17) = estimatedAHA(7:10);
                obj.eX(20) = estimatedAHA(11);
            end

            % GPS
            if (isfield(obj, 'gpsreceiver'))
                obj.gpsreceiver.step(obj.X);

                estimatedPosNED = obj.gpsreceiver.getMeasurement(obj.X);
                obj.eX(1:3) = estimatedPosNED(1:3);
                obj.eX(18:19) = estimatedPosNED(4:5);
            end
          
            %return values
            %obj.eX = [estimatedPosNED(1:3);estimatedAHA(1:3);zeros(3,1);...
            %    estimatedAHA(4:6);0;estimatedAHA(7:10);estimatedPosNED(4:5);estimatedAHA(11)];
   end
       
       function obj = updateGraphics(obj, ~)
                               % graphics      
            if(obj.graphicsOn)
                obj.graphics.update(obj.X);
                obj.updateAdditionalGraphics(obj.X);
            end
       end
    end
end