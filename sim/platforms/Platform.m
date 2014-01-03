classdef Platform<Entity
    % Class that implements a platform. A platform extends an entity in
    % some way! It's probably that it's instrumented in some manner.
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
            %   obj=Entity(objparams);
            
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