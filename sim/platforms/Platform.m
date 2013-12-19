classdef Platform<Entity
    % Class that implements a platform. A platform extends an entity in
    % some way! It's probably that it's instrumented in some manner.
    %
    
    properties (Access = protected)
        gpsreceiver; % handle to the gps receiver
        ahars ;      % handle to the attitude heading altitude reference system
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
    end             
end