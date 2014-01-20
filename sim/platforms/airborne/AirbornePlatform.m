classdef AirbornePlatform<Platform
    % Class that implements an airborne platform. This extends a regular
    % platform by adding turbulence
    
    properties (Access = protected)
        aerodynamicTurbulence;  % handle to the aerodynamic turbulence
    end
    
    methods (Access = public)
        function obj = AirbornePlatform(objparams)
            % constructs the platform object and initialises its subcomponent
            % The configuration of the type and parameters of the subcomponents are read
            % from the platform config file e.g. pelican_config.m
            %
            % Example:
            %
            %   obj=Platform(objparams);
            
            obj=obj@Platform(objparams);
            
            %instantiation of sensor and wind objects, with some "manual" type checking
            
            % TURBULENCE
            objparams.aerodynamicturbulence.DT = objparams.DT;
            objparams.aerodynamicturbulence.dt = objparams.dt;
            objparams.aerodynamicturbulence.state = objparams.state;
            if(objparams.aerodynamicturbulence.on)
                
                assert(isfield(objparams.aerodynamicturbulence,'type'),'airborneplatform:noaerodynamicturbulencetype',...
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
        
        function obj = setX(obj,X)

            obj.aerodynamicTurbulence.setState(obj.X);
            obj.aerodynamicTurbulence.reset();

            % Call base function
            obj = setX@Platform(obj,X);
            size(obj.X)
            keyboard
        end
    end
end