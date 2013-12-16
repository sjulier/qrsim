classdef BoxWithSteppablesArea<BoxArea
    
    % Defines a simple box shaped area in which the ground is split in
    % areas of different classes and there are persons present at randomly
    % generated locations.
    %
    % BoxWithPersonsArea Methods:
    %    BoxWithPersonsArea(objparams)  - constructs the object
    %    reset()                        - does nothing
    %    getOriginUTMCoords()           - returns origin
    %    getLimits()                    - returns limits
    %                                     a one if the person was found in the current timestep
    %    getSteppables()                - returns the array of steppables objects
    %    getTerrainClass(pts)           - returns the terrain class at the specified gound points
    %    
    properties (Access=protected)
        steppables;       % the array of the steppables stored in this object
    end
    
    methods (Access=public)
        function obj = BoxWithSteppablesArea(objparams)
            % constructs the object
            %
            % Example:
            %
            %   obj=BoxWithSteppablesArea(objparams)
            %               objparams.limits - x,y,z limits of the area
            %               objparams.originutmcoords - structure containing the origin in utm coord
            %               objparams.graphics.type - class type for the graphics object
            %                                         (only needed if the 3D display is active)
            %               objparams.graphics.backgroundimage - background image
            %               objparams.state - handle to the simulator state
            %
            
            if(~isfield(objparams,'dt'))
                % this is a static object
                objparams.dt = 3600*objparams.DT;
            end
            objparams.on = 1;
            
            % call parents constructors
            obj=obj@BoxArea(objparams);
        end
        
        function num_steppables = addSteppable(obj, newSteppable)
                assert(isfield(newSteppable, 'simState'),'boxwithsteppablesarea:notasteppable','Only steppable objects can be added');
                obj.steppables = [obj.steppables newSteppable];
                num_steppables = size(obj.steppables);
        end
        
        function steppables = getSteppables(obj)
                steppables = obj.steppables;
        end
        
        function on = isGraphicsOn(obj)
            % returns true if the 3D graphics is on
            on = obj.graphicsOn;
        end
    end
    
    methods (Access=public)
        function obj = reset(obj)
           % Delegate whatever is required in the base class
           
           obj = BoxArea.reset(obj);        
           % clear the set of steppables
           obj.steppables = [];
        end
    end
    
    methods (Access=protected)        
        function obj = update(obj, args)
            % Step all the steppables in the environment
            for k = 1 : length(obj.steppables)
                obj.steppables(k).update(args);
            end
        end
    end
end
