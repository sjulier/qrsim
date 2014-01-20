classdef Entity<Steppable
    % Class that implements an entity. This is a physical object that is
    % present in the environment and has a mutable state of some kind
    %
    
    properties (Access = protected)
        graphics;    % handle to the graphics
        collisionD;  % distance from any other object that defines a collision
        behaviourIfStateNotValid = 'warning'; % what to do when the state is not valid
        prngIds;     % ids of the prng stream used by this object
        X;           % state [px;py;pz;phi;theta;psi;u;v;w;p;q;r;thrust]
        stateLimits; % 13 by 2 vector of allowed values of the state
        valid;       % the state of the platform is invalid
        graphicsOn;  % true if graphics is on
        dynNoise;    % standard deviation of the noise dynamics
        physicalSize;% the physical size of the object
    end
    
    methods (Access = public)
        function obj = Entity(objparams)
            % constructs the platform object and initialises its subcomponent
            % The configuration of the type and parameters of the subcomponents are read
            % from the platform config file e.g. pelican_config.m
            %
            % Example:
            %
            %   obj=Entity(objparams);
            %                objparams.dt - timestep of this object
            %                objparams.on - 1 if the object is active
            %                objparams.sensors.ahars - ahrs parameters
            %                objparams.sensors.gpsreceiver - gps receiver parameters
            %                objparams.graphics - graphics parameters
            %                objparams.stateLimits - 13 by 2 vector of allowed values of the state
            %                objparams.collisionDistance - distance from any other object that defines a collision
            %                objparams.dynNoise -  standard deviation of the noise dynamics
            %                objparams.state - handle to simulator state
            %                objparams.physicalSize - the physical size of
            %                the entity (length of sides of axis-aligned
            %                bounding box?)
            %
            
            obj=obj@Steppable(objparams);
            
            assert(isfield(objparams,'stateLimits'),'entity:nostatelimits',...
                'the platform config file must define the stateLimits parameter');
            obj.stateLimits = objparams.stateLimits;
            
            assert(isfield(objparams,'collisionDistance'),'entity:nocollisiondistance',...
                'the platform config file must define the collisionDistance parameter');
            obj.collisionD = objparams.collisionDistance;
                        
            if(isfield(objparams,'behaviourIfStateNotValid'))
                obj.behaviourIfStateNotValid = objparams.behaviourIfStateNotValid;
            end

            if(isfield(objparams,'physicalSize'))
                obj.physicalSize = objparams.physicalSize;
            else
                obj.physicalSize = zeros(3, 1);
            end
                        
            assert(isfield(objparams,'dynNoise'),'entity:nodynnoise',...
                'the platform config file must define the dynNoise parameter');
            obj.dynNoise = objparams.dynNoise;
                        
            % GRAPHICS
            assert(isfield(objparams,'graphics')&&isfield(objparams.graphics,'on'),'entity:nographics',...
                'the platform config file must define a graphics parameter if not needed set graphics.on = 0');
            objparams.graphics.DT = objparams.DT;
            objparams.graphics.state = objparams.state;
            if(objparams.graphics.on)
                obj.graphicsOn = 1;
                assert(isfield(objparams.graphics,'type'),'entity:nographicstype',...
                    'the platform config file must define a graphics.type');
                obj.graphics=feval(objparams.graphics.type,objparams.graphics);
            else
                obj.graphicsOn = 0;
            end
        end
                
        function iv = isValid(obj)
            % true if the state is valid
            iv = obj.valid;
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
        
        function obj = setX(obj,X)
            % reinitialise the current state and noise
            %
            % Example:
            %
            %   obj.setState(X)
            %       X - platform new state vector [px,py,pz,phi,theta,psi,u,v,w,p,q,r]
            %           if the length of the X vector is 6, all the velocities are set to zero
            %
            
            assert((size(X,1)==4)||(size(X,1)==6)||(size(X,1)==12)||(size(X,1)==13),'entity:wrongsetstate',...
                'setState() on an entity object requires an input of length 4, 6, 12 or 13 instead we have %d',size(X,1));
            
            X
            
            assert(obj.thisStateIsWithinLimits(X),'entity:settingoobstate',...
                'the state passed through setState() is not valid (i.e. out of limits)');

            if(size(X,1)==4)
                X = [X;zeros(2,1)];
            end
            
            if(size(X,1)==6)
                X = [X;zeros(6,1)];
            end
            
            if(size(X,1)==12)
                X = [X;abs(obj.MASS*obj.G)];
            end
            
            obj.X = X;

            % update the graphics (if enabled)
            if(obj.graphicsOn)
                obj.graphics.reset();
            end
            
            obj.valid = 1;
            
            obj.bootstrapped = 1;

        end
        
        function obj = reset(obj)
            % resets all the platform subcomponents
            %
            % Example:
            %   obj.reset();
            %
            assert(false,'car:person',['A platform can not be simply reset since that would its state undefined',...
                ' use setX instead, that will take care of resetting what necessary']);
        end
        
        function d = getCollisionDistance(obj)
            % returns collision distance
            d = obj.collisionD;
        end
        
        function physicalSize = getPhysicalSize(obj)
            physicalSize = obj.physicalSize;
        end
    end   

    
    methods (Access=protected)
        
        function coll = inCollision(obj)
            % returns 1 if a collision is occourring
            coll = 0;
            for i=1:length(obj.simState.platforms),
                if(obj.simState.platforms{i} ~= obj)
                    if(norm(obj.simState.platforms{i}.X(1:3)-obj.X(1:3))< obj.collisionD)
                        coll = 1;
                    end
                end
            end
            if ((coll == 0) && isfield(obj.simState, 'entities'))
                for i=1:length(obj.simState.entities),
                    if(obj.simState.entities{i} ~= obj)
                        if(norm(obj.simState.entities{i}.X(1:3)-obj.X(1:3))< obj.collisionD)
                            coll = 1;
                        end
                    end
                end
            end
        end
        
        function obj = printStateNotValidError(obj)
            % display state error info
            
            obj.X
            dbstack
            keyboard
            
            if(strcmp(obj.behaviourIfStateNotValid,'continue'))
                
            else
                if(strcmp(obj.behaviourIfStateNotValid,'error'))
                    if(obj.inCollision())
                        error('entity state not valid, in collision!\n');
                    else
                        error('entity state not valid, values out of bounds!\n');
                    end
                else
                    if(obj.inCollision())
                        fprintf(['warning: entity state not valid, in collision!\n Normally this should not happen; ',...
                            'however if you think this is fine and you want to stop this warning use the task parameter behaviourIfStateNotValid\n']);
                    else
                        ids = (obj.X(1:12) < obj.stateLimits(:,1)) | (obj.X(1:12) > obj.stateLimits(:,2));
                        problematics = '';
                        for k=1:size(ids)
                            if(ids(k))
                                problematics = [problematics,',',obj.labels{k}]; %#ok<AGROW>
                            end
                        end
                        fprintf(['warning: entity state not valid, values out of bounds (',problematics,')!\n',num2str(obj.X'),'\nNormally this should not happen; ',...
                            'however if you think this is fine and you want to stop this warning use the task parameter behaviourIfStateNotValid\n']);
                    end
                end
            end
        end
    end
    
    
    methods (Sealed,Access=protected)        
        function valid = thisStateIsWithinLimits(obj,X)
            % returns 0 if the state is out of bounds
            to = min(size(X,1),size(obj.stateLimits,1));
            
            valid = all(X(1:to)>=obj.stateLimits(1:to,1)) && all(X(1:to)<=obj.stateLimits(1:to,2));
        end        
    end
 
   methods (Access = protected)
       % used by subclasses to define the behaviour
       function obj = updateEntityState(obj, US)
       end
   end
       
    methods (Access = protected)
        
        function obj=resetAdditional(obj)
           % used by subclasses to reset additional stuff 
        end
        
        function obj=updateAdditional(obj,~)
           % used by subclasses to update additional stuff 
        end
         
        function obj=updateAdditionalGraphics(obj,~)
           % used by subclasses to update additional graphics stuff 
        end
    end
end