classdef Platform<Steppable
    % Class that implements a platform.
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
            %                objparams.dt - timestep of this object
            %                objparams.on - 1 if the object is active
            %                objparams.sensors.ahars - ahrs parameters
            %                objparams.sensors.gpsreceiver - gps receiver parameters
            %                objparams.graphics - graphics parameters
            %                objparams.stateLimits - 13 by 2 vector of allowed values of the state
            %                objparams.collisionDistance - distance from any other object that defines a collision
            %                objparams.dynNoise -  standard deviation of the noise dynamics
            %                objparams.state - handle to simulator state
            %
            
            obj=obj@Steppable(objparams);
            
            % GRAPHICS
            assert(isfield(objparams,'graphics')&&isfield(objparams.graphics,'on'),'platform:nographics',...
                'the platform config file must define a graphics parameter if not needed set graphics.on = 0');
            objparams.graphics.DT = objparams.DT;
            objparams.graphics.state = objparams.state;
            if(objparams.graphics.on)
                obj.graphicsOn = 1;
                assert(isfield(objparams.graphics,'type'),'platform:nographicstype',...
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
    end
    
    methods (Access = public, Abstract)        
        setX(obj,X);
        % sets the platform state to the value passed             
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
        end
        
        function obj = printStateNotValidError(obj)
            % display state error info
            if(strcmp(obj.behaviourIfStateNotValid,'continue'))
                
            else
                if(strcmp(obj.behaviourIfStateNotValid,'error'))
                    if(obj.inCollision())
                        error('platform state not valid, in collision!\n');
                    else
                        error('platform state not valid, values out of bounds!\n');
                    end
                else
                    if(obj.inCollision())
                        fprintf(['warning: platform state not valid, in collision!\n Normally this should not happen; ',...
                            'however if you think this is fine and you want to stop this warning use the task parameter behaviourIfStateNotValid\n']);
                    else
                        ids = (obj.X(1:12) < obj.stateLimits(:,1)) | (obj.X(1:12) > obj.stateLimits(:,2));
                        problematics = '';
                        for k=1:size(ids)
                            if(ids(k))
                                problematics = [problematics,',',obj.labels{k}]; %#ok<AGROW>
                            end
                        end
                        fprintf(['warning: platform state not valid, values out of bounds (',problematics,')!\n',num2str(obj.X'),'\nNormally this should not happen; ',...
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
 
    methods (Access=protected)
        
        function obj=resetAdditional(obj)
           % used by subclasses to reset additional stuff 
        end
        
        function obj=updateAdditional(obj,U)
           % used by subclasses to update additional stuff 
        end
         
        function obj=updateAdditionalGraphics(obj,X)
           % used by subclasses to update additional graphics stuff 
        end
    end
end