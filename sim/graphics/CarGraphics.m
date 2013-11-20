classdef CarGraphics<PlatformGraphics
    % Class that handles the 3D visualization of a car helicopter
    % This implementation is very basic but has the advantage of not
    % depending on any additional toolbox
    %
    % CarGraphics Methods:
    %   CarGraphics(initX,params)  - constructs the object
    %   update()                       - updates the visualization according to the current state
    %
    
    properties (Access = private)
        
        % body
        BW % body width m
        BT % body thickness m
        
        % wheels
        R % wheel radius m
        DFT % distance from truss m
        
        gHandle         % graphic handle
        plotTrj         % 1 to enable trajectory plotting
        X               % state
        alphaValue;     % transparency value
        superimposedRenderingInterval % if defined the quadrotor pose is rendered anew every
                                      % interval steps and superimposed to the past poses
        renderCnt = 0;  % ref counter for this graphics object      
        lw;             % trajectory line widthd
        trjLength;      % displayed trajectory length  
        
        color;          % color
    end
    
    methods (Sealed)
        function obj=CarGraphics(objparams)
            % constructs the object
            %
            % Example:
            %   obj =  QuadrotorGraphics(params);
            %          params.AL - arm length m
            %          params.AT - arm width m
            %          params.AW - arm thickness m
            %          params.BW - body width m
            %          params.BT - body thickness m
            %          params.R - rotor radius m
            %          params.DFT - distance from truss m
            %          params.on - 1 if graphics is active
            %          params.trajectory - 1 if plotting of trajectory is active
            %          params.trjLw - trajectory line width
            %          params.trjLength - trajectory length
            %          params.alphaValue - transparency value
            %          params.color - color character
            
            obj=obj@PlatformGraphics(objparams);
            
            assert(isfield(objparams,'BW'),'cargraphics:nopar',...
                'the platform configuration file need to define the parameter BW');
            assert(isfield(objparams,'BT'),'cargraphics:nopar',...
                'the platform configuration file need to define the parameter BT');
            assert(isfield(objparams,'R'),'cargraphics:nopar',...
                'the platform configuration file need to define the parameter R');
            assert(isfield(objparams,'DFT'),'cargraphics:nopar',...
                'the platform configuration file need to define the parameter DFT');
            assert(isfield(objparams,'trajectory'),'cargraphics:nopar',...
                'the platform configuration file need to define the parameter trajectory');
            
            if(isfield(objparams,'superimposedrenderinginterval'))
                obj.superimposedRenderingInterval = objparams.superimposedrenderinginterval;
            end
            
            if (isfield(objparams,'trjLw'))
                obj.lw = objparams.trjLw;
            else
                obj.lw = 2;
            end
            
            if (isfield(objparams,'trjLength'))
                obj.trjLength = objparams.trjLength;
            else
                obj.trjLength = 100;
            end
            
            if (isfield(objparams,'alphaValue'))
                obj.alphaValue = objparams.alphaValue;
            else
                obj.alphaValue = 1;
            end
            
            if (isfield(objparams,'color'))
                obj.color = objparams.color;
            else
                obj.color = 'b';
            end
            
            % body
            obj.BW = objparams.BW; % body width m
            obj.BT = objparams.BT; % body thickness m
            
            % wheels
            obj.R = objparams.R; % wheel radius m
            obj.DFT = objparams.DFT; % distance from truss m
            
            % trajectory
            obj.plotTrj = objparams.trajectory;
            
            % arbitrary initial position and orientation
            obj.X=zeros(6,1);
            
            obj.initGlobalGraphics();
            obj.createGraphicsHandlers();
        end
        
        function obj = update(obj,X)
            % updates the visualization according to the current state
            %
            % Example:
            %   updateGraphics()
            %
            
            obj.X = X(1:6);
            
            set(0,'CurrentFigure',obj.simState.display3d.figure)
            
            % rotations and translation
            C = dcm(obj.X);
            T = [obj.X(1),obj.X(2),obj.X(3)];
            
            % body translation
            TTB = repmat(T,size(obj.simState.display3d.cargraphicobject.b{1},1),1);
           
            % wheel translation
            TTR = repmat(T,size(obj.simState.display3d.cargraphicobject.wheel{1},1),1);
            
            if(isempty(obj.superimposedRenderingInterval))
                % update body
                for i=1:size(obj.simState.display3d.cargraphicobject.b,2),
                    b = ((obj.simState.display3d.cargraphicobject.b{i})*C)+TTB;
                    set(obj.gHandle.b(i),'Vertices',b);
                end
                
                % update wheels
                for i=1:size(obj.simState.display3d.cargraphicobject.wheel,2),
                    r = ((obj.simState.display3d.cargraphicobject.wheel{i})*C)+TTR;
                    set(obj.gHandle.r(i),'XData',r(:,1));
                    set(obj.gHandle.r(i),'YData',r(:,2));
                    set(obj.gHandle.r(i),'ZData',r(:,3));
                end
                
            else
                if(mod(obj.renderCnt,obj.superimposedRenderingInterval)==0)
                    for i=1:size(obj.simState.display3d.cargraphicobject.b,2),
                        b = ((obj.simState.display3d.cargraphicobject.b{i})*C)+TTB;
                        obj.gHandle.b(i) = patch('Vertices',b,'Faces',obj.simState.display3d.cargraphicobject.bf);
                        set(obj.gHandle.b(i) ,'FaceAlpha',obj.alphaValue,'EdgeAlpha',0);
                    end
                    
                    for i=1:size(obj.simState.display3d.cargraphicobject.rotor,2),
                        r = ((obj.simState.display3d.cargraphicobject.rotor{i})*C)+TTR;
                        if (i==1)
                            obj.gHandle.r(i)= patch(r(:,1),r(:,2),r(:,3),'r');
                        else
                            obj.gHandle.r(i) = patch(r(:,1),r(:,2),r(:,3),obj.color);
                        end
                        set(obj.gHandle.r(i) ,'FaceAlpha',obj.alphaValue,'EdgeAlpha',0);
                    end
                end
            end
                       
            if (obj.plotTrj)
                s = max(1,length(obj.gHandle.trjData.x)-obj.trjLength);
                if(isempty(obj.gHandle.trjData.x))
                    obj.gHandle.trjData.x = obj.X(1);
                    obj.gHandle.trjData.y = obj.X(2);
                    obj.gHandle.trjData.z = obj.X(3);
                    obj.gHandle.trjLine = line('XData',obj.gHandle.trjData.x,'YData',obj.gHandle.trjData.y,...
                    'ZData',obj.gHandle.trjData.z,'LineWidth',obj.lw);
                else
                    obj.gHandle.trjData.x = [obj.gHandle.trjData.x(s:end) obj.X(1)];
                    obj.gHandle.trjData.y = [obj.gHandle.trjData.y(s:end) obj.X(2)];
                    obj.gHandle.trjData.z = [obj.gHandle.trjData.z(s:end) obj.X(3)];
                end
                set(obj.gHandle.trjLine,'XData',obj.gHandle.trjData.x);
                set(obj.gHandle.trjLine,'YData',obj.gHandle.trjData.y);                
                set(obj.gHandle.trjLine,'ZData',obj.gHandle.trjData.z);
            end
            obj.renderCnt = obj.renderCnt + 1;
        end
        
        function obj = reset(obj)
            % clears visualizations
            obj.gHandle.trjData.x = [];
            obj.gHandle.trjData.y = [];
            obj.gHandle.trjData.z = [];
        end
        
        function obj = setTrjLw(obj,lw)
            % sets trajectory line width
            obj.lw = lw;
        end
    end
    
    methods (Sealed,Access=private)        
        function obj=createGraphicsHandlers(obj)
            % creates the necessary graphics handlers and stores them
            %
            % Example:
            %    obj.createGraphicsHandlers()
            %
            
            set(0,'CurrentFigure',obj.simState.display3d.figure)
            
            % initial translation and orientation
            C = dcm(obj.X);
            T = [obj.X(1),obj.X(2),obj.X(3)];
            
            TTB = repmat(T,size(obj.simState.display3d.cargraphicobject.b{1},1),1);
            for i=1:size(obj.simState.display3d.cargraphicobject.b,2),
                b = ((obj.simState.display3d.cargraphicobject.b{i})*C)+TTB;
                obj.gHandle.b(i) = patch('Vertices',b,'Faces',obj.simState.display3d.cargraphicobject.bf);
                set(obj.gHandle.b(i) ,'FaceAlpha',obj.alphaValue,'EdgeAlpha',0);
            end
            
            TTR = repmat(T,size(obj.simState.display3d.cargraphicobject.wheel{1},1),1);
            for i=1:size(obj.simState.display3d.cargraphicobject.wheel,2),
                r = ((obj.simState.display3d.cargraphicobject.wheel{i})*C)+TTR;
                if (i==1)
                    obj.gHandle.r(i)= patch(r(:,1),r(:,2),r(:,3),'r');
                else
                    obj.gHandle.r(i) = patch(r(:,1),r(:,2),r(:,3),obj.color);
                end
                set(obj.gHandle.r(i) ,'FaceAlpha',obj.alphaValue,'EdgeAlpha',0);
            end
            
            obj.gHandle.trjData.x = [];
            obj.gHandle.trjData.y = [];
            obj.gHandle.trjData.z = [];
            
        end
        
        function obj = initGlobalGraphics(obj)
            % creates the necessary graphics primitives only once for all helicopters
            % Dimension according to the constants AL,AT,AW,BW,BT,R,DFT
            %
            % Example:
            %    obj.initGlobalGraphics()
            %
            
            if(~exist('obj.simState.display3d.carGexists','var'))
                %%% body
                bw = obj.BW/2; % half body width
                bt = obj.BT/2; % half body thickness
                bd = obj.BW*sqrt(2); % body diagonal
%                cube = [-1, 1, 1;-1, 1,-1;-1,-1,-1;-1,-1, 1; 1, 1, 1; 1, 1,-1; 1,-1,-1;1,-1, 1];
                cube = [-1, 1, 1;-1, 1,-1;-1,-1,-1;-1,-1, 1; 1, 1, 1; 1, 1,-1; 1,-1,-1;1,-1, 1];
                
                % body
                obj.simState.display3d.cargraphicobject.b{1} =  (cube.*repmat([bw,bw,bt],size(cube,1),1));
                
                obj.simState.display3d.cargraphicobject.bf = [1 2 3 4; 5 6 7 8; 4 3 7 8; 1 5 6 2; 1 4 8 5; 6 7 3 2];
                
                %%% wheels
                r = 0:pi/8:2*pi;
                sr = size(r,2);
                disc = [sin(r).*obj.R;-zeros(1,sr);cos(r).*obj.R]';
                %[xc, yc, zc]=cylinder();
                %sr = size(xc,2);
                %wheel=[xc;yc;zc];                
                
                %obj.simState.display3d.cargraphicobject.wheel{1}.xc = xc + repmat(bw,sr,2);
                %obj.simState.display3d.cargraphicobject.wheel{1}.yc = yc + 0 * repmat(bw,sr,2);
                %obj.simState.display3d.cargraphicobject.wheel{2}.zc = zc + 0 * repmat(bw,sr,2);

                obj.simState.display3d.cargraphicobject.wheel{1} = disc + repmat([bw,bw,0],sr,1);
                obj.simState.display3d.cargraphicobject.wheel{2} = disc + repmat([-bw,bw,0],sr,1);
                obj.simState.display3d.cargraphicobject.wheel{3} = disc + repmat([-bw,-bw,0],sr,1);
                obj.simState.display3d.cargraphicobject.wheel{4} = disc + repmat([bw,-bw,0],sr,1);
                obj.simState.display3d.cargraphicobject.waypoint = (disc + repmat([-bw,0,0],sr,1))*10;
                
                obj.simState.display3d.carGexists=1;
                
            end
        end
    end
    
end

