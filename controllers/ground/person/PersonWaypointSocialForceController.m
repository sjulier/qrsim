classdef PersonWaypointSocialForceController<Controller
    % A basic class which handles the waypoint controls for a vehicle
    %
    properties (Access=protected)
        wp;  % waypoints
        iwp; % index to current waypoint
        simstate; % the simstate
        valid_simstate; % flag shows if the simstate is good
    end
    
    properties (Constant)
        minD = 4;         % minimum distance from a waypoint before switching to the next one
        maxS = 2;           % max speed in m/s
    end
    
    methods (Access = public)
        
        function obj = PersonWaypointSocialForceController(controllerparams) 
            %  Creates a controller object:
            %      
            %  use:
            %    pid = CarWaypointController(controllerparams)
            %      DT - control timestep (i.e. inverse of the control rate)
            %
            
            obj = obj@Controller(controllerparams);
            obj.wp = controllerparams.wp;
            obj.reset();
        end
        
        function setSimState(obj, simstate)
            obj.simstate = simstate;
            obj.valid_simstate = 1;
        end
        
        function U = computeU(obj,X)
            %  Computes the person control signals given the current state
            %  and a desired waypoint; this consists of the acceleration

            %assert(obj.valid_simstate,...
            %    'personwaypointsocialforcecontroller:invalidsimstate',['the controller must be provided with a simstate.']);
           
            % Find the current waypoint and check if it's still valid. If
            % not, identify the next waypoint
            cwp= obj.wp(:,obj.iwp);
            d2= (cwp(1)-X(1))^2 + (cwp(2)-X(2))^2;
            if d2 < obj.minD^2
                obj.iwp= obj.iwp+1; % switch to next
                if obj.iwp > size(obj.wp,2) % reached final waypoint, flag and return
                    obj.iwp=1;
                end    
                cwp= obj.wp(:,obj.iwp); % next waypoint
            end

            % compute the difference to the current waypoint and the
            % location of the pedestrian
            deltaX = [cwp(1:2)-X(1:2)];
            
            U = 0.02 * deltaX;
            
            % smoothly scale back the acceleration as we get close to the
            % maximum speed
            currentSpeed = norm(X(7:9));
           
            U = U * max(1 - currentSpeed / obj.maxS, 0);            
        end
        
        function obj = reset(obj)
            % reset controller
            %
            % use:
            %  pid.reset();
            %
            obj.iwp = 1;
            obj.simstate = [];
            obj.valid_simstate = 0;
        end    
    end    
end
