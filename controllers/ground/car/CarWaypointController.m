classdef CarWaypointController<handle
    % A basic class which handles the waypoint controls for a vehicle
    %
    properties (Access=protected)
        wp;  % waypoints
        iwp; % index to current waypoint
        DT;  % control timestep
        G;   % current steer angle
        V;   % speed
    end
    
    properties (Constant)
        minD = 4;         % minimum distance from a waypoint before switching to the next one
        rateG = pi / 18;    % max steering rate (rad/s)
        maxG = pi / 6;      % max steering angle (rad)
        maxV = 5;           % max speed in m/s
    end
    
    methods (Access = public)
        
        function obj = CarWaypointController(wp, DT) 
            %  Creates a WaypointPID object:
            %      
            %  use:
            %    pid = WaypointPID(DT)
            %      DT - control timestep (i.e. inverse of the control rate)
            %
            obj.DT = DT;
            obj.wp = wp;
            obj.reset();
        end
        
        function U = computeU(obj,X)
            %  Computes the car control signals given the current state and a desired waypoint
 
            % Find the current waypoint and check if it's still valid. If
            % not, identify the next waypoint
            cwp= obj.wp(:,obj.iwp);
            d2= (cwp(1)-X(1))^2 + (cwp(2)-X(2))^2
            if d2 < obj.minD^2
                obj.iwp= obj.iwp+1; % switch to next
                if obj.iwp > size(obj.wp,2) % reached final waypoint, flag and return
                    obj.iwp=1;
                end    
                cwp= obj.wp(:,obj.iwp); % next waypoint
            end

            % compute change in G to point towards current waypoint
            deltaG= atan2(cwp(2)-X(2), cwp(1)-X(1)) - X(6) - obj.G;
            if (deltaG < -pi)
                deltaG = deltaG + 2 * pi;
            elseif (deltaG > pi)
                deltaG = deltaG - 2 * pi;
            end
                
            % limit rate
            maxDelta= obj.rateG * obj.DT;
            if abs(deltaG) > maxDelta
                deltaG= sign(deltaG)*maxDelta;
            end

            % limit angle
            obj.G= obj.G+deltaG;
            if abs(obj.G) > obj.maxG
                obj.G= sign(obj.G)*obj.maxG;
            end
            
            U=[obj.V;obj.G];
        end
        
        function obj = reset(obj)
            % reset controller
            %
            % use:
            %  pid.reset();
            %
            obj.iwp = 1;
            obj.G = 0;
            obj.V = obj.maxV;
        end    
    end    
end
