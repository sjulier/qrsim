classdef QuadrotorGraphics<PlatformGraphics
    % Abstract class for the 3D visualization of a quadrotor helicopter
    % This implementation is very basic but has the advantage of not
    % depending on any additional toolbox

    methods (Sealed)
        function obj=QuadrotorGraphics(objparams)
            obj=obj@PlatformGraphics(objparams);
        end
    end
end