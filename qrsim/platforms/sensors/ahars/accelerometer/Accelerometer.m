classdef Accelerometer<Sensor
    % Abstract class for a generic Accelerometer sensor.
    % This is a simple wrapper, it does not include any code, its only purpouse is to
    % allow for runtime type checking.
    %
    % Accelerometer Methods:
    %    Accelerometer(objparams) - constructs the object, to be called only from derived subclasses.
    %
    methods (Sealed)
        function obj = Accelerometer(objparams)
            % constructs the object
            %
            % Example:
            %
            %   obj=Accelerometer(objparams)
            %                objparams.dt - timestep of this object
            %                objparams.DT - global simulation timestep
            %                objparams.on - 1 if the object is active
            %                objparams.seed - prng seed, random if 0
            %
            % Note:
            % this is an abstract class so this contructor is meant to be called by any
            % subclass.
            %
            obj = obj@Sensor(objparams);
        end
    end
end
