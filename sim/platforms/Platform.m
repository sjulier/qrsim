classdef Platform<Entity
    % Class that implements a platform. A platform extends an entity in
    % some way! It's probably that it's instrumented in some manner.
    %
    
    properties (Access = protected)
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
            
        end
    end             
end