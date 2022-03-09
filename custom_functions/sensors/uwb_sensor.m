classdef uwb_sensor < matlab.System
    % Ultra-Wide-Band simulator system block.
    %
    % uwb = uwb_sensor('beacon_position', b_pos (matrix [number_beacons × 3]) [m], ...
    %                     'error_variance', var (scalar) [m^2], ...
    %                     'range', range (scalar) [m]);
    %
    % [dist, id, index] = uwb(target_position, unobstructed_id)
    %
    % The error is modeled as a simple white noise. NLOS effect are not
    % considered. The output is directly a distance (not a time).
    % To be quantized.

    % Public, tunable properties
    properties
        % Array of beacon positions (matrix [number_beacons × 3]) [m]
        beacon_position
        
        % Beacon id
        beacon_id
        
        % Error variance of the uwb [m^2]
        error_variance = 0.01
        
        % Maximum range of each beacon [m]
        range = 40
        
        % Minimum measurable range [m]
        min_dist = 0.05;
        
        % Random number generator seed
        seed = 1
    end
    
    properties(DiscreteState)
        % Random number generator state (to change its seed)
        rng_state
    end
    
    methods
        % Constructor
        function obj = uwb_sensor(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end

    methods(Access = protected)
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end

        function [dist, id, index2] = stepImpl(obj, target_position, input_id)
            % Total number of beacons
            num_beacons = size(obj.beacon_position, 1);
            
            % Number of accessible beacons (they are un-accessible if walls
            % limitate the signal)
            num_input = length(input_id);
            
            % Beacon id
            b_id = obj.beacon_id;
            
            % Index of the beacons initialization
            index = zeros(1, num_input);
            
            % Output initialization
            dist = [];
            id = [];
            index2 = [];
            
            % Find the indexes (the position of the id in the array of ids)
            % of the beacons
            for jj = 1:num_beacons
                for kk = 1:num_input
                    if input_id(kk) == b_id(jj)
                        index(kk) = jj;
                    end
                end
            end
            
            
            % Cycled over the indexes
            for ii = index
                [measurement, meas_ideal] = beacon_sim(obj, ii, target_position);
                
                % If the measurement exceeds the maximum range, the beacon
                % is not able to sense the target (no measure).
                if meas_ideal < obj.range
                    dist(end+1, :) = measurement;
                    
                    id(end+1) = b_id(ii);
                    
                    index2(end+1) = ii;
                end
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.rng_state = rng(obj.seed);
        end
        
        % Simulate the distance measured by the UWB system.
        function [dist, dist_ideal] = beacon_sim(obj, index, target_position)
            beacon_pos = obj.beacon_position(index, :);
            
            % Measurement error modeled as a white noise with zero mean
            % (using randn).
            error = randn * obj.error_variance^0.5;
            
            % Distance output. If with the error the distance is negative,
            % it is corrected to zero.
            dist_ideal = norm(beacon_pos - target_position);
            dist = dist_ideal + error;
            if dist < obj.min_dist
                dist = obj.min_dist;
            end
        end
    end
end
