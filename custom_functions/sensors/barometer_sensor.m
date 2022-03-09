classdef barometer_sensor < matlab.System
    % Barometer simulator system block.
    %
    % bar = barometer_sensor(...);
    %
    % h = bar(drone_pos)
    %
    % The error is modeled as White Noise + Bias Instability (First-Order
    % Markov process). The output is quantized.
    % It is supposed that the sensor does internally the conversion between
    % pressure and altitude. The zero-altitude (and thus the zero pressure)
    % is re-initialized when the drone starts its flight.

    % Public, tunable properties
    properties
        % White Noise error variance [m^2]
        wn_variance = 0.5^2
        
        % Bias instability variance [m^2]
        bi_variance = 0.1^2
        
        % Bias instability decay factor (in [0, 1])
        % 0 = White Noise   1 = Random Walk
        decay_factor = 0.001
        
        % Quantization
        quantization = 0.1
        
        % Random number generator seed
        seed = 4
    end

    properties(DiscreteState)
        % Random number generator state (to change its seed)
        rng_state
        
        % Bias instability value
        bias_inst
    end

    % Pre-computed constants
    properties(Access = private)

    end
    
    methods
        % Constructor
        function obj = barometer_sensor(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end

    methods(Access = protected)
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end

        function h = stepImpl(obj,pos)
            % Real height
            h = - pos(3);
            
            
            % Add noise
            
            % White noise
            noise = randn * obj.wn_variance^0.5;
            
            % Bias instability
            bi = obj.bias_inst;
            bi = bi * obj.decay_factor + randn * obj.bi_variance;
            
            % Real measure affected by errors
            h = h + noise + bi;
                
                
            % Quantization
            quant = obj.quantization;
            h = fix(h / quant) * quant;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.rng_state = rng(obj.seed);
            
            obj.bias_inst = 0;
        end
    end
end
