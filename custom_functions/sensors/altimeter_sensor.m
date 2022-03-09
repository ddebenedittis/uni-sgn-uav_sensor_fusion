classdef altimeter_sensor < matlab.System
    % Altimeter sensor simulator system block.
    %
    % alt = altimeter_sensor(...);
    %
    % h_meas = alt(drone_pos, drone_quat_orient);
    %
    % The error is modeled as White Noise + Bias Instability (First-Order
    % Markov process). The output is quantized.
    % If the altitude is greater than a predefined value, the block outputs
    % NaN.
    % If the measure is less than the full scale, the output is equal to
    % the full scale.

    % Public, tunable properties
    properties
        % White Noise error variance [m^2]
        wn_variance = 0.05^2
        
        % Second White Noise error variance, quadratic with the distance [m^-2]
        qwn_variance = 0.3^2 / 5^4
        
        % Bias instability variance [m^2]
        % BI modeled as a first-orded Markov process
        bi_variance = 0.01^2
        
        % Bias instability decay factor (in [0, 1])
        % 0 = White Noise   1 = Random Walk
        decay_factor = 0.1
        
        % Minimum and maximum measurable altitude
        min_alt = 0.1
        max_alt = 5
        
        % Quantization
        quantization = 0.01
        
        % Position wrt drone COM (or its point of interest) in a local NED
        % frame
        p_0 = [0 0 0.1] 
        
        % Random number generator seed
        seed = 3
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
        function obj = altimeter_sensor(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end

    methods(Access = protected)
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end

        function h_meas = stepImpl(obj, pos, q, ground_height)
            % The effect of the drone rotation is considering by dividing
            % the perpendicular distance (the sensor altitude) by the
            % cosine of the angle the formed by the altimeter direction 
            % with the vertical direction
            
            % Cosinne
            R = quat2rotm(q);
            cosine = R(3,3);
            
            % Rotated altimeter displacement (q_0)
            rot_h_0 = obj.h_rot(obj.p_0, (q));
            
            % Measured height considering the rotation of the drone
            h_meas = (- pos(3) - rot_h_0 - ground_height) / cosine;
            
            
            % Limit the maximum and minimum measurable altitude
            if h_meas > obj.max_alt || h_meas <= 0
                h_meas = NaN;
            elseif h_meas < obj.min_alt
                h_meas = obj.min_alt;
            else
                % The measure is accepted
                % Add noise
                
                % Gaussian white noise
                noise = randn * obj.wn_variance^0.5;
                
                % Gaussian white noise multiplied by the square of the
                % ground distance
                q_noise = randn * obj.qwn_variance^0.5 * h_meas^2;
            
                % Bias instability
                bi = obj.bias_inst;
                bi = bi * obj.decay_factor + randn * obj.bi_variance;
            
                % Measurement affected by errors
                h_meas = h_meas + noise + q_noise + bi;
                
                
                % Quantization
                quant = obj.quantization;
                h_meas = fix(h_meas / quant) * quant;
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.rng_state = rng(obj.seed);
            
            obj.bias_inst = 0;
        end
    end
    
    methods(Static)
        % Rotation with quaternions function
        function h = h_rot(v, q)
            quat_v = quaternion([0, v(1), v(2), v(3)]);
    
            quat_v = q * quat_v * conj(q);
    
            % Quaternion parts extraction
            [~, ~, ~, part3] = parts(quat_v);
    
            h = part3;
        end
    end
end
