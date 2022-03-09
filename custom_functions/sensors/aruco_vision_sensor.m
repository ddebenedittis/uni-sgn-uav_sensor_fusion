classdef aruco_vision_sensor < matlab.System
    % ArUco vision simulator system block.
    %
    % aruco = aruco_vision_sensor()
    %
    % [id, Tcm, visualized_markers_index] = aruco(p_bg_g, drone_quat_orient)
    %
    % The error is modeled as two white noises components, one constant and
    % one linear with the distance of the target.
    % The noise variance is different depending on the axis.
    % TODO: add another noise component dependent on the relative angle of
    % the marker (to the camera).

    % Public, tunable properties
    properties
        % Camera detection cone semi-angle [radians]
        camera_cone_semi_angle = 50 * pi / 180
        
        % Maximum distance that the vision sensor is able to measure
        camera_max_distance = 20
        
        % Minimum distance that the vision sensor is able to measure
        camera_min_distance = 0.05
        
        % The noise on both position and orientation is modeled as the sum
        % of a constant noise and a noise linear with the distance of the
        % target from the camera.
        % In addition, the noise, for both orientation and position, is
        % different if the component is perpendicular or parallel to the
        % line of sight.
        % For position, the error is bigger on the parallel component.
        noise_perpendicular_position_constant = (0.1)^2
        noise_parallel_position_constant      = (0.2)^2
        noise_perpendicular_position_linear   = (0.2 / 20)^2
        noise_parallel_position_linear        = (0.4 / 20)^2
        
        % For orientation, the error is bigger on the perpendicular
        % component.
        noise_perpendicular_orientation_constant = (2*pi/180)^2
        noise_parallel_orientation_constant      = (1*pi/180)^2
        noise_perpendicular_orientation_linear   = (4*pi/180 / 20)^2
        noise_parallel_orientation_linear        = (2*pi/180 / 20)^2
        
        % Camera frame orientation with respect to the body frame,
        % specified as euler angles [yaw, pitch, roll].
        camera_orientation = [0, 0, 0]
        
        % Optical center position with respect to the body center of mass 
        % in the body frame.
        % p_cb_b
        camera_position = [0.1, 0, 0.1]
        
        % Marker id, orientations and positions
        marker_id
        marker_orientation      % euler angles [yaw, pitch, roll] [rad]
        marker_position         % p_mb_b
        
        % Random number generator seed
        seed = 2
    end
    
    properties(DiscreteState)
        % Random number generator state (to change its seed)
        rng_state
    end
    
    methods
        % Constructor
        function obj = aruco_vision_sensor(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end

    methods(Access = protected)
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end

        function [Tcm, id, visualized_markers_index] = stepImpl(obj, p_bg_g, q)
            % Convention: roto-traslation matrix
            % Tab = [ Rab,  p_ba_a ]  =  [ Rab,  - p_ab_a ]
            %       [   0,       1 ]     [   0,         1 ]
            %
            % p^a = Rab * p^b
            % p_ba_a = p_b^a - p_a^a
            
            % Marker id
            m_id = obj.marker_id;
            
            
            % Body to camera reference frame (RF) transformation (roto-traslation form)
            Rcb = eul2rotm(obj.camera_orientation).';
            p_cb_b = obj.camera_position.';
            p_cb_c = Rcb * p_cb_b;                      % the position is transformed in local RF
            Tcb = [Rcb,        -p_cb_c;
                   zeros(1,3),  1];
            
            % Global to body RF transformation
            Rbg = quat2rotm(q).';
            p_bg_b = Rbg * p_bg_g.';                    % the position is transformed in local RF
            Tbg = [Rbg,         -p_bg_b;
                   zeros(1,3),  1];
            
            % Global to camera RF transformation 
            Tcg = Tcb * Tbg;
            Rcg = Tcg(1:3, 1:3);
            
            
            % Total number of markers
            num_marker = size(obj.marker_position, 1);
            
            % Marker position in global frame wrt g frame
            p_mg_g = obj.marker_position.';
            hp_mg_g = [p_mg_g;
                       ones(1, num_marker)];            % homogeneous position vector, by column, [4 x number_markers]
            
            % Marker position in camera frame
            hp_mc_c = Tcg * hp_mg_g;
            p_mc_c = hp_mc_c(1:3, :);
            
            
            % Vector containing the ids of the marker which the camera is
            % able to see at the current instant.
            visualized_markers_index = [];
            
            % Cicle over all the markers to determine the visualized ones.
            for ii = 1:num_marker
                if ( p_mc_c(1, ii) > obj.camera_min_distance ) && ( p_mc_c(1, ii) < obj.camera_max_distance )
                    r = p_mc_c(1, ii) * tan(obj.camera_cone_semi_angle);
                    if ( p_mc_c(2, ii)^2 + p_mc_c(3, ii)^2 ) < r^2
                        % The marker position falls within the vision cone
                        % region of the camera.
                        
                        % The indexes (not the id) of the visualized
                        % markers: beacon_id(index) = visualized_m_id
                        visualized_markers_index(end+1) = ii;
                    end
                end
            end
            
            % Output initialization
            % Tcm is chosen now, can be changed to Tmc by uncommenting some
            % lines and commenting their corresponding acrive line.
            id = [];
            %Tmc = [];
            Tcm = [];
            
            % Cycle over al the visualized markers.
            for ii = visualized_markers_index
                id(end+1) = m_id(ii);
                
                % Exact roto-traslation matrix calculation
                Rmg = eul2rotm(obj.marker_orientation(ii, :)).';
                
                Rgc = Rcg.';
                Rmc = Rmg * Rgc;
                
                % pos_mg^m: position of m wrt g in m frame
                p_mg_m = Rmg * p_mg_g(:, ii);
                
                % pos_gc^c: position of g wrt c in c frame
                p_gc_c = Tcg(1:3, 4);
                
                % pos_mc^m: position of m wrt c in m frame
                p_mc_m = p_mg_m + Rmc * p_gc_c;
                
                %Tmc(:,:,end+1) = [Rmc,        -p_mc_m;
                %                  zeros(1,3),  1];
                
                Tcm(:,:,end+1) = [Rmc.',        Rmc.'*p_mc_m;
                                  zeros(1,3),   1];
                
                % Add measurement noise
                %Tmc(:,:,end) = add_noise(obj, Tmc(:,:,end));
                Tcm(:,:,end) = add_noise(obj, Tcm(:,:,end));
            end
            
            %Tmc = Tmc(:,:,2:end);       % This eliminates an initial zero matrix, which I don't understand why is here in the first place :|
            Tcm = Tcm(:,:,2:end);       % This eliminates an initial zero matrix, which I don't understand why is here in the first place :|
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.rng_state = rng(obj.seed);
        end
        
        % Add noise to the measurement: a GWN with constand standard
        % deviation and a GWN with standard deviation proportional with the
        % marker distance
        % The noise is a gaussian white noise on the position x y and z
        % components.
        % For the orientation, the real orientation ismultiplied by a small
        % rotation matrix corresponding to small euler angles with gaussian
        % white noise distribution.
        function out = add_noise(obj, Tmc)
            n_perp_p_c = obj.noise_perpendicular_position_constant;
            n_para_p_c = obj.noise_parallel_position_constant;
            n_perp_p_l = obj.noise_perpendicular_position_linear;
            n_para_p_l = obj.noise_parallel_position_linear;
        
            n_perp_o_c = obj.noise_perpendicular_orientation_constant;
            n_para_o_c = obj.noise_parallel_orientation_constant;
            n_perp_o_l = obj.noise_perpendicular_orientation_linear;
            n_para_o_l = obj.noise_parallel_orientation_linear;
            
            % Marker - camera distance
            dist = norm(Tmc(1:3, 4));
        
            % Position error
            p_err = [randn * n_para_p_c^0.5  +  randn * n_para_p_l^0.5 * dist;
                     randn * n_perp_p_c^0.5  +  randn * n_perp_p_l^0.5 * dist;
                     randn * n_perp_p_c^0.5  +  randn * n_perp_p_l^0.5 * dist];
            
            % Orientation error
            theta_err = [randn * n_perp_o_c^0.5  +  randn * n_perp_o_l^0.5 * dist;      % yaw (z)
                         randn * n_perp_o_c^0.5  +  randn * n_perp_o_l^0.5 * dist;      % pitch (y)
                         randn * n_para_o_c^0.5  +  randn * n_para_o_l^0.5 * dist];     % roll (x)
            
            % Matrix multiplicative error
            Rmc = Tmc(1:3, 1:3);
            Rmc_error = eul2rotm(theta_err.').' * Rmc;       % perturber orientation matrix
            
            % Additive error
            p_m = Tmc(1:3, 4);
            p_m_error = p_m + p_err;
            
            % Output
            out = [Rmc_error,   p_m_error;
                   zeros(1,3),  1];
        end
    end
end
