classdef detect_marker < matlab.System
    % Kalman filter to estimate the position of an aruco marker using the
    % UAV position estimate and the camera measurements.
    %
    % detect_aruco = detect_marker(...);

    % Public, tunable properties
    properties
        sample_time = 0.01          % Hz
        
        static_markers = true       % flag, true if it is known that all the unknown markers are static (v = 0)
        
        camera_pos = [0 0 0]        % p_cb_b
        
        camera_orient = [0 0 0]     % camera to body euler angles [yaw pitch roll] [rad]
        
        
        % Process noises
        w_q = 1e-5                  % orientation
        w_p = 1e-5                  % position
        w_v = 1e-5                  % velocity
        
        
        % Measurement noises
        v_q = 1e-2                  % orientation constant
        v_q_lin = 1/20^2            % position linear with distance
        v_p = 1e-2                  % position constant
        v_p_lin = 1/20^2            % position linear with distance
    end

    properties(DiscreteState)
        rec_id  % recorded id
        
        state               % state
        
        state_covariance	% error state covariance matrix
        
        time                % time
    end
    
    % Pre-computed constants
    properties(Access = private)
        pos_bc_b
        quat_cb
    end
    
    methods
        % Constructor
        function obj = detect_marker(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
        
        function update_private_properties(obj)
            obj.quat_cb = conj(quaternion(eul2quat(obj.camera_orient)));
            
            obj.pos_bc_b = - obj.camera_pos.';
        end
    end

    methods(Access = protected)
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end

        function [q_corr, p_corr, v_corr] = stepImpl(obj, Tcm, id, p_bn_n, q_bn, fcnt)
            %% Geometry initialization
            
            % Geometry computations
            q_cb = obj.quat_cb;
            p_bc_b = obj.pos_bc_b;
            
            % Output initialization
            p_corr = zeros(length(id), 3);
            v_corr = zeros(length(id), 3);
            q_corr = quaternion([1 0 0 0]) * zeros(length(id), 1);
            
            % Repeat this for all the different unknown markers detected
            for ii = 1:length(id)
                % More geometry, id-dependent
                q_cm = quaternion(rotm2quat(Tcm(1:3, 1:3, ii)));
                p_mc_c = Tcm(1:3,4, ii);

                q_mc = conj(q_cm);

                q_mb = q_mc * q_cb;

                % Measurements
                q_mn_meas = q_mb * q_bn;

                q_nc =  conj(q_bn) * conj(q_cb);

                p_mn_n_meas = obj.quat_rot(p_mc_c, q_nc) + obj.quat_rot(-p_bc_b, conj(q_bn)) + p_bn_n.';


                %%
                
                % Chack whether the marker id belongs to an already seen
                % marker or not.
                if not(ismember(id(ii), obj.rec_id))
                    %% New marker not previously detected
                    
                    % If the id is new add a new entry for the discrete
                    % states equal to the actual measure.

                    obj.rec_id(end+1) = id(ii);

                    q_corr(ii) = q_mn_meas;
                    p_corr(ii,:) = p_mn_n_meas.';
                    
                    obj.state(:, end+1) = [ compact(q_corr).'
                                            p_corr(ii,:).'
                                            zeros(3,1) ];

                    if sum(size(obj.state_covariance)) == 0
                        obj.state_covariance(:,:,1) = 0.0001*eye(9);
                    else
                        obj.state_covariance(:,:,end+1) = 0.0001*eye(9);
                    end
                    
                    obj.time(end+1) = fcnt;
                else 
                    %% Marker previously detected
                    
                    % The id is not new ==> kalman filter
                    
                    % Find the index corresponding to the id
                    for jj = 1:length(obj.rec_id)
                        if id(ii) == obj.rec_id(jj)
                            index = jj;
                        end
                    end
                    
                    
                    %% Kalman filterp prediction
                    
                    x = obj.state(:, index);
                    P = obj.state_covariance(:, :, index);
                    
                    % Process noise
                    Q = diag([ obj.w_q*ones(3,1)
                               obj.w_p*ones(3,1)
                               obj.w_v*ones(3,1) ]);
                    
                           
                    % Time elapsed since last observation (can be different
                    % for each detected marker)
                    dt = (fcnt - obj.time(index)) * obj.sample_time;
                    obj.time(index) = fcnt;
                    
                    
                    q = quaternion( x(1:4).' );
                    p = x(5:7);
                    v = x(8:10);
                    
                    
                    % Set the velocity to zero if this flag is true
                    if obj.static_markers == true
                        v = [0;0;0];
                    end
                    
                    % Predict the position
                    p = p + v * dt;
                    
                    % Predict P
                    F = eye(9) + [ zeros(3), zeros(3),  zeros(3)
                                   zeros(3), zeros(3), dt*eye(3)
                                   zeros(3), zeros(3),  zeros(3) ];
                    
                    if obj.static_markers == false
                        P_pred = F * P * F.' + Q;
                    else
                        P_pred = P + Q;
                    end
                    
                    
                    %% Measurement model
                    
                    % Measurement = meas_value - predict_value
                    z_o = rotvec(q_mn_meas * conj(q)).';
                    
                    z_p = p_mn_n_meas - p;
                
                    z = [ z_o
                          z_p ];
                
                
                    %% Observation model
                    
                    % Observation matrix and covariance matrices
                    dist = norm(p_mc_c);
                    
                    H = [ - eye(3), zeros(3), zeros(3)
                          zeros(3), - eye(3), zeros(3) ];
                      
                    R = diag([ obj.v_q*ones(3,1)+obj.v_q_lin*ones(3,1)*dist^2
                               obj.v_p*ones(3,1)+obj.v_p_lin*ones(3,1)*dist^2 ]);
                
                           
                    %% Kalman equations
                    
                    if obj.static_markers == true
                        H = H(:,1:6);
                        
                        P_pred = P_pred(1:6,1:6);
                    end
                    
                    [x_err, P_cor] = obj.kalman_eq(P_pred, R, H, z);
                    
                    x_err = [x_err; zeros(3,1)];
                    P_cor = blkdiag(P_cor, zeros(3));
                
                    
                    %% Correction 
                    
                    % Update the state
                    q = q * quaternion(- x_err(1:3).', 'rotvec');
                    x(1:4) = compact(q).';
                    x(5:end) = x(5:end) - x_err(4:end);
                    obj.state(:, index) = x;
                    
                    % Update the state covariance
                    obj.state_covariance(:,:,index) = P_cor;
                
                
                    % Update the output
                    p_corr(ii,:) = x(5:7).';
                    v_corr(ii,:) = x(8:10).';
                
                    q_corr(ii) = q;
                end
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.rec_id = [];
            
            obj.state = [];

            obj.state_covariance = [];
            
            obj.time = [];
        end
    end
    
    methods(Static)
        % Kalman equations.
        function [x_err, P] = kalman_eq(P, R, H, z)
            % Innovation covariance
            S = R + H * P * H.';
            
            % Kalman gain
            K = P * H.' / S;
            
            % Error estimate covariance update
            P = P - K * H * P;
            
            x_err = K * z;
        end
        
        function v_out = quat_rot(v, q)
            quat_v = quaternion([0, v(1), v(2), v(3)]);
    
            quat_v_out = q * quat_v * conj(q);
    
            % Quaternion parts extraction.
            % By doing this, the output is in the same form of the input
            % (row or column vector)
            [~, v(1), v(2), v(3)] = parts(quat_v_out);
            
            v_out = v;
        end
    end
end
