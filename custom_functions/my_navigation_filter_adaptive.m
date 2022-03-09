classdef my_navigation_filter_adaptive < matlab.System
    % ( ͡° ͜ʖ ͡°)
    %
    % Version 1.0
    %
    % Fusion of accelerometer, gyroscope, magnetometer, GPS (position and
    % velocity), UWB (with known and unknown beacons), computer vision
    % (ArUco markers with known and unknown positions), barometer and
    % altimeter.
    %
    % Indirect Quaternion-Based Sequential Extended Kalman Filter
    %
    % Adaptively-robust filter with Mahalanobis based statistical test 

    % Public, tunable properties
    properties
        %% Sampling properties
        
    	sample_rate = 100                       % [Hz]           
        decimation_factor = 1                   % Number of IMU samples joined together to reduce the computational cost
        
        
        %% Sensor properties
        
        % Gyroscope
        gyro_noise = 1e-0                       % [rad^2/s^2]
        gyro_drift_noise = 1e-0                 % [(rad/s)^2/s]
        q_sens_gyro = quaternion([1 0 0 0])     % orientation of the gyroscope sensor frame wrt body frame
        
        % Accelerometer
        acc_noise = 1e+2;                       % [(m/s^2)^2]
        acc_drift_noise = 1e-0                  % [(m/s^2)^2/s]
        lin_acc_noise = 1e-0                    % [(m/s^2)^2/s]
        lin_acc_decay_factor = 0.1/100          % [1/s] if the linear acceleration is quickly changing set to a high value; ∈ [0, 1/Ts]
        q_sens_acc = quaternion([1 0 0 0])      % orientation of the acceleration sensor frame wrt body frame
        
        % Magnetometer
        magn_noise = 1e+2                       % [μTesla^2]
        magn_disturbance_noise = 0.5            % [μTesla^2/s]
        magn_disturbance_decay_factor = 0.1/100 % [1/s] if the magnetic disturbance is quickly changing set to a high value; ∈ [0, 1]
        q_sens_magn = quaternion([1 0 0 0])     % orientation of the magnetometer sensor frame wrt body frame
        
        % GPS
        GPS_pos_noise = 1e+0                    % [m^2]
        GPS_vel_noise = 1e-1                    % [(m/s)^2]
        
        % UWB
        uwb_noise = 1e+2                        % [m^2]
        uwb_beacon_position                     % [m]
        uwb_beacon_id                           % 
        
        % UWB unknown
        uwb_unkn_meas_noise = 1e-2              % [m^2]
        uwb_unkn_proc_noise = 0e-3              %
        uwb_unkn_init_type = 't'                % 't' (time) if the initialization stops after x seconds, 'p' if the initializations stops after it has traveled at least x [m] in all three directions 
        uwb_unkn_init_threshold = 50            % [s] or [m] depending on uwb_unkn_init_type
        
        % Vision
        vision_pos_noise = 1e+0;                % 
        vision_orient_noise = 1e+0;             %
        camera_orient                           % euler angles [yaw, pitch, roll] [rad]
        camera_pos                              % camera position p_cb_b [m]
        marker_id                               % 
        marker_orient                           % euler angles [yaw, pitch, roll] [rad]
        marker_pos                              % p_mb_b
        
        % Barometer
        bar_noise = 1e+0                        % [m^2]
        
        % Altimeter
        alt_pos = [0, 0, 0.1]                   % p_ab_b [m]
        alt_noise = 1e-3                        % [m^2]
        alt_threshold = 0.3                     % [m] altimeter threshold: if the measure of the altitude by the altimeter and and its estimate
                                                % differs by more than this value, it is assumed that there was a sudden ground height change.
        
        
        %% Parameters used in the Kalman covariance matrices
        
        % Position and velocity process noises
        pos_noise = 1e-0;                       % [m^2/s^2]
        vel_noise = 1e-0;                       % [(m/s)^2/s]
        
        % Ground height
        ground_height_noise = 1e-1              % [m^2/s] process noise of the ground altitude process
        ground_height_decay_factor = 0.1/100    % [1/s] if the ground height is quickly changing set to a high value
        
        % P0
        init_process_noise = diag([1e-0*ones(3,1)
                                   1e-1*ones(3,1)
                                   1e-1*ones(3,1)
                                   1e-2*ones(3,1)
                                   1e-0*ones(3,1)
                                   1e-0*ones(3,1)
                                   1e-0*ones(3,1)
                                   1e-0*ones(1,1)])
        
        
        %% Earth parameters
        
        magn_field_NED                          % [μTesla]
        gravity = 9.81                          % [m/s]
        reference_location                      % latitude, longitude [°], altitude [m]
    end
    
    
    %% Discrete State
    
    properties(DiscreteState)
        % State:
        %         ⌈ quaternion orientation |   ⌈ q   ⌉
        %         | gyroscope bias         |   | ω_b |
        %         | accelerometer bias     |   | a_b |
        % state = | linear acceleration    | = | acc | ∈ R^23
        %         | magnetic disturbance   |   | m_d |
        %         | position               |   | p   |
        %         | velocity               |   | v   |
        %         ⌊ ground altitude        ⌋   ⌊ h_g ⌋
        state
        
        % Error state:
        %             ⌈ orientation error          ⌉   ⌈ θ_ε   ⌉
        %             | gyroscope bias error       |   | ω_b_ε |
        %             | accelerometer bias error   |   | a_b_ε |
        % state_err = | linear acceleration error  | = | acc_ε | ∈ R^22
        %             | magnetic disturbance error |   | m_d_ε |
        %             | position error             |   | p_ε   |
        %             | velocity error             |   | v_ε   |
        %             ⌊ ground altitude error      ⌋   ⌊ h_g_ε ⌋
        % state_err
        
        % Acceleration measurements store variable
        store_accel

        % Angular velocity at the previous time step
        ang_vel_old
        
        % State noise covariance matrix
        state_covariance
        
        % Steps passed since the last absolute position reference
        steps_last_pos
        
        % Set of currently detected unknown beacons ids
        uwb_unkn_rec_id

        % 
        uwb_unkn_position

        % 
        uwb_unkn_state_covariance
        
        % Matrix H of Hx=z used in the initialization of the uwb unknown
        % position
        uwb_unkn_H
        
        % Vector z of Hx=z used in the initialization of the uwb unknown
        % position
        uwb_unkn_z
    end

    %% Private properties
    
    % Must be computed/updated with set_discrete_properties
    properties(Access = private)
        % Sensors sampling time
        sample_time     % Kalman filter sample time
        KF_Ts           % decimation_factor / sample_time
        
        % Covariance of the process model
        process_covariance
        
        % Covariance of the measurement model
        measurement_covariance
        
        % Vision
        quat_gm         % quaternion orientation marker to global
        quat_cb      	% quaternion orientation body to camera
        
        pos_bc_b        % position in frame b of body wrt camera
        pos_mg_g
    end

    
    %% Public Methods
    
    methods
        %% Constructor
        
        function my_navigation_filter_v2(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
            
            % private_properties_update(obj)    % Used in version 1, in version 2 produces errors because the parameters are specified after the creation fo the object
        end
        
        %% Set initial state
        
        % Set the initial DiscreteState properties.
        % To be used if additional informations are available, for example
        % from an initial calibration.
        % Most of the argumens can be both a scalar or a vector with three
        % components (row or column vector is the same)
        function set_initial_state(obj, varargin)
            for ii = 1:2:nargin-1
                switch varargin{ii}
                    case 'state'
                        obj.state = varargin(ii+1);
                    case 'q'
                        obj.state( 1: 4) = varargin{ii+1};
                    case 'gyro_offset'    
                        obj.state( 5: 7) = varargin{ii+1};
                    case 'acc_bias'
                        obj.state( 8:10) = varargin{ii+1};
                    case 'lin_accel'
                        obj.state(11:13) = varargin{ii+1};
                    case 'magn_dist'
                        obj.state(14:16) = varargin{ii+1};
                    case 'pos'
                        obj.state(17:19) = varargin{ii+1};
                    case 'vel'
                        obj.state(20:22) = varargin{ii+1};
                    case 'h_g'
                        obj.state(23) = varargin{ii+1};
                    case 'P'
                        obj.state_covariance = varargin{ii+1};
                end
            end
        end
        
        %% Get Pose
        
        % Returns the estimated orientation and position of the UAV
        function [q, p] = get_pose(obj)
            q = quaternion(obj.state(1:4).');
            
            p = obj.state(17:19).';
        end
        
        %% Predict
        
        % Predict the orientation, the position and the velocity using the
        % accelerometer and the gyroscope measurements.
        % This is to be performed every time the gyroscope and the
        % accelerometer signal arrive (they are assumed to arrive together)
        % independently of the decimation factor
        function predict(obj, gyro_meas, acc_meas)
            % Rotate the measure from its sensor frame to the body frame
            acc_meas = obj.quat_rot(acc_meas, obj.q_sens_acc);
            gyro_meas = obj.quat_rot(gyro_meas, obj.q_sens_gyro);

            % Transform into a column vector
            acc_meas = acc_meas(:);
            gyro_meas = gyro_meas(:);

            % The measured acceleration is the opposite of the acceleration
            % acting on the body
            acc = - acc_meas;
            
            obj.state_predict(gyro_meas, acc);
            
            obj.state_cov_predict(gyro_meas, acc);
            
            
            % The correction using the accelerometer sensor is performed
            % every decimation_factor steps. To consider all the
            % accelerations in this interval, store_accel computes their
            % average over the interval of interest. It is also taken into
            % account how much the sensor has rotated (using the gyroscope
            % measurement and the estimated bias).
            gyro_b = obj.state(5:7);
            Ts = obj.sample_time;
            rot = quat2rotm( exp(1/2*Ts * quaternion([0; gyro_meas - gyro_b].')) );
            
            obj.store_accel = rot.' * obj.store_accel + (acc_meas / obj.decimation_factor);
        end
        
        
        %% Reset store_accel
        
        % Used in main_v1 when the accelerometer correction is deactivated
        function reset_store_accel(obj)
            obj.store_accel = zeros(3,1);
        end
        
        
        %% Fuse Accelerometer
        
        % Correct the estimate using the accelerometer.
        function z_g = fuse_accel(obj)
            if obj.steps_last_pos / obj.sample_rate < 5
                z_g = imu_kalman(obj);
            else
                z_g = imu_kalman_reduced(obj);
            end
            
        end
        
        %% Fuse Magnetometer
        
        % Correct the estimate using the magnetometer.
        function z_m = fuse_magn(obj, magn_meas)
            % Rotate the measure from its sensor frame to the body frame
            magn_meas = obj.quat_rot(magn_meas, obj.q_sens_magn);
            
            if obj.steps_last_pos / obj.sample_rate < 5
                z_m = magn_kalman(obj, magn_meas.');
            else
                z_m = magn_kalman_reduced(obj, magn_meas.');
            end
            
        end
        
        %% Fuse GPS
        
        % Correct the estimate of the position and velocity using the GPS
        % measurements (of position and velocity).
        function z_GPS = fuse_GPS(obj, GPS_pos_meas, GPS_vel_meas)
            z_GPS = GPS_kalman(obj, GPS_pos_meas.', GPS_vel_meas.');
        end
        
        %% Fuse UWB
        
        % Correct using the UWB measurements.
        function z_uwb = fuse_uwb(obj, uwb_meas, id)
            z_uwb = uwb_kalman(obj, uwb_meas, id);
        end
        
        % Correct using the "unknown" UWB measurements.
        function [beacon_pos, z_uwb] = fuse_unknown_uwb(obj, uwb_meas, id)
            rec_id = obj.uwb_unkn_rec_id;
            
            n = length(id);
            
            % Initialize the output variables
            beacon_pos = NaN * ones(n,3);
            
            z_uwb = NaN * ones(n,1);
            
            
            for ii = 1:length(id)
                if size(rec_id,1) == 0
                    % The id is a new one, create a new entry
                    
                    uwb_unkn_add_new(obj, uwb_meas(ii), id(ii));
                elseif not( ismember(id(ii), rec_id(1,:)) )
                    % The id is a new one, create a new entry
                    
                    uwb_unkn_add_new(obj, uwb_meas(ii), id(ii));
                else
                    % The id is not a new one
                    
                    % Find the index of the id in the rec_unkn_id variable
                    for jj = 1:size(rec_id,2)
                        if id(ii) == rec_id(1,jj)
                            index = jj;
                        end
                    end
                    
                    if rec_id(2,index) == 0
                        % Continue the initialization process
                        
                        [beacon_pos(ii,:)] = uwb_unkn_init(obj, uwb_meas(ii), index);
                    else
                        % Run Kalman filter
                        
                        [beacon_pos(ii,:), z_uwb(ii)] = uwb_unkn_kalman_nav(obj, uwb_meas(ii), index);
                    end
                end
            end
        end
        
        % Detect only the UWB beacon
        function [beacon_pos, z_uwb] = detect_unknown_uwb(obj, uwb_meas, id)
            rec_id = obj.uwb_unkn_rec_id;
            
            n = length(id);
            
            % Initialize the output variables
            beacon_pos = NaN * ones(n,3);
            
            z_uwb = NaN * ones(n,1);
            
            
            for ii = 1:length(id)
                if size(rec_id,1) == 0
                    % The id is a new one, create a new entry
                    
                    uwb_unkn_add_new(obj, uwb_meas(ii), id(ii));
                elseif not( ismember(id(ii), rec_id(1,:)) )
                    % The id is a new one, create a new entry
                    
                    uwb_unkn_add_new(obj, uwb_meas(ii), id(ii));
                else
                    % The id is not a new one
                    
                    % Find the index of the id in the rec_unkn_id variable
                    for jj = 1:size(rec_id,2)
                        if id(ii) == rec_id(1,jj)
                            index = jj;
                        end
                    end
                    
                    if rec_id(2,index) == 0
                        % Continue the initialization process
                        
                        [beacon_pos(ii,:)] = uwb_unkn_init(obj, uwb_meas(ii), index);
                    else
                        % Run Kalman filter
                        
                        [beacon_pos(ii,:), z_uwb(ii)] = uwb_unkn_kalman_detect(obj, uwb_meas(ii), index);
                    end
                end
            end
                    
        end
        
        %% Fuse Vision
        
        % Correct using the vision measurements.
        function [z_vis, n_meas] = fuse_vis(obj, Tmc, id)
            [z_vis, n_meas] = vis_kalman(obj, Tmc, id);
        end
        
        %% Fuse Barometer
        
        % Correct using the barometer measurements.
        function z_bar = fuse_bar(obj, bar_meas)
            z_bar = bar_kalman(obj, bar_meas);
        end
        
        %% Fuse Altimeter
        
        % Correct using the altimeter measurements.
        function z_alt = fuse_alt(obj, alt_meas)
            z_alt = alt_kalman(obj, alt_meas);
        end
        
        %% Private properties update
        
        % To be executed after changing one of the relevant properties of
        % the filter.
        function private_properties_update(obj)
            % Time constants definition.
            obj.sample_time = 1 / obj.sample_rate;
            obj.KF_Ts = obj.decimation_factor / obj.sample_rate;
            
            Ts = obj.sample_time;
            
            
            % Covariance of the process model
            obj.process_covariance = diag([obj.gyro_noise * Ts^2 * ones(3,1)
                                           obj.gyro_drift_noise * Ts * ones(3,1)
                                           obj.acc_drift_noise * Ts * ones(3,1)
                                           obj.lin_acc_noise * Ts * ones(3,1)
                                           obj.magn_disturbance_noise * Ts * ones(3,1)
                                           obj.pos_noise * Ts^2 * ones(3,1)
                                           obj.vel_noise * Ts * ones(3,1)
                                           obj.ground_height_noise * Ts]);
            
            % Covariance of the measurement model
            obj.measurement_covariance = diag([obj.acc_noise * ones(3,1)
                                               obj.magn_noise * ones(3,1)
                                               obj.GPS_pos_noise * ones(3,1)
                                               obj.GPS_vel_noise * ones(3,1)
                                               obj.uwb_noise
                                               obj.vision_pos_noise
                                               obj.vision_orient_noise
                                               obj.bar_noise
                                               obj.alt_noise]);
            
            
            % Vision useful quantities
            if  not(isempty(obj.marker_orient))
                obj.quat_cb = conj(quaternion(eul2quat(obj.camera_orient)));
                obj.quat_gm = quaternion(eul2quat(obj.marker_orient));
            
                obj.pos_bc_b = - obj.camera_pos.';
                obj.pos_mg_g = obj.marker_pos.';
            end
        end
    end

    
    %% Protected Methods
    
    methods(Access = protected)
        %% Basic Functions
        
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end

        function y = stepImpl(~, u)
            % Implement algorithm. Calculate y as a function of input u and discrete states.
            y = u;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.state = zeros(23,1);
            obj.state(1) = 1;
            
            obj.store_accel = zeros(3,1);

            obj.ang_vel_old = zeros(3,1);
            
            obj.state_covariance = obj.init_process_noise;
            
            obj.steps_last_pos = 0;
            
            obj.uwb_unkn_rec_id = [];
            
            obj.uwb_unkn_position = [];

            obj.uwb_unkn_state_covariance = [];
            
            obj.uwb_unkn_H = {};
            
            obj.uwb_unkn_z = {};
        end
        
        
        %% Backup/Restore Functions
        
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            % obj.myproperty = s.myproperty; 

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        

        %% Advanced Functions
        
        % Not really needeed...
        
%         function validateInputsImpl(obj,u)
%             % Validate inputs to the step method at initialization
%         end
% 
%         function validatePropertiesImpl(obj)
%             % Validate related or interdependent property values
%         end

        % Use the getDiscreteState method to ... (suspance)
        % get the discrete state.
%         function ds = getDiscreteStateImpl(obj)
%             % Return structure of properties with DiscreteState attribute
%             ds = struct();
%         end

%         function processTunedPropertiesImpl(obj)
%             % Perform actions when tunable properties change
%             % between calls to the System object
%         end
% 
%         function flag = isInputSizeMutableImpl(obj,index)
%             % Return false if input size cannot change
%             % between calls to the System object
%             flag = false;
%         end
% 
%         function flag = isInactivePropertyImpl(obj,prop)
%             % Return false if property is visible based on object 
%             % configuration, for the command line and System block dialog
%             flag = false;
%         end
        
        
        %% Accelerometer Correction
        
        function z = imu_kalman(obj)
            %% Measurement model
            
            q = quaternion( obj.state(1:4).' );
            acc_bias = obj.state(8:10);
            lin_accel = obj.state(11:13);
            acc_meas = obj.store_accel;             % - lin_acc + a_b + g
            g_n = [0; 0; obj.gravity];              % gravity vector in n frame
            
            % Reset to zero for the next step
            obj.store_accel = zeros(3,1);
            
            % Gravity vector in body frame
            R_bn = quat2rotm(q).';                  % Reference frame rotation matrix from n to b
            g_b = R_bn * g_n;
                        
            % Error model
            z = acc_meas - ( - lin_accel + acc_bias + g_b);
            
            
            %% Observation model
            
            Xdx = obj.X_deltax(q, 15);
            
            [qw, qx, qy, qz] = parts(q);
            
            dRgdq = 9.81 * [ -2*qy,  2*qz, -2*qw, 2*qx
                              2*qx,  2*qw,  2*qz, 2*qy
                              2*qw, -2*qx, -2*qy, 2*qz ];

            % Measurementt matrix computation
            %x = [   θ_ε,    ω_b_ε, acc_b_ε, lin_acc_ε,      p_ε,      v_ε ].'
            Hx = [ dRgdq, zeros(3),  eye(3),  - eye(3), zeros(3), zeros(3) ];

            Hk = Hx * Xdx;
            
            % Measurement covariance
            R = obj.measurement_covariance(1:3,1:3);
            
            
            %% The rest (but i) is in common with all the other sensors
            
            % Interval containing the states that will be modified
            i = [1,12, 16,21];
            
            method = "standard";
            
            gen_kalman_correction(obj, i, R, Hk, z, method)
        end
        
        
        %% Accelerometer correction without position and velocity
        
        % This simplified version does not include the position and the
        % velocity in the measurement model. It provides better performance
        % when the position noise is high
        function z = imu_kalman_reduced(obj)
            %% Measurement model
            
            q = quaternion( obj.state(1:4).' );
            acc_bias = obj.state(8:10);
            lin_accel = obj.state(11:13);
            acc_meas = obj.store_accel;             % lin_acc + a_b - g
            g_n = [0; 0; obj.gravity];              % gravity vector in n frame
            
            % Reset to zero for the next step
            obj.store_accel = zeros(3,1);
            
            % Gravity vector in body frame
            R_bn = quat2rotm(q).';                  % Reference frame rotation matrix from n to b
            g_b = R_bn * g_n;
                        
            % Error model
            z = acc_meas - ( - lin_accel + acc_bias + g_b);
            
            
            %% Observation model
            
%             Xdx = obj.X_deltax(q, 15);
            Xdx = obj.X_deltax(q, 9);
            
            [qw, qx, qy, qz] = parts(q);
            
            dRgdq = 9.81 * [ -2*qy,  2*qz, -2*qw, 2*qx
                              2*qx,  2*qw,  2*qz, 2*qy
                              2*qw, -2*qx, -2*qy, 2*qz ];

            % Measurementt matrix computation
            %x = [   θ_ε,    ω_b_ε, acc_b_ε, lin_acc_ε ].'
            Hx = [ dRgdq, zeros(3),  eye(3),  - eye(3) ];

            Hk = Hx * Xdx;
            
            % Measurement covariance
            R = obj.measurement_covariance(1:3,1:3);
            
            
            %% The rest (but i) is in common with all the other sensors
            
            % Interval containing the states that will be modified
%             i = [1,12, 16,21];
            i = [1,6, 7,12];
            
            method = "standard";
            
            gen_kalman_correction(obj, i, R, Hk, z, method)
        end
        
        %% General Kalman equations and correction
        
        % Used for all the sensors, with an opportune interval i
        function gen_kalman_correction(obj, i, R, Hk, z, method)
            a = i(1); b = i(2); c = i(3); d = i(4);
            
            n = b-a+1 + d-c+1 - 3;
            
            % State covariance matrix
            P = obj.state_covariance;
            P_pre = [ P(a:b, a:b), P(a:b, c:d)
                      P(c:d, a:b), P(c:d, c:d) ];
            
            
            % Kalman equations
            
            if method == "standard" || obj.magn_disturbance_decay_factor ~= 1 * obj.sample_rate
                [x_err, P_cor] = obj.kalman_eq(P_pre, R, Hk, z);
            elseif method == "mahalanobis adaptive"
                [alpha, R_bar] = obj.mahalanobis_adaptively_robust(P_pre, R, Hk, z);
                
                if not( isnan(alpha) )
                    [x_err, P_cor] = obj.adaptive_factor_kalman_eq(P_pre, R, Hk, z, alpha);
                else
                    [x_err, P_cor] = obj.kalman_eq(P_pre, R_bar, Hk, z);
                end
            end
            

            Gk = blkdiag( eye(3)-skew(1/2*x_err(1:3)), eye(n) );

            P_cor = Gk * P_cor * Gk.';
            
            
            % Correction
            
            % Compute the state error
            state_err = substitute_state_err_and_state_cov(obj, x_err, P_cor, i);

            % Update the state using the state error
            obj.correct(state_err);
        end
        
        %% Correct
        
        % General correction step only
        function correct(obj, state_err)
            obj.state(1:4) = compact( quaternion(obj.state(1:4).') * quaternion(state_err(1:3).', 'rotvec') );
            %obj.state(1:4) = normalize(obj.state(1:4), 'norm');
            obj.state(5:end) = obj.state(5:end) + state_err(4:end);
        end
        
        
        %% Magnetometer Correction
        
        function z = magn_kalman(obj, magn_meas)
            %% Measurement model
            
            q = quaternion( obj.state(1:4).' );
            magn_dist = obj.state(14:16);
            m_n = obj.magn_field_NED;

            mx = m_n(1); my = m_n(2); mz = m_n(3);

            % Earth magnetic field in body frame
            R_bn = quat2rotm(q).';                  % Reference frame rotation matrix from n to b
            m_b = R_bn * m_n;
            
            % Error model (with normalized magnetic field)
            z = magn_meas - (m_b + magn_dist);


            %% Observation model
            
            Xdx = obj.X_deltax(q, 15);
%             Xdx = obj.X_deltax(q, 9);
            
            [qw, qx, qy, qz] = parts(q);
            
            dRmdq = [ 2*mx*qw + 2*my*qz - 2*mz*qy, 2*mx*qx + 2*my*qy + 2*mz*qz, 2*my*qx - 2*mx*qy - 2*mz*qw, 2*my*qw - 2*mx*qz + 2*mz*qx
                      2*my*qw - 2*mx*qz + 2*mz*qx, 2*mx*qy - 2*my*qx + 2*mz*qw, 2*mx*qx + 2*my*qy + 2*mz*qz, 2*mz*qy - 2*my*qz - 2*mx*qw
                      2*mx*qy - 2*my*qx + 2*mz*qw, 2*mx*qz - 2*my*qw - 2*mz*qx, 2*mx*qw + 2*my*qz - 2*mz*qy, 2*mx*qx + 2*my*qy + 2*mz*qz ];
            
            % Measurementt matrix computation
            %x =  [   θ_ε,   ω_b_ε,  acc_b_ε,    m_d,      p_ε,      v_ε ].'
            Hx = [ dRmdq, zeros(3), zeros(3), eye(3), zeros(3), zeros(3) ];
%             Hx = [ dRmdq, zeros(3), zeros(3), eye(3)];

            Hk = Hx * Xdx;
            
            % Measurement covariance
            R = obj.measurement_covariance(4:6,4:6);
            
            
            %% The rest (but i) is in common with all the other sensors
            
            % Interval containing the states that will be modified
            i = [1,9, 13,21];
%             i = [1,9, 13,15];
            
            method = "standard";
            
            gen_kalman_correction(obj, i, R, Hk, z, method)
        end
        
        
        %% Magnetometer correction without position and velocity
        
        % This simplified version does not include the position and the
        % velocity in the measurement model. It provides better performance
        % when the position noise is high
        function z = magn_kalman_reduced(obj, magn_meas)
            %% Measurement model
            
            q = quaternion( obj.state(1:4).' );
            magn_dist = obj.state(14:16);
            m_n = obj.magn_field_NED;

            mx = m_n(1); my = m_n(2); mz = m_n(3);

            % Earth magnetic field in body frame
            R_bn = quat2rotm(q).';                  % Reference frame rotation matrix from n to b
            m_b = R_bn * m_n;
            
            % Error model (with normalized magnetic field)
            z = magn_meas - (m_b + magn_dist);


            %% Observation model
            
%             Xdx = obj.X_deltax(q, 15);
            Xdx = obj.X_deltax(q, 9);
            
            [qw, qx, qy, qz] = parts(q);
            
            dRmdq = [ 2*mx*qw + 2*my*qz - 2*mz*qy, 2*mx*qx + 2*my*qy + 2*mz*qz, 2*my*qx - 2*mx*qy - 2*mz*qw, 2*my*qw - 2*mx*qz + 2*mz*qx
                      2*my*qw - 2*mx*qz + 2*mz*qx, 2*mx*qy - 2*my*qx + 2*mz*qw, 2*mx*qx + 2*my*qy + 2*mz*qz, 2*mz*qy - 2*my*qz - 2*mx*qw
                      2*mx*qy - 2*my*qx + 2*mz*qw, 2*mx*qz - 2*my*qw - 2*mz*qx, 2*mx*qw + 2*my*qz - 2*mz*qy, 2*mx*qx + 2*my*qy + 2*mz*qz ];
            
            % Measurementt matrix computation
            %x = [   θ_ε,    ω_b_ε,  acc_b_ε,    m_d ].'
            Hx = [ dRmdq, zeros(3), zeros(3), eye(3) ];

            Hk = Hx * Xdx;
            
            % Measurement covariance
            R = obj.measurement_covariance(4:6,4:6);
            
            
            %% The rest (but i) is in common with all the other sensors
            
            % Interval containing the states that will be modified
%             i = [1,9, 13,21];
            i = [1,9, 13,15];
            
            method = "standard";
            
            gen_kalman_correction(obj, i, R, Hk, z, method)
        end
        
        
        %% GPS Correction
        
        function z = GPS_kalman(obj, GPS_p_meas, GPS_v_meas)
            %% Measurement model
            
            % Geodetic to NED conversion
            refloc = obj.reference_location;
            lat0 = refloc(1); lon0 = refloc(2); h0 = refloc(3);
            
            % Compute the NED coordinates from the geodetic coordinates
            [xNorth, yEast, zDown] = geodetic2ned(GPS_p_meas(1), GPS_p_meas(2), GPS_p_meas(3), lat0, lon0, h0, wgs84Ellipsoid);
            
            % Predicted state
            pos = obj.state(17:19);
            vel = obj.state(20:22);
            
            % GPS measurement
            pos_GPS = [xNorth; yEast; zDown];
            vel_GPS = GPS_v_meas;
            
            % Error model
            z_pos = pos_GPS - pos;
            z_vel = vel_GPS - vel;
            
            z = [z_pos
                 z_vel];
            
            
            %% Observation model
            
            % Measurementt matrix computation
            %x = [      θ_ε,    ω_b_ε,  acc_b_ε,      p_ε,      v_ε ].'
            Hk = [ zeros(3), zeros(3), zeros(3),   eye(3), zeros(3)
                   zeros(3), zeros(3), zeros(3), zeros(3),   eye(3) ];
            
            % Measurement covariance
            R = obj.measurement_covariance(7:12,7:12);
            
            
            %% The rest (but i) is in common with all the other sensors
            
            % Interval containing the states that will be modified
            i = [1,9, 16,21];
            
            increase_P_last_pos(obj);
            
            method = "mahalanobis adaptive";
%             method = "standard";
            
            gen_kalman_correction(obj, i, R, Hk, z, method)
        end
        
        
        %% increase_P_last_pos
        
        % Increase some parts of P with a term quadratic with time. This is
        % useful if there's no absolute reference for a significant
        % interval, because increasing the part of P associated with the
        % position noise covariance will enable the filter to instantly
        % recover the correct position whenever a signal is available.
        function increase_P_last_pos(obj)
            P = obj.state_covariance;
            dt = obj.sample_time * obj.steps_last_pos;
            
            if dt>10
                P(:,16:21) = 0;
                P(16:21,:) = 0;
                P(16:18,16:18) = obj.state_covariance(16:18,16:18);
                
                % Increase the position covariance
                Q = zeros( size(P) );
                Q(16:18, 16:18) = diag( obj.pos_noise * ones(1,3) ) * dt^2;
            
                % Update P
                P = P + Q;
            
                obj.state_covariance = P;
            end
            
            
            % Reset steps_last_pos to zero
            obj.steps_last_pos = 0;
        end
        
        
        %% Ultra Wide Band Correction
        
        % Ultra Wide Band error model
        function z = uwb_kalman(obj, uwb_meas, uwb_id)
            %% Measurement model
            
            beacon_pos = obj.uwb_beacon_position;
            b_id = obj.uwb_beacon_id;
            
            pos = obj.state(17:19).';
            
            % Initialize index as an empty vector
            index = [];
            
            % Cycle to determine the indexes of the known beacons
            % id =/= index: the id is a real world value, the index is the
            % position in which the beacon is stored in this simulation.
            % Some of the beacon "viewed" do not belong to the known beacon
            % set.
            for kk = 1:length(uwb_id)
                for ii = 1:length(b_id)
                    if uwb_id(kk) == b_id(ii)
                        index(end+1) = ii;
                    end
                end
            end
            
            % Initialization
            dist_imu = zeros(length(index), 1);    % predicted estimate of the distance measured by the uwb
            diff_imu = zeros(length(index), 3);    % difference between the beacon position and the target predicted position
            
            % Cycle for all the beacons which the target is able to detect
            for ii = 1:length(index)
                diff_imu(ii, :) = pos - beacon_pos(index(ii), :);
                
                dist_imu(ii) = norm(beacon_pos(index(ii), :) - pos);
            end
            
            % Error
            z = uwb_meas - dist_imu;
            
            
            %% Observation model
            
            % A matrix of all zeros of opportune size
            zz = zeros(length(z), 3);
            
            % Measurementt matrix computation
            %x = [ θ_ε, ω_b_ε, acc_b_ε,                  p_ε, v_ε ].'
            Hk = [  zz,    zz,      zz, diff_imu ./ uwb_meas,  zz ];
            
            % Measurement covariance
            R = eye(length(z)) * obj.measurement_covariance(13,13);
            
            
            %% The rest (but i) is in common with all the other sensors
            
            % Interval containing the states that will be modified
            i = [1,9, 16,21];
            
            increase_P_last_pos(obj);
            
            method = "mahalanobis adaptive";
            
            gen_kalman_correction(obj, i, R, Hk, z, method)
        end
        
        
        %% Unknown Ultra Wide Band Correction
        
        % Used when a new unknown id is detected, initialize everything.
        function uwb_unkn_add_new(obj, dist_meas, id)
            % Add a new id to the recorded ids variable.
            % The 0 is used to know that the initialization phase of this
            % id is still not completed
            obj.uwb_unkn_rec_id(:, end+1) = [ id; 0 ];
            
            % Position as a row vector
            p = obj.state(17:19).';
            
            % These are cell arrays because they can have different
            % dimensions
            obj.uwb_unkn_H{end+1} = [1, -2*p];
            
            obj.uwb_unkn_z{end+1} = dist_meas^2 - norm(p)^2;
        end
        
        % Add new data for the initialization phase. After a number equal
        % to uwb_unkn_init_threshold data points have been acquired, the
        % initialization phase is concluded.
        function [beacon_pos] = uwb_unkn_init(obj, dist_meas, index)
            % Position as a row vector
            p = obj.state(17:19).';
            
            % Initialize the output
            beacon_pos = NaN * ones(1,3);
            
            
            % Get H and z corresponding to the index
            H = obj.uwb_unkn_H{index};
            
            z = obj.uwb_unkn_z{index};
            
            
            % Update H and z
            H(end+1, :) = [1, -2*p];
            obj.uwb_unkn_H{index} = H;
            
            % (end+1, :) to make it a column vector
            z(end+1, :) = dist_meas^2 - norm(p)^2;
            obj.uwb_unkn_z{index} = z;
            
            
            % If enough data points have been acquired, complete the
            % initialization phase
            if size(H, 1) == obj.uwb_unkn_init_threshold
                x = (H.' * H)^-1 * H.' * z;
                
                obj.uwb_unkn_position(:, index) = x(1:4);
                beacon_pos = x(2:4);
                
                obj.uwb_unkn_state_covariance(:,:,index) = x(1) * eye(4);
                
                obj.uwb_unkn_rec_id(2, index) = 1;
            end
        end
        
        % 
        function [beacon_pos, z] = uwb_unkn_kalman_detect(obj, dist_meas, index)
            %% Measurement model
            
            x = obj.uwb_unkn_position(:, index);
            
            % Position
            p = obj.state(17:19);
            
            % Error model
            z = dist_meas^2 - norm(p,2)^2;
            
            
            %% Observation model
            H = 2 * [ 1/2,  -p(1), -p(2), -p(3)];
            
            % Measurement covariance
            R = obj.uwb_unkn_meas_noise;
            
            
            %% Kalman equations
            P_pre = obj.uwb_unkn_state_covariance(:,:,index);
            
            z = z - H * x;
            
            [x_cor, P_cor] = obj.kalman_eq(P_pre, R, H, z);
            
            
            % Correction
            obj.uwb_unkn_position(:, index) = x + x_cor;
            beacon_pos = x(2:4);
            
            obj.uwb_unkn_state_covariance(:,:,index) = P_cor;
        end
        
        % Kalman filter to fuse the unknown UWB beacons measurement. This
        % step estimates the UWB beacon position and the AUV position at
        % the same time.
        function [beacon_pos, z] = uwb_unkn_kalman_nav(obj, dist_meas, index)
            %% Measurement model
            
            beacon_pos = obj.uwb_unkn_position(2:4, index);
            
            % Position
            p = obj.state(17:19);
            
            % Error model
            diff_imu = beacon_pos - p;
            dist_imu = norm(diff_imu);
            
            z = dist_meas - dist_imu;
            
            
            %% Observation model
            
            J = (diff_imu / dist_meas).';
            
            %x = [      θ_ε,    ω_b_ε,  acc_b_ε, p_ε,      v_ε, p_ε ]
            Hk = [ zeros(1,3), zeros(1,3), zeros(1,3),  -J, zeros(1,3),   J ];
            
            % Measurement covariance
            R = obj.uwb_unkn_meas_noise;
            
            
            %% Kalman equations
            
            P_b = obj.uwb_unkn_state_covariance(2:4,2:4,index);
            
            P = obj.state_covariance;
            
            P_pre = [P( 1: 9, 1: 9), P( 1: 9,16:21), zeros(9,3)
                     P(16:21, 1: 9), P(16:21,16:21), zeros(6,3)
                         zeros(3,9),     zeros(3,6),       P_b ];
            
            % Innovation covariance
            S = R + Hk * P_pre * Hk.';
            
            % Kalman gain
            K = P_pre * Hk.' / S;
            
            % Error estimate covariance update
            P_cor = (eye(18) - K * Hk) * P_pre;
                        
            x_err = K * z;
            
%             Gk = eye(3) - skew(1/2*x_err(1:3));
%             
%             P_cor(1:3,1:3) = Gk * P_cor(1:3,1:3) * Gk.';
            
            
            %% Correction step
            
            % Drone
            
            state_err = zeros(22,1);
            state_err(1:9) = x_err(1:9);
            state_err(16:21) = x_err(10:15);
            
            obj.state(1:4) = compact( quaternion(obj.state(1:4).') * quaternion(state_err(1:3).', 'rotvec') );
            obj.state(5:end) = obj.state(5:end) + state_err(4:end);
            
            obj.state_covariance(1:9,1:9) = P_cor(1:9,1:9);
            obj.state_covariance(16:21,1:9) = P_cor(10:15,1:9);
            obj.state_covariance(1:9,16:21) = P_cor(1:9,10:15);
            obj.state_covariance(16:21,16:21) = P_cor(10:15,10:15);
            
            % Beacon
            
            beacon_pos = beacon_pos + x_err(end-2:end);
            obj.uwb_unkn_position(2:4, index) = beacon_pos;
            obj.uwb_unkn_position(1, index) = norm(beacon_pos);
            
            obj.uwb_unkn_state_covariance(2:4,2:4,index) = P_cor(end-2:end,end-2:end);
        end
        
        
        %% ArUco Vision Sensor Correction
        
        % ArUco vision sensor error model
        function [z, n_meas] = vis_kalman(obj, Tcm, id)
            %% Measurement model
            
            p_bc_b = obj.pos_bc_b;
            p_mg_g = obj.pos_mg_g;
            
            q_cb = obj.quat_cb;
            q_gm = obj.quat_gm;
            
            m_id = obj.marker_id;
            
            index = [];
            
            for jj = 1:length(id)
                for ii = 1:length(m_id)
                    if m_id(ii) == id(jj)
                        index(end+1) = ii;
                    end
                end
            end
            
            % Number of sensed known ArUco markers
            n_meas = length(index);
            
            % Initialization
            q_gb_vis = quaternion([1 0 0 0]) * ones(n_meas, 1);
            p_bg_g_vis = zeros(3, n_meas);
            
            for ii = 1:n_meas
                q_mc = conj(quaternion(rotm2quat(Tcm(1:3, 1:3, ii))));
                
                q_gb_vis(ii) = q_gm(index(ii)) * q_mc * q_cb;
                
                p_bc_g_quat = q_gb_vis(ii) * quaternion([0, p_bc_b.']) * conj(q_gb_vis(ii));
                p_bc_g_quat_comp = compact(p_bc_g_quat);
                p_bc_g = p_bc_g_quat_comp(2:4);
                
                p_cm_m = - Tcm(1:3, 1:3, ii).' * Tcm(1:3, 4, ii);
                p_cm_g_quat = q_gm(index(ii)) * quaternion([0, p_cm_m.']) * conj(q_gm(index(ii)));
                p_cm_g_quat_comp = compact(p_cm_g_quat);
                p_cm_g = p_cm_g_quat_comp(2:4);
                
                p_bg_g_vis(:, ii) = p_bc_g.' + p_cm_g.' + p_mg_g(:, index(ii));
            end
            
            vis_pos_meas = p_bg_g_vis;
            vis_orient_meas = q_gb_vis;
            
            % Error
            q = quaternion( obj.state(1:4).' );
            pos = obj.state(17:19);
            
            z_pos_vis = vis_pos_meas - pos;
            z_pos_vis = reshape(z_pos_vis, [3*n_meas, 1]);                      % Reshape in the form of a column vector
            
            z_orient_vis = q * conj(vis_orient_meas);
            
            z_orient_vis = reshape(rotvec(z_orient_vis).', [3*n_meas, 1]);      % Reshape in the form of a column vector
            
            z = [z_pos_vis
                 z_orient_vis];
            
            
            %% Observation model
            
            % Measurement matrix
            I = zeros(3*n_meas, 3);
            
            for ii = 1:n_meas
                I(1+(ii-1)*3:(ii)*3, 1:3) = eye(3);
            end
            
            %x = [             θ_ε,      ω_b_ε,  acc_b_ε,        p_ε,        v_ε  ].'
            Hk = [ I * [zeros(3,3), zeros(3,3), zeros(3),     eye(3), zeros(3,3)]
                   I * [    eye(3), zeros(3,3), zeros(3), zeros(3,3), zeros(3,3)] ];
            
            % Measurement covariance
            R1 = obj.measurement_covariance(14,14);
            R2 = obj.measurement_covariance(15,15);
            
            R = [eye(3*n_meas) * R1,    zeros(3*n_meas)
                    zeros(3*n_meas), eye(3*n_meas) * R2];
            
            
            %% The rest (but i) is in common with all the other sensors
            
            % Interval containing the states that will be modified
            i = [1,9, 16,21];
            
            increase_P_last_pos(obj);
            
            method = "standard";
            
            gen_kalman_correction(obj, i, R, Hk, z, method)
        end
        
        
        %% Barometer Correction
        
        % Barometer error model
        function z = bar_kalman(obj, bar_meas)
            if isnan(bar_meas)
                z = NaN;
            else
                %% Measurement model
                
                z = - bar_meas - obj.state(19);
                
                
                %% Observatiton model
                
                % Measurement matrix
                Hk = [ zeros(1,3), zeros(1,3), zeros(1,3), 0, 0, 1, zeros(1,3) ];
                
                % Measurement covariance
                R = obj.measurement_covariance(16,16);
                
                
                %% The rest (but i) is in common with all the other sensors
            
                % Interval containing the states that will be modified
                i = [1,9, 16,21];
            
                method = "standard";
            
                gen_kalman_correction(obj, i, R, Hk, z, method)
            end
        end
        
        
        %% Altimeter Correction
        
        % Altimeter error model
        function z = alt_kalman(obj, alt_meas)
            if isnan(alt_meas)
                % The altimeter is unable to measure the altitude
                z = NaN;
            else
                %% Measurement model
                
                % Relevant quantities
                q = quaternion( obj.state(1:4).' );
                z_down = obj.state(19);                 % z coordinate
                h = - z_down;                           % height
                ground_height = obj.state(23);
                
                p0 = obj.alt_pos.';                     % position of the altimeter with respect to the COM in body frame
                p0x = p0(1); p0y = p0(2); p0z = p0(3);  % components of p0
                
                % Rotation matrix R_b^n
                Rnb = quat2rotm(q);
                
                % Cosine of the angle of the laser with the vertical
                % direction.
                cosine = Rnb(3,3);
                
                % Predicted altimeter measurement
                alt_imu = (- z_down - [0,0,1]*Rnb*p0 - ground_height) / cosine;
                
                % Error
                z = alt_meas - alt_imu;
                
                factor = 10^1;
                                
                % Change the ground height if the error is greater than a
                % certain threshold.
                if abs(z) > obj.alt_threshold
                    ground_height = - z_down - [0,0,1]*Rnb*p0 - alt_meas * cosine;
                    obj.state(23) = ground_height;
                    
                    obj.state_covariance(22,22) = obj.state_covariance(22,22) * factor;
                else
                    %% Observation model
                
                    Xdx = obj.X_deltax(q, 13);
                    
                    [qw, qx, qy, qz] = parts(q);
                    
                    dAltdq = [ - (2*p0z*qw - 2*p0y*qx + 2*p0x*qy)/(qw^2 - qx^2 - qy^2 + qz^2) - (2*qw*(h - p0z*(qw^2 - qx^2 - qy^2 + qz^2) - p0x*(2*qw*qy + 2*qx*qz) + p0y*(2*qw*qx - 2*qy*qz)))/(qw^2 - qx^2 - qy^2 + qz^2)^2, (2*p0y*qw + 2*p0z*qx - 2*p0x*qz)/(qw^2 - qx^2 - qy^2 + qz^2) + (2*qx*(h - p0z*(qw^2 - qx^2 - qy^2 + qz^2) - p0x*(2*qw*qy + 2*qx*qz) + p0y*(2*qw*qx - 2*qy*qz)))/(qw^2 - qx^2 - qy^2 + qz^2)^2, (2*qy*(h - p0z*(qw^2 - qx^2 - qy^2 + qz^2) - p0x*(2*qw*qy + 2*qx*qz) + p0y*(2*qw*qx - 2*qy*qz)))/(qw^2 - qx^2 - qy^2 + qz^2)^2 - (2*p0x*qw - 2*p0z*qy + 2*p0y*qz)/(qw^2 - qx^2 - qy^2 + qz^2), - (2*p0x*qx + 2*p0y*qy + 2*p0z*qz)/(qw^2 - qx^2 - qy^2 + qz^2) - (2*qz*(h - p0z*(qw^2 - qx^2 - qy^2 + qz^2) - p0x*(2*qw*qy + 2*qx*qz) + p0y*(2*qw*qx - 2*qy*qz)))/(qw^2 - qx^2 - qy^2 + qz^2)^2];
                    dAltdz = - 1 / cosine;
                    
                    % Measurement matrix
                    %x = [    θ_ε,      ω_b_ε,    acc_b_ε,              p_ε,        v_ε,    h_ε ].'
                    Hx = [ dAltdq, zeros(1,3), zeros(1,3),   0,   0, dAltdz, zeros(1,3), dAltdz ];
                    
                    Hk = Hx * Xdx;
                    
%                     Hk = Hk(:,10:end);
                    
                    % Measurement covariance
                    R = obj.measurement_covariance(17,17);
                    
                    
                    %% The rest (but i) is in common with all the other sensors
                    
                    % Interval containing the states that will be modified
                    i = [1,9, 16,22];
%                     i = [16,20, 21,22];
                    
                    method = "standard";
            
                    gen_kalman_correction(obj, i, R, Hk, z, method)
                end
            end
        end
        
        %%
        function x = state_predict(obj, gyro_meas, acc)
            % Relevant quantities
            x = obj.state;
            
            dt = obj.sample_time;
            
            % States
            q = x(1:4);
            w_b = x(5:7);
            acc_b = x(8:10);
            lin_acc = x(11:13);
            mag_d = x(14:16);
            p = x(17:19);
            v = x(20:22);
            h_g = x(23);
            
            
            % Angular velocities
            w_new = gyro_meas - w_b;

            w_old = obj.ang_vel_old;

            w_avg = 1/2 * (w_new + w_old);

            
            g = obj.gravity * [0;0;1];

            q_old = q;

          	
            % State prediction
            % quaternion(w_avg.'*dt, 'rotvec') = exp(1/2*dt*quaternion([0,w_avg.']))
            q = compact( quaternion(q_old.') * ( quaternion(w_avg.'*dt, 'rotvec') + dt^2/24*quaternion([0,cross(w_old, w_new).']) ) ).';
            %q = normalize(q, 'norm');
            %w_b = w_b;
            %acc_b = acc_b;
            lin_acc = ( (1 - obj.lin_acc_decay_factor*dt) * eye(3) ) * lin_acc;
            mag_d = ( (1 - obj.magn_disturbance_decay_factor*dt) * eye(3) ) * mag_d;
            
            q = normalize(q, 'norm');
            q_w = q(1);
            q_v = [q(2), q(3), q(4)];
            R = (q_w^2 - q_v*q_v.') * eye(3) + 2*(q_v.'*q_v) + 2*q_w*skew(q_v);
            
            p = p + v*dt + 1/2*dt^2 * ( R*(acc + acc_b) + g );
            v = v + dt * ( R*(acc + acc_b) + g );
            h_g = ( 1 - obj.ground_height_decay_factor*dt ) * h_g;
            

            % State vector
            x = [q
                 w_b
                 acc_b
                 lin_acc
                 mag_d
                 p
                 v
                 h_g];
            
            obj.state = x;
        end

        function P = state_cov_predict(obj, gyro_meas, acc)
            % Relevant quantities
            dt = obj.sample_time;
            
            ni = obj.lin_acc_decay_factor;
            sigma = obj.magn_disturbance_decay_factor;
            lambda = obj.ground_height_decay_factor;

            w_new = gyro_meas - obj.state(5:7);
            obj.ang_vel_old = w_new;
            
            q = obj.state(1:4);
            q_v = q(2:4);
            R = eye(3) + 2*skew(q_v) * ( q(1)*eye(3) + skew(q_v) );
            
            acc_bias = obj.state(8:10);

            
            % Error state transition matrix
            F_err = [           eye(3) - dt*skew(w_new), - dt*eye(3),     zeros(3),         zeros(3),            zeros(3),   zeros(3),   zeros(3),           zeros(3,1)
                                               zeros(3),      eye(3),     zeros(3),         zeros(3),            zeros(3),   zeros(3),   zeros(3),           zeros(3,1)
                                               zeros(3),    zeros(3),       eye(3),         zeros(3),            zeros(3),   zeros(3),   zeros(3),           zeros(3,1)
                                               zeros(3),    zeros(3),     zeros(3), (1-ni*dt)*eye(3),            zeros(3),   zeros(3),   zeros(3),           zeros(3,1)
                                               zeros(3),    zeros(3),     zeros(3),         zeros(3), (1-sigma*dt)*eye(3),   zeros(3),   zeros(3),           zeros(3,1)
                      - 1/2*dt^2*R*skew(acc + acc_bias),    zeros(3), + 1/2*dt^2*R,         zeros(3),            zeros(3),     eye(3),  dt*eye(3),           zeros(3,1)
                            - dt*R*skew(acc + acc_bias),    zeros(3),       + dt*R,         zeros(3),            zeros(3),   zeros(3),     eye(3),           zeros(3,1) 
                                             zeros(1,3),  zeros(1,3),   zeros(1,3),       zeros(1,3),          zeros(1,3), zeros(1,3), zeros(1,3), (1-lambda*dt)*eye(1) ];
            
            
            % Process covariance prediction
            Q = obj.process_covariance;

            P = obj.state_covariance;

            P = F_err * P * F_err.' + Q;

            obj.state_covariance = P;
            
            
            % State covariance associated with the unknown beacons posotion
            obj.uwb_unkn_state_covariance = obj.uwb_unkn_state_covariance + obj.uwb_unkn_proc_noise*dt;
            
            
            % Steps passed since the last absolute position correction
            obj.steps_last_pos = obj.steps_last_pos + 1;
        end
        
        function state_err = substitute_state_err_and_state_cov(obj, x_err, P_cor, i)
            % The interval is i(1):i(2) + i(3):i(4) or a:b + c:d
            a = i(1); b = i(2); c = i(3); d = i(4);

            % The other interval is 1:h, divided in 1:f and g:h
            f = b - a + 1;
            g = f + 1;
            h = f + d - c + 1;
            
            % Initialize state_err
            state_err = zeros(22,1);
            
            % Populate state_err
            state_err(a:b) = x_err(1:f);
            state_err(c:d) = x_err(g:h);

            % Update state_covariance
            state_cov = obj.state_covariance;

            state_cov(a:b,a:b) = P_cor(1:f,1:f);
            state_cov(c:d,a:b) = P_cor(g:h,1:f);
            state_cov(a:b,c:d) = P_cor(1:f,g:h);
            state_cov(c:d,c:d) = P_cor(g:h,g:h);

            obj.state_covariance = state_cov;
        end
    end
    
    
    %% Static Methods
    
    methods(Static)
        % Rotation with quaternions
        function v_out = quat_rot(v, q)
            quat_v = quaternion([0, v(1), v(2), v(3)]);
    
            quat_v_out = q * quat_v * conj(q);
    
            % Quaternion parts extraction.
            % By doing this, the output is in the same form of the input
            % (row or column vector)
            [~, v(1), v(2), v(3)] = parts(quat_v_out);
            
            v_out = v;
        end
        
        % Kalman equations.
        function [x_err, P] = kalman_eq(P, R, H, z)
            % Innovation covariance
            S = R + H * P * H.';
            
            % Kalman gain
            K = P * H.' / S;
            
            % Error estimate covariance update
            P = P - K * H * P;
%             P = (eye(size(P,1))-K*H) * P * (eye(size(P,1))-K*H).' + K * R * K.';
            
            x_err = K * z;
        end
        
        function [alpha, R_bar] = mahalanobis_adaptively_robust(P, R, H, z)
            % Output initialization
            % Only one between alpha and R is not NaN
            alpha = NaN;
            R_bar = NaN;
            
            % Covariance of the residual
            P_z = H * P * H.' + R;
                        
            % Mahalanobis distance computation
            mah = z.' * P_z^-1 * z;
            
            % sign_v = 0.01;                           % significant value
            % dof = 6;                                  % degrees of freedom
            % chi_sqr_sign = chi2inv(1-sign_v, dof);
            % The value of chi_sqrt_sign is not computed again at every
            % iteration
%             chi_sqr_sign = 22.4577;
            chi_sqr_sign = 16.8119;

            % Mahanobis distance used as statistical test
            if mah <= chi_sqr_sign
                % Adaptive filter
                Delta_z = ( (z.' * z) / sqrt(trace(P_z)) )^0.5;
                
                % Threshold
                c = 1.3;
                
                % Adaptive factor computation
                if abs(Delta_z) <= c
                    alpha = 1;
                else
                    alpha = c / abs(Delta_z);
                end
                
            else
                % Robust filter
                c = 1.0;
                
                % Robust factor initialization
                lambda = zeros(size(R));
                
                % Normalized residuals
                z_norm = ( z.^2 ./ (diag(P_z).^(0.5)) );
                
                % One factor for each measurement
                for i=1:size(R,1)
                    if abs(z_norm(i)) <= c
                        lambda(i,i) = 1;
                    else
%                         lambda(i,i) = abs(z_norm(i)) / c;
                        lambda(i,i) = (mah / chi_sqr_sign);
                    end
                end
                
                % Robust measurement covariance
                R_bar = lambda * R;
            end
        end
        
        % Adaptive Kalman filter equations with adaptive factor.
        function [x_err, P] = adaptive_factor_kalman_eq(P, R, H, z, alpha)
            % Innovation covariance
            S = R + 1/alpha * H * P * H.';
            
            % Kalman gain
            K = 1/alpha * P * H.' / S;
            
            % Error estimate covariance update
            P = P - K * H * P;
%             P = (eye(size(P,1))-K*H) * P * (eye(size(P,1))-K*H).' + K * R * K.';
            
            x_err = K * z;
        end

        % Auxiliary for the Kalman filter
        function [Xdx] = X_deltax(q, n)
            [qw, qx, qy, qz] = parts(q);
            qv = [qx, qy, qz];

            q_matrix = [ qw  ,                - qv
                         qv.', qw*eye(3) + skew(qv) ];

            Q_deltatheta = q_matrix * 1/2 * [zeros(1,3); eye(3)];
    
            Xdx = [ Q_deltatheta, zeros(4,n)
                      zeros(n,3),     eye(n) ];
        end
    end
 end
