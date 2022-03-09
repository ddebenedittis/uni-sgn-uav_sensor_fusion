%% Description
%
% Generate a coil trajectory. This trajectory is useful when there are
% unknown UWB beacons and no other absolute position sensors available.
% This type of trajectory allows the system to estimate the UWB anchor
% position and at the same time use them for navigation (SLAM). The beacon
% position is first be initialized with an appropriate algorithm.


%% Initialization

clc
clear variables
close all

sch = 'shuffle';

rng(sch)

fprintf('Started. \n \n')

% Add the folder and the sub-folders to MATLAB path
addpath(genpath('custom_functions'))
addpath(genpath('datasets'))


%% Simulation Setup

% Sensor frequencies [Hz]
imuFs = 100;        	% IMU (accelerometer and gyroscope)
magFs = 20;             % magnetometer
uwbFs = 10;             % UWB

Ts = 1/imuFs;           % IMU sampling time

duration = 40;                  % simulation total duration [s]
loopBound = duration*imuFs;     % number of samples of the simulation


%% Initial position and Earth parameters

% Starting position
refloc = [43.70853, 10.4036, 57];                       	% Latitude, longitude [°], height [m]


%% Coil trajectory generation

radius = 2;         % [m ] rotation radius
speed = 0.5;        % [m/s] forward speed
omega = 1.5;        % [rad/s] trajectory rotation angular speed
initialYaw = 0;     % [deg]
initialPitch = 0;   % [deg]
initialRoll = 0;    % [deg]

% Initial position, velocity and speed
initPos = [0, 0, 0];
initVel = [speed, 0, omega*radius];
initOrientation = quaternion([initialYaw,initialPitch,initialRoll],'eulerd','zyx','frame');

% Kinematic trajectory generation
trajectory = kinematicTrajectory('SampleRate',imuFs, ...
    'Velocity',initVel, ...
    'Position',initPos, ...
    'Orientation',initOrientation);

timeVector = (0:(loopBound-1))/imuFs;

accBody = zeros(loopBound,3);
accBody(:,1) = 0.05;
accBody(:,2) = omega^2*radius * cos(omega * timeVector);
accBody(:,3) = - omega^2*radius * sin(omega * timeVector);

angVelBody = zeros(loopBound,3);

pitchRotation = quaternion([0,initialPitch,0],'eulerd','zyx','frame');
angVelBody = rotateframe(pitchRotation,angVelBody);
accBody = rotateframe(pitchRotation,accBody);

[trajPos, trajOrient, trajVel, trajAcc, trajAngVel] = trajectory(accBody,angVelBody);


%% IMU Sensors

imu = imuSensor('accel-gyro-mag', 'SampleRate', imuFs);

% Earth magnetic field in the UAV starting position.
% MATLAB 2020 needed for this command. If a different (older) version is
% being used, you can change the year and the model used (the '2020') to
% make it work for your current version.
magnfield = (wrldmagm(refloc(3), refloc(1), refloc(2), decyear(date, 'dd-mm-yyyy'), '2020') / 1000).';
imu.MagneticField = magnfield;

% Real(istic) Sensors

real_acc_bias = 0.3905 * [0, -0, 0];
real_gyro_bias = deg2rad(3) * [0, -0, 0];
real_mag_dist = [0, 0, 0];

% Accelerometer
imu.Accelerometer.MeasurementRange =  19.62;
imu.Accelerometer.Resolution = 0.00059875;
imu.Accelerometer.ConstantBias = real_acc_bias;
imu.Accelerometer.NoiseDensity = 0.003924;

% Gyroscope
imu.Gyroscope.MeasurementRange = deg2rad(250);
imu.Gyroscope.Resolution = deg2rad(0.00625);
imu.Gyroscope.ConstantBias = real_gyro_bias;
imu.Gyroscope.AxesMisalignment = 1.5*0;
imu.Gyroscope.NoiseDensity = deg2rad(0.025);

% Magnetometer
imu.Magnetometer.MeasurementRange = 1000;
imu.Magnetometer.Resolution = 0.1;
imu.Magnetometer.ConstantBias = 0;  % Supposed to have been previously compensated during the sensor setup
imu.Magnetometer.NoiseDensity = 0.3/ sqrt(50);


% Sensor orientations (s_i to body)
% They should be all the same if AHRS
q_sens_acc  = quaternion([1 0 0 0]);
q_sens_gyro = quaternion([1 0 0 0]);
q_sens_magn = quaternion([1 0 0 0]);


%% Ultra Wide Band Sensor

% The beacons are divided in known and unknown, depending on if they are
% known by the navigation system or not.

% Beacon (anchor) positions and id with known coordinates (by the 
% navigation sys)
known_beacon_pos = [ 1,   1,  1];     % 1

known_beacon_id = linspace(1,size(known_beacon_pos,1),size(known_beacon_pos,1));


% Beacon (anchor) position nad id with unknown coordinates (by the
% navigation sys)
unknown_beacon_pos = [  2,   2,   0
                       10,   3,  -1
                        4,   8,   0
                        6,  -2, -0.5
                        30,  8,  -2
                        40, -5,   2
                        50,  0,   0
                        60, -1, -1];

gne = 101;
unkn_num =  size(unknown_beacon_pos, 1);
unknown_beacon_id = linspace(11, gne+unkn_num-1, unkn_num);


% Beacons which have a position already known to the navigation system.
% This division is only useful to simplify the code.

% UWB sensor system object creation
uwb_known = uwb_sensor('beacon_position', known_beacon_pos, ...
                       'beacon_id', known_beacon_id, ...
                       'error_variance', 0.0119^2, ...
                       'range', 30, ...
                       'seed', sch);


% UWB sensor system object istance
% Simulates only the beacons with unknown position to the nav system.
uwb_unknown = clone(uwb_known);

uwb_unknown.beacon_position = unknown_beacon_pos;
uwb_unknown.beacon_id = unknown_beacon_id;
uwb_unknown.error_variance = 0.0119^2;
uwb_unknown.seed = sch;


%% Barometer Sensor

bar = barometer_sensor;

bar.wn_variance = 0.5^2;
bar.bi_variance = 0.03^2;
bar.decay_factor = 0.0001;

% The output is quantized
bar.quantization = 0.05;


%% Laser (?) Altimeter Sensor

alt = altimeter_sensor;

alt.wn_variance = 0.05^2;
alt.qwn_variance = 0.3^2 / 7^4;
alt.bi_variance = 0.01^2;
alt.decay_factor = 0.2;

% Minimum and maximum measurable distances
alt.min_alt = 0.1;
alt.max_alt = 5;

% The output is quantized
alt.quantization = 0.001;

% The sensor is supposed to be perfectly perpendicular to the ground and
% with the following displacement relative to body frame
alt.p_0 = [0 0 0.1];


%% Filter setup

% % Number of accelerometer and gyroscope measurements which are considered
% % together in the Kalman filter. Increase this value to reduce the
% % computational cost, at the expense of estimation precision.
decimation_factor = 1;

% Filter call and properties initialization
FUSE = my_navigation_filter_adaptive;

dt = Ts;

% Sample rates
FUSE.sample_rate = imuFs;

% Decimation factor
FUSE.decimation_factor = decimation_factor;

% Some process noises
FUSE.pos_noise = 2e-4;
FUSE.vel_noise = 1e-6;

% Accel, gyro, magn noise
FUSE.acc_noise = 1e+8;
FUSE.magn_noise = 1e+8;
FUSE.gyro_noise = 1e-5;

% IMU sensor frames orientation
FUSE.q_sens_acc = q_sens_acc;
FUSE.q_sens_gyro = q_sens_gyro;
FUSE.q_sens_magn = q_sens_magn;

% Gyroscope drift
FUSE.gyro_drift_noise = 1e-5;

% Linear acceleration
FUSE.lin_acc_noise = 0e-10 / dt;
FUSE.lin_acc_decay_factor = 0 / dt;

% Magnetometer disturbance
FUSE.magn_disturbance_noise = 1e-1 / dt;
FUSE.magn_disturbance_decay_factor = 1 * imuFs;

% GPS
FUSE.GPS_pos_noise = 1e+2;
FUSE.GPS_vel_noise = 1e+0;

% Acceleration drift (bias)
FUSE.acc_drift_noise = 1e-6 / dt;

% UWB
FUSE.uwb_noise = 1e+1;
FUSE.uwb_beacon_position = known_beacon_pos;
FUSE.uwb_beacon_id = known_beacon_id;

% Barometer
FUSE.bar_noise = 1e+2;

% Altimeter
FUSE.alt_pos = [0 0 0.1];
FUSE.alt_noise = 1e-1;
FUSE.alt_threshold = 0.3;

FUSE.ground_height_noise = 1e+0 / dt;
FUSE.ground_height_decay_factor = 1e-1 / dt;


FUSE.init_process_noise = diag([0e-0 * ones(3,1)
                                1e-1 * ones(3,1)
                                1e-2 * ones(3,1)
                                1e-1 * ones(3,1)
                                0e+0 * ones(3,1)
                                1e+0 * ones(3,1)
                                1e+0 * ones(3,1)
                                1e+0 * ones(1,1)]);

FUSE.magn_field_NED = magnfield.';

FUSE.reference_location = refloc;

FUSE.private_properties_update;


FUSE(1);    % To initialize the discrete state. TODO: eliminate the need for this.


% Initialize some filter states
FUSE.set_initial_state('q', compact(trajOrient(1)).', ...
                       'gyro_offset', [0,0,0], ...
                       'pos', trajPos(1,:).', ...
                       'vel', trajVel(1,:).', ...
                       'acc_bias', [0,0,0]);


%% Plot setup

% Colors
default_colors = colororder;

used_colors = [0 0 1;
               1 .647 0;
               1 1 0;
               1 0 1;
               0 1 0;
               0 1 1;
               1 0 0];

uwb_beacon_dim = 50;


% Main view
fig = figure();
fig.WindowState = 'maximized';

hax1 = axes;

% Real trajectory plot
plot3(trajPos(:,1),trajPos(:,2),trajPos(:,3))
xlabel('North (m)')
ylabel('East (m)')
zlabel('Down (m)')
title('Trajectory')
hold on
grid on
axis equal

% Estimated trajectory
est_traj = animatedline(hax1, 'Color', 'black', 'LineWidth', 1);
est_pos = scatter3(NaN, NaN, NaN, 25, 'black', 'o');

% Known uwb beacons plot
scatter3(known_beacon_pos(:,1), known_beacon_pos(:,2), known_beacon_pos(:,3), uwb_beacon_dim, used_colors(6,:), 'd', 'LineWidth', 1);

% Unknown uwb beacons plot
scatter3(unknown_beacon_pos(:,1), unknown_beacon_pos(:,2), unknown_beacon_pos(:,3), uwb_beacon_dim, used_colors(7,:), 'd', 'LineWidth', 2)

% UWB beacons position estimate container.
ii = length(unknown_beacon_id);
beacon_estimate_plot(ii) = scatter3(hax1, [], [], []);


% States animatedline plot initialization
% [anim_est_states, anim_real_states, fig3] = anim_states_create(default_colors);


%% Simulation Loop

% Log data for final metric computation.
pqorient = quaternion.zeros(loopBound,1);
pqpos = zeros(loopBound,3);
pqvel = zeros(loopBound,3);
pqacc = zeros(loopBound,3);

% Number of uwb anchors in range
uwb_known_beacon_number = 0;

% Memory of the uwb beacons indexes in range, used to change their color in
% the plot.
known_beacon_id_old = [];
uwb_beacon_id_old = [];

% Memory of the uwb unknown beacons indexes in range, used to change their
% color in the plot.
unknown_beacon_id_old = [];
uwb_beacon_detect_id_old = [];

% Memory of the unknown  ArUco markers indexes in range, used to change
% their color in the plot.
vis_id_old = [];
vis_detect_id_old = [];


fcnt = 2;

while fcnt <= loopBound
    % Simulate the IMU data from the current pose.
    % [position, orientation, velocity, acceleration, angularVelocity] = trajectory()
    [accel, gyro, mag] = imu(trajAcc(fcnt,:), ...
                             trajAngVel(fcnt, :), ...
                             trajOrient(fcnt));
	
	% Sensor output
    % accel = - f_b;
    % gyro = omega_ib_b;
	
	% Rotate the outputs because the sensor frame does not coincide with
	% the body frame.
    accel = quat_rot(accel, conj(q_sens_acc));      % conj because we rotate from the boody frame to the sensor frame
    gyro = quat_rot(gyro, conj(q_sens_gyro));
    mag = quat_rot(mag, conj(q_sens_magn));
    
    
    % The filter predict step is done at the IMU (accelerometer and gyro)
    % frequency.
    FUSE.predict(gyro, accel);
    
    
    % P_i is updated only if there is the need to perform a correction step
    % using that specific part of P. This justifies the need for the if
    % cycle on the update of P_i and the variables P_i_already_predicted.
    % Some useless optimizations on the cycle and on the variable update
    % have been performing by commenting an unused line.
    
%     % The correction using the accelerometer data is done every
%     % decimation_factor sample times.
%     if mod(fcnt, fix(decimation_factor)) == 0
%         % Accelerometer correction
%        z_acc = FUSE.fuse_accel;
%     end
%     
%     
%     % Magnetometer correction
%     if mod(fcnt, fix(imuFs/magFs)) == 0
%         % Fuse the magnetomer measurement
%         z_mag = FUSE.fuse_magn(mag);
%     end
    
%     % UWB correction step
%     if mod(fcnt, fix(imuFs/uwbFs)) == 0
%         %Simulate sensor
%         [uwb_meas_dist, uwb_beacon_id, uwb_index] = uwb_known(trajPos(fcnt,:), known_beacon_id);
%         
%         if isempty(uwb_beacon_id) == false
%             FUSE.fuse_uwb(uwb_meas_dist, uwb_beacon_id);
%         end
%         
%         
%         % Change the color of the UWB anchors depending on their state.
%         % Legend:   dark blue:      known, obstructed
%         %           light blue:     known, not obstructed
%         %           green:          known, received
%         %           red:            unknown, not received
%         %           purple:         unknown, received
%         %           orange:         unknown estimate
%         if length(known_beacon_id) ~= length(known_beacon_id_old) || length(uwb_beacon_id) ~= length(uwb_beacon_id_old)
%             known_beacon_id_old = known_beacon_id;
%             uwb_beacon_id_old = uwb_beacon_id;
%             
%             % Reset the color
%             scatter3(hax1, known_beacon_pos(:,1), known_beacon_pos(:,2), known_beacon_pos(:,3), uwb_beacon_dim, used_colors(1,:), 'd', 'LineWidth', 1)
%             
%             % Plot the not obscured ones
%             accessible_uwb_plot = scatter3(hax1, known_beacon_pos(known_beacon_id_old,1), known_beacon_pos(known_beacon_id_old,2), known_beacon_pos(known_beacon_id_old,3), uwb_beacon_dim, used_colors(6,:), 'd', 'LineWidth', 1);
%             
%             % Plot the received ones
%             visible_uwb_plot = scatter3(hax1, known_beacon_pos(uwb_index,1), known_beacon_pos(uwb_index,2), known_beacon_pos(uwb_index,3), uwb_beacon_dim, used_colors(5,:), 'd', 'LineWidth', 1);
%         elseif sum(known_beacon_id ~= known_beacon_id_old) || sum(uwb_beacon_id ~= uwb_beacon_id_old)
%             known_beacon_id_old = known_beacon_id;
%             uwb_beacon_id_old = uwb_beacon_id;
%             
%             % Reset the color
%             scatter3(hax1, known_beacon_pos(:,1), known_beacon_pos(:,2), known_beacon_pos(:,3), uwb_beacon_dim, used_colors(1,:), 'd', 'LineWidth', 1)
%             
%             % Plot the not obscured ones
%             accessible_uwb_plot = scatter3(hax1, known_beacon_pos(known_beacon_id_old,1), known_beacon_pos(known_beacon_id_old,2), known_beacon_pos(known_beacon_id_old,3), uwb_beacon_dim, used_colors(6,:), 'd', 'LineWidth', 1);
%             
%             % Plot the received ones
%             visible_uwb_plot = scatter3(hax1, known_beacon_pos(uwb_index,1), known_beacon_pos(uwb_index,2), known_beacon_pos(uwb_index,3), uwb_beacon_dim, used_colors(5,:), 'd', 'LineWidth', 1);
%         end
%     end
    
    
    % UWB beacon detect step. This is done at a lower frequency compared
    % to the uwb frequency.
    if mod(fcnt, fix(imuFs/uwbFs)) == 0
        % Detect the UWB anchor position
        
        % Simulate unknown sensor
        [uwb_unkn_meas_dist, uwb_unkn_beacon_id, uwb_unkn_index] = uwb_unknown(trajPos(fcnt,:), unknown_beacon_id);
        
        % Detect the beacon only if there is a measure available
        if isempty(uwb_unkn_beacon_id) == false
            [uwb_anchor, z_uwb] = FUSE.fuse_unknown_uwb(uwb_unkn_meas_dist, uwb_unkn_beacon_id);
        end

        % Draw the estimate.
        if mod(fcnt, 5 * fix(imuFs/uwbFs)) == 0
            for ii = 1:length(uwb_unkn_beacon_id)
                delete(beacon_estimate_plot(uwb_unkn_index(ii)));
            
                beacon_estimate_plot(uwb_unkn_index(ii)) = scatter3(hax1, uwb_anchor(ii, 1), uwb_anchor(ii, 2), uwb_anchor(ii, 3), uwb_beacon_dim, used_colors(2,:), 'LineWidth', 2);
            end
        end
        
        % Change the color of the UWB anchors depending on their state.
        % Legend:   dark blue:      known, obstructed
        %           light blue:     known, not obstructed
        %           green:          known, received
        %           red:            unknown, not received
        %           purple:         unknown, received
        %           orange:         unknown estimate
        if length(unknown_beacon_id) ~= length(unknown_beacon_id_old) || length(uwb_unkn_beacon_id) ~= length(uwb_beacon_detect_id_old)
            unknown_beacon_id_old = unknown_beacon_id;
            uwb_beacon_detect_id_old = uwb_unkn_beacon_id;
            
            % Reset the color
            scatter3(hax1, unknown_beacon_pos(:,1), unknown_beacon_pos(:,2), unknown_beacon_pos(:,3), uwb_beacon_dim, used_colors(7,:), 'd', 'LineWidth', 2)
            
            % Plot the received ones
            visible_uwb_plot = scatter3(hax1, unknown_beacon_pos(uwb_unkn_index,1), unknown_beacon_pos(uwb_unkn_index,2), unknown_beacon_pos(uwb_unkn_index,3), uwb_beacon_dim, used_colors(4,:), 'd', 'LineWidth', 2);
            
        elseif sum(unknown_beacon_id ~= unknown_beacon_id_old) || sum(uwb_unkn_beacon_id ~= uwb_beacon_detect_id_old)
            unknown_beacon_id_old = unknown_beacon_id;
            uwb_beacon_detect_id_old = uwb_unkn_beacon_id;
            
            % Reset the color
            scatter3(hax1, unknown_beacon_pos(:,1), unknown_beacon_pos(:,2), unknown_beacon_pos(:,3), uwb_beacon_dim, used_colors(7,:), 'd', 'LineWidth', 2)
            
            % Plot the received ones
            visible_uuwb_plot = scatter3(hax1, unknown_beacon_pos(uwb_unkn_index,1), unknown_beacon_pos(uwb_unkn_index,2), unknown_beacon_pos(uwb_unkn_index,3), uwb_beacon_dim, used_colors(4,:), 'd', 'LineWidth', 2);
        end
    end
    
    
    [pqorient(fcnt), pqpos(fcnt, :)] = FUSE.get_pose();
    
    addpoints(est_traj, pqpos(fcnt, 1),pqpos(fcnt, 2),pqpos(fcnt, 3));
    set(est_pos, 'XData', pqpos(fcnt, 1), 'YData', pqpos(fcnt, 2), 'ZData', pqpos(fcnt, 3))
    drawnow limitrate
    
    time_x = fcnt / imuFs;
    
%     anim_states_add(anim_est_states, anim_real_states, time_x, FUSE.getDiscreteState.state, real_gyro_bias, real_acc_bias, trajAcc(fcnt,:), real_mag_dist, trajVel(fcnt,:))
    
    
    
    % Increase the counter
    fcnt = fcnt + 1;
%     pause(0.01)
end


%% Functions

% Rotation with quaternions
function v_out = quat_rot(v, q)
    quat_v = quaternion([0, v(1), v(2), v(3)]);
    
    quat_v_out = q * quat_v * conj(q);
    
    % Quaternion parts extraction
    [~, part1, part2, part3] = parts(quat_v_out);
    
    % With this way, the output is in the same form of the input (row or
    % column vector)
    v(1) = part1;
    v(2) = part2;
    v(3) = part3;
    
    v_out = v;
end

