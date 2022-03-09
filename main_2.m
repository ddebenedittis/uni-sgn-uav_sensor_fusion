%% Description
%
% Generate a waypoint trajectory in a semi-structured environment (manually
% created) and visualize the estimate of the position of the drone.

% Once the trajectory (not the scenario) has been changed it is necessary
% to recreate it by selecting the correct choice in the corresponding
% questdlg. Once this has been done, if there are no further changes in the
% trajectory, for later simulation it can be reused bu loading the saved
% trajectory data (this avoids the slow trajectory generation process).


%% Initialization

clc
clear variables
close all

rng('shuffle')

fprintf('Started. \n \n')

% Add the folder and the sub-folders to MATLAB path
addpath(genpath('custom_functions'))
addpath(genpath('datasets'))


%% Simulation Setup

% Sensors frequencies [Hz]
imuFs = 020;    % IMU (accelerometer and gyroscope)
magFs =  20;    % magnetometer
gpsFs =   5;    % GPS
uwbFs =  20;    % Ultra Wide Band
visFs =   5;    % vision system (ArUco marker)
barFs =   1;    % barometer
altFs =  20;	% altimeter

Ts = 1/imuFs;   % IMU sampling time


%% Initial position and Earth parameters

% Starting position
refloc = [43.70853, 10.4036, 57];                       	% Latitude, longitude [°], height [m]

% Earth parameters and their influence on navigation are neglected.

% % Earth rotation rate
% omega_ei_i = 72.92115*10^-6 * [0; 0; 1];                    % Earth angular velocity in ECI [rad/s]
% omega_ie_i = - omega_ei_i;
% omega_ie = 72.92115*10^-6 * [cos(lat_0); 0; -sin(lon_0)];	  % Earth angular velocity in NED [rad/s]
% 
% % WGS84 initialization
% wgs84 = wgs84Ellipsoid;
% re = wgs84.SemimajorAxis;
% eccentricity = wgs84.Eccentricity;
% % Gravity approximation parameters
% g_wgs0 = 9.7803267714;
% g_wgs1 = 0.00193185138639;


%% Generate UAV Trajectory

% Text for the questdlg
traj_opt_1 = 'Create a new one ex novo';
traj_opt_2 = 'Use a previously generated one';

load_traj = questdlg('Would you like to load the previously generated trajectory or to use a new one?', ...
                     'Trajectory creation', ...
                     traj_opt_1, traj_opt_2, traj_opt_2);

% Switch between creating the trajectory ex novo or loading the previous
% one
switch load_traj
    case traj_opt_1         % The trajectory is created ex novo
        % Total initialization time
        init_end = 5;
        
        % Orientations at the waypoints
        orientation_init = quaternion(eul2quat([.5, -1,1.5
                                                 0,  0,  0
                                                 0, pi,  0
                                                pi, pi,  0
                                                pi, pi, pi]));
        
        
        % Real trajectory
        
        % Create the waypoints for the trajectory
        waypoints = [ zeros(5,2), -40*ones(5,1)     % Initialization
                       0    0  -40;
                      30    0  -10;
                      50    0   -2;
                      50    0   -2;
                      75    0   -1.5;	% 10
                      80    7.5 -0.2;
                      85   15   -3.0;
                     102   15   -1.5;
                     110   +5   -1.5;
                     102   -5   -1.5;   % 15
                      95   -5   -1.5;
                      95   -5   -1.5;
                      95  -15   -1.5;
                      95  -15   -1.5;
                     110  -15   -1.5];  % 20
        
        delta_toa = [0, init_end*ones(1,5)/5, 30, 20, 10, 15,     5, 5, 20, 15, 15,     15, 2, 15, 2, 15];
        
        % You may not like it, but this is what peak variable naming looks
        % like.
        Mr_MATLAB_comma_speed_up_the_fkn_simulation_please = 1;
        delta_toa = [delta_toa(1:6), delta_toa(7:end)/Mr_MATLAB_comma_speed_up_the_fkn_simulation_please];
        
        % Time of arrival (at the waypoint)
        waypoint_toa = cumsum(delta_toa);
        
        % Velocity at the waypoint
        waypoint_vel = zeros(size(waypoints,1), 3);
        waypoint_vel(10, :) = [ 1, 0, 0];
        waypoint_vel(11, :) = [ 0, 1, 0];
        waypoint_vel(12, :) = [ 1, 0, 0];
        waypoint_vel(13, :) = [ 1, 0, 0];
        waypoint_vel(14, :) = [ 0,-1, 0];
        waypoint_vel(15, :) = [-1, 0, 0];
        
        % Orientation at the waypoints
        waypoint_orient = quaternion([eul2quat([.5, -1,1.5
                                                 0,  0,  0
                                                 0, pi,  0
                                                 pi, pi,  0
                                                 pi, pi, pi])
                                      ones(5,1) * [1 0 0 0] % 10
                                      0.7071 0 0 -0.7071
                                      1      0 0  0
                                      1      0 0  0
                                      0.7071 0 0  0.7071
                                      0      0 0  1     	% 15
                                      0      0 0  1
                                      0.7071 0 0  0.7071
                                      0.7071 0 0  0.7071
                                      1      0 0  0
                                      1      0 0  0]);  	% 20
        
        % Waypoint trajectory creation
        trajData = waypointTrajectory(waypoints, ...
                                      'TimeOfArrival', waypoint_toa, ...
                                      'Velocities', waypoint_vel, ...
                                      'Orientation', waypoint_orient, ...
                                      'SampleRate', imuFs, ...
                                      'AutoPitch', false, ...
                                      'AutoBank', false);
        
        % The output of the trajectory is in this form:
        % [position, orientation, velocity, acceleration, angularVelocity] = trajectory()
        
        
        % Generate all the trajectory points (slow process).
        
        fprintf('Generating trajectory... \n\n')
        
        % Number of trajectory points to generate
        loopBound = floor(imuFs * waypoint_toa(end) + 1);
        
        % Trajectory variables initialization
        trajPos = zeros(loopBound, 3);
        trajOrient = quaternion([1 0 0 0]) * ones(loopBound, 1);
        trajVel = zeros(loopBound, 3);
        trajAcc = zeros(loopBound, 3);
        trajAngVel = zeros(loopBound, 3);
        
        
        trajPos(1,:) = waypoints(1,:);
        trajOrient(1) = waypoint_orient(1);
        
        
        % The trajectory starts from 2 because the first position at time 0
        % (and only if the starting time is 0) is not recorded.
        fcnt = 2;
        
        % Real trajectory
        while ~isDone(trajData)
            [trajPos(fcnt,:), trajOrient(fcnt), trajVel(fcnt,:), trajAcc(fcnt,:), trajAngVel(fcnt,:)] = trajData();
            fcnt = fcnt+1;
        end
        
        % Add the final points
        trajPos(end,:)  = waypoints(end,:);
        trajOrient(end) = waypoint_orient(end);
        trajVel(end,:)  = waypoint_vel(end,:);
        
        
        fprintf('Trajectory generation complete. \n \n')


        % Save the trajectory variables of interest in a .mat file
        save('trajectory_data.mat', 'init_end', 'waypoints', 'waypoint_toa', 'loopBound', 'trajPos', 'trajOrient', 'trajVel', 'trajAcc', 'trajAngVel')
        
    case traj_opt_2     % Load the trajectory previously generated
        load('trajectory_data.mat')

        fprintf('Trajectory loading complete. \n \n')
end


% Initialize the random number generator used in the simulation of sensor
% noise.
rng(1)


%% Scenario selector

scenario = menu('Choose a scenario used for the simulation', ...
                'Scenario 1: Standard scenario', ...
                'Scenario 2: Sensor shortage near the end of the trajectory', ...
                'Scenario 3: Magnetometer disturbance', ...
                'Scenario 4: Faulty GPS measurements near the building', ...
                'Scenario 5: NLOS UWB errors');


%% IMU Sensors

imu = imuSensor('accel-gyro-mag', 'SampleRate', imuFs);

% Earth magnetic field in the UAV starting position.
% MATLAB 2020 needed for this command. If a different (older) version is
% being used, you can change the year and the model used (the '2020') to
% make it work for your current version.
magnfield = (wrldmagm(refloc(3), refloc(1), refloc(2), decyear(date, 'dd-mm-yyyy'), '2020') / 1000).';
imu.MagneticField = magnfield;

% Real(istic) Sensors

real_acc_bias = 0.3905 * [2, -1, 3];
real_gyro_bias = deg2rad(3) * [1, -2, 3];
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
imu.Magnetometer.ConstantBias = real_mag_dist;  % Supposed to have been previously compensated during the sensor setup
imu.Magnetometer.NoiseDensity = 0.3/ sqrt(50);


% Sensor orientations (s_i to body)
% They should be all the same if AHRS
q_sens_acc  = quaternion([1 0 0 0]);
q_sens_gyro = quaternion([1 0 0 0]);
q_sens_magn = quaternion([1 0 0 0]);


magnetometer_disturbance = zeros(loopBound, 3);

% When the chosen scenario is 3 a constant magnetometer disturbance is 
% introduced, otherwise it is null during all the navigation.
if scenario == 3
    NN = waypoint_toa(12)*imuFs - waypoint_toa(6)*imuFs + 1;
    magnetometer_disturbance(waypoint_toa(6)*imuFs : waypoint_toa(12)*imuFs, :) = ones(NN,1) * [1, -2, 3];
end


%% GPS Sensor

gps = gpsSensor('UpdateRate', gpsFs);
gps.ReferenceLocation = refloc;
gps.DecayFactor = 0.1;              	 % Random walk noise parameter
gps.HorizontalPositionAccuracy = 1.6;
gps.VerticalPositionAccuracy = 3;
gps.VelocityAccuracy = 0.1;


%% Ultra Wide Band Sensor

% The beacons are divided in known and unknown, depending on if they are
% known by the navigation system or not.

% Beacon (anchor) positions and id with known coordinates (by the 
% navigation sys)
known_beacon_pos = [ 61,  19,  0;   	% 1
                     61, -19,  0;       % 2
                     89,  19,  0;       % 3
                     89, -19,  0;       % 4
                    115,  15, -1;       % 5
                     95, 7.5,  0;       % 6
                     95,   0,  0;       % 7
                    100, -10,  0.5 ];   % 8  

% TODO: check.
known_beacon_id = linspace(1,size(known_beacon_pos,1),size(known_beacon_pos,1));


% Beacon (anchor) position and id with unknown coordinates (to the
% navigation system)
if scenario == 1
    unknown_beacon_pos = [70, +10, 0];
    
    unknown_beacon_id = 10;
else
    unknown_beacon_pos = [];
    
    unknown_beacon_id = [];
end


% Beacons which have a position already known to the navigation system.
% This division is only useful to simplify the code.

% UWB sensor system object creation
uwb_known = uwb_sensor('beacon_position', known_beacon_pos, ...
                       'beacon_id', known_beacon_id, ...
                       'error_variance', 0.0119^2, ...
                       'range', 30);

% Ideal sensor, used to compare uwb measurements with ground truth
uwb_known_ideal = uwb_sensor('beacon_position', known_beacon_pos, ...
                             'beacon_id', known_beacon_id, ...
                             'error_variance', 0, ...
                             'range', 300);


% Unknown UWB sensor system object istance
% Simulates only the beacons with unknown position to the nav system.
uwb_unknown = clone(uwb_known);

uwb_unknown.beacon_position = unknown_beacon_pos;
uwb_unknown.beacon_id = unknown_beacon_id;
uwb_unknown.error_variance = 0.0119^2;


%% Vision Sensor

% Camera parameters

% Camera detection cone semi-angle [radians]
camera_cone_semi_angle = 40 * pi / 180;

% Maximum distance that the vision sensor is able to measure
camera_max_distance = 40;

% Minimum distance that the vision sensor is able to measure
camera_min_distance = 0.05;

% The noise on both position and orientation is modeled as the sum
% of a constant noise and a noise linear with the distance of the
% target from the camera.
% In addition, the noise, for both orientation and position, is
% different if the component is perpendicular or parallel to the
% line of sight.
% For position, the error is bigger on the parallel component.
noise_perpendicular_position_constant = (0.1)^2;
noise_parallel_position_constant      = (0.2)^2;
noise_perpendicular_position_linear   = (0.02 / camera_max_distance)^2;
noise_parallel_position_linear        = (0.04 / camera_max_distance)^2;

% For orientation, the error is bigger on the perpendicular
% component.
noise_perpendicular_orientation_constant = (2*pi/180)^2;
noise_parallel_orientation_constant      = (1*pi/180)^2;
noise_perpendicular_orientation_linear   = (4*pi/180 / camera_max_distance)^2;
noise_parallel_orientation_linear        = (2*pi/180 / camera_max_distance)^2;

% Camera frame orientation with respect to the body frame,
% specified as euler angles [yaw, pitch, roll].
camera_orientation = [0, -pi/3, 0];

% Optical center position with respect to the body center of mass 
% in the body frame.
% p_cb_b
camera_position = [0.1, 0, 0.1];


% Marker parameters

if scenario == 1

    % Markers reference frame orientation wrt global frame, specified as
    % [yaw, pitch, roll] and in [rad]. Row vector.
    known_marker_orientation = [0, pi/2, 0;
                                0, pi/2, 0;
                                0, pi/2, 0;
                                0, pi/2, 0;
                                0, pi/2, 0];

    % Markers position in global frame [m].
    known_marker_position = [109,  10,   0;
                             109,   0,   0;
                             100,  -5,   0;
                              95, -15,   0;
                             100,  15,   0];

    % Known markers (by the navigation system)
    % Marker id
    known_marker_id = linspace(1, size(known_marker_orientation, 1), size(known_marker_orientation, 1));


    % Unknown markers (by the navigation system)
    unknown_marker_id = [17, 18];

    unknown_marker_orientation = [0 0 0;
                                  0 0 0];

    unknown_marker_position = [ 50    0  0;
                               110, -15, 0 ];
elseif scenario ~= 2

    % Markers reference frame orientation wrt global frame, specified as
    % [yaw, pitch, roll] and in [rad]. Row vector.
    known_marker_orientation = [0, pi/2, 0;
                                0, pi/2, 0;
                                0, pi/2, 0;
                                0, pi/2, 0;
                                0, pi/2, 0];

    % Markers position in global frame [m].
    known_marker_position = [109,  10,   0;
                             109,   0,   0;
                             100,  -5,   0;
                              95, -15,   0;
                             100,  15,   0];

    % Known markers (by the navigation system)
    % Marker id
    known_marker_id = linspace(1, size(known_marker_orientation, 1), size(known_marker_orientation, 1));
    
    
    unknown_marker_id = [];

    unknown_marker_orientation = [];

    unknown_marker_position = [];
elseif scenario == 2
    % Markers reference frame orientation wrt global frame, specified as
    % [yaw, pitch, roll] and in [rad]. Row vector.
    known_marker_orientation = [];

    % Markers position in global frame [m].
    known_marker_position = [];

    % Known markers (by the navigation system)
    % Marker id
    known_marker_id = [];
    
    
    unknown_marker_id = [];

    unknown_marker_orientation = [];

    unknown_marker_position = [];
end


% Aruco sensor simulator full setup ( ͡ʘ ͜ʖ ͡ʘ)
aruco = aruco_vision_sensor;
aruco.camera_cone_semi_angle = camera_cone_semi_angle;
aruco.camera_max_distance = camera_max_distance;
aruco.camera_min_distance = camera_min_distance;

aruco.noise_perpendicular_position_constant = noise_perpendicular_position_constant;
aruco.noise_parallel_position_constant = noise_parallel_position_constant;
aruco.noise_perpendicular_position_linear = noise_perpendicular_position_linear;
aruco.noise_parallel_position_linear = noise_parallel_position_linear;

aruco.noise_perpendicular_orientation_constant = noise_perpendicular_orientation_constant;
aruco.noise_parallel_orientation_constant = noise_parallel_orientation_constant;
aruco.noise_perpendicular_orientation_linear = noise_perpendicular_orientation_linear;
aruco.noise_parallel_orientation_linear = noise_parallel_orientation_linear;

aruco.camera_orientation = camera_orientation;
aruco.camera_position = camera_position;

aruco.marker_id = known_marker_id;
aruco.marker_orientation = known_marker_orientation;
aruco.marker_position = known_marker_position;


% Unknown aruco sensor simulator
% It is divided from the other only to simplify the coding.
aruco2 = clone(aruco);

aruco2.marker_id = unknown_marker_id;
aruco2.marker_orientation = unknown_marker_orientation;
aruco2.marker_position = unknown_marker_position;


%% Detect marker object
% This object represents the kalman filter that estimates the unknown ArUco
% marker position, orientation and eventually velocity.

% Object creation
detect_aruco = detect_marker;

% Parameters


detect_aruco.camera_pos = camera_position;
detect_aruco.camera_orient = camera_orientation;

detect_aruco.w_q = 0;
detect_aruco.w_p = 1e-2;
detect_aruco.w_v = 0;

detect_aruco.static_markers = true;

detect_aruco.update_private_properties;


%% Barometer Sensor

bar = barometer_sensor;

bar.wn_variance = 0.3^2;
bar.bi_variance = 0*0.005^2;
bar.decay_factor = 0.01;

% The output is quantized
bar.quantization = 0.05;

bar.seed = 'shuffle';


%% Laser Altimeter Sensor

altimeter_position = [0, 0, 0.1];

alt = altimeter_sensor;

alt.wn_variance = 0.05^2;
alt.qwn_variance = 0.1^2 / 7^4;
alt.bi_variance = 0.01^2;
alt.decay_factor = 0.2;

% Minimum and maximum measurable distances
alt.min_alt = 0.1;
alt.max_alt = 5;

% The output is quantized
alt.quantization = 0.001;

% The sensor is supposed to be perfectly perpendicular to the ground and
% with the following displacement relative to body frame
alt.p_0 = altimeter_position;


% Ground height initialization
ground_height = zeros(loopBound, 1);

% The same ground height for all simulations, this may be changed if some
% other configurations are interesting to simulate.
if scenario
    NN = waypoint_toa(11)*imuFs - waypoint_toa(10)*imuFs + 1;
    ground_height(waypoint_toa(10)*imuFs : waypoint_toa(11)*imuFs) = linspace(0, -1.5, NN);
    
    NN = waypoint_toa(12)*imuFs - waypoint_toa(11)*imuFs + 1;
    ground_height(waypoint_toa(11)*imuFs : waypoint_toa(12)*imuFs) = linspace(-1.5, +2.5, NN);
    
    NN = waypoint_toa(13)*imuFs - waypoint_toa(12)*imuFs + 1;
    ground_height(waypoint_toa(12)*imuFs : waypoint_toa(13)*imuFs) = linspace(2.5, 1, NN);
    
    ground_height(waypoint_toa(13)*imuFs : end) = 1;
end


%% Filter setup

% Number of accelerometer and gyroscope measurements which are considered
% together in the Kalman filter. Increase this value to reduce the
% computational cost, at the expense of estimation precision.
decimation_factor = 4;

xxx = 1e-1;

% Filter call and properties initialization
FUSE = my_navigation_filter_adaptive;
% FUSE = my_navigation_filter;

dt = Ts;

% Sample rates
FUSE.sample_rate = imuFs;

% Decimation factor
FUSE.decimation_factor = decimation_factor;

% Some process noises
FUSE.pos_noise = 2e-3;
FUSE.vel_noise = 1e-5;

% Accel, gyro, magn noise
FUSE.acc_noise = 1e+0*xxx;
FUSE.magn_noise = 1e+0*xxx;
FUSE.gyro_noise = 1e-3*xxx;

% IMU sensor frames orientation
FUSE.q_sens_acc = q_sens_acc;
FUSE.q_sens_gyro = q_sens_gyro;
FUSE.q_sens_magn = q_sens_magn;

% Gyroscope drift
FUSE.gyro_drift_noise = 5e-2*xxx;

% Linear acceleration
FUSE.lin_acc_noise = 1e+1 / dt*xxx;
FUSE.lin_acc_decay_factor = 1 / dt;

% Magnetometer disturbance
FUSE.magn_disturbance_noise = 1e-1 / dt*xxx;
FUSE.magn_disturbance_decay_factor = 1 * imuFs;

% GPS
FUSE.GPS_pos_noise = 2e+0;
FUSE.GPS_vel_noise = 1e+0;

% Acceleration drift (bias)
FUSE.acc_drift_noise = 1e-2 / dt*xxx;

% UWB
FUSE.uwb_noise = 1e-1;
FUSE.uwb_beacon_position = known_beacon_pos;
FUSE.uwb_beacon_id = known_beacon_id;

% Computer vision
FUSE.camera_orient = camera_orientation;
FUSE.camera_pos = camera_position;
FUSE.marker_id = known_marker_id;
FUSE.marker_orient = known_marker_orientation;
FUSE.marker_pos = known_marker_position;
FUSE.vision_pos_noise = 1e+1;
FUSE.vision_orient_noise = 1e+10*xxx;

% Barometer
FUSE.bar_noise = 1e-1;

% Altimeter
FUSE.alt_pos = altimeter_position;
FUSE.alt_noise = 1e+1;
FUSE.alt_threshold = 0.3;

FUSE.ground_height_noise = 1e-2 / dt;
FUSE.ground_height_decay_factor = 0e-3 / dt;


FUSE.init_process_noise = diag([5e-0 * ones(3,1)
                                1e-1 * ones(3,1)
                                1e-2 * ones(3,1)
                                1e-1 * ones(3,1)
                                1e+0 * ones(3,1)
                                1e+0 * ones(3,1)
                                1e+0 * ones(3,1)
                                1e+0 * ones(1,1)]);

FUSE.magn_field_NED = magnfield.';

FUSE.reference_location = refloc;

FUSE.private_properties_update;


FUSE(1);    % To initialize the discrete state. TODO: eliminate the need for this.


% Initialize some filter states
FUSE.set_initial_state('q', [1,0,0,0], ...
                       'gyro_offset', [0,0,0], ...
                       'pos', trajPos(1,:).', ...
                       'vel', trajVel(1,:).', ...
                       'acc_bias', [0,0,0]);


%% Plots setup

% Colors
default_colors = colororder;

used_colors = [0 0 1;
               1 .647 0;
               1 1 0;
               1 0 1;
               0 1 0;
               0 1 1;
               1 0 0];

% Top view plot
fig1 = figure('Name', 'Bottom View', ...
              'NumberTitle', 'off', ...
              'Units', 'normalized', ...
              'OuterPosition', [0, 0, 0.35, 0.5]);

% This is used to avoid calling figure(1) when selecting where to plot.
% This way if the plot is continuously updated, it does not pop up over
% everything.
% Do: plot(hax1, ...)
hax1 = axes;

hold on
axis equal
grid on

title('Top View')
xlabel('x-North [m]')
ylabel('y-East [m]')

% Ideal trajectory plot
plot(trajPos(:,1), trajPos(:,2), 'b:', 'LineWidth', 1)

% Environment plot
% This matrix is used to plot the walls of the house :O
house1 = [ 60,     5;
           60,    20;
           90,    20;
           90,    17.5;
           90,    20;
          120,    20;
          120,     5;
          115,     5;
          120,     5;
          120,   -10;
          105,   -10;
          120,   -10;
          120,   -20;
           90,   -20;
           90,  12.5;
           90,     5;
          105,     5;
           90,     5;
           90,   -20;
           60,   -20;
           60,   -5];
       
plot(house1(:,1), house1(:,2), 'black', 'LineWidth', 3)


% Aruco markers plot
aruco_marker_dim = 250;     % dimension of the aruco marker in the plot

if not(isempty(known_marker_id))
    scatter(known_marker_position(:,1), known_marker_position(:,2), aruco_marker_dim, used_colors(1,:), 's', 'LineWidth', 2)   	% dark blue
end

if not(isempty(unknown_marker_id))
    scatter(hax1, unknown_marker_position(:,1), unknown_marker_position(:,2), aruco_marker_dim, used_colors(7,:), 's', 'LineWidth', 2)
    
    % Aruco markers position estimate container.
    aruco_estimate_plot(length(unknown_marker_id)) = scatter(hax1, [], []);
end


% UWB anchors plot
uwb_beacon_dim = 100;

if not(isempty(known_beacon_id))
    scatter(known_beacon_pos(:,1), known_beacon_pos(:,2), uwb_beacon_dim, used_colors(1,:), 'd', 'LineWidth', 2)                 % dark blue
end

if not(isempty(unknown_beacon_id))
    scatter(unknown_beacon_pos(:,1), unknown_beacon_pos(:,2), uwb_beacon_dim, used_colors(7,:), 'd', 'LineWidth', 2)            	% red
end

% UWB beacons position estimate container.
if not(isempty(unknown_beacon_id))
    beacon_estimate_plot(length(unknown_beacon_id)) = scatter(hax1, [], []);
end


% Estimated trajectory plot
est_traj = animatedline(hax1, 'Color', 'k', 'LineWidth', 1);                                                            % black

real_pos = scatter(hax1, NaN, NaN, 'b', '*');
est_pos = scatter(hax1, NaN, NaN, 'k', 'o');


% Interface plot
fig2 = figure('Name', 'Interface', ...
              'NumberTitle', 'off', ...
              'Units', 'normalized', ...
              'OuterPosition', [0.35, 0, 0.15, 0.4]);

% Interface setup
hold off
axis([0 1 0 1])
daspect([1 1 1])
set(gca,'visible','off')
set(gca,'xtick',[])
set(gca,'ytick',[])


% Initialize the interface
% Interface rectangles dimensions
ann_gps_rect_dim = [.05 .85 .9 .1];
ann_uwb_rect_dim = [.05 .70 .9 .1];
ann_vis_rect_dim = [.05 .55 .9 .1];
ann_alt_rect_dim = [.05 .40 .9 .1];


% GPS signal flag
ann_gps_col = annotation(fig2, 'rectangle', ann_gps_rect_dim, 'FaceColor','red','FaceAlpha',.2);
ann_gps_text = annotation(fig2, 'textbox', ann_gps_rect_dim, 'String', 'GPS signseal absent', 'FitBoxToText','off');

% UWB signal flag
ann_uwb_col = annotation(fig2, 'rectangle', ann_uwb_rect_dim, 'FaceColor','red','FaceAlpha',.2);
ann_uwb_text = annotation(fig2, 'textbox', ann_uwb_rect_dim, 'String', 'UWB signal absent', 'FitBoxToText','off');

% Vision signal flag
ann_vis_col = annotation(fig2, 'rectangle', ann_vis_rect_dim, 'FaceColor','red','FaceAlpha',.2);
ann_vis_text = annotation(fig2, 'textbox', ann_vis_rect_dim, 'String', 'No ArUco marker detected', 'FitBoxToText','off');

% Barometer signal flag
ann_alt_col = annotation(fig2, 'rectangle', ann_alt_rect_dim, 'FaceColor','red','FaceAlpha',.2);
ann_alt_text = annotation(fig2, 'textbox', ann_alt_rect_dim, 'String', 'Altimeter signal absent', 'FitBoxToText','off');


% States animatedline plot

% fig3 = figure('Name', 'States', ...
%               'NumberTitle', 'off', ...
%               'Units', 'normalized', ...
%               'OuterPosition', [0.5, 0, 0.5, 1]);
% 
% hax3 = subplot(3,2,1);
% title('Gyroscope bias')
% 
% hax4 = subplot(3,2,2);
% title('Accelerometer bias')
% 
% hax5 = subplot(3,2,3);
% title('Linear acceleration')
% 
% hax6 = subplot(3,2,4);
% title('Magnetometer disturbance')
% 
% hax7 = subplot(3,2,5);
% title('Velocity')
% 
% hax8 = subplot(3,2,6);
% title('Ground height')
% 
% % Estimated values
% 
% a_g_b_1 = animatedline(hax3, 'Color', default_colors(1,:));
% a_g_b_2 = animatedline(hax3, 'Color', default_colors(2,:));
% a_g_b_3 = animatedline(hax3, 'Color', default_colors(3,:));
% 
% a_a_b_1 = animatedline(hax4, 'Color', default_colors(1,:));
% a_a_b_2 = animatedline(hax4, 'Color', default_colors(2,:));
% a_a_b_3 = animatedline(hax4, 'Color', default_colors(3,:));
% 
% a_l_a_1 = animatedline(hax5, 'Color', default_colors(1,:));
% a_l_a_2 = animatedline(hax5, 'Color', default_colors(2,:));
% a_l_a_3 = animatedline(hax5, 'Color', default_colors(3,:));
% 
% a_m_d_1 = animatedline(hax6, 'Color', default_colors(1,:));
% a_m_d_2 = animatedline(hax6, 'Color', default_colors(2,:));
% a_m_d_3 = animatedline(hax6, 'Color', default_colors(3,:));
% 
% a_vel_1 = animatedline(hax7, 'Color', default_colors(1,:));
% a_vel_2 = animatedline(hax7, 'Color', default_colors(2,:));
% a_vel_3 = animatedline(hax7, 'Color', default_colors(3,:));
% 
% a_g_h   = animatedline(hax8, 'Color', default_colors(1,:));
% 
% 
% % Real values
% 
% r_a_g_b_1 = animatedline(hax3, 'Color', default_colors(1,:), 'LineStyle', ':');
% r_a_g_b_2 = animatedline(hax3, 'Color', default_colors(2,:), 'LineStyle', ':');
% r_a_g_b_3 = animatedline(hax3, 'Color', default_colors(3,:), 'LineStyle', ':');
% 
% r_a_a_b_1 = animatedline(hax4, 'Color', default_colors(1,:), 'LineStyle', ':');
% r_a_a_b_2 = animatedline(hax4, 'Color', default_colors(2,:), 'LineStyle', ':');
% r_a_a_b_3 = animatedline(hax4, 'Color', default_colors(3,:), 'LineStyle', ':');
% 
% r_a_l_a_1 = animatedline(hax5, 'Color', default_colors(1,:), 'LineStyle', ':');
% r_a_l_a_2 = animatedline(hax5, 'Color', default_colors(2,:), 'LineStyle', ':');
% r_a_l_a_3 = animatedline(hax5, 'Color', default_colors(3,:), 'LineStyle', ':');
% 
% r_a_m_d_1 = animatedline(hax6, 'Color', default_colors(1,:), 'LineStyle', ':');
% r_a_m_d_2 = animatedline(hax6, 'Color', default_colors(2,:), 'LineStyle', ':');
% r_a_m_d_3 = animatedline(hax6, 'Color', default_colors(3,:), 'LineStyle', ':');
% 
% r_a_vel_1 = animatedline(hax7, 'Color', default_colors(1,:), 'LineStyle', ':');
% r_a_vel_2 = animatedline(hax7, 'Color', default_colors(2,:), 'LineStyle', ':');
% r_a_vel_3 = animatedline(hax7, 'Color', default_colors(3,:), 'LineStyle', ':');
% 
% r_a_g_h   = animatedline(hax8, 'Color', default_colors(1,:), 'LineStyle', ':');

[anim_est_states, anim_real_states, fig3] = anim_states_create(default_colors);


%% Initialize Scopes

useErrScope = true;     % Turn on the streaming error plot

if useErrScope
    errscope = HelperScrollingPlotter(...
        'NumInputs', 4, ...
        'TimeSpan', 10, ...
        'SampleRate', imuFs, ...
        'YLabel', {'degrees', ...
        'meters', ...
        'meters', ...
        'meters'}, ...
        'Title', {'Quaternion Distance', ...
        'Position X Error', ...
        'Position Y Error', ...
        'Position Z Error'}, ...
        'YLimits', ...
        [ 0, 10
          0,  4
          0,  4
          0,  4]);
end

f = gcf;    % current figure handle


%% Simulation Loop

% Log data for final metric computation.
pqorient = quaternion.zeros(loopBound,1);
pqpos = zeros(loopBound,3);
pqvel = zeros(loopBound,3);
pqacc = zeros(loopBound,3);


% Parameters used to update the figure colors.

% Flags, true if the corresponding sensor measurement is received or not
% accessible
% Used to update the interface
is_gps = false;
is_uwb = false;
is_vis = false;
is_alt = false;
is_vis_detect = false;

% Number of uwb anchors in range
uwb_known_beacon_number = 0;

% Memory of the uwb beacons indexes in range, used to change their color in
% the plot.
unobstr_known_beacon_id_old = [];
uwb_beacon_id_old = [];

% Memory of the uwb unknown beacons indexes in range, used to change their
% color in the plot.
unobstr_unknown_beacon_id_old = [];
uwb_beacon_detect_id_old = [];

% Memory of the unknown  ArUco markers indexes in range, used to change
% their color in the plot.
vis_id_old = [];
vis_detect_id_old = [];


% Saved sensors error for logging and debugging
% saved_z_acc = zeros(loopBound, 3);
% saved_z_mag = zeros(loopBound, 3);


% The starting orientation is computed using only the accelerometer and the
% magnetometer measurements.
[accel, gyro, mag] = imu(trajAcc(1,:), ...
                         trajAngVel(1, :), ...
                         trajOrient(1));


% Initialize the filter orientation using the accelerometer and the
% magnetometer measurement (electronic compass)
pqorient(1) = ecompass(accel, mag);
FUSE.set_initial_state('q', compact(pqorient(1)));


% The position and the velocity is supposed to be initialized using the
% GPS.
[lla, gpsvel] = gps( trajPos(1,:), trajVel(1,:) );

lat0 = refloc(1); lon0 = refloc(2); h0 = refloc(3);
[xNorth, yEast, zDown] = geodetic2ned(lla(1), lla(2), lla(3), lat0, lon0, h0, wgs84Ellipsoid);

pqpos(1,:) = [xNorth, yEast, zDown];
FUSE.set_initial_state('pos',pqpos(1,:).');

pqvel(1,:) = gpsvel(1,:);
FUSE.set_initial_state('vel',pqvel(1,:).');


% Loop variable
fcnt = 2;


alt_meas = NaN(loopBound,1);
alt_imu =  NaN(loopBound,1);


% Some useful imformations printed on the screen
fprintf('Simulation started...\n \n')
fprintf('Total simulated time (seconds): %d \n \n', round(loopBound/imuFs))
diesci = 5;
fprintf('Current simulated time (seconds): \n \n')


% Initialize the variables used for some plots at the end of the simulation
ecompass_err = NaN(loopBound, 1);
estim_orient_err = NaN(loopBound, 1);

meas_GPS_NED_err = NaN(ceil(loopBound / fix(imuFs/gpsFs)),3);
meas_GPS_NED_time = linspace(0, ceil(loopBound / fix(imuFs/gpsFs)) * fix(imuFs/gpsFs)/imuFs, ceil(loopBound / fix(imuFs/gpsFs)));

meas_UWB_dist_err = NaN(ceil(loopBound / fix(imuFs/uwbFs)), size(known_beacon_pos,1));
estim_UWB_dist_err = NaN(ceil(loopBound / fix(imuFs/uwbFs)), size(known_beacon_pos,1));
meas_UWB_dist_time = linspace(0, ceil(loopBound / fix(imuFs/uwbFs)) * fix(imuFs/uwbFs)/imuFs, ceil(loopBound / fix(imuFs/uwbFs)));

meas_bar = NaN(ceil(loopBound / fix(imuFs/barFs)),1);
meas_bar_time = linspace(0, ceil(loopBound / fix(imuFs/barFs)) * fix(imuFs/barFs)/imuFs, ceil(loopBound / fix(imuFs/barFs)));

meas_alt = NaN(ceil(loopBound / fix(imuFs/altFs)),1);
meas_alt_time = linspace(0, ceil(loopBound / fix(imuFs/altFs)) * fix(imuFs/altFs)/imuFs, ceil(loopBound / fix(imuFs/altFs)));


while(fcnt <= loopBound)
    if mod(fcnt, imuFs*diesci) == 0
        fprintf('%d \n', fcnt/imuFs)
    end
    
    % Change the filter properties after some time is passed. These
    % parameters are for when the initialization phase is concluded.
    if fcnt == imuFs * init_end
        FUSE.gyro_noise = 1e-5*xxx;
        
        FUSE.gyro_drift_noise = 1e-5*xxx;
        
        FUSE.acc_drift_noise = 1e-6*xxx;
        
        FUSE.lin_acc_noise = 1e+0 / dt*xxx;
        FUSE.lin_acc_decay_factor = 0.001 / dt;
        
        FUSE.magn_noise = 1e+0*xxx;
        
        FUSE.magn_disturbance_noise = 1e+1 / dt*xxx;
        FUSE.magn_disturbance_decay_factor = 0.025 * imuFs;
        
        FUSE.GPS_pos_noise = 1e+2;
        
        FUSE.private_properties_update;
    end
    
    % Simulate the IMU data from the current pose.
    % [position, orientation, velocity, acceleration, angularVelocity] = trajectory()
    [accel, gyro, mag] = imu(trajAcc(fcnt,:), ...
                             trajAngVel(fcnt, :), ...
                             trajOrient(fcnt));
	
	% Sensor output
    % accel = - f_b;
    % gyro = omega_ib_b;
    
    ecompass_err(fcnt) = rad2deg(dist(ecompass(accel, mag), trajOrient(fcnt) ));
	
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
    
    % The correction using the accelerometer data is done every
    % decimation_factor sample times.
    if mod(fcnt, fix(decimation_factor)) == 0
        % Accelerometer correction
       z_acc = FUSE.fuse_accel;
    end
    
    
    % Magnetometer correction
    if mod(fcnt, fix(imuFs/magFs)) == 0
        if sum(real_mag_dist ~= magnetometer_disturbance(fcnt, :))
            real_mag_dist = magnetometer_disturbance(fcnt, :);
            imu.Magnetometer.ConstantBias = magnetometer_disturbance(fcnt, :);
        end
        
        % Fuse the magnetomer measurement
        z_mag = FUSE.fuse_magn(mag);
    end
    
    
    % GPS correction step (at the GPS sample rate)
    if mod(fcnt, fix(imuFs/gpsFs)) == 0
        % Update the interface plot, only if its state is changed
        if is_gps ~= yes_gps(trajPos(fcnt,:))
            is_gps = yes_gps(trajPos(fcnt,:));
            
            if is_gps
                % GPS available
                figure(2)
                delete(ann_gps_col);
                delete(ann_gps_text);
                
                ann_gps_col = annotation(fig2, 'rectangle', ann_gps_rect_dim, 'FaceColor','green','FaceAlpha',.2);
                ann_gps_text = annotation(fig2, 'textbox', ann_gps_rect_dim, 'String', 'GPS signal received', 'FitBoxToText','off');
            else
                % GPS unavailable
                figure(2)
                delete(ann_gps_col);
                delete(ann_gps_text);
                
                ann_gps_col = annotation(fig2, 'rectangle', ann_gps_rect_dim, 'FaceColor','red','FaceAlpha',.2);
                ann_gps_text = annotation(fig2, 'textbox', ann_gps_rect_dim, 'String', 'GPS signal absent', 'FitBoxToText','off');
            end
        end
        
        % Fuse the gps signal, only if available
        if is_gps
            % Simulate the GPS output based on the current pose.
            [lla, gpsvel] = gps( trajPos(fcnt,:), trajVel(fcnt,:) );
            

            if scenario == 4
                if rand > 0.9
                    lla = lla + 2 * 100 * 180 / (6300 * 1000 * pi) * (rand(1,3)-0.5);
                    gpsvel = gpsvel + 2 * 10 * (rand(1,3)-0.5);
                    
                    % Compute the NED coordinates from the geodetic coordinates
                    [xNorth, yEast, zDown] = geodetic2ned(lla(1), lla(2), lla(3), refloc(1), refloc(2), refloc(3), wgs84Ellipsoid);

                    scatter(hax1, xNorth, yEast, 5, default_colors(7,:))
                else
                    % Compute the NED coordinates from the geodetic coordinates
                    [xNorth, yEast, zDown] = geodetic2ned(lla(1), lla(2), lla(3), refloc(1), refloc(2), refloc(3), wgs84Ellipsoid);

                    scatter(hax1, xNorth, yEast, 5, default_colors(5,:))
                end
            end
            
            [xNorthSaved, yEastSaved, zDownSaved] = geodetic2ned(lla(1), lla(2), lla(3), refloc(1), refloc(2), refloc(3), wgs84Ellipsoid);
            
            meas_GPS_NED_err( fcnt / fix(imuFs/gpsFs), : ) = [xNorthSaved, yEastSaved, zDownSaved] - trajPos(fcnt,:);
    
            % Correct the filter states based on the GPS data measurements.
            z_gps = FUSE.fuse_GPS(lla, gpsvel);
        end
    end

    if fcnt == imuFs * init_end
        if scenario ~= 3
            FUSE.magn_disturbance_decay_factor = 1 * imuFs;
        end
    end
    
    
    % UWB correction step
    if mod(fcnt, fix(imuFs/uwbFs)) == 0 && not(isempty(known_beacon_id))
        % Returns a flag if in the current "room" there are beacons which
        % are not obstructed by walls, and their ids.
        [y1, unobstr_known_beacon_id] = yes_uwb(trajPos(fcnt,:));
        
        % Fuse the UWB signal, if available
        if y1
            %Simulate sensor
            [uwb_meas_dist, uwb_beacon_id, uwb_index] = uwb_known(trajPos(fcnt,:), unobstr_known_beacon_id);
            
            [uwb_meas_dist_ideal] = uwb_known_ideal(trajPos(fcnt,:), uwb_beacon_id);
            
            
            if scenario == 5
                for ij = 1:length(uwb_meas_dist)
                    if rand > 0.9
                        uwb_NLOS_relative_error = 100 / 100;
                        
                        uwb_meas_dist(ij) = uwb_meas_dist(ij) * (1 + 2 * uwb_NLOS_relative_error * (rand() - 1/2 ));
                    end
                end
            end
            
            meas_UWB_dist_err( fcnt / fix(imuFs/uwbFs), uwb_index ) = uwb_meas_dist.' - uwb_meas_dist_ideal.';
            
            if isempty(uwb_beacon_id) == false
                FUSE.fuse_uwb(uwb_meas_dist, uwb_beacon_id);
                
                [pqorient(fcnt), pqpos(fcnt, :)] = FUSE.get_pose();
                
                [uwb_estim_dist] = uwb_known_ideal(pqpos(fcnt,:), uwb_beacon_id);
            
                estim_UWB_dist_err( fcnt / fix(imuFs/uwbFs), uwb_index ) = uwb_estim_dist.' - uwb_meas_dist_ideal.';
            end
        else
            uwb_beacon_id = [];
            uwb_index = [];
        end
        
        % Update the interface plot, only if its state is changed
        if is_uwb ~= ( yes_uwb(trajPos(fcnt,:)) & not(isempty(uwb_beacon_id)) ) ... % the uwb is active AND at least an anchor is in range
                || uwb_known_beacon_number ~= length(uwb_beacon_id)
            is_uwb = ( yes_uwb(trajPos(fcnt,:)) & not(isempty(uwb_beacon_id)) );
            uwb_known_beacon_number = length(uwb_beacon_id);
            
            if is_uwb
                % UWB available
                figure(2)
                delete(ann_uwb_col);
                delete(ann_uwb_text);
                
                if length(uwb_beacon_id) >= 4
                    ann_uwb_col = annotation(fig2, 'rectangle', ann_uwb_rect_dim, 'FaceColor','green','FaceAlpha',.2);
                    ann_uwb_text = annotation(fig2, 'textbox', ann_uwb_rect_dim, 'String', 'UWB signal received', 'FitBoxToText','off');
                else
                    ann_uwb_col = annotation(fig2, 'rectangle', ann_uwb_rect_dim, 'FaceColor','yellow','FaceAlpha',.2);
                    ann_uwb_text = annotation(fig2, 'textbox', ann_uwb_rect_dim, 'String', 'UWB signal received', 'FitBoxToText','off');
                end
            else
                % UWB unavailable
                figure(2)
                delete(ann_uwb_col);
                delete(ann_uwb_text);
                
                ann_uwb_col = annotation(fig2, 'rectangle', ann_uwb_rect_dim, 'FaceColor','red','FaceAlpha',.2);
                ann_uwb_text = annotation(fig2, 'textbox', ann_uwb_rect_dim, 'String', 'UWB signal absent', 'FitBoxToText','off');
            end
        end
        
        
        % Change the color of the UWB anchors depending on their state.
        % Legend:   dark blue:      known, obstructed
        %           light blue:     known, not obstructed
        %           green:          known, received
        %           red:            unknown, not received
        %           purple:         unknown, received
        %           orange:         unknown estimate
        if length(unobstr_known_beacon_id) ~= length(unobstr_known_beacon_id_old) || length(uwb_beacon_id) ~= length(uwb_beacon_id_old)
            unobstr_known_beacon_id_old = unobstr_known_beacon_id;
            uwb_beacon_id_old = uwb_beacon_id;
            
            % Reset the color
            scatter(hax1, known_beacon_pos(:,1), known_beacon_pos(:,2), uwb_beacon_dim, used_colors(1,:), 'd', 'LineWidth', 2)
            
            % Plot the not obscured ones
            accessible_uwb_plot = scatter(hax1, known_beacon_pos(unobstr_known_beacon_id_old,1), known_beacon_pos(unobstr_known_beacon_id_old,2), uwb_beacon_dim, used_colors(6,:), 'd', 'LineWidth', 2);
            
            % Plot the received ones
            visible_uwb_plot = scatter(hax1, known_beacon_pos(uwb_index,1), known_beacon_pos(uwb_index,2), uwb_beacon_dim, used_colors(5,:), 'd', 'LineWidth', 2);
        elseif sum(unobstr_known_beacon_id ~= unobstr_known_beacon_id_old) || sum(uwb_beacon_id ~= uwb_beacon_id_old)
            unobstr_known_beacon_id_old = unobstr_known_beacon_id;
            uwb_beacon_id_old = uwb_beacon_id;
            
            % Reset the color
            scatter(hax1, known_beacon_pos(:,1), known_beacon_pos(:,2), uwb_beacon_dim, used_colors(1,:), 'd', 'LineWidth', 2)
            
            % Plot the not obscured ones
            accessible_uwb_plot = scatter(hax1, known_beacon_pos(unobstr_known_beacon_id_old,1), known_beacon_pos(unobstr_known_beacon_id_old,2), uwb_beacon_dim, used_colors(6,:), 'd', 'LineWidth', 2);
            
            % Plot the received ones
            visible_uwb_plot = scatter(hax1, known_beacon_pos(uwb_index,1), known_beacon_pos(uwb_index,2), uwb_beacon_dim, used_colors(5,:), 'd', 'LineWidth', 2);
        end
    end
    
    
    % UWB beacon detect step. This is done at a lower frequency compared
    % to the uwb frequency.
    if mod(fcnt, 1 * fix(imuFs/uwbFs)) == 0 && not(isempty(unknown_beacon_id))
        % Returns a flag if in the current "room" there are beacons which
        % are not obstructed by walls, and their ids.
        if scenario == 1
            [yu, unobstr_unknown_beacon_id] = yes_uwb_unknown_1(trajPos(fcnt,:));
        else
            [yu, unobstr_unknown_beacon_id] = yes_uwb_unknown_1(trajPos(fcnt,:));
        end
        
        % Detect the UWB anchor position, if the signal is available.
        if yu
            %Simulate unknown sensor
            [uwb_unkn_meas_dist, uwb_unkn_beacon_id, uwb_unkn_index] = uwb_unknown(trajPos(fcnt,:), unobstr_unknown_beacon_id);
            
            % Detect the beacon only if there is a measure available
            if isempty(uwb_unkn_beacon_id) == false
                [uwb_anchor, z_uwb] = FUSE.detect_unknown_uwb(uwb_unkn_meas_dist, uwb_unkn_beacon_id);
            end
            
            % Draw the estimate.
            if mod(fcnt, 5 * fix(imuFs/uwbFs)) == 0
                for ii = 1:length(uwb_unkn_index)
                    delete(beacon_estimate_plot(uwb_unkn_index(ii)));
                
                    beacon_estimate_plot(uwb_unkn_index(ii)) = scatter(hax1, uwb_anchor(ii, 1), uwb_anchor(ii, 2), uwb_beacon_dim, used_colors(2,:), 'd', 'LineWidth', 2);
                end
            end
        else
            uwb_unkn_beacon_id = [];
            uwb_unkn_index = [];
        end
    end
    
    % UWB beacon detect plot step. This is done at an even lower frequency
    % compared to the uwb frequency.
    if mod(fcnt, 5 * fix(imuFs/uwbFs)) == 0 && not(isempty(unknown_beacon_id))
        
    % Change the color of the UWB anchors depending on their state.
        % Legend:   dark blue:      known, obstructed
        %           light blue:     known, not obstructed
        %           green:          known, received
        %           red:            unknown, not received
        %           purple:         unknown, received
        %           orange:         unknown estimate
        if length(unobstr_unknown_beacon_id) ~= length(unobstr_unknown_beacon_id_old) || length(uwb_unkn_beacon_id) ~= length(uwb_beacon_detect_id_old)
            unobstr_unknown_beacon_id_old = unobstr_unknown_beacon_id;
            uwb_beacon_detect_id_old = uwb_unkn_beacon_id;
            
            % Reset the color
            scatter(hax1, unknown_beacon_pos(:,1), unknown_beacon_pos(:,2), uwb_beacon_dim, used_colors(7,:), 'd', 'LineWidth', 2)
            
            % Plot the received ones
            visible_uwb_plot = scatter(hax1, unknown_beacon_pos(uwb_unkn_index,1), unknown_beacon_pos(uwb_unkn_index,2), uwb_beacon_dim, used_colors(4,:), 'd', 'LineWidth', 2);
            
        elseif sum(unobstr_unknown_beacon_id ~= unobstr_unknown_beacon_id_old) || sum(uwb_unkn_beacon_id ~= uwb_beacon_detect_id_old)
            unobstr_unknown_beacon_id_old = unobstr_unknown_beacon_id;
            uwb_beacon_detect_id_old = uwb_unkn_beacon_id;
            
            % Reset the color
            scatter(hax1, unknown_beacon_pos(:,1), unknown_beacon_pos(:,2), uwb_beacon_dim, used_colors(7,:), 'd', 'LineWidth', 2)
            
            % Plot the received ones
            visible_uuwb_plot = scatter(hax1, unknown_beacon_pos(uwb_unkn_index,1), unknown_beacon_pos(uwb_unkn_index,2), uwb_beacon_dim, used_colors(4,:), 'd', 'LineWidth', 2);
        end
        
    end


    % Vision correction step
    if mod(fcnt, fix(imuFs/visFs)) == 0 && not(isempty(known_marker_id))
        % Simulate sensor
        [Tmc, vis_id, vis_index] = aruco(trajPos(fcnt,:), trajOrient(fcnt));
        
        % Predict P_2, only if it has not been already done
        if isempty(vis_id) == false
            [z_vis, n_meas] = FUSE.fuse_vis(Tmc, vis_id);
        end
        
        % Update the interface plot, only if its state is changed
        if is_vis ~= not(isempty(vis_id))
            is_vis = not(isempty(vis_id));
            
            if is_vis
                % Marker detected
                figure(2)
                delete(ann_vis_col);
                delete(ann_vis_text);
                
                ann_vis_col = annotation(fig2, 'rectangle', ann_vis_rect_dim, 'FaceColor','green','FaceAlpha',.2);
                ann_vis_text = annotation(fig2, 'textbox', ann_vis_rect_dim, 'String', 'ArUco marker detected', 'FitBoxToText','off');
            else
                % No marker detected
                figure(2)
                delete(ann_vis_col);
                delete(ann_vis_text);
                
                ann_vis_col = annotation(fig2, 'rectangle', ann_vis_rect_dim, 'FaceColor','red','FaceAlpha',.2);
                ann_vis_text = annotation(fig2, 'textbox', ann_vis_rect_dim, 'String', 'No ArUco marker detected', 'FitBoxToText','off');
            end
        end
        
        % Change the color of the vis markers depending on their state.
        % Legend:   dark blue:      known, not observed
        %           green:          known, observed
        %           red:            unknown, not observed
        %           purple:         unknown, received
        %           orange:         unknown estimate
        if length(vis_id) ~= length(vis_id_old)
            vis_id_old = vis_id;
            
            % Reset the color
            scatter(hax1, known_marker_position(:,1), known_marker_position(:,2), aruco_marker_dim, used_colors(1,:), 's', 'LineWidth', 2)
            
            % Plot the received ones
            scatter(hax1, known_marker_position(vis_index,1), known_marker_position(vis_index,2), aruco_marker_dim, used_colors(5,:), 's', 'LineWidth', 2)
        elseif sum(vis_id ~= vis_id_old)
            vis_id_old = vis_id;
            
            % Reset the color
            scatter(hax1, known_marker_position(:,1), known_marker_position(:,2), aruco_marker_dim, used_colors(1,:), 's', 'LineWidth', 2)
            
            % Plot the received ones
            scatter(hax1, known_marker_position(vis_index,1), known_marker_position(vis_index,2), aruco_marker_dim, used_colors(5,:), 's', 'LineWidth', 2)
        end
    end


    % Aruco marker detection step
    if mod(fcnt, fix(imuFs/visFs)) == 0 && not(isempty(unknown_marker_id))
        % Simulate sensor (only markers with unknown position)
        [Tmc, vis_detect_id, vis_index] = aruco2(trajPos(fcnt,:), trajOrient(fcnt));
        
        if not(isempty(vis_detect_id))
            [pqorient(fcnt), pqpos(fcnt, :)] = FUSE.get_pose();
            [marker_q_est, marker_p_est] = detect_aruco(Tmc, vis_detect_id, pqpos(fcnt, :), pqorient(fcnt), fcnt);

            for ii = 1:length(vis_index)
                delete(aruco_estimate_plot(vis_index(ii)));
                
                aruco_estimate_plot(vis_index(ii)) = scatter(hax1, marker_p_est((ii), 1), marker_p_est((ii), 2), aruco_marker_dim, used_colors(2,:), 's', 'LineWidth', 2);
            end
        end
        
        % Change the color of the vis markers depending on their state.
        % Legend:   dark blue:      known, not observed
        %           green:          known, observed
        %           red:            unknown, not observed
        %           purple:         unknown, received
        %           orange:         unknown estimate
        if length(vis_detect_id) ~= length(vis_detect_id_old)
            vis_detect_id_old = vis_detect_id;
            
            % Reset the color
            scatter(hax1, unknown_marker_position(:,1), unknown_marker_position(:,2), aruco_marker_dim, used_colors(7,:), 's', 'LineWidth', 2)
            
            % Plot the received ones
            scatter(hax1, unknown_marker_position(vis_index,1), unknown_marker_position(vis_index,2), aruco_marker_dim, used_colors(4,:), 's', 'LineWidth', 2)
        elseif sum(vis_detect_id ~= vis_detect_id_old)
            vis_detect_id_old = vis_detect_id;
            
            % Reset the color
            scatter(hax1, unknown_marker_position(:,1), unknown_marker_position(:,2), aruco_marker_dim, used_colors(7,:), 's', 'LineWidth', 2)
            
            % Plot the received ones
            scatter(hax1, unknown_marker_position(vis_index,1), unknown_marker_position(vis_index,2), aruco_marker_dim, used_colors(4,:), 's', 'LineWidth', 2)
        end
    end
    
    
    % Barometer correction step
    if mod(fcnt, fix(imuFs/barFs)) == 0
        % Predict P_2, only if it has not been already done
        % Simulate sensor
        bar_h_meas = bar(trajPos(fcnt,:));
        
        meas_bar( fcnt / fix(imuFs/barFs), : ) = bar_h_meas;
        
        % Fuse the sensor measurement
        FUSE.fuse_bar(bar_h_meas);
    end
    
    
    % Altimeter correction step
    if mod(fcnt, fix(imuFs/altFs)) == 0
        
        % Simulate sensor
        alt_h_meas = alt(trajPos(fcnt,:), trajOrient(fcnt,:), ground_height(fcnt));
        
        meas_alt( fcnt / fix(imuFs/altFs), : ) = alt_h_meas;
        
        
        
        % Predict P_2, only if it has not been already done
        if not(isnan(alt_h_meas))
            z_alt = FUSE.fuse_alt(alt_h_meas);
        end
        
        
        % Update the interface plot, if necessary
        if is_alt ~= not(isnan(alt_h_meas))
            is_alt = not(isnan(alt_h_meas));
            
            if is_alt
                % Ground detected
                figure(2)
                delete(ann_alt_col);
                delete(ann_alt_text);
                
                ann_alt_col = annotation(fig2, 'rectangle', ann_alt_rect_dim, 'FaceColor','green','FaceAlpha',.2);
                ann_alt_text = annotation(fig2, 'textbox', ann_alt_rect_dim, 'String', 'Ground detected', 'FitBoxToText','off');
            else
                % No ground detected
                figure(2)
                delete(ann_alt_col);
                delete(ann_alt_text);
                
                ann_alt_col = annotation(fig2, 'rectangle', ann_alt_rect_dim, 'FaceColor','red','FaceAlpha',.2);
                ann_alt_text = annotation(fig2, 'textbox', ann_alt_rect_dim, 'String', 'No ground detected', 'FitBoxToText','off');
            end
        end
    end


    [pqorient(fcnt), pqpos(fcnt, :)] = FUSE.get_pose();
    
    
    % Update the top view
    addpoints(est_traj,pqpos(fcnt, 1),pqpos(fcnt, 2));
    
    drawnow limitrate   % limits the maximum update frequency
    
    time_x = fcnt / imuFs;
    
    % Estimated
%     addpoints(a_g_b_1,time_x, FUSE.getDiscreteState.state( 5));
%     addpoints(a_g_b_2,time_x, FUSE.getDiscreteState.state( 6));
%     addpoints(a_g_b_3,time_x, FUSE.getDiscreteState.state( 7));
% 
%     addpoints(a_a_b_1,time_x, FUSE.getDiscreteState.state( 8));
%     addpoints(a_a_b_2,time_x, FUSE.getDiscreteState.state( 9));
%     addpoints(a_a_b_3,time_x, FUSE.getDiscreteState.state(10));
% 
%     addpoints(a_l_a_1,time_x, FUSE.getDiscreteState.state(11));
%     addpoints(a_l_a_2,time_x, FUSE.getDiscreteState.state(12));
%     addpoints(a_l_a_3,time_x, FUSE.getDiscreteState.state(13));
% 
%     addpoints(a_m_d_1,time_x, FUSE.getDiscreteState.state(14));
%     addpoints(a_m_d_2,time_x, FUSE.getDiscreteState.state(15));
%     addpoints(a_m_d_3,time_x, FUSE.getDiscreteState.state(16));
% 
%     addpoints(a_vel_1,time_x, FUSE.getDiscreteState.state(20));
%     addpoints(a_vel_2,time_x, FUSE.getDiscreteState.state(21));
%     addpoints(a_vel_3,time_x, FUSE.getDiscreteState.state(22));
% 
%     addpoints(  a_g_h,time_x, FUSE.getDiscreteState.state(23));
%     
%     % Real
%     addpoints(r_a_g_b_1,time_x, real_gyro_bias(1));
%     addpoints(r_a_g_b_2,time_x, real_gyro_bias(2));
%     addpoints(r_a_g_b_3,time_x, real_gyro_bias(3));
% 
%     addpoints(r_a_a_b_1,time_x, real_acc_bias(1));
%     addpoints(r_a_a_b_2,time_x, real_acc_bias(2));
%     addpoints(r_a_a_b_3,time_x, real_acc_bias(3));
% 
%     addpoints(r_a_l_a_1,time_x, trajAcc(fcnt, 1));
%     addpoints(r_a_l_a_2,time_x, trajAcc(fcnt, 2));
%     addpoints(r_a_l_a_3,time_x, trajAcc(fcnt, 3));
% 
%     addpoints(r_a_m_d_1,time_x, real_mag_dist(1));
%     addpoints(r_a_m_d_2,time_x, real_mag_dist(2));
%     addpoints(r_a_m_d_3,time_x, real_mag_dist(3));
% 
%     addpoints(r_a_vel_1,time_x, trajVel(fcnt, 1));
%     addpoints(r_a_vel_2,time_x, trajVel(fcnt, 2));
%     addpoints(r_a_vel_3,time_x, trajVel(fcnt, 3));
% 
%     addpoints(  r_a_g_h,time_x, 0);
%     
%     drawnow limitrate   % limits the maximum update frequency

    anim_states_add(anim_est_states, anim_real_states, time_x, FUSE.getDiscreteState.state, real_gyro_bias, real_acc_bias, trajAcc(fcnt,:), real_mag_dist, trajVel(fcnt,:), ground_height(fcnt))
    
    set(real_pos, 'XData', trajPos(fcnt, 1), 'YData', trajPos(fcnt, 2));
    set(est_pos, 'XData', pqpos(fcnt, 1), 'YData', pqpos(fcnt, 2));


    % Error scope variables
    fusedOrient = pqorient(fcnt,:);
    fusedPos = pqpos(fcnt,:);
    
    % Compute the errors and plot.
    if useErrScope
        orientErr = rad2deg(dist(fusedOrient, ...
            trajOrient(fcnt) ));
        posErr = abs(fusedPos - trajPos(fcnt,:));
        errscope(orientErr, posErr(1), posErr(2), posErr(3));
    end
    
    estim_orient_err(fcnt) = orientErr;
    
    
    % Increase the counter
    fcnt = fcnt + 1;
end


%% Error Metric Computation
% Position and orientation estimates were logged throughout the
% simulation. Now compute an end-to-end root mean squared error for both
% position and orientation.

posd = pqpos(1:loopBound,:) - trajPos( 1:loopBound, :);

% For orientation, quaternion distance is a much better alternative to
% subtracting Euler angles, which have discontinuities. The quaternion
% distance can be computed with the |dist| function, which gives the
% angular difference in orientation in radians. Convert to degrees
% for display in the command window. 

quatd = rad2deg(dist(pqorient(1:loopBound), trajOrient(1:loopBound)) );

% Display RMS errors in the command window.
fprintf('\n\nEnd-to-End Simulation Position RMS Error\n');
msep = sqrt(mean(posd.^2));
fprintf('\tX: %.2f , Y: %.2f, Z: %.2f   (meters)\n\n',msep(1), ...
    msep(2), msep(3));

fprintf('End-to-End Quaternion Distance RMS Error (degrees) \n');
fprintf('\t%.2f (degrees)\n\n', sqrt(mean(quatd.^2)));


% Error metric plots

% Orient
figure()

ttt = linspace(0,size(pqpos,1)/imuFs, size(pqpos,1));

grid on
hold on
plot(ttt, ecompass_err)
plot(ttt, estim_orient_err)
xlabel('time [s]')
ylabel('[degrees]')
title('Orientation quaternion error')
legend('Ecompass orientation error', 'Estimated orientation error')


% GPS
% figure()
% 
% estim_NED_err = pqpos - trajPos;
% 
% subplot(3,1,1)
% hold on
% grid on
% plot(meas_GPS_NED_time, meas_GPS_NED_err(:,1))
% pause(0)
% axis 'auto y'
% plot(ttt, estim_NED_err(:,1));
% xlabel('time [s]')
% ylabel('position error [m]')
% title('Error on the x coordinate')
% legend('Error on the GPS measure', 'Error on the estimated value')
% 
% subplot(3,1,2)
% hold on
% grid on
% plot(meas_GPS_NED_time, meas_GPS_NED_err(:,2))
% pause(0)
% axis 'auto y'
% plot(ttt, estim_NED_err(:,2))
% xlabel('time [s]')
% ylabel('position error [m]')
% title('Error on the y coordinate')
% legend('Error on the GPS measure', 'Error on the estimated value')
% 
% subplot(3,1,3)
% hold on
% grid on
% plot(meas_GPS_NED_time, meas_GPS_NED_err(:,3))
% pause(0)
% axis 'auto y'
% plot(ttt, estim_NED_err(:,3))
% xlabel('time [s]')
% ylabel('position error [m]')
% title('Error on the z coordinate')
% legend('Error on the GPS measure', 'Error on the estimated value')

figure()

estim_NED_err = pqpos - trajPos;
estim_NED_err = log(1+abs(estim_NED_err));

meas_GPS_NED_err = log(1+abs(meas_GPS_NED_err));


subplot(3,1,1)
hold on
grid on
plot(meas_GPS_NED_time, meas_GPS_NED_err(:,1))
pause(0)
axis 'auto y'
plot(ttt, estim_NED_err(:,1));
xlabel('time [s]')
ylabel('log(1+abs(position error))')
title('Error on the x coordinate')
legend('Error on the GPS measure', 'Error on the estimated value')

subplot(3,1,2)
hold on
grid on
plot(meas_GPS_NED_time, meas_GPS_NED_err(:,2))
pause(0)
axis 'auto y'
plot(ttt, estim_NED_err(:,2))
xlabel('time [s]')
ylabel('log(1+abs(position error))')
title('Error on the y coordinate')
legend('Error on the GPS measure', 'Error on the estimated value')

subplot(3,1,3)
hold on
grid on
plot(meas_GPS_NED_time, meas_GPS_NED_err(:,3))
pause(0)
axis 'auto y'
plot(ttt, estim_NED_err(:,3))
xlabel('time [s]')
ylabel('log(1+abs(position error))')
title('Error on the z coordinate')
legend('Error on the GPS measure', 'Error on the estimated value')


% UWB distances
figure()

id_interval = [1,3,5,7];


for jj = 1:length(id_interval)
    subplot(2,2,jj)
    ii = id_interval(jj);
    
    hold on; grid on;
    plot(meas_UWB_dist_time, meas_UWB_dist_err(:,ii))
    plot(meas_UWB_dist_time, estim_UWB_dist_err(:,ii))
    
    xlabel('time [s]')
    ylabel('UWB distance error [m]')
    title(['Anchor id = ', num2str(known_beacon_id(ii))])
    legend('Error on the UWB distance measure', 'Error on the estimated distance value')
end


% Height
figure()

hold on
grid on

plot(meas_bar_time, meas_bar)
plot(meas_alt_time, meas_alt)
plot(ttt, - pqpos(:,3))
plot(ttt, - trajPos(:,3))

xlim([55,180])

xlabel('time [s]')
ylabel('height [m]')
title('Height measurements vs height estimate vs ground-truth height')

legend('Barometer measurement','Altimeter measurement', 'Estimated height', 'True height')


%% Quaternions

% CONVENTION: Quaternion (used by MATLAB) (Hamilton convention)
%	q = (q_scalar, q_vectorial) = (q_w, q_v) = (q_w, q_x, q_y, q_z)
%   Right-handed: ij = k
%   Passive: rotate frames (not vectors)
%   Local-to-Global: q = q_GL = q_L2G
%                    x_G = q * x_L * conj(q)

% CONVENTION: Euler angles (used by MATLAB):
%   eul = [yaw (z), pitch (y), roll (x)] (in this order)
%   rotation order = ZYX: Rz -> Ry -> Rx

% Quaternions are used to avoid singularities in the representation of the
% orientation.

% Alternative method (less computational cost, worse accuracy).
% Uncomment if you want to use this one and comment the other function.
% function [qt_p] = quat_updt(qt, omega, Ts)
%     % Euler approximation
%     qt_p = qt + 1/2*Ts*quaternion([0,omega])*qt;
%     
%     % Output quaternion normalization, it is necessary in this case.
%     qt_p = 1/norm(qt_p) * qt_p;
% end

% Alternative method (small computational cost, medium accuracy).
% Uncomment if you want to use this one and comment the other function.
% function [qt_p] = quat_updt(qt, omega, Ts)
%     % Euler approximation
%     qt_p = quaternion([1,1/2*Ts*omega]) * qt;
%     
%     % Output quaternion normalization, it is necessary in this case.
%     qt_p = 1/norm(qt_p) * qt_p;
% end

% function [qt_p] = quat_updt(qt, omega, Ts)
% % Quaternion update function, uses the quaternion exponential method.
%     qt_p = exp(1/2*Ts*quaternion([0,omega]))*qt;
%     
%     % The normalization should be unnecessary with this method, it is
%     % still used to prevent the accumulation of numerical errors from
%     % affecting the quaternion unit norm.
%     qt_p = 1/norm(qt_p) * qt_p;
% end


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


%% Available sensors

% Returns true if the drone is in a zone where the GPS is available.
function flag = yes_gps(pos)
    if (pos(1) > 60 ) && (pos(1) < 120) && (pos(2) > - 20) && (pos(2) < 20)
        flag = false;
    else
        flag = true;
    end
end

% If the drone is in a zone where UWB is available returns true, and the
% available beacons (which may still be too far away)
function [flag, id] = yes_uwb(pos)
    if (pos(1) > 60 ) && (pos(1) < 90) && (pos(2) > - 20) && (pos(2) < 20)
        flag = true;
        id = linspace(1,4,4);
    elseif pos(1) > 90 && pos(1) < 120 && pos(2) > 5 && pos(2) < 20
        flag = true;
        id = [5,6];
    elseif pos(1) > 90 && pos(1) < 120 && pos(2) > -20 && pos(2) < 5
        flag = true;
        id = [7,8];
    else
        flag = false;
        id = [];
    end
end

% If the drone is in a zone where UWB is available returns true, and the
% available beacons (which may still be too far away)

% Scenario 1
function [flag, id] = yes_uwb_unknown_1(pos)
    if (pos(1) > 60 ) && (pos(1) < 90) && (pos(2) > - 20) && (pos(2) < 20)
        flag = true;
        id = 10;
    else
        flag = false;
        id = [];
    end
end

% % Scenario 2
% function [flag, id] = yes_uwb_unknown_2(pos)
%     x = pos(1);
%     y = pos(2);
% 
%     if (x > 60 ) && (x < 120) && (y > - 20) && (y < 20)
%         flag = true;
%         id = 1;
%         if x > 60 && x < 120 && y > 0 && y < 20
%             if x > 105 && x < 115 && y < 20
%                 id = [1 2 3];
%             else
%                 id = [1 2];
%             end
%         end
%         if x>90 && x<120 && y<0 && y>-20
%             id = [2 3 4];
%         end
%     else
%         flag = false;
%         id = [];
%     end
% end


%% Rotations

% CONVENTION: rotation matrices nomenclature
%	Cnb corresponds to a change of the reference frame from frame b to
%   frame n.
%   v_n = Cnb * v_b
%	Cnb = C_b2n = C_b_n

% function out = rotCni(lat, long, time, omega_ie)
% % Rotation matrix from the ECI frame to the navigation frame.
%     lat = lat*pi/180;
%     long = long*pi/180;
%     
%     lon = long + omega_ie*time;
% 
%     out = [[-sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat)];
%            [-sin(lon), cos(lon), 0];
%            [-cos(lat)*cos(lon), -cos(lat)*sin(lon), -sin(lat)]];
% end
% 
% function out = rotCnb(eul)
% % Rotation matrix from the body frame to the navigation frame after a
% % yaw-pitch-roll rotation (ZYX).
%     yaw = eul(1);
%     pitch = eul(2);
%     roll = eul(3);
% 
%     out = ( rotCx(roll)*rotCy(pitch)*rotCz(yaw) ).';
% end
% 
% function out = rotCx(ang)
%     out = [[1, 0, 0];
%            [0, cos(ang), sin(ang)];
%            [0, -sin(ang), cos(ang)]];
% end
% 
% function out = rotCy(ang)
%     out = [[cos(ang), 0, -sin(ang)];
%            [0, 1, 0];
%            [sin(ang), 0, cos(ang)]];
% end
% 
% function out = rotCz(ang)
%     out = [[cos(ang), sin(ang), 0];
%            [-sin(ang), cos(ang), 0];
%            [0, 0, 1]];
% end


%% Earth

% function [rn, rm, g] = ellwgs84(re, eccentricity, lat, h, g_wgs0, g_wgs1)
%     rn = re/(1-eccentricity^2*sin(lat)^2)^0.5;                      % R_normal
%     rm = rn * (1-eccentricity^2)/(1-eccentricity^2*sin(lat)^2);     % R_meridian
%     
%     g = g_wgs0 * (1 + g_wgs1 * sin(lat)^2)/(1 - eccentricity^2 * sin(lat)^2) * re^2/(re + h)^2;
% end
% 
% function [lat, lon] = coord_updt(coord, v_n, v_e, rn, rm, Ts)
% 
%     % v_n = v_North     rn = R_normal       Ts = sampling time
%     % v_e = v_East      rm = R_meridian
% 
%     lat = coord(1);
%     lon = coord(2);
%     h = coord(3);
% 
%     latd = v_n / (rm + h);
%     lond = v_e / ((rn + h) * cos(lat));
%     
%     lat = lat + latd * Ts;
%     lon = lon + lond * Ts;
% end
% 
% function omega_en_n = x2omega_en(lat, h, v_n, v_e)
%     latd = v_n / (rm + h);
%     lond = v_e / ((rn + h) * cos(lat));
% 
%     omega_en_n = [ lond * cos(lat);
%                   -latd;
%                   -lond * sin(lat)];
% end

