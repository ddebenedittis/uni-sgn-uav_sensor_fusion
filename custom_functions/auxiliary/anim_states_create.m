function [anim_est_states, anim_real_states, fig3] = anim_states_create(default_colors)
    fig3 = figure('Name', 'States', ...
                  'NumberTitle', 'off', ...
                  'Units', 'normalized', ...
                  'OuterPosition', [0.5, 0, 0.5, 1]);

    hax3 = subplot(3,2,1);
    title('Gyroscope bias')

    hax4 = subplot(3,2,2);
    title('Accelerometer bias')

    hax5 = subplot(3,2,3);
    title('Linear acceleration')

    hax6 = subplot(3,2,4);
    title('Magnetometer disturbance')

    hax7 = subplot(3,2,5);
    title('Velocity')

    hax8 = subplot(3,2,6);
    title('Ground height')

    % Estimated values

    % Gyro bias
    anim_est_states( 1) = animatedline(hax3, 'Color', default_colors(1,:));
    anim_est_states( 2) = animatedline(hax3, 'Color', default_colors(2,:));
    anim_est_states( 3) = animatedline(hax3, 'Color', default_colors(3,:));

    % Accel bias
    anim_est_states( 4) = animatedline(hax4, 'Color', default_colors(1,:));
    anim_est_states( 5) = animatedline(hax4, 'Color', default_colors(2,:));
    anim_est_states( 6) = animatedline(hax4, 'Color', default_colors(3,:));

    % Lin accel
    anim_est_states( 7) = animatedline(hax5, 'Color', default_colors(1,:));
    anim_est_states( 8) = animatedline(hax5, 'Color', default_colors(2,:));
    anim_est_states( 9) = animatedline(hax5, 'Color', default_colors(3,:));

    % Mag dist
    anim_est_states(10) = animatedline(hax6, 'Color', default_colors(1,:));
    anim_est_states(11) = animatedline(hax6, 'Color', default_colors(2,:));
    anim_est_states(12) = animatedline(hax6, 'Color', default_colors(3,:));

    % Vel
    anim_est_states(13) = animatedline(hax7, 'Color', default_colors(1,:));
    anim_est_states(14) = animatedline(hax7, 'Color', default_colors(2,:));
    anim_est_states(15) = animatedline(hax7, 'Color', default_colors(3,:));

    % Ground height
    anim_est_states(16)   = animatedline(hax8, 'Color', default_colors(1,:));


    % Real values

    anim_real_states( 1) = animatedline(hax3, 'Color', default_colors(1,:), 'LineStyle', ':');
    anim_real_states( 2) = animatedline(hax3, 'Color', default_colors(2,:), 'LineStyle', ':');
    anim_real_states( 3) = animatedline(hax3, 'Color', default_colors(3,:), 'LineStyle', ':');

    anim_real_states( 4) = animatedline(hax4, 'Color', default_colors(1,:), 'LineStyle', ':');
    anim_real_states( 5) = animatedline(hax4, 'Color', default_colors(2,:), 'LineStyle', ':');
    anim_real_states( 6) = animatedline(hax4, 'Color', default_colors(3,:), 'LineStyle', ':');

    anim_real_states( 7) = animatedline(hax5, 'Color', default_colors(1,:), 'LineStyle', ':');
    anim_real_states( 8) = animatedline(hax5, 'Color', default_colors(2,:), 'LineStyle', ':');
    anim_real_states( 9) = animatedline(hax5, 'Color', default_colors(3,:), 'LineStyle', ':');

    anim_real_states(10) = animatedline(hax6, 'Color', default_colors(1,:), 'LineStyle', ':');
    anim_real_states(11) = animatedline(hax6, 'Color', default_colors(2,:), 'LineStyle', ':');
    anim_real_states(12) = animatedline(hax6, 'Color', default_colors(3,:), 'LineStyle', ':');

    anim_real_states(13) = animatedline(hax7, 'Color', default_colors(1,:), 'LineStyle', ':');
    anim_real_states(14) = animatedline(hax7, 'Color', default_colors(2,:), 'LineStyle', ':');
    anim_real_states(15) = animatedline(hax7, 'Color', default_colors(3,:), 'LineStyle', ':');

    anim_real_states(16)   = animatedline(hax8, 'Color', default_colors(1,:), 'LineStyle', ':');
end