clc; clear all; close all

%% Path setup
path_step = transpose(0:.01:500);
x_path = path_step/17 + 2*sin(path_step/50);
y_path = sqrt(path_step) + 5*sin(path_step/10);

position_matrix = [x_path,y_path];
distance_between_points = diff(position_matrix,1);
dist_from_vertex_to_vertex = hypot(distance_between_points(:,1), distance_between_points(:,2));
cumulative_dist_along_path = [0; cumsum(dist_from_vertex_to_vertex,1)];


%% Cart setup
saturation_limit = 4;
motor_coef = .5;
motor_model = @(input_command) min(saturation_limit, max(-saturation_limit, input_command * motor_coef)) + .3 * randn(1);
k_p = 3;
k_i = 1;
k_d = .1;


%% Simulation and initial conditions setup and array initialization
time_step = .1;
sim_time = 100;
t = transpose(0:time_step:sim_time);
distance_actual = zeros(size(t));
err = zeros(size(t));

err_integrated = 0;
err_derivative = 0;
distance_actual(1) = 20; % actual starting point


%% Desired trajectory following pattern
velocity = .5 + 2*sin(t/13).^8;
velocity=[velocity(1:end-100); zeros(100,1)];
reference_starting_point = 40;
desired_travel_distances = cumsum(velocity .* time_step,1) + reference_starting_point; 
desired_trajectory = interp1(cumulative_dist_along_path, position_matrix, desired_travel_distances);
x_pos = desired_trajectory(:,1);
y_pos = desired_trajectory(:,2);

%% Run simulation
for sim_step = 1:sim_time/time_step
    err(sim_step) = desired_travel_distances(sim_step) - distance_actual(sim_step); % calculate error
    err_integrated = err_integrated + err(sim_step) * time_step;
    if sim_step>1
        err_derivative = (err(sim_step) - err(sim_step-1)) / time_step;
    end
    u_motor = k_p * err(sim_step) + k_d * err_derivative + k_i * err_integrated; % apply PID magic and calculate control input
    actual_velocity(sim_step) = motor_model(u_motor); % Send the command to the motor and it will have a new velocity based on that
    distance_actual(sim_step+1) = actual_velocity(sim_step) * time_step + distance_actual(sim_step); % calculate traveled distance from new
end


%% How good it follows the desired velocity pattern
figure()
title('Velocity Profile')
xlabel('Simulation Step')
ylabel('Velocity')
plot(velocity,'LineWidth',3)
hold on
plot(actual_velocity, ':')
hold off
legend('Desired', 'Actual')


%% Error analysis
figure()
title('Error')
xlabel('Simulation Step')
ylabel('Error')
plot(err)


%% Make an animation
actual_position = interp1(cumulative_dist_along_path, position_matrix, distance_actual);
figure()
title('Simulation')
xlabel('X')
ylabel('Y')
plot(x_path, y_path)
hold on
h1 = plot(x_pos(1), y_pos(1), 'rx');
h2 = plot(actual_position(1, 1), actual_position(1, 2), 'ko');
legend('Path', 'Desired Position', 'Actual Position')
disp('Press key ...')
waitforbuttonpress()
for sim_step=1:sim_time/time_step
    delete(h1)
    delete(h2)
    clc
    disp('Time:')
    t = sprintf('%0.3f', sim_step * time_step);
    disp(t)
    h1 = plot(x_pos(sim_step), y_pos(sim_step), 'rx');
    h2 = plot(actual_position(sim_step, 1), actual_position(sim_step, 2), 'ko');
    drawnow
    pause(.05)
end
clc
disp('Finished!')
