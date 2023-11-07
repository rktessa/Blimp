clear all;
close all;
clc;

%% Extract data

% blimp_data = readmatrix('log_Blimp_data_2023-07-20 21-45-02.txt');
% blimp_navigation = readmatrix('log_Blimp_navigation_2023-07-20 21-45-02.txt');
blimp_data = readmatrix('log_Blimp_data_2023-07-20 22-34-50.txt');
blimp_navigation = readmatrix('log_Blimp_navigation_2023-07-20 22-34-50.txt');


time_data = blimp_data(:,1);

acc_x = blimp_data(:,2);
acc_y = blimp_data(:,3);
acc_z = blimp_data(:,4);
gyro_x = blimp_data(:,5);
gyro_y = blimp_data(:,6);
gyro_z = blimp_data(:,7);
mag_x = blimp_data(:,8);
mag_y = blimp_data(:,9);
mag_z = blimp_data(:,10);

x_pos = blimp_data(:,11);
y_pos = blimp_data(:,12);
z_pos = blimp_data(:,13);
psi = blimp_data(:,14);

for i = 1:length(psi)
    if psi(i) < 10
        psi(i) = 360 - psi(i);
    end
end

time_nav = blimp_navigation(:,1);
l_pwm = blimp_navigation(:,2);
r_pwm = blimp_navigation(:,3);
z_pwm = blimp_navigation(:,4);
goal_distance = blimp_navigation(:,5);
psi_ref = blimp_navigation(:,6);
z_ref = blimp_navigation(:,7);

for i = 1:length(psi_ref)
    if psi_ref(i) < -80
        psi_ref(i) = 360 + psi_ref(i);
    end
end
%% Mean of the Z measured by Ultrasonic sensor

z_list = [];
z_counter = 0;
z_mean = [];
flag_z = 0;

for i = 1:length(z_pos)
    if length(z_list) < 10
        z_list(end+1) = z_pos(i);
    else
        [z_list, flag_z] = pos_mean(z_list, z_pos(i));
    end
                
    if flag_z == 1
        z_counter = z_counter + 1;
    else
        z_counter = 0;
    end
            
    if z_counter >= 10
        z_list(end+1) = z_pos(i);
        z_list(1) = [];
    end

    z_mean(end+1) = sum(z_list) / length(z_list);

end

z_pwm_pos = [];

for j = 1:length(z_pwm)
    if z_pwm(j) < 0
        z_pwm_pos(j) = 0;
    else
        z_pwm_pos(j) = z_pwm(j);
    end
end

%% Mean of x and y coordinate position

x_list = [];
y_list = [];
x_counter = 0;
y_counter = 0;
x_mean = [];
y_mean = [];
flag_x = 0;
flag_y = 0;

for i = 1:length(x_pos)
    if length(x_list) < 5
        x_list(end+1) = x_pos(i);
        y_list(end+1) = y_pos(i);
    else
        [x_list, flag_x] = pos_mean(x_list, x_pos(i));
        [y_list, flag_y] = pos_mean(y_list, y_pos(i));
    end
                
    if flag_x == 1
        x_counter = x_counter + 1;
    else
        x_counter = 0;
    end

    if flag_y == 1
        y_counter = y_counter + 1;
    else
        y_counter = 0;
    end
            
    if x_counter >= 3
        x_list(end+1) = x_pos(i);
        x_list(1) = [];
    end

    if y_counter >= 3
        y_list(end+1) = y_pos(i);
        y_list(1) = [];
    end

    x_mean(end+1) = sum(x_list) / length(x_list);
    y_mean(end+1) = sum(y_list) / length(y_list);

end

%% Psi mean

psi_list = [];
psi_mean = [];

for i = 1:length(psi)
    psi_list(end+1) = psi(i);
    if length(psi_list) > 3
        psi_list(1) = [];
    end

    psi_mean(end+1) = psi_mean_function(psi_list, psi(i));

end

%% Plot results

figure()
plot(time_data, z_mean)
hold on
plot(time_nav, z_ref)
legend("z position", "z reference")
xlabel("Time [s]")
ylabel("Altitude [m]")
title("Altitude control")

figure
plot(time_nav, z_pwm)
hold on
plot(time_nav, z_pwm_pos)
legend("z PWM", "Positive z PWM")
xlabel("Time [s]")
ylabel("z PWM [%]")
title("Altitude control")

%%

figure()
plot(x_mean(1), y_mean(1),"*")
hold on
plot(x_mean, y_mean, '.')
hold on 
plot(3,2,"*")
legend("Blimp starting point", "Blimp trajectory point", "Goal")
xlabel("x [m]")
ylabel("y [m]")
title("Blimp point tracking trajectory")
xlim([0,5])
ylim([0,10])
axis equal
pbaspect([1 2 1])

%%

figure()
tmp_ax1 = subplot(3,1,1);
plot(time_data, psi_mean)
hold on
plot(time_nav, psi_ref)
legend("psi", "psi reference")
xlabel("Time [s]")
ylabel("Yaw angle [°]")
title("Yaw angle control")
tmp_ax2 = subplot(3,1,2);
plot(time_nav, goal_distance)
xlabel("Time [s]")
ylabel("Goal distance [m]")
title("Goal distance control")
ylim([0,10])
tmp_ax3 = subplot(3,1,3);
plot(time_nav, l_pwm)
hold on
plot(time_nav, r_pwm)
legend("Left motor PWM", "Right motor PWM")
xlabel("Time [s]")
ylabel("Lateral motors PWM")
title("Lateral motors PWM evolution")

linkaxes([tmp_ax1, tmp_ax2, tmp_ax3],'x')


%% Write into a file x_mean, y_mean, yaw
fileID = fopen('prova_22_34.txt','w');
matrix =  [time_data, x_mean', y_mean', psi_mean'];
fprintf(fileID, '%f %f %f %f\n', matrix');
