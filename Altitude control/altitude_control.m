
clear all;
close all;
clc;

%% Extract data

blimp_data = readmatrix('log_Blimp_data_2023-07-20 21-52-03.txt');

blimp_navigation = readmatrix('log_Blimp_navigation_2023-07-20 21-52-03.txt');

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

time_nav = blimp_navigation(:,1);
l_pwm = blimp_navigation(:,2);
r_pwm = blimp_navigation(:,3);
z_pwm = blimp_navigation(:,4);
goal_distance = blimp_navigation(:,5);
psi_ref = blimp_navigation(:,6);
z_ref = blimp_navigation(:,7);

%%

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


%%
i = 0;

i = i +1;
figure(i)
plot(time_data, z_mean)
hold on
plot(time_nav, z_ref)
legend("z position", "z reference")
xlabel("Time [s]")
ylabel("z data")
title("Altitude control")

i = i +1;
figure(i)
plot(time_nav, z_pwm)
hold on
plot(time_nav, z_pwm_pos)
legend("z PWM", "Positive z PWM")
xlabel("Time [s]")
ylabel("z PWM")
title("Altitude control")