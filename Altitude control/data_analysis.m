
clear all;
close all;
clc;

%% Extract data

%  blimp_data = readmatrix('log_Blimp_data_2023-07-20 22-34-50.txt');
%  blimp_navigation = readmatrix('log_Blimp_navigation_2023-07-20 22-34-50.txt');

 blimp_data = readmatrix('log_Blimp_data_2023-07-20 21-45-02.txt');
 blimp_navigation = readmatrix('log_Blimp_navigation_2023-07-20 21-45-02.txt');

%  blimp_data = readmatrix('log_Blimp_data_2023-07-20 21-52-03.txt');
%  blimp_navigation = readmatrix('log_Blimp_navigation_2023-07-20 21-52-03.txt');

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

%% Time Analysis
% Delta time of acquisition is constant? 
time_data_delta = [];
time_nav_delta = [];

for l = 1:(length(time_data)-1)
time_data_delta(l) = time_data(l+1) - time_data(l);
end

mean_delta_data_time = mean(time_data_delta);
var_delta_data_time = var(time_data_delta);


for l = 1:(length(time_nav)-1)
time_nav_delta(l) = time_nav(l+1) - time_nav(l);
end

mean_delta_nav_time = mean(time_nav_delta);
var_delta_nav_time = var(time_nav_delta);



%% Inerzia

%calcolo tante volte l'inerzia finchè il Blimp gira
% 113 or 57
inerzia_vec = [];
L = 0.135;
omega = (pi/2)/34;
time_zero = time_nav(1);
for l = 1:57
inerzia_vec(l) = ((l_pwm(l)/100) * 0.1 * L * (time_nav(l) - time_zero)^2  )/( pi - omega * (time_nav(l) - time_zero));
end

Mean_inerzia = mean(inerzia_vec);
var_inerzia  = var(inerzia_vec);
%% Varianza UWB
% Considero i primi 50 punti in cui dovrebbe star ruotando staticamente su
% se stesso 
var_posx = var(x_pos(1:50));
var_posy = var(y_pos(1:50));

%% Varianza Yaw angle

var_yaw = var(psi(1:10));

%% Propagazione errore per il mio controllo

syms Pl Pr sPl Pr dt sdt mpwm smpwm massa smassa Ine sIne l sl

Tk = (Pl + Pr)/massa * (0.5* dt^2)*mpwm;

deltaAngle = (Pl - Pr)/Ine *l*0.5 *(0.5* dt^2)*mpwm;

%Calcolo le derivate parziali
dTk_Pl = diff(Tk,Pl);
dTk_Pr = diff(Tk,Pr);
dTk_massa = diff(Tk,massa);
dTk_dt = diff(Tk,dt);
dTk_mpwm = diff(Tk,mpwm);


ddeltaA_Pl = diff(deltaAngle, Pl);
ddeltaA_Pr = diff(deltaAngle, Pr);
ddeltaA_l = diff(deltaAngle, l);
ddeltaA_dt = diff(deltaAngle, dt);
ddeltaA_mpwm = diff(deltaAngle, mpwm);
ddeltaA_Ine = diff(deltaAngle, Ine);


error_Tk = sqrt( (dTk_Pl* sPl)^2 + (dTk_Pr* sPl)^2 + (dTk_massa* smassa)^2 + (dTk_dt* sdt)^2 + (dTk_mpwm* smpwm)^2 );
error_deltaAngle = sqrt( (ddeltaA_Pl* sPl)^2 + (ddeltaA_Pr* sPl)^2 + (ddeltaA_l* sl)^2 + (ddeltaA_dt* sdt)^2 + (ddeltaA_mpwm* smpwm)^2 + (ddeltaA_Ine* sIne)^2 );


Pl = 7/100; sPl = 0.1/100;
Pr = 7/100;
dt = 0.2915;  sdt = 0.0033;
mpwm = 0.1; smpwm = 0.01;
massa = 0.250; smassa = 0.0001;
Ine = 0.0363; sIne = 9.888078498361200e-04;
l = 0.135/2; sl = 0.001;


num_error_Tk = eval(error_Tk);
num_error_deltaAngle = eval(error_deltaAngle);


%% Plot of Figure
i = 0;
i = i +1;
figure(i)
tiledlayout(3,1)
% Top plot
nexttile
plot(time_data, smooth(acc_x), 'r')
title('Accelerations of Blimp')
xlabel('time [s]')
ylabel('acceleration [m/s^2]')
legend('acc_x')
% Middle plot
nexttile
plot(time_data, smooth(acc_y), 'b')
title('Accelerations of Blimp')
xlabel('time [s]')
ylabel('acceleration [m/s^2]')
legend('acc_y')
% Bottom plot
nexttile
plot(time_data, smooth(acc_z), 'k')
title('Accelerations of Blimp')
xlabel('time [s]')
ylabel('acceleration [m/s^2]')
legend('acc_z')


% Gyroscope Data
i = i +1;
figure(i)
tiledlayout(3,1)
% Top plot
nexttile
title('Rotational velocity of Blimp')
plot(time_data, smooth(gyro_x), 'r')
xlabel('time [s]')
ylabel('angular velocity [deg/s]')
legend('gyro_x')
% Middle
nexttile
plot(time_data, smooth(gyro_y), 'b')
xlabel('time [s]')
ylabel('angular velocity [deg/s]')
legend('gyro_y')
% Bottom
nexttile
plot(time_data, smooth(gyro_z), 'k')

xlabel('time [s]')
ylabel('angular velocity [deg/s]')
legend('gyro_z')


% Motors value
i = i +1;
figure(i)
hold on
plot(time_nav, l_pwm, 'r')
plot(time_nav, r_pwm, 'b')
plot(time_nav, z_pwm, 'k')
title('PWM motors of Blimp')
xlabel('time [s]')
ylabel('PWM ')
legend('left motor', 'right motor', 'z motor')

% Blimp position
i = i +1;
figure(i)
hold on
plot(x_pos, y_pos, '*')
xlim([0 12])
ylim([0 12])
title('Blimp coordinates over time')
xlabel('x [m]')
ylabel('y [m] ')
legend('Blimpo position')


% Psi value

i = i +1;
figure(i)
hold on
plot( time_nav, psi_ref, 'b')
plot( time_data, psi, 'r')
title('Yaw of Blimp')
xlabel('time [s]')
ylabel('Orientation [degrees] ')
legend('yaw reference', 'yaw blimp')


i = i +1;
figure(i)
hold on
plot(time_data, 'b')
plot(time_nav, 'r')
xlabel('numero dati')
ylabel('Time [s] ')
legend('data', 'nav')

i = i +1;
figure(i)
hold on
plot((1./time_nav_delta), '*')
plot((1./time_data_delta), 'd')
legend('nav', 'data')


i = i +1;
figure(i)
hold on
plot((inerzia_vec), '*')
legend('Inerzia Blimp')

% i = i +1;
% figure(i)
% plot(time_data, z_mean)
% hold on
% plot(time_nav, z_ref)
% legend("z position", "z reference")
% xlabel("Time [s]")
% ylabel("z data")
% title("Altitude control")
% 
% i = i +1;
% figure(i)
% plot(time_nav, z_pwm)
% hold on
% plot(time_nav, z_pwm_pos)
% legend("z PWM", "Positive z PWM")
% xlabel("Time [s]")
% ylabel("z PWM")
% title("Altitude control")

%% Extended Kalman filter

% Define Q and R covariance matrix and alpha vector.

Q = [0.001  0.00  0.00;  % Each variable is only related itself.
     0.00  0.001  0.00; 
     0.00  0.00  0.001];
 
RR = [0.03 0 0;
      0 0.002 0;
      0 0 1e-11];

alpha= [ 0.9 0.3 0.601 0.001 ];
% Define noise parameters.
varianceV= 0.01; %.3 
varianceW= 0.017; % .1
varianceR = 0.5;
varianceT = 0.05;

sigma = [0 0 0;  %Covariance matrix start with all elements zero
         0 0 0; 
         0 0 0];
