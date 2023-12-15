clc;
clear all; 
close all; 

%dt = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]; 
dt = 0.0001;  % CHE DT VA MESSO QUI? 
% QUALE è L'ORDINE GIUSTO PER STI TEMPI DALLE ANCORE? 
T = [1., 1., 1.];
c = 299792458; %#m/s


%Embedded Lab system anchor position
fileName = 'C:\Volume_D\Programming\Blimp_git\Blimp\log.txt'; % filename in JSON extension 
str = fileread(fileName); % dedicated for reading files as text 
data = strsplit(str,' ');
data = bin2dec(data);
%data = str2double(data);
data(end) = []; % Remove the last element
%data = jsondecode(str); % Using the jsondecode function to parse JSON from string 
n_data = length(data);

times = reshape( data,[(n_data/24) 24]);
   

stringData = split(stringData);

            for i=1:6
                ts(i,1) = eval(sprintf("bin2dec(stringData(%d));",i));
                ts(i,2) = eval(sprintf("bin2dec(stringData(%d+6));",i));
                ts(i,3) = eval(sprintf("bin2dec(stringData(%d+12));",i));
                ts(i,4) = eval(sprintf("bin2dec(stringData(%d+18));",i));
            end

            t6_rx1 = double(ts(1,1)) * 15.65e-12;
            t1_rx1 = double(ts(2,1)) * 15.65e-12;
            t2_rx1 = double(ts(3,1)) * 15.65e-12;
            t3_rx1 = double(ts(4,1)) * 15.65e-12;
            t4_rx1 = double(ts(5,1)) * 15.65e-12;
            t5_rx1 = double(ts(6,1)) * 15.65e-12;

            t6_rx2 = double(ts(1,2)) * 15.65e-12;
            t1_rx2 = double(ts(2,2)) * 15.65e-12;
            t2_rx2 = double(ts(3,2)) * 15.65e-12;
            t3_rx2 = double(ts(4,2)) * 15.65e-12;
            t4_rx2 = double(ts(5,2)) * 15.65e-12;
            t5_rx2 = double(ts(6,2)) * 15.65e-12; %double(1/(63.8976 * 1000000000) );

            t6_tx1 = double(ts(1,3)) * 15.65e-12;
            t1_tx1 = double(ts(2,3)) * 15.65e-12;
            t2_tx1 = double(ts(3,3)) * 15.65e-12;
            t3_tx1 = double(ts(4,3)) * 15.65e-12;
            t4_tx1 = double(ts(5,3)) * 15.65e-12;
            t5_tx1 = double(ts(6,3)) * 15.65e-12;

            t6_tx2 = double(ts(1,4)) * 15.65e-12;
            t1_tx2 = double(ts(2,4)) * 15.65e-12;
            t2_tx2 = double(ts(3,4)) * 15.65e-12;
            t3_tx2 = double(ts(4,4)) * 15.65e-12;
            t4_tx2 = double(ts(5,4)) * 15.65e-12;
            t5_tx2 = double(ts(6,4)) * 15.65e-12;

            %real measurements
            toa_tx = double([ t6_tx1,t6_tx2; t1_tx1, t1_tx2; t2_tx1,t2_tx2; t3_tx1,t3_tx2; t4_tx1,t4_tx2; t5_tx1,t5_tx2]);
            toa_rx = double([t6_rx1,t6_rx2; t1_rx1,t1_rx2; t2_rx1,t2_rx2; t3_rx1,t3_rx2; t4_rx1,t4_rx2; t5_rx1,t5_rx2]);

            %Drift tag
            eval(sprintf("dt_new%d(:,j) = (toa_rx(:,2)-toa_rx(:,1))./(toa_tx(:,2)-toa_tx(:,1));",kk));

            for i = 1:num_anch
                eval(sprintf("dt%d(%d,1) = mean(rmoutliers(dt_new%d(%d,:)),2);",kk,i,kk,i));
            end





[x_t, dt_new] = rTDoApippo(times(2,:), dt)



function [x_t, dt_new] = rTDoApippo(ts, dt)

t6_rx1 = double(ts(1)) * 15.65e-12;
t1_rx1 = double(ts(2)) * 15.65e-12;
t2_rx1 = double(ts(3)) * 15.65e-12;
t3_rx1 = double(ts(4)) * 15.65e-12;
t4_rx1 = double(ts(5)) * 15.65e-12;
t5_rx1 = double(ts(6)) * 15.65e-12;

t6_rx2 = double(ts(7)) * 15.65e-12;
t1_rx2 = double(ts(8)) * 15.65e-12;
t2_rx2 = double(ts(9)) * 15.65e-12;
t3_rx2 = double(ts(10)) * 15.65e-12;
t4_rx2 = double(ts(11)) * 15.65e-12;
t5_rx2 = double(ts(12)) * 15.65e-12; %double(1/(63.8976 * 1000000000) );

t6_tx1 = double(ts(13)) * 15.65e-12;
t1_tx1 = double(ts(14)) * 15.65e-12;
t2_tx1 = double(ts(15)) * 15.65e-12;
t3_tx1 = double(ts(16)) * 15.65e-12;
t4_tx1 = double(ts(17)) * 15.65e-12;
t5_tx1 = double(ts(18)) * 15.65e-12;

t6_tx2 = double(ts(19)) * 15.65e-12;
t1_tx2 = double(ts(20)) * 15.65e-12;
t2_tx2 = double(ts(21)) * 15.65e-12;
t3_tx2 = double(ts(22)) * 15.65e-12;
t4_tx2 = double(ts(23)) * 15.65e-12;
t5_tx2 = double(ts(24)) * 15.65e-12; %double(1/(63.8976 * 1000000000) );

% Embedded Lab system
 A_n1 = [0.00; 7.19; 2.15];
 A_n2 = [0.00; 3.62; 3.15];
 A_n3 = [0.00; 0.00; 2.15];
 A_n4 = [4.79; 1.85; 3.15];
 A_n5 = [4.79; 5.45; 2.15];
 A_n6  = [3.00; 9.35; 3.15];

% Mechatronics Lab system
% anchor_offset = [0;0;0.2];
% A_n1 = [1.000; 0.00; 2.413] + anchor_offset;
% A_n2 = [10.990; 0.521; 2.411]  + anchor_offset;
% A_n3 = [17.058; 0.631; 2.386] + anchor_offset;
% A_n4 = [17.512; 6.231; 2.416]  + anchor_offset;
% A_n5 = [5.679; 6.507; 2.386]  + anchor_offset;
% A_n6 = [0.123; 5.921; 2.436]  + anchor_offset;

A_n = [A_n6 A_n1 A_n2 A_n3 A_n4 A_n5]
c = 299792458;
n = length(A_n);

TOF_MA = sqrt(sum((A_n6*ones(1,6) - A_n).^2,1))/c;      % ToF anchor/master  

%real measurements
toa_tx = double([t6_tx1,t6_tx2; t1_tx1, t1_tx2; t2_tx1,t2_tx2; t3_tx1,t3_tx2; t4_tx1,t4_tx2; t5_tx1,t5_tx2]);
toa_rx = double([t6_rx1,t6_rx2; t1_rx1,t1_rx2; t2_rx1,t2_rx2; t3_rx1,t3_rx2; t4_rx1,t4_rx2; t5_rx1,t5_rx2]);

%TOA = toa_rx(:,2) 
%Drift tag
dt_new = (toa_rx(:,2)-toa_rx(:,1))./(toa_tx(:,2)-toa_tx(:,1));

tmp_rx(:,1) = toa_rx(:,1) - toa_rx(1,1) - (toa_tx(:,1).*dt - toa_tx(1,1)*dt(1));
tmp_rx(:,2) = toa_rx(:,2) - toa_rx(1,2) - (toa_tx(:,2).*dt - toa_tx(1,2)*dt(1));

%% TDoA
%     tdoa = tmp_rx(:,2) - tmp_tx(:,2);
tdoa = tmp_rx(:,2)
tdoa(1)=[]; %elimina il primo zero

D = c*tdoa
%------Trilateration linear equations system-------------------
A = 2*[(A_n6(1)-A_n(1, 2:n)'), (A_n6(2)-A_n(2,2:n)'),(A_n6(3)-A_n(3,2:n)'), D]
b = D.^2 + norm(A_n6)^2 - sum((A_n(:,2:n)'.^2),2)
x_t0 = pinv(A)*b

%-----Non linear correction (Taylor Expansion)-----------------
x_t_0 = [x_t0(1); x_t0(2); x_t0(3)];
f = zeros(n-1,1);
del_f = zeros(n-1,3);

for ii=2:n
    f(ii-1)=norm(x_t_0-A_n(:,ii))-norm(x_t_0-A_n(:,1));
    del_f(ii-1,1) = (x_t_0(1)-A_n(1,ii))*norm(x_t_0-A_n(:,ii))^-1 - (x_t_0(1) - A_n(1,1))*norm(x_t_0-A_n(:,1))^-1;
    del_f(ii-1,2) = (x_t_0(2)-A_n(2,ii))*norm(x_t_0-A_n(:,ii))^-1 - (x_t_0(2) - A_n(2,1))*norm(x_t_0-A_n(:,1))^-1;
    del_f(ii-1,3) = (x_t_0(3)-A_n(3,ii))*norm(x_t_0-A_n(:,ii))^-1 - (x_t_0(3) - A_n(3,1))*norm(x_t_0-A_n(:,1))^-1;
end

x_t = pinv(del_f)*(D-f) + x_t_0;
end