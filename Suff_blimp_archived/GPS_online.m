% Author:   Luca Santoro: luca.santoro@unitn.it
%           Matteo Nardello: matteo.nardello@unitn.it
% Created:  12.02.2021
%
% University of Trento

% Clean workspace
clear all
close all
clc
instrreset

% Define squared fence dimensions
type = 'p';
% center = [2.5 4 -1];
% dx = 2;
% dy = 3;
% dz = 4;

center = [7 6 4];
dx = 10;
dy = 12;
dz = 7;
r = sqrt(dx^2 + dy^2);
% Generate fence area
out = generate_fance(type,r,center,dx,dy,dz);
bond = boundary(out',0);

% PV variables
con = 1;
c = 299792458;

num_anch = 6;

% A_n1 = [0.00; 7.19; 2.15];
% A_n2 = [0.00; 3.62; 3.15];
% A_n3 = [0.00; 0.00; 2.15];
% A_n4 = [4.79; 1.85; 3.15];
% A_n5 = [4.79; 5.45; 2.15];
% A_n6  = [3.00; 9.35; 3.15];

A_n1 = [0.00; 0.00; 5.68];
A_n2 = [5.65; 0.00; 5.74];
A_n3 = [3.05; 4.95; 6.26];
A_n4 = [8.01; 7.35; 7.03];
A_n5 = [13.22; 4.95; 6.22];
A_n6  = [2.92; 11.05; 6.68];

A_n = [A_n1 A_n2 A_n3 A_n4 A_n5 A_n6];
n = length(A_n);

pos1 = [2.57;3.65];
pos2 = [2.57;3.65];
pos3 = [2.57;3.65];

data = zeros(1,24);
fixlen = 1;
dt1 = zeros(6,fixlen);
dt2= zeros(6,fixlen);
dt3 = zeros(6,fixlen);

num_tag = 3;

if con
    myMQTT = mqtt('tcp://192.168.0.100');
    
    for i=1:num_tag
        eval(sprintf("rec%d = subscribe(myMQTT,'tag%d');",i,i));
    end
else
    s = serial('COM21');
    s.BaudRate = 921600;
    s.InputBufferSize = 1500;
    fopen(s);
end

%Figure control handle
DlgH = figure('name','GPS');
H = uicontrol('Style', 'PushButton', ...
    'String', 'Stop', ...
    'Callback', 'delete(gcbf)');
hold on
grid on
axis equal
plot3(A_n(1,:),A_n(2,:), A_n(3,:),'o','MarkerSize',10,'MarkerEdgeColor','blue','MarkerFaceColor',[.1 .2 .8]);
view([-2 -2 2])
% Localisation
disp("Start Localisation");

counter = 1;
dur = 1e4;
j = 1;

for i=1:num_tag
    eval(sprintf("j%d = 1;",i));
    eval(sprintf("run%d = 0;", i));
end

x_t_old = [0,0];

while (j<=dur && (ishandle(H)))
    for kk=1:num_tag
        try
            if con

                pause(0.001);
                eval(sprintf("stringData = rec%d.read();",kk));

            else
                tmp = fscanf(s,'%c');
                data = textscan(tmp,'%s','delimiter','\r\n','whitespace',' ');
                stringData = string(data{:});
            end

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

%             tmpdtmean(:,j) = dt;
    %         dt = mean(rmoutliers(dt_new),2);
            eval(sprintf("[x_t, dt%d] = rTDoA(ts, dt%d);",kk,kk));
%             eval(sprintf("err%d(j%d) = norm(pos%d-x_t(1:2));",kk,kk,kk));
            eval(sprintf("history%d(j%d,:) = x_t(1:2);",kk,kk));

            trisurf(bond,out(1,:),out(2,:),out(3,:),'Facecolor','red','FaceAlpha',0.1)
            if abs(x_t(1)) < 1e1 || abs(x_t(2)) < 1e1
                eval(sprintf("h%d(1,j%d) = plot3(history%d(j%d,1),history%d(j%d,2),x_t(3),'o','MarkerSize',7,'MarkerFaceColor',[.8 .2 .1]);",kk,kk,kk,kk,kk,kk));
                eval(sprintf("j%d = j%d + 1;",kk,kk));
                drawnow
            end
            if eval(sprintf("run%d == 1", kk))
                eval(sprintf("delete(h%d(1,j%d-1));",kk,kk));
            end

            x_t_old = x_t;
            eval(sprintf("run%d = 1;", kk)); 


        catch

        end
    end
    sprintf("Status: %d %", j*100/dur)
    j = j + 1; 

end

hold off
if con==0
    fclose(s);
end
%% Plotting
figure('Name', 'Estimated Position')
hold on

for i=1:num_tag
    
    eval(char(sprintf(" if exist('history%d','var')\n plot(history%d(:,1),history%d(:,2))\n end",i,i,i)))
end

hold off

% %%Create the labels
% mnlabelx1 = sprintf('Meanx 1 -- %3.2d', mean(history1(:,1)));
% stdlabelx1 = sprintf('Std Deviation 1 -- %3.2d', var(history1(:,1)));
% mnlabely1 = sprintf('Meany 1 -- %3.2d', mean(history1(:,2)));
% stdlabely1 = sprintf('Std Deviation 1 -- %3.2d', var(history1(:,2)));
% 
% mnlabelx2 = sprintf('Meanx 2 -- %3.2d', mean(history2(:,1)));
% stdlabelx2 = sprintf('Std Deviation 2 -- %3.2d', var(history2(:,1)));
% mnlabely2 = sprintf('Meany 2 -- %3.2d', mean(history2(:,2)));
% stdlabely2 = sprintf('Std Deviation 2 -- %3.2d', var(history2(:,2)));
% 
% mnlabelx3 = sprintf('Meanx 3 -- %3.2d', mean(history3(:,1)));
% stdlabelx3 = sprintf('Std Deviation 3 -- %3.2d', var(history3(:,1)));
% mnlabely3 = sprintf('Meany 3 -- %3.2d', mean(history3(:,2)));
% stdlabely3 = sprintf('Std Deviation 3 -- %3.2d', var(history3(:,2)));
% %%%Create the textbox
% h = annotation('textbox',[0.8 0.8 0.1 0.1]);
% set(h,'String',{mnlabelx1,stdlabelx1,mnlabely1,stdlabely1, mnlabelx2,stdlabelx2,mnlabely2,stdlabely2, mnlabelx3,stdlabelx3,mnlabely3,stdlabely3});
% hold off








