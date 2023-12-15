% Author:   Luca Santoro: luca.santoro@unitn.it
%           Matteo Nardello: matteo.nardello@unitn.it
% Created:  12.02.2021
%
% University of Trento

% Clean workspace
clc
clear all
close all
set(0,'DefaultFigureWindowStyle','Docked');

%% ----------Structure initialization-----------------------------------------------------
% Define squared fence dimensions
type = 'p';
center = [2.5 5 1.5];
dx = 5;
dy = 10;
dz = 3;
r = sqrt(dx^2 + dy^2);
% Define how many anchor are connected
range_T = 7;
range_S = 1.5;
% Define speed of light
c = 299792458;
% Generate fence area
%out = generate_fance(type,r,center,dx,dy,dz);
%bond = boundary(out',0);
% A_n1 = [0.00; 0.00; 2];
% A_n2 = [3.80; 0.00; 2];
% A_n3 = [2.00; 3.30; 2];
% A_n4 = [5.40; 4.90; 2];
% A_n5 = [8.80; 3.30; 2];
% A_n6  = [2.00; 7.40; 2];

A_n1 = [0.00; 7.19; 2.15];
A_n2 = [0.00; 3.62; 3.15];
A_n3 = [0.00; 0.00; 2.15];
A_n4 = [4.79; 1.85; 3.15];
A_n5 = [4.79; 5.45; 2.15];
A_n6  = [3.00; 9.35; 3.15];
% A_n = [A_n1 A_n2 A_n3 A_n4 A_n5 A_n6];
A_n = [A_n1 A_n2 A_n3 A_n4 A_n5 A_n6];
% Define Reference anchors coordinates matrices

n = length(A_n);
% ID tags
tags = [1,2,3];


%% -----------MQTT Data Aquisition---------------------------------------------------------
myMQTT = mqtt('tcp://192.168.1.200');


for i=1:length(tags)
    eval(sprintf("tag%d = subscribe(myMQTT,'DS_TWR/%d');",tags(i),tags(i)));
    eval(sprintf("P_Tx%d = [];",tags(i)));
    eval(sprintf("P_Ty%d = [];",tags(i)));
    eval(sprintf("P_Tz%d = [];",tags(i)));
    eval(sprintf("indx%d = 1;", tags(i)));
    eval(sprintf("run%d = 0;", tags(i)));
    eval(sprintf("anch_range%d = [];", tags(i)));
    eval(sprintf("anch_range_old%d = [];", tags(i)));
end

%Figure control handle
DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
    'String', 'Stop', ...
    'Callback', 'delete(gcbf)');
% Set plot
plot3(A_n(1,:),A_n(2,:), A_n(3,:),'o','MarkerSize',10,'MarkerEdgeColor','blue','MarkerFaceColor',[.1 .2 .8]);
hold on
text(A_n(1,1), A_n(2,1), A_n(3,1)+0.3, 'A_n_1');
text(A_n(1,2), A_n(2,2), A_n(3,2)+0.3, 'A_n_2');
text(A_n(1,3), A_n(2,3), A_n(3,3)+0.3, 'A_n_3');
text(A_n(1,4), A_n(2,4), A_n(3,4)+0.3, 'A_n_4');
text(A_n(1,5), A_n(2,5), A_n(3,5)+0.3, 'A_n_5');
text(A_n(1,6), A_n(2,6), A_n(3,6)+0.3, 'A_n_6');
grid on
axis equal
xlabel('x-axis'); ylabel('y-axis');
xlim([-range_T 2*range_T]);
ylim([-0.4*range_T 2.2*range_T]);
zlim([0 range_T+2]);
%% ------------------------------TWO WAY RANGING----------------------------------
while(ishandle(H))
    for i=1:length(tags)
        try
            eval(sprintf("res = tag%d.read();",tags(i)));
            eval(sprintf("anch_range%d = str2double(jsondecode(res));",tags(i)))
            if (eval(sprintf("~isequal(anch_range%d,anch_range_old%d)", tags(i), tags(i))))
                x_t = eval(sprintf("trilateration(anch_range%d, A_n');", tags(i)));
%                 x_t = eval(sprintf("tdoa_test(ts%d, c, A_n, A_n3);", tags(i)));
                sprintf("position tag%d: %d %d %d\r\n", tags(i), x_t(1), x_t(2), x_t(3))
                eval(sprintf("P_Tx%d = [P_Tx%d; x_t(1)];", tags(i), tags(i)));
                eval(sprintf("P_Ty%d = [P_Ty%d; x_t(2)];", tags(i), tags(i)));
                eval(sprintf("P_Tz%d = [P_Tz%d; x_t(3)];", tags(i), tags(i)));
                trisurf(bond,out(1,:),out(2,:),out(3,:),'Facecolor','red','FaceAlpha',0.1)
                eval(sprintf("h%d(1,indx%d) = plot3(x_t(1), x_t(2), 1.7,'o','MarkerSize',7,'MarkerFaceColor',[.8 .2 .1]);", tags(i), tags(i)))
                eval(sprintf("g%d(1,indx%d) = text(x_t(1), x_t(2), 1.7,'Tag%d','Color','blue','FontSize',10);", tags(i), tags(i), tags(i)));
                pause(.1)
                
                if eval(sprintf("run%d == 1", tags(i)))
                    eval(sprintf("delete(h%d(1,indx%d-1));", tags(i), tags(i)));
                    eval(sprintf("delete(g%d(1,indx%d-1));", tags(i), tags(i)));
                end
                eval(sprintf("indx%d = indx%d + 1;", tags(i), tags(i)));
                eval(sprintf("anch_range_old%d = anch_range%d;", tags(i), tags(i)));
            end
            eval(sprintf("run%d = 1;", tags(i))); 
            eval(sprintf("anch_range_old%d = ts%d;", tags(i), tags(i)));
        catch

        end
    end
    pause(0.001) 
end

%% % ---------------TRAJECTORY PLOT-------------------------------------------------------------------------------------
% 
figure('Name','Trajectory')
% plot3(A_n(1,:),A_n(2,:), A_n(3,:),'o','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor',[.1 .2 .8]);
plot(A_n(1,:),A_n(2,:),'o','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor',[.1 .2 .8]);

hold on
% plot3(P_Tx1,P_Ty1,P_Tz1,'-o')
plot(movmean(P_Tx1,3),movmean(P_Ty1,3))
% plot(P_Tx2,P_Ty2,'-o')
grid on
xlabel('x-axis'); ylabel('y-axis'); %zlabel('z-axis');