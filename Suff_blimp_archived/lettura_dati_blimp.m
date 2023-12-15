clc;
clear all;
close all;

%% Lettura del file

fileID = fopen('C:\Volume_D\Programming\Blimp_git\log_Blimp_00_data.txt','r');
formatSpec = '%f';
sizeA = [14 Inf];

A = fscanf(fileID,formatSpec, sizeA);


time = A(1,:);
psi = A(14,:);
x_pos = A(12,:);
y_pos = A(13,:);

figure(1);
plot(time, psi)

figure(2);
plot(x_pos, y_pos, '*')