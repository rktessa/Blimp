function [pos,C] = trilateration(anch_range, anch_loc)
A = zeros(3,3); 
b = zeros(3,1);
[m,n] = size(anch_loc);
x = anch_loc(:,1); y = anch_loc(:,2); z = anch_loc(:,3); %Position of the anchors
for i=2:m
    A(i-1,:) = [x(i)-x(1), y(i)-y(1), z(i)-z(1)];
     b(i-1,:) = anch_range(1)^2 - anch_range(i)^2 + x(i)^2 + y(i)^2 + z(i)^2 - x(1)^2 - y(1)^2 - z(1)^2;
end
POS = pinv(A)*b/2; %Least Square Method, transform into WLS
pos = [POS(1) POS(2) POS(3)];