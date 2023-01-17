%% Parallel Robot
% Plot workspace for a given orientation alpha
% Degree system
% Author: Tunwu Li

%%
clear
clc
close all

%% Initialization
% mm
SA = 170;  
L = 130; 
r = 130; 
R = 290;

% Input the orientation of the platform (alpha)
a = input ('Orientation of the platform: ');

%% Points of the base (BPB_i)
% Radian system
Base = zeros(2, 3); % row1: X, row2: Y
for i=1:3
    Base(1, i) = -R * cosd(90 + (i-1)*120);
    Base(2, i) = -R * sind(90 + (i-1)*120);
end

%% Coordinates of {C}
x_c = -150 : 6 : 150;
y_c = -150 : 6 : 150;

%% Preallocate space
platform = zeros(2,3);
e_1 = zeros(1,3);
e_2 = zeros(1,3);
e_3 = zeros(1,3);
t = zeros(1,3);
theta = zeros(1,3);
PBPP = zeros(2,3);

n = 1;
points = zeros(2, 2*180);

%% Calculate possible angles theta_i
% Discard the imaginary number angle
for j=1:length(x_c)
    for k=1:length(y_c)
        for i=1:3 
            platform(1, i) = x_c(j) - r * cosd(270 + a + 120*(i-1));
            platform(2, i) = y_c(k) - r * sind(270 + a + 120*(i-1));

            PBPP(1, i) = Base(1, i) + platform(1, i);
            PBPP(2, i) = Base(2, i) + platform(2, i);

            e_1(i) = -2 * PBPP(2, i) * SA;
            e_2(i) = -2 * PBPP(1, i) * SA;
            e_3(i) = PBPP(1, i)^2 + PBPP(2, i)^2 + SA^2 - L^2;

            t(i) = (-e_1(i) - sqrt(e_1(i)^2 + e_2(i)^2 - e_3(i)^2)) / (e_3(i)-e_2(i));
            theta(i) = 2 * atand(t(i));
        end
        
        % Determines whether theta is a real number
        if abs(imag(theta)) == 0
            points(1, n) = x_c(j);
            points(2, n) = y_c(k);
            n = n + 1;
        end
    end
end

%% Plot workspace
plot(0, 0, 'red*') % {B}
hold on

base=[-Base(1, :) -Base(1, 1);
      -Base(2, :) -Base(2, 1)];
line(base(1,:),base(2,:), 'Color', 'red', 'linewidth', 2); % enclose base

% workspace
if points ==0
else
    scatter(points(1, :), points(2, :),'b.')
end

grid on
axis equal












