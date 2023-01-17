%% Parallel Robot
% Inverse Kinematics for a 3 DoF planar parallel robot: https://link.springer.com/chapter/10.1007/978-3-030-91892-7_21/figures/1
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
R_plat = 130; 
R_base = 290;

% Input the orientation a of the robot and the centre point of the platform
alpha = input ('Orientation of the platform: ');
x_c = input('X-coordinate of {C}: ');
y_c = input('Y-coordinate of {C}: ');

%% Points of the platform (CPP_i)
% Degree system
Platform = zeros(2, 3); % row1: X, row2: Y
for i=1:3
    Platform(1, i) = x_c - R_plat * cosd(alpha + 270 + 120*(i-1));
    Platform(2, i) = y_c - R_plat * sind(alpha + 270 + 120*(i-1));
end

%% Points of the base (BPB_i)
% Degree system
Base = zeros(2, 3); % row1: X, row2: Y
for i=1:3
    Base(1, i) = -R_base * cosd(90 + (i-1)*120);
    Base(2, i) = -R_base * sind(90 + (i-1)*120);
end

%% PB_iPP_i
PBPP = zeros(2, 3); % row1: X, row2: Y
for i=1:3
    PBPP(1, i) = Base(1, i) + Platform(1, i);
    PBPP(2, i) = Base(2, i) + Platform(2, i);
end

%% The joints connect upper and lower section (e_i)
e1 = zeros(1, 3);
e2 = zeros(1, 3);
e3 = zeros(1, 3);
theta = zeros(1, 3);

for i=1:3
    theta(i) = atan2d(PBPP(2, i), PBPP(1, i));
    e1(i) = -SA * 2 * PBPP(2, i);
    e2(i) = -SA * 2 * PBPP(1, i);
    e3(i) = PBPP(1,i)^2 + PBPP(2, i)^2 + SA^2 - L^2;

    % Degree system
    theta(i) = 2 * atan2d(-e1(i) + sqrt(e1(i)^2 + e2(i)^2 - e3(i)^2), e3(i) - e2(i));
end

%% Calculate the positions of the joints
% Degree system
Joints = zeros(2,3); % row1: X, row2: Y
for i=1:3
    Joints(1, i) = SA * cosd(theta(i)) - Base(1, i);
    Joints(2, i) = SA * sind(theta(i)) - Base(2, i);
end


%% Assign the platform
platform = [Platform(1, :) Platform(1, 1);
            Platform(2, :) Platform(2, 1)];

base = [-Base(1, :) -Base(1, 1);
        -Base(2, :) -Base(2, 1)];

% Links
link_1 = [-Base(1, 1) Joints(1, 1) platform(1, 1);
          -Base(2, 1) Joints(2, 1) platform(2, 1)];

link_2 = [-Base(1, 2) Joints(1, 2) platform(1, 2);
          -Base(2, 2) Joints(2, 2) platform(2, 2)];

link_3 = [-Base(1, 3) Joints(1, 3) platform(1, 3);
          -Base(2, 3) Joints(2, 3) platform(2, 3)];

%% Plot the kinematic model in two positions
% Platform——blue
% Base——red
% Links——black

plot(x_c, y_c, 'blue*') % {C}
hold on % keep the drawing
plot(0, 0, 'red*') % {B}

line(platform(1, :), platform(2, :), 'Color', 'blue', 'linewidth', 2) % enclose platform
line(base(1, :), base(2, :), 'Color', 'red', 'linewidth', 2) % enclose base

% Plot links
plot(link_1(1, :), link_1(2, :), 'k-o', 'MarkerSize', 3, 'linewidth', 1)
plot(link_2(1, :), link_2(2, :), 'k-o', 'MarkerSize', 3, 'linewidth', 1)
plot(link_3(1, :), link_3(2, :), 'k-o', 'MarkerSize', 3, 'linewidth', 1)

grid on
axis equal
