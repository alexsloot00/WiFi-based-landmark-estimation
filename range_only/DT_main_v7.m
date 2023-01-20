%% Range-only landmark estimation
% (2023) Original code: Matteo Marcantoni - DTPA - University of Groningen
% (2023) Modified by:   Alex Sloot - DTPA - University of Groningen
clear; clc; close all;

%% Simulation variables
rng(1)                  % set rng seed
sim_time = 1e4;         % simulation time
time_step = 1e-1;       % sampling time 
k = 4*1e1;              % correction gain
R_rot = [0, 1; -1, 0];  % rotation matrix
u_mag = 1e-2;           % magnitude of input   
ui = [-1; -1];          % unit vector of input direction 
u0 = u_mag*ui;          % movement input
p0 = [0; 0];            % initial position
dim = length(p0);       % number of dimensions
ang_err0 = -pi;         % initial angle error 
l_rad = 15;             % distance to landmark
l_ang = deg2rad(45);    % angle between origin and landmark
l_star = l_rad*[cos(l_ang); sin(l_ang)];    % true landmark position
z_true0 = l_star - p0;                      % true landmark-robot vector
theta_true0 = atan2(z_true0(2),z_true0(1)); % true angle robot to landmark

%% Initial estimates
theta_est0 = l_ang + ang_err0;              % angle estimate
theta_est0 = angle_adjustment(theta_est0);  % angle estimate [-pi, pi]
y0 = norm(z_true0);                         % distance measurement
z_est0 = y0*[cos(theta_est0); sin(theta_est0)]; % landmark-robot vector
l_est0 = z_est0 + p0;                       % landmark estimation
v0 = z_est0/norm(z_est0);                   % unit vector of z_est0 
w0 = R_rot*v0;                              % orthogonal vector to v0 

%% Inizialize vectors
p = zeros(dim,sim_time);% agent positions
z_est = p;              % estimated relative distance
z_true = p;             % true relative distance
l_est = p;              % estimated landmark position
v = p;                  % unit vector pointing like z_est
w = p;                  % unit vector orthogonal to v
u = p;                  % input of the agent
e_l = p;                % error on landmark position
e_z = p;                % error on relative position
y = zeros(1,sim_time);  % true measurement
y_pred = y;             % predicted measurement (n-1)  
e_y = y;                % measurement error (UNUSED)
theta_true = y;         % true angle
theta_est = y;          % estimated angle
theta_update = y;       % updates (n-1)
theta_helper = y;       % variable for computation
theta_old = y;          % variable for comparing
e_theta = y;            % angle error

%% Insert initial conditions
p(:,1) = p0;            % position
z_est(:,1) = z_est0;    % estimated landmark-robot vector 
z_true(:,1) = z_true0;  % true landmark-robot vector
l_est(:,1) = l_est0;    % estimated landmark
v(:,1) = v0;            % unit vector of z_est
w(:,1) = w0;            % orthogonal vector to v
u(:,1) = u0;            % movement input
y(1) = y0;              % true distance to landmark
y_pred(1) = y0;         % predicted distance to landmark
theta_true(1) = theta_true0;    % true angle
theta_est(1) = theta_est0;      % estimated angle
theta_update(1) = 0;            % angle update value
theta_helper(1) = theta_est0;   % variable for computation
theta_old(1) = theta_est0;      % variable fr comparing
e_l(:,1) = l_star - l_est0;     % landmark error
e_z(:,1) = z_true0 - z_est0;    % landmark-robot vector error
e_y(1) = theta_update(1);       % distance to landmark error (UNUSED)
e_theta(1) = theta_true0 - theta_est0; % angle error

%% Simulate the evolution
for i=2:sim_time
    % agent is moving orthogonally to estimated landmark 
    p(:,i) = p(:,i-1) + time_step*u(:,i-1);

    % relative position vectors evolving accordingly
    z_true(:,i) = l_star - p(:,i);
    theta_true(i) = atan2(z_true(2,i),z_true(1,i));
    z_est(:,i) = l_est(:,i-1) - p(:,i);

    % prediction of measurement 
    y_pred(i) = norm(z_est(:,i));

    % measurement
    y(i) = norm(z_true(:,i)); 

    % difference between prediction and measure
    theta_update(i) = (y(i)^2 - y_pred(i)^2); 

    % update anle estimation
    theta_est(i) = theta_est(i-1) + time_step*u(:,i-1)'*w(:,i-1) + ...
                    k*sign(u(:,i-1)'*w(:,i-1))*theta_update(i);
    theta_est(i) = angle_adjustment(theta_est(i));
    
    % relative position correction
    z_est(:,i) = y(i)*[cos(theta_est(i)); sin(theta_est(i))];
    
    % landmark correction
    l_est(:,i) = z_est(:,i) + p(:,i);
    
    % unit vector for next movement
    v(:,i) = z_est(:,i)/norm(z_est(:,i));
    w(:,i) = R_rot*v(:,i);

    % next input
    u(:,i) = u0;

    % errors
    e_l(:,i) = l_star - l_est(:,i);
    e_z(:,i) = z_true(:,i) - z_est(:,i);
    e_y(i) = theta_update(i);
    e_theta(i) = theta_true(i) - theta_est(i);
end
%% Debug plot
figure;
plot(theta_true,'DisplayName','\theta_{true}(t)')
hold on; 
plot(theta_est,'DisplayName','\theta_{est}(t)')
plot(theta_update(2:end),'DisplayName','\theta_{update}(t)');
title('Angles over time')
xlabel('angle [rad]');
ylabel('t [s]');
legend; hold off;

%% Plots
% positions of true- and estimated landmark and robot over time
figure;
p(1) = plot(p(1,:),p(2,:),'r','DisplayName','p(t)'); hold on;
p(2) = plot(l_est(1,:),l_est(2,:),'bo','DisplayName','l_{est}'); hold on;
p(3) = plot(l_star(1,:),l_star(2,:),'gx','DisplayName','l^*'); hold on;
title('Landmark and agent positions over time')
xlabel('x [m]');
ylabel('y [m]');
axis equal;
legend(p(1:3)); hold off;

% errors on estimated landmark position over time
figure;
e(1) = plot(e_l(1,:),'b-','DisplayName','e^x_{l}(t)'); hold on;
e(2) = plot(e_l(2,:),'r-','DisplayName','e^y_{l}(t)'); hold on;
title('Evolution of e_{l}(t) over time')
xlabel('time [s]');
ylabel('error [m]');
legend(e(1:2)); hold off;

% errors on estimated angle over time
figure;
plot(e_theta,'r-','DisplayName','e_{\theta}(t)'); hold on;
title('Evolution of e_{\theta}(t) over time')
xlabel('time [s]');
ylabel('error [rad]');
legend; hold off;

% errors on estimated angle over time
figure;
plot(rad2deg(e_theta),'r-','DisplayName','e_{\theta}(t)'); hold on;
title('Evolution of e_{\theta}(t) over time')
xlabel('time [s]');
ylabel('error [deg]');
legend; hold off;

%% Kerstin plots
xLdist = bsxfun(@minus,l_est,l_star);
xLdist = arrayfun(@(j)norm(xLdist(:,j)),1:size(xLdist,2));

f5 = figure(5); clf(f5);
set(f5,'units','centimeters', 'color', 'white','position',[0 1 20 15],'PaperPositionMode','auto');
tiledlayout(2,1);
nexttile; hold on; grid;
title('\theta angle over trajectory');
plot(theta_true); plot(theta_est); 
nexttile; hold on; grid;
title('distance of l^* versus l_{est} over trajectory');
plot(xLdist,'k');
set(gca, 'YScale', 'log');

%% Adjust angle to -pi and pi
function out = angle_adjustment(a)
    out = a;
    if a > 2*pi
        out = out - 2*pi;
    elseif a < -2*pi
        out = out  + 2*pi;
    end
end
