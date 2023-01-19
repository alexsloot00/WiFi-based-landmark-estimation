%% This script runs the WSR framework for a 3D helix
% Author: Alex Sloot, University of Groningen
% robot_i position: Helix motion
% robot_j position: stationary
clear; clc; close all;
print = 0;
simple = 0;
%% Time variables 
tk = 0;                 % start time
tl = 300;               % end time 
M = tl - tk;            % number of packets
N = 1 + M;              % easier notation for index

%% Variables
% check wikipedia for frequency of different wifi channels: 
% https://en.wikipedia.org/wiki/List_of_WLAN_channels#5_GHz_(802.11a/h/j/n/ac/ax)
centerfreq = 5.52e9;    % Channel 104
theta_predict = 3e8;    % Speed of light = 3e8 m/s
lambda = theta_predict./centerfreq; % WiFi wavelength is roughly 6cm      

% robot i and j initial positions and distance vectors
p_i = readmatrix('rx_trajectory_2021-06-27_202550_.csv');
p_i = p_i(tk+1:tl+1,3:5).';
p_i0 = p_i(:,1);    % initial position receiving robot i position
% p_i = zeros(3,N);     % initialized vector of zeros
rho_kl = zeros(1,N);    % distance vector p_i and p_i0
phi_kl = zeros(1,N);    % angle transmitting robot j and X-axis
E_kl = zeros(1,N);      % angle transmitting robot j and Z-axis
theta_g = zeros(1,N);   % groundtruth theta
phi_g = zeros(1,N);     % groundtruth phi

% t = 1 (initial values)
p_i(:,1) = p_i0;        % initial position robot i 
p_j(:,1) = [-2.42; 2.65; 0.72]; % transmitting robot j position   %%NJ- The distance between transmitting and receiving robot should be substantial for if the displacement of the receiving robot is large
X_ij(:,1) = -p_i(:,1) + p_j(:,1); % [x,y,z] distance robot i and j
phi_g(1,:) = atan2(X_ij(2,1), X_ij(1,1)); % elevation
theta_g(1,:) = atan2(X_ij(3,1), norm(X_ij(1:2,1))); % azimuth
d_ij = norm(X_ij(:,1)); % distance between robot i and j
% rho_kl(1) = norm(p_i0(:)-p_i0(:)); = 0

% initialize AOA and h_ij terms
h_ij = zeros(N,1);

%NJ- added step
step = 100;
step_elevation = 0.1/N ; %%NJ: limit the elevation displacement to 0.1 meters
elevation = 0;

%% Loop over t from tk:tl
for t = 2:N
    % next position robot i and j over tk:tl
%     p_i(:,t) = [cos(t/step)/2; sin(t/step); elevation];
%     p_i(:,t) = p_i0 + [t/step; t/step; 0];
    p_j(:,t) = [-2.42; 2.65; 0.72];
%     elevation = elevation + step_elevation;
    
    % azimuth angle robot i and p_i0 over tk:tl
    phi_kl(t) = atan2(p_i(2,t)-p_i0(2), p_i(1,t)-p_i0(1));
    
    % elevation angle robot i and p_i0 over tk:tl
    E_kl(t) = atan2(norm(p_i(1:2,t)-p_i0(1:2)), p_i(3,t));
    
    % [x,y,z] vector between robot i and j over tk:tl (pj-pi not pi-pj)
    X_ij(:,t) = -p_i(:,t) + p_j(:,t);  
    % distance between robot i and j over tk:tl
    d_ij(t) = norm(X_ij(:,t));  
    % [x,y,z] distance vector p_i and p_i0 over tk:tl
    rho_kl(t) = norm(p_i(:,t)-p_i0(:));

    % groundtruth angles, index 1 shows angle between p_i0 and p_j
    phi_g(t) = atan2(X_ij(2,t), X_ij(1,t)); % true angle p_i and p_j
    theta_g(t) = atan2(X_ij(3,t), norm(X_ij(1:2,t))); % true angle p_i and p_j 
    %theta_g = acos(X_ij(3,1)/d_ij(1)) same thing as line above

    % compute h_ij(t), normally gather data with the CSI toolbox
    h_ij(t) = 1/d_ij(t) * exp((-2*pi*1i)/lambda*d_ij(t));
end

%% Compute the AOA profile
nbeta = 360;%180;
ngamma = 180; %90;
beta_min = deg2rad(-180);%-pi;
beta_max = deg2rad(180);%pi;
gamma_min = deg2rad(0);%0*pi/180;
gamma_max = deg2rad(180);%90*pi/180; %70*pi/180 % 80*pi/180; % pi/2 - 10*pi/180;

betaList = linspace(beta_min, beta_max, nbeta).';
gammaList = linspace(gamma_min, gamma_max, ngamma);

hrList = h_ij;
yawList = phi_kl.';
pitchList  = E_kl.';
rhoList = rho_kl.';

% Creates the AOA profile
AOA_profile = bartlett_AOA_estimator(hrList, yawList, ...
    pitchList, rhoList, lambda, betaList, gammaList, nbeta, ngamma, simple);
%% Plots 
% the signal phase
size_val_hlist = length(h_ij);
x_href = 1:size_val_hlist;
figure(1)
scatter(x_href, angle(h_ij), 'x');
title('Antenna 1 Signal Phase');
xlabel('Packets');
ylabel('Phase (radians)');

% Robot movement relative to the landmark
figure(2)
plot3(p_i(1,:), p_i(2,:), p_i(3,:), 'r')
hold on
plot3(p_j(1,1), p_j(2,1), p_j(3,1), 'g*', 'LineWidth', 4)
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('robot position', 'landmark')

% The AOA profile
figure(3);
subplot(2,1,1)
surf(rad2deg(betaList), rad2deg(gammaList), AOA_profile.', 'EdgeColor', 'none');
set(gcf,'Renderer','Zbuffer')            
xlabel('Azimuth (Degree)');
ylabel('Elevation (Degree)');
title(sprintf('AOA profile (side view)'));         
subplot(2,1,2)
surf(rad2deg(betaList), rad2deg(gammaList), AOA_profile.', 'EdgeColor', 'none');
set(gcf,'Renderer','Zbuffer');
view(2)
title('AOA profile Top View');
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');

%% Find the peak and display the found and true angles
peak = max(max(AOA_profile));
[idphi,idtheta]=find(AOA_profile==peak, 1, 'first');
phi = betaList(idphi);
theta = gammaList(idtheta);

% convert to degrees
theta_true = rad2deg(theta_g(end));
phi_true = rad2deg(phi_g(end));
theta_predict = rad2deg(theta);
phi_predict = rad2deg(phi);

% print the values
fprintf("The true and found angles (degrees) are:\n")
fprintf("\t\t True \t\t Found \t\t Error\n")
fprintf("Azimuth: \t %.f \t\t %.f \t\t %.f\n", phi_true, phi_predict, phi_true-phi_predict)
fprintf("Elevation: \t %.f \t\t %.f \t\t %.f\n", theta_true, theta_predict, theta_true-theta_predict)
%% test if true angles give correct estimate
if print
    dij1 = norm(p_i0(:)-p_j(:,1));
    pos1_predict = dij1 * [cos(phi_predict); sin(phi_predict); sin(theta_predict)] + p_i0
    pos1 = dij1 * [cos(phi_g(1)); sin(phi_g(1)); sin(theta_g(1))] + p_i07264*real(h_ij) - WSRreal
    
    dije = norm(p_i(:,end)-p_j(:,end));
    posend_predict = dij1 * [cos(phi_predict); sin(phi_predict); sin(theta_predict)] + p_i(:,end)
    posend = dije * [cos(phi_g(end)); sin(phi_g(end)); sin(theta_g(end))] + p_i(:,end)
end

%% The original phase from the dataset
% fname = 'jsondata.json'; % all 1543 packets
fname = 'jsondata_sliced.json'; % roughly 870 packets
fid = fopen(fname); 
str = char(fread(fid,inf)'); 
fclose(fid); 
val = jsondecode(str);

fn = fieldnames(val.channel_packets);
WSR_CSI = zeros(1,numel(fn));
for k=0:numel(fn)-1
    name = ['x' num2str(k)];
    dummy = val.channel_packets.(name);
    WSR_CSI(k+1) = dummy.center_subcarrier_phase;
    % Comment this out when using 'jsondata.json'
    WSRreal(k+1) = dummy.real;
    WSRimag(k+1) = dummy.imag;
    WSRh_ij(k+1) = WSRreal(k+1) + 1i*WSRimag(k+1);
end

% the signal phase
figure(7)
scatter(1:length(WSR_CSI), WSR_CSI, 'x', 'LineWidth', 1.5);
title('Antenna 1 Signal Phase');
xlabel('Packets');
ylabel('Phase (radians)');

%% Calculate AOA profile with WSR h_ij data
% hrList = WSRh_ij(1:N).';
% AOA_profile = bartlett_AOA_estimator(hrList, yawList, ...
%     pitchList, rhoList, lambda, betaList, gammaList, nbeta, ngamma, simple);

%% Plotting h_ij from formula vs WSRh_ij
figure(8)
subplot(2,2,1)
scatter(1:N, real(h_ij), 'x'); 
title('Formula real part')
xlabel('Number of packets')
ylabel('Value')

subplot(2,2,2)
scatter(1:N, WSRreal(1:N), 'rx')
title('WSR data real part')
xlabel('Number of packets')
ylabel('Value')

subplot(2,2,3)
scatter(1:N, imag(h_ij), 'bo'); 
title('Formula imaginary part')
xlabel('Number of packets')
ylabel('Value')

subplot(2,2,4)
scatter(1:N, WSRimag(1:N), 'ro')
title('WSR data imaginary part')
xlabel('Number of packets')
ylabel('Value')
%% Better when: h_ij*7264 { =norm(WSRreal)/norm(real(h_ij)) }
figure(8)
subplot(2,1,1)
scatter(1:N, 7264*real(h_ij), 'x');
hold on;
scatter(1:N, WSRreal(1:N), 'rx')
title('Real part')
legend('Formula', 'WSR data')
xlabel('Number of packets')
ylabel('Value')

subplot(2,1,2)
scatter(1:N, 7264*imag(h_ij), 'bo'); 
hold on;
scatter(1:N, WSRimag(1:N), 'ro')
title('Imaginary part')
legend('Formula', 'WSR data')
xlabel('Number of packets')
ylabel('Value')
%% The AOA profile from the C++ WSR toolbox
WSR_AOA = readmatrix('aoa_WSR.csv');
gammaList2 = linspace(gamma_min, 90, 90);
figure(6);
subplot(2,1,1)
surf(rad2deg(betaList), rad2deg(gammaList2), WSR_AOA.', 'EdgeColor', 'none');
set(gcf,'Renderer','Zbuffer')            
xlabel('Azimuth (Degree)');
ylabel('Elevation (Degree)');
title(sprintf('AOA profile (side view)'));         
subplot(2,1,2)
surf(rad2deg(betaList), rad2deg(gammaList2), WSR_AOA.', 'EdgeColor', 'none');
set(gcf,'Renderer','Zbuffer');
view(2)
title('AOA profile Top View');
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');
