%% This script runs the WSR framework for a 3D helix
% Author: Alex Sloot, University of Groningen
% robot_i position: Helix motion
% robot_j position: stationary
clear; clc; close all;
print = 0;
%% Time variables 
tk = 0;                 % start time
tl = 200;               % end time  %%NJ: Need large number of packets
M = tl - tk;            % number of packets
N = 1 + M;              % easier notation for index

%% Variables
% check wikipedia for frequency of different wifi channels: 
% https://en.wikipedia.org/wiki/List_of_WLAN_channels#5_GHz_(802.11a/h/j/n/ac/ax)
centerfreq = 5.52e9;    % Channel 104
theta_predict = 3e8;    % Speed of light = 3e8 m/s
lambda = theta_predict./centerfreq; % WiFi wavelength is roughly 6cm      
% delta_f = 0;            % frequency offset, assume = 0 at first

% robot i and j initial positions and distance vectors
p_i0 = [0.5;0.01;0];    % initial position receiving robot i position
p_i = zeros(3,N);       % initialized vector of zeros
rho_kl = zeros(1,N);    % distance vector p_i and p_i0
phi_kl = zeros(1,N);    % angle transmitting robot j and X-axis
E_kl = zeros(1,N);      % angle transmitting robot j and Z-axis
theta_g = zeros(1,N);   % groundtruth theta
phi_g = zeros(1,N);     % groundtruth phi

% t = 1 (initial values)
p_i(:,1) = p_i0;        % initial position robot i 
p_j(:,1) = [1; 20; 1]; % transmitting robot j position   %%NJ- The distance between transmitting and receiving robot should be substantial for if the displacement of the receiving robot is large
X_ij(:,1) = -p_i(:,1) + p_j(:,1); % [x,y,z] distance robot i and j
phi_g(1,:) = atan2(X_ij(2,1), X_ij(1,1)); % elevation
theta_g(1,:) = atan2(X_ij(3,1), norm(X_ij(1:2,1))); % azimuth
d_ij = norm(X_ij(:,1)); % distance between robot i and j
% rho_kl(1) = norm(p_i0(:)-p_i0(:)); = 0

% initialize AOA and h_ij terms
% AOA = zeros(360,180);
% AOA_true = zeros(360,180);
% AOA_true1 = zeros(360,180);
h_ij = zeros(N,1);
% h_ij_rev = zeros(N,1);
% h_ij_hat = zeros(N,1);
% h_ij_hat_rev = zeros(N,1);

%NJ- added step
step = 100;
step_elevation = 0.1/N ; %%NJ: limit the elevation displacement to 0.1 meters
elevation = 0;

%% Loop over t from tk:tl
for t = 2:N
    % next position robot i and j over tk:tl
    p_i(:,t) = [cos(t/step)/2; sin(t/step); elevation];
%     p_i(:,t) = p_i0 + [t/step; t/step; 0];
    p_j(:,t) = [1; 20; 1];
    elevation = elevation + step_elevation;
    
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

    % No need to simulate h_ij_hat since its only used for cancelling 
    % Carrier Frequency Offset for real data.
%     h_ij_rev = 1/d_ij(t) * exp((-2*pi*1i)/lambda*d_ij(t)); 
%     h_ij_hat(t) = h_ij(t);
%     h_ij_hat_rev(t) = 1; % first simple version            
%     h_ij_hat(t) = h_ij(t) * exp(-2*pi*delta_f*(t-tk)); 
%     h_ij_hat_rev(t) = h_ij_rev * exp(2*pi*delta_f*(t-tk));  
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
[betaProfile] = bartlett_AOA_estimator(hrList, yawList, ...
    pitchList, rhoList, lambda, betaList, gammaList, nbeta, ngamma);
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
plot3(p_j(1), p_j(2), p_j(3), 'g*', 'LineWidth', 4)
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('robot position', 'landmark')

% The AOA profile
figure(3);
subplot(2,1,1)
surf(rad2deg(betaList), rad2deg(gammaList), betaProfile.', 'EdgeColor', 'none');
set(gcf,'Renderer','Zbuffer')            
xlabel('Azimuth (Degree)');
ylabel('Elevation (Degree)');
title(sprintf('AOA profile (side view)'));         
subplot(2,1,2)
surf(rad2deg(betaList), rad2deg(gammaList), betaProfile.', 'EdgeColor', 'none');
set(gcf,'Renderer','Zbuffer');
view(2)
title('AOA profile Top View');
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');

%% Find the peak and display the found and true angles
peak = max(max(betaProfile));
[idphi,idtheta]=find(betaProfile==peak, 1, 'first');
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
    pos1 = dij1 * [cos(phi_g(1)); sin(phi_g(1)); sin(theta_g(1))] + p_i0
    
    dije = norm(p_i(:,end)-p_j(:,end));
    posend_predict = dij1 * [cos(phi_predict); sin(phi_predict); sin(theta_predict)] + p_i(:,end)
    posend = dije * [cos(phi_g(end)); sin(phi_g(end)); sin(theta_g(end))] + p_i(:,end)
end
 
% a = get_a(tk+1, tl+1, rho_kl, theta_g, phi_g, lambda);
% AOA = get_AOA_profile(tk+1, tl+1, h_ij, ones(size(h_ij)), a);
% figure(4)
% surf(AOA)
% 
% function a = get_a(tk, tl, p_kl, E_kl, phi_kl, lambda)
%     % This function computes the steering angle at a time range from tk
%     % till tl
%     % p_kl   is [1, tk:tl]
%     % E_kl   is [1, tk:tl]
%     % phi_kl is [1, tk:tl]
%     % lambda is [1, 1]
%     a = zeros(360,180,401);
%     for phi = 1:360
%         for theta = 1:180
%             for t = tk:tl
%                 a(phi, theta, t) = exp((2*pi*p_kl(t)*1i)/lambda * sin(theta)*sin(E_kl(t)).*cos((phi-181)-phi_kl(t))+cos(E_kl(t)).*cos(theta));
%             end
%         end
%     end
%     % vector form?
% %     a = exp((2*pi.*p_kl*1i)/lambda .* sin(theta).*sin(E_kl).*cos(phi-phi_kl)+cos(E_kl).*cos(theta));
% end
% 
% function AOA = get_AOA_profile(tk, tl, h_ij_hat, h_ij_hat_rev, a)
%     % This function computes the AOA profile at a certain time
%     dummy = zeros(360,180); % matrix of [360,1] as only theta = 0 exists;
%     % the sum part of the equation
%     for t = tk:tl
%         dummy = dummy + h_ij_hat(t) * h_ij_hat_rev(t) * a(:,:,t);
%     end
%     AOA = abs(dummy).^2;
% end
