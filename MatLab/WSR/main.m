%% This script runs the WSR toolbox
% (2023) Original code: Alex Sloot - DTPA - University of Groningen
% Receiver (RX) moves around
% Transmitter (TX) is stationary
clear; clc; close all;
use_real_world_data = 1;
%% define the inputs and compute the AOA profile
% Load real_world data or simulate a circular trajectory
if use_real_world_data
    file_name = 'tx1_10cm_1_packet_dist.json';
    rx_trajectory = get_json_trajectory(file_name);
    csi_data_file = 'tx1_10cm_1_sliced_channel_data.json';
else
    % Circular
    radius = 0.70;
    rx_trajectory = get_circle_trajectory(radius);
    % Straight (move -x=forward, move y=right) 
%     rx_trajectory = [-linspace(0,1,800); zeros(1,800); zeros(1,800)];
    csi_data_file = false;
end

% Stationary transmitter
tx_position = [-3.9; 1.9; 0];

% Settings
settings.simple = 0;                % boolean for 2D
settings.print = 1;                 % boolean for printing
settings.nbeta = 360;               % azimuth resolution
settings.ngamma = 30;               % elevation resolution
settings.beta_min = deg2rad(-180);  % minimum azimuth
settings.beta_max = deg2rad(180);   % maximum azimuth
settings.gamma_min = deg2rad(0);    % minimum elevation
settings.gamma_max = deg2rad(90);   % maximum elevation

% Obtain the AOA profile
[AOA_profile, betaList, gammaList, h_ij, WSRh_ij] = create_AOA(rx_trajectory, ...
                            tx_position, settings, csi_data_file);

%% Robot movement relative to the landmark
figure(2)
plot3(rx_trajectory(1,:), rx_trajectory(2,:), rx_trajectory(3,:), 'r')
hold on
plot3(tx_position(1), tx_position(2), tx_position(3), 'g*', 'LineWidth', 4)
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('robot position', 'landmark')

%% The AOA profile
figure(3);
title('found AOA from simulation')
subplot(2,1,1)
surf(rad2deg(betaList), rad2deg(gammaList), AOA_profile.', ...
                                    'EdgeColor', 'none');
set(gcf,'Renderer','Zbuffer')            
xlabel('Azimuth (Degree)');
ylabel('Elevation (Degree)');
zlim([0 max(max(AOA_profile))])
title(sprintf('AOA profile (side view)'));         
subplot(2,1,2)
surf(rad2deg(betaList), rad2deg(gammaList), AOA_profile.', ...
                                    'EdgeColor', 'none');
set(gcf,'Renderer','Zbuffer');
view(2)
title('AOA profile Top View');
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');

%% Only create phase comparison plots if real-world phase exists
if isa(WSRh_ij, 'logical')
    fprintf("\nNo real-world CSI data was provided. " + ...
        "No phase plots are created.\n")
    return
end
%% Simuated and real-world CSI data
N = length(rx_trajectory);

% Obtain amplitude CSI data values using root mean squared
real_amplitude_h_ij = rms(real(h_ij));
imag_amplitude_h_ij = rms(imag(h_ij));
real_amplitude_WSRh_ij = rms(real(WSRh_ij));
imag_amplitude_WSRh_ij = rms(imag(WSRh_ij));

% Constants to relate the two signals
real_frac = real_amplitude_WSRh_ij / real_amplitude_h_ij;
imag_frac = imag_amplitude_WSRh_ij / imag_amplitude_h_ij;
mean_frac = (real_frac + imag_frac) / 2;
%% Separate plots
figure;
subplot(2,2,1)
scatter(1:N, real(h_ij), 'x'); 
title('Formula real part')
xlabel('Number of packets')
ylabel('Value')
subplot(2,2,2)
scatter(1:N, real(WSRh_ij), 'rx')
title('WSR data real part')
xlabel('Number of packets')
ylabel('Value')
subplot(2,2,3)
scatter(1:N, imag(h_ij), 'bo'); 
title('Formula imaginary part')
xlabel('Number of packets')
ylabel('Value')
subplot(2,2,4)
scatter(1:N, imag(WSRh_ij), 'ro')
title('WSR data imaginary part')
xlabel('Number of packets')
ylabel('Value')

%% Overlayed plots
figure;
subplot(2,1,1)
scatter(1:N, real_frac .* real(h_ij), 'x'); 
hold on;
scatter(1:N, real(WSRh_ij), 'rx');
title('CSI data real part')
ylim([-1000 1000])
xlabel('Number of packets')
ylabel('Value')
legend('simulated', 'real-world')
hold off;
subplot(2,1,2)
scatter(1:N, imag_frac .* imag(h_ij), 'x'); 
hold on;
scatter(1:N, imag(WSRh_ij), 'rx'); 
title('CSI data imaginary part')
ylim([-1000 1000])
xlabel('Number of packets')
ylabel('Value')
legend('simulated', 'real-world')
hold off;
%% Real and imaginary axes
figure;
plot(mean_frac*h_ij(2:end), 'x', LineWidth=2); 
hold on; 
plot(WSRh_ij, 'rx');
title('CSI data')
xlim([-1500 1500])
ylim([-1500 1500])
xlabel('Real values')
ylabel('Imaginary values')
legend( 'simulated', 'real-world')
%% CSI data phase
figure;
scatter(1:N, angle(h_ij), 'x')
hold on;
scatter(1:N, angle(WSRh_ij), 'x')
title('CSI phase data')
ylim([-1.02*pi 1.02*pi])
xlabel('Number of packets')
ylabel('Phase (rad)')
legend( 'simulated', 'real-world')
%% Helper function for loading json data
function rx_trajectory = get_json_trajectory(fname)
    % INPUT: fname - a string of the .json filename
    % OUTPUT rx_trajectory - a [x;y;z] trajectory over time
    fid = fopen(fname); 
    str = char(fread(fid,inf)'); 
    fclose(fid); 
    val = jsondecode(str);
    fn = fieldnames(val.pose_list);
    rx_trajectory = zeros(3,numel(fn));
    for k=0:numel(fn)-1
        name = ['x' num2str(k)];
        dummy = val.pose_list.(name);
        rx_trajectory(:,k+1) = [dummy.x; dummy.y; dummy.z];
    end
end
%% Helper function for circular trajectory
function rx_trajectory = get_circle_trajectory(radius)
    % INPUT: radius - the radius of the circle
    % OUTPUT rx_trajectory - a [x;y;z] trajectory over time
    % we make the circle 800 time steps
    t = linspace(0,2*pi,800);
    x = -radius + radius*cos(t);
    y = radius*sin(t);
    z = zeros(1,800);
    rx_trajectory = [x; y; z];
end