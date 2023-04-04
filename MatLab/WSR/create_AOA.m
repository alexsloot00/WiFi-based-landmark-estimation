function [AOA_profile, betaList, gammaList, h_ij, WSRh_ij] = create_AOA(rx_trajectory, ...
                              tx_position, settings, csi_data_file)
    % (2023) Original code: Alex Sloot - DTPA - University of Groningen

    % This function uses the bartlett_AOA_estimator() to estimate an AOA
    % profile based on the given trajectory. The CSI data can be
    % measured and given as an input, if csi_data_file is not provided the
    % function will emulate CSI data. The receiver (RX) is represented as
    % robot i and the transmitter (TX) is represented by robot j.

    % INPUTS:  rx_trajectory - receiver trajectory [x;y;z] over time
    %          tx_position   - stationary transmitter [x;y;z] position 
    %          settings      - additional accuracy settings
    %          csi_data_file - string name of .json file with CSI data *
    %                        * means the input is optional
        
    % OUTPUTS: AOA_profile   - AOA profile by the receiver (360x180)
    %          betaList      - list of azimuth angles (-180:180)
    %          gammaList     - list of elevation angles (0:90)

    % Check optional parameter
    if ~exist('csi_data_file','var')
         csi_data_file = false;   
    end

    % Settings variables
    print = settings.print;             % boolean for 2D computation
    simple = settings.simple;           % boolean for printing        
    nbeta = settings.nbeta;             % azimuth resolution
    ngamma = settings.ngamma;           % elevation resolution
    beta_min = settings.beta_min;       % minimum azimuth
    beta_max = settings.beta_max;       % maximum azimuth
    gamma_min = settings.gamma_min;     % minimum elevation
    gamma_max = settings.gamma_max;     % maximum elevation
    betaList = linspace(beta_min, beta_max, nbeta).';      % azimuth 
    gammaList = linspace(gamma_min, gamma_max, ngamma);    % elevation 

    % Load the CSI data from a dataset (if provided)
    if csi_data_file
        fname = csi_data_file;
        fid = fopen(fname); 
        str = char(fread(fid,inf)'); 
        fclose(fid); 
        val = jsondecode(str);
        fn = fieldnames(val.channel_packets);
        WSRh_ij = zeros(1,numel(fn));
        for k=0:numel(fn)-1
            name = ['x' num2str(k)];
            dummy = val.channel_packets.(name);
            WSRh_ij(k+1) = dummy.real + 1i*dummy.imag;
        end
        M = length(WSRh_ij);            % number of packets
    else
        M = length(rx_trajectory);      % number of packets
        WSRh_ij = false;                % needs an output value
    end

    % Variables
    p_i = rx_trajectory;                % list of robot i's position
    p_j = tx_position;                  % stationary robot j position
    N = M;                              % number of packets to actually use
    p_i = p_i(:, 1:N);                  % to run with less iterations
    centerfreq = 5.54e9;                % Channel 108
    c = 3e8;                            % Speed of light = 3e8 m/s
    lambda = c/centerfreq;              % WiFi wavelength is roughly 6cm      
    
    % Initialize vectors
    p_i0 = p_i(:,1);                    % initial position robot i
    rho_kl = zeros(1,N);                % distance vector p_i and p_i0
    phi_kl = zeros(1,N);                % angle robot j and X-axis
    E_kl = zeros(1,N);                  % angle robot j and Z-axis
    d_ij = zeros(1,N);                  % distance vector p_i and p_j
    h_ij = zeros(N,1);                  % CSI data (complex values)

    % Initial positions and distance vectors
    X_ij(:,1) = -p_i(:,1) + p_j;        % [x,y,z] distance robot i and j
    d_ij(1) = norm(X_ij(:,1));          % distance between robot i and j
    rho_kl(1) = 0;                      % initial distance to self is 0
    phi_kl(1) = deg2rad(90);            % reference frame makes it 90
    E_kl(1) = atan2(p_i0(3), norm(p_i0(1:2)-p_i0(1:2)));  % pitch
    
    % Loop over t from tk:tl
    for t = 2:N        
        % azimuth angle robot i and p_i0 
        phi_kl(t) = atan2(p_i(2,t)-p_i0(2), p_i(1,t)-p_i0(1));
        
        % elevation angle (0 in 2D) robot i and p_i0 
        E_kl(t) = atan2(p_i(3,t), norm(p_i(1:2,t)-p_i0(1:2)));
        
        % [x,y,z] vector between robot i and j (pj-pi not pi-pj)
        X_ij(:,t) = -p_i(:,t) + p_j;  

        % distance between robot i and j 
        d_ij(t) = norm(X_ij(:,t));  
        
        % [x,y,z] distance vector p_i and p_i0 
        rho_kl(t) = norm(p_i(:,t)-p_i0(:));

        % compute h_ij(t)
        h_ij(t) = 1/d_ij(t) * exp((-2*pi*1i)/lambda*d_ij(t));
    end

    % transpose and choose which lists to use for AOA computation
    if csi_data_file
        hrList = WSRh_ij.';
    else
        hrList = h_ij;
    end
    yawList = phi_kl.';
    pitchList  = E_kl.';
    rhoList = rho_kl.';
    
    % Compute the AOA profile
    AOA_profile = bartlett_AOA_estimator(hrList, yawList, pitchList, ...
        rhoList, lambda, betaList, gammaList, nbeta, ngamma, simple);
    
    % Find the peak and display the found and true angles
    peak = max(max(AOA_profile));
    [idphi,idtheta]=find(AOA_profile==peak, 1, 'first');
    phi = betaList(idphi);
    theta = gammaList(idtheta);
    if simple
        theta = deg2rad(90);
    end
    
    % True azimuth and elevation, respectively 
    phi_g = atan2(X_ij(2,end), X_ij(1,end));                  
    theta_g = deg2rad(90) - atan2(X_ij(3,end), norm(X_ij(1:2,end))); 

    % convert to degrees
    theta_true = rad2deg(theta_g);
    phi_true = rad2deg(phi_g);
    theta_predict = rad2deg(theta);
    phi_predict = rad2deg(phi);
    
    % Print the values
    if print
        % Print the angle estimates and true values
        fprintf("The true and found angles (degrees) are:\n")
        fprintf("\t\t True \t\t Found \t\t Error\n")
        fprintf("Azimuth: \t % .2f \t % .2f \t % .2f\n", ...
                phi_true, phi_predict, phi_true-phi_predict)
        fprintf("Elevation: \t % .2f \t % .2f \t % .2f\n\n", ...
            theta_true, theta_predict, theta_true-theta_predict)
    
        % z = cos(theta), because theta is defined as 90 - normal_theta
        tx_position_estimate = d_ij(end) * [cos(phi); ...
                    sin(phi); cos(theta)] + p_i0;
        tx_position_true = d_ij(end) * [cos(phi_g); ...
                    sin(phi_g); cos(theta_g)] + p_i0;

        % Print the position estimates and true values
        fprintf("The true and found estimated position of TX are:\n")
        fprintf("\t\t True \t\t Found \t\t Error\n")
        fprintf("\tx: \t % .2f \t\t % .2f \t\t % .2f\n", ...
            tx_position_true(1), tx_position_estimate(1), ...
            tx_position_true(1)-tx_position_estimate(1))
        fprintf("\ty: \t % .2f \t\t % .2f \t\t % .2f\n", ...
            tx_position_true(2), tx_position_estimate(2), ...
            tx_position_true(2)-tx_position_estimate(2))
        fprintf("\tz: \t % .2f \t\t % .2f \t\t % .2f\n", ...
            tx_position_true(3), tx_position_estimate(3), ...
            tx_position_true(3)-tx_position_estimate(3))    
    end
end