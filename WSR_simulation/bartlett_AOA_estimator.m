function profile = bartlett_AOA_estimator(hrList, yawList, pitchList, rhoList, lambda, betaList, gammaList, nbeta, ngamma, simple)          
    % (2022) Original code: Ninad Jadhav - REACT LAB - Harvard University
    %                       Weiying Wang - REACT LAB - Harvard University
    % (2023) Modified by:   Alex Sloot - University of Groningen

    % This function estimates an AOA profile with the resolution of
    % (nbeta x ngamma) using 1 subcarrier. The peak of the AOA refers to 
    % the azimuth and elevation angle between the transmitter (tx) and 
    % receiver (rx). rx and tx may be swapped based on which is moving.

    % INPUTS: hrList    - contains CSI data as complex values
    %         yawList   - azimuth angles of rx and rx's initial position
    %         pitchList - elevation angles of rx and rx's initial position
    %         rhoList   - distance between rx and rx's initial position 
    %         lambda    - the wavelength of the WiFi signal
    %         betaList  - contains all values for azimuth to run
    %         gammaList - contains all values for elevation to run
    %         nbeta     - number of elements in betaList
    %         ngamma    - number of elements in gammaList
    %         simple    - optional boolean for 2D easier computation
                    
    % OUTPUT: profile   - the AOA profile between tx and rx

    % Check optional parameter 
    if ~exist('simple','var')
         simple = false;   
    end    

    % If simple change computation method (only gamma = 0:1)
    if simple
        ngamma_old = ngamma;
         ngamma = 2;
         gammaList = linspace(0, 1, ngamma);     
    end
    
    % Create repetitive matrices
    nfft = 1; %number of subcarriers
    betaRep = repmat(betaList, [1, ngamma]);
    gammaRep = repmat(gammaList, [nbeta, 1]); 
    lambdaRep = repmat(reshape(lambda, 1, 1, []), [nbeta, ngamma, 1]);
    
    % Change sizes
    len = length(yawList);           
    betaRep1 = repmat(betaRep, [1, 1, nfft, len]);
    gammaRep1 = repmat(gammaRep, [1, 1, nfft, len]);
    lambdaRep1 = repmat(lambdaRep, [1, 1, 1, len]);
    yawList1 = repmat(reshape(yawList, [1 , 1, 1, len]), [nbeta ngamma nfft, 1]);
    pitchList1 = repmat(reshape(pitchList, [1 , 1, 1, len]), [nbeta ngamma nfft, 1]);
    rhoList1 = repmat(reshape(rhoList, [1 , 1, 1, len]), [nbeta ngamma nfft, 1]);
   
    % Compute the angle of arrival, i.e. a(phi,theta)(t)
    if simple
        a_phi_theta = exp((-1i*4*pi*rhoList1.*(cos(pitchList1).*cos(betaRep1-yawList1)))./lambdaRep1);
    else
        a_phi_theta = exp((-1i*4*pi*rhoList1.*(sin(gammaRep1).*cos(pitchList1).*cos(betaRep1-yawList1)+sin(pitchList1).*cos(gammaRep1)))./lambdaRep1); 
    end

    % Compute h_ij(t) * a(phi,theta)(t)
    leftMat= double(squeeze(a_phi_theta(:, :, 1, :))); % 360 * 90 * 135
    rightMat = hrList;  
    resultMat = double(prodND_fast(leftMat, rightMat));

    % Compute F_ij(t) using the Bartlett estimator 
    betaArray = permute((double(abs(resultMat)).^2), [3, 1, 2]); 
    betaProfileBig( :, :, 1) = betaArray;

    % Normalize F_ij(t)
    dummy = betaProfileBig(:, :, 1);  
    dummy = dummy ./ sum(sum(dummy));     
    profile = dummy;

    % Create output of correct size (only for plotting in main_csv.m)
    if simple
        % other elevation values all 0? 
        % profile = zeros(nbeta, ngamma_old);
        % profile(1:size(dummy,1),1:size(dummy,2)) = dummy;
        % or copy same values?
        for k = 3:ngamma_old
            profile(:,k) = profile(:,1);
        end
    end
%     whos('leftMat')
%     whos('rightMat')
end
        
