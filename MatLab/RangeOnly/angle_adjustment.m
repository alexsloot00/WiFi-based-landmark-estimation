function adjusted_angle = angle_adjustment(angle)
    % (2023) Original code: Alex Sloot - DTPA - University of Groningen
    
    % This function converts an angle in radians to an angle in radians
    % between -pi and pi.

    % INPUT:    angle - any angle in radians
    % OUTPUT:   adjusted_angle - the angle between -pi and pi

    adjusted_angle = angle;
    if angle > 2*pi
        adjusted_angle = adjusted_angle - 2*pi;
    elseif angle < -2*pi
        adjusted_angle = adjusted_angle  + 2*pi;
    end
end