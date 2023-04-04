%% Adjust angle to -pi and pi
function out = angle_adjustment(a)
    out = a;
    if a > 2*pi
        out = out - 2*pi;
    elseif a < -2*pi
        out = out  + 2*pi;
    end
end