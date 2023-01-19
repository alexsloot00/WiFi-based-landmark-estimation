function result = prodND_fast(leftMat, rightMat)
    % (2022) Original code: Ninad Jadhav - Harvard University
    lM = permute(leftMat, [2, 3, 1]);
    rM = repmat(rightMat, [1, 1, size(leftMat, 1)]);
    result = mtimesx(lM, rM, 'SPEED');
end
