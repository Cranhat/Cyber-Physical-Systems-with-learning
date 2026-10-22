function pose = rigidTransform(A, B)
    centroidA = mean(A, 1);
    centroidB = mean(B, 1);

    AA = A - centroidA;
    BB = B - centroidB;

    H = AA' * BB;

    [U, ~, V] = svd(H);

    R = V * U';

    if det(R) < 0
        V(:, end) = -V(:, end);
        R = V * U';
    end

    t = centroidB' - R * centroidA';

    angle = atan2(R(2,1), R(1,1));

    pose = [t(1), t(2), angle];
end
