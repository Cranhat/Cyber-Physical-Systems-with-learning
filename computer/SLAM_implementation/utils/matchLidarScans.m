function delta = matchLidarScans(scan, refScan, initialPosition, maxIter)
    sourcePoints = scan.Points;
    refPoints = refScan.Points;

    if size(sourcePoints, 1) < 20 || size(refPoints, 1) < 20
        delta = initialPosition;
        return;
    end

    %reducing number of points
    sourcePoints = sourcePoints(1:  max(1, floor(size(sourcePoints, 1) / 120)): end, :);
    refPoints = refPoints(1:    max(1, floor(size(refPoints, 1) / 120)):    end, :);

    pose = initialPosition(:)';

    for iter = 1:maxIter

        transformed = transformPoints(sourcePoints, pose);

        idx = nearestNeighbors(transformed, refPoints);
        matchedrefPoints = refPoints(idx, :);

        errors = matchedrefPoints - transformed;
        dists = sqrt(sum(errors.^2, 2));

        keep = dists < 0.25;

        if sum(keep) < 10
            break;
        end

        A = transformed(keep, :);
        B = matchedrefPoints(keep, :);

        correction = rigidTransform(A, B);

        pose = moveRobot(pose, correction);

        if norm(correction(1:2)) < 1e-4 && abs(correction(3)) < deg2rad(0.05)
            break;
        end
    end

    delta = pose;
end

