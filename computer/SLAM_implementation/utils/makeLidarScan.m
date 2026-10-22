function scan = makeLidarScan(ranges, angles, maxRange)
    ranges = ranges(:);
    angles = angles(:);

    valid = ranges > 0 & ranges < maxRange * 0.98;

    ranges = ranges(valid);
    angles = angles(valid);

    x = ranges .* cos(angles);
    y = ranges .* sin(angles);

    scan.Ranges = ranges;
    scan.Angles = angles;
    scan.Points = [x, y];
end