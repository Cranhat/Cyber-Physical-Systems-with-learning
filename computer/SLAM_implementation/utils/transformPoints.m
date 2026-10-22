function pointsOut = transformPoints(points, pose)
    x = pose(1);
    y = pose(2);
    a = pose(3);

    R = [
        cos(a), -sin(a);
        sin(a),  cos(a)
    ];

    pointsOut = points * R' + [x, y];
end