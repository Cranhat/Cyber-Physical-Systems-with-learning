function new_posision = moveRobot(startPosision, endPosision)
    x1 = startPosision(1);
    y1 = startPosision(2);
    a1 = startPosision(3);

    x2 = endPosision(1);
    y2 = endPosision(2);
    a2 = endPosision(3);

    R = [cos(a1) -sin(a1);
         sin(a1)  cos(a1)];

    p = [x1; y1] + R * [x2; y2];

    new_posision = [p(1), p(2), wrapToPi(a1 + a2)];
end