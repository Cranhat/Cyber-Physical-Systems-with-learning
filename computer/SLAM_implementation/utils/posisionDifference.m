function dif = posisionDifference(posision1, posision2)
    x1 = posision1(1);
    y1 = posision1(2);
    a1 = posision1(3);

    x2 = posision2(1);
    y2 = posision2(2);
    a2 = posision2(3);

    dx = x2 - x1;
    dy = y2 - y1;

    R = [ cos(a1) sin(a1);
         -sin(a1) cos(a1)];

    localDelta = R * [dx; dy];

    dif = [localDelta(1), localDelta(2), wrapToPi(a2 - a1)];
    
end