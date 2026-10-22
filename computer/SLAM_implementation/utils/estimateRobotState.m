function [r_next, F] = estimateRobotState(r, u, enDt, imuDt, L)
    %r_next - estimated robot state
    %F - state transition

    %r - currect robot state
    %u - input [distanceR, distanceL, gyro, accelerationX, accelerationY]
    %enDt - time dif for encoder data
    %imuDt - time dif for imu data
    %L - distance between wheels

    distanceR = u(1);
    distanceL = u(2);
    gyro = u(3);

    distance = (distanceR + distanceL)/2;

    angle = r(3);
    angleGyro = angle + gyro * imuDt;
    angleEn = angle + (distanceL - distanceR) / L;
    finalAngle = wrapToPi(0.8 * angleGyro + 0.2 * angleEn);
    
    if enDt > 0
        v_new = distance / enDt;
    else
        v_new = r(4);
    end

    r_next = [r(1) + distance * cos(finalAngle);
              r(2) + distance * sin(finalAngle);
              wrapToPi(finalAngle);
              v_new];

    F = [1 0 -distance * sin(finalAngle)    0;
         0 1  distance * cos(finalAngle)    0;
         0 0  1                             0;
         0 0  0                             1];
end