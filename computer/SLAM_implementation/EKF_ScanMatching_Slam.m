clear; clc;
addpath('utils/');

%% Parameters
L = 0.165;      % wheel distance [m]
maxRange = 5;   % lidar max range [m]
Q = diag([1e-3, 1e-3, deg2rad(4)^2, 1e-1]);
lidarR = diag([0.20^2, 0.20^2, deg2rad(10)^2]);

% EKF state: [x; y; angle; v]
x_est = [0; 0; 0; 0];
P = diag([1, 1, 0.5, 1]);

% Occupancy map
mapResolution = 20;
mapWidth = 4;
mapHeight = 4;
mapOrigin = [-2 -2];

occMap = occupancyMap(mapWidth, mapHeight, mapResolution);
occMap.GridLocationInWorld = mapOrigin;

% Histories
scanHistory = {};
ekfHist = [];
ekfPoseHist = []; 
lidar_hist = [];
poseGraphHist = [];

prevScan = [];
nodeId = 0;

% Forgetting / optimization
maxStoredScans = 200;     % max number of scans in memory
mapUpdateStep = 5;        % frequency of rebuilding map 
graphOptimizeStep = 10;   % frequency of optimization of graph
drawStep = 5;             % frequency of redrawing the map

%% MAIN LOOP %%

tcpObj = tcpserver("127.0.0.1", 5005);

while true
    %% Read TCP packet
    if tcpObj.NumBytesAvailable <= 0
        pause(0.01);
        continue;
    end

    line = readline(tcpObj);

    if strcmp(strtrim(line), "STOP")
        disp("Received STOP command");
        break;
    end

    dataRead = str2double(split(strtrim(line), ","));

    if numel(dataRead) < 8 || any(isnan(dataRead(1:8)))
        disp("Bad TCP packet header");
        continue;
    end
    
    %packet: [enDt, distanceR, distanceL, imuDt, gyro, accelerationX,
    %           accelerationY, lidarNum, (lidar_dist_n)x n, (lidar_ang_n)x n]
    enDt = dataRead(1);
    distanceR = dataRead(2); 
    distanceL = dataRead(3);
    imuDt = dataRead(4);
    gyro = (dataRead(5));
    gyro = deg2rad((gyro*250)/32768);

    accelerationX = dataRead(6);
    accelerationY = dataRead(7);
    lidarNum = dataRead(8);

    expectedLen = 8 + 2*lidarNum;

    if numel(dataRead) < expectedLen || any(isnan(dataRead(1:expectedLen)))
        disp("Bad TCP packet");
        continue;
    end

    lidar_dist = dataRead(9: 8 + lidarNum)';
    lidar_ang = dataRead(9 + lidarNum: 8 + lidarNum * 2)';

    lidar_dist(lidar_dist <= 0) = maxRange;
    lidar_dist(lidar_dist > maxRange) = maxRange;
    lidar_ang = wrapToPi(-deg2rad(lidar_ang));
    
    scan = makeLidarScan(lidar_dist, lidar_ang, maxRange);

    %% Prediction step of EKF
    u = [distanceR; distanceL; gyro; accelerationX; accelerationY];

    [x_pred, F] = estimateRobotState(x_est, u, enDt, imuDt, L);
    P_pred = F * P * F' + Q;


    x_est = x_pred;
    P = P_pred;

    %% Save EKF pose and scan
    ekfPose = x_est(1:3)';
    
    nodeId = nodeId + 1;
    scanHistory{nodeId} = scan;
    
    % Forget old scans
    if numel(scanHistory) > maxStoredScans
        oldIdx = nodeId - maxStoredScans;
        if oldIdx >= 1
            scanHistory{oldIdx} = [];
        end
    end

   %% Scan initialization
    if isempty(prevScan)
        prevScan = scan;
        prevPoseEKF = ekfPose;

        ekfPoseHist(nodeId, :) = ekfPose;
        ekfHist(:, end+1) = x_est(1:2);
    
        drawMap(occMap, ekfHist, x_est);
        continue;
    end

    %% Scan matching
    ekfDelta = posisionDifference(prevPoseEKF, ekfPose);

    useScanMatch = true;

    try
        scanDelta = matchLidarScans(prevScan, scan, ekfDelta, 50);

        if any(isnan(scanDelta)) || ...
           norm(scanDelta(1:2) - ekfDelta(1:2)) > 0.1 || ...
           abs(wrapToPi(scanDelta(3) - ekfDelta(3))) > deg2rad(2)
                useScanMatch = false;
        end

    catch
        useScanMatch = false;
    end

    if useScanMatch
        lidarAlpha = 0.5;   % influence of scanDelta over x state
    
        relPose = ekfDelta;
        relPose(1:2) = (1-lidarAlpha)*ekfDelta(1:2) + lidarAlpha*scanDelta(1:2);
    else
        relPose = ekfDelta;
    end

    %% Update step of EKF
    if useScanMatch
        lidarPose = moveRobot(prevPoseEKF, relPose);

        z = lidarPose(:);

        H = [1 0 0 0;
             0 1 0 0;
             0 0 1 0];

        innovation = z - H * x_est;
        innovation(3) = wrapToPi(innovation(3));

        K = P * H' / (H * P * H' + lidarR);

        x_est = x_est + K * innovation;
        x_est(3) = wrapToPi(x_est(3));

        P = (eye(4) - K * H) * P;
    end

    ekfPose = x_est(1:3)';
    ekfPoseHist(nodeId, :) = ekfPose;
    ekfHist(:, end+1) = x_est(1:2);
    
   %% Map rebuild
    if mod(nodeId, mapUpdateStep) == 0
    
        occMap = occupancyMap(mapWidth, mapHeight, mapResolution);
        occMap.GridLocationInWorld = mapOrigin;
    
        startIdx = max(1, nodeId - maxStoredScans + 1);
        endIdx = nodeId;
    
        for i = startIdx:endIdx
            if i > numel(scanHistory)
                continue;
            end
    
            thisScan = scanHistory{i};
    
            if isempty(thisScan)
                continue;
            end
            
            insertRay(occMap, ekfPoseHist(i, :), ...
                thisScan.Ranges, thisScan.Angles, maxRange);
        end
    end

    prevScan = scan;
    prevPoseEKF = ekfPose;

    if mod(nodeId, drawStep) == 0
          drawMap(occMap, ekfHist, x_est);
    end
end

clear tcpObj;




