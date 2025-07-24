% -------------------------------------------------------------------------
% CMAC Approximation of a q function
% -------------------------------------------------------------------------
function [cmac, qvals] = cmacModel(app)

cmac= app.cmac;
% Initialize arrays with zeros
cmac.pred = 0;

cmac.inputRanges = repmat([0 app.robot.cmac.rangeMax], app.robot.sensor.lidar.n, 1);
% max distance from objective is norm of track dims
cmac.inputRanges = [cmac.inputRanges;[0, norm([app.robot.track.wall1_length, app.robot.track.wall2_length])]];
% min/max difference in heading from robot to obstacle
cmac.inputRanges= [cmac.inputRanges;[-1, 1]];
cmac.inputRanges= [cmac.inputRanges;[-1, 1]];

cmac.numInputs = app.robot.sensor.lidar.n + 3;

cmac.inputBins = zeros(1, cmac.numInputs);
cmac.inputBins(1:app.robot.sensor.lidar.n) = cmac.sensorRes;
cmac.inputBins(app.robot.sensor.lidar.n + 1) = cmac.distanceRes;
cmac.inputBins(app.robot.sensor.lidar.n + 2: app.robot.sensor.lidar.n + 3) = cmac.headingRes;

numActions = 3;

cmac.wMatrix = zeros(numActions, cmac.N);
cmac.adMatrix = zeros(numActions, cmac.numInputs, cmac.c);
cmac.sMatrix = zeros(numActions, cmac.numInputs, cmac.c);

cmac.w_1 = zeros(numActions, cmac.N);
cmac.w_2= zeros(numActions, cmac.N);
cmac.d_w= zeros(numActions, cmac.N);



% System Input (Input layer)
u = app.robot.sensor.lidar.distances;

% Create vector from robot to objective
objX = app.robot.obj.x - app.robot.center(1);
objY = app.robot.obj.y - app.robot.center(2);
dist = norm(objX, objY);
u(end+1) = dist;


% calculate angle between robot's heading and obstacle
robDir = app.robot.kinematics.theta;
objDir = atan2(objY, objX);
%headingDiff = wrapToPi(objDir - robDir);
headingDiff = objDir - robDir;

% two heading inputs
% using cos and sin allows continuity as the angle moves past 180
app.robot.headingError = [cos(headingDiff), sin(headingDiff)];
u(end+1) = app.robot.headingError(1);
u(end+1) = app.robot.headingError(2);
app.robot.headingDiff = headingDiff;


% clamp input values to input range;
u = min(max(u, cmac.inputRanges(:,1)), cmac.inputRanges(:,2));


% concept mapping and actual mapping
% Mapping U --> A
% Mapping A --> P (hashing)

uMin = cmac.inputRanges(:,1);
uMax = cmac.inputRanges(:,2);

%normalize input vals
normalU = (u - uMin)./(uMax-uMin);
%quantize inputs into bins
binnedU = round(normalU .* cmac.inputBins');

%create offsets matrix
offsets = repmat(1:cmac.c, cmac.numInputs,1);

%add offsets to quantized inputs to create generalized conceptual addresses
s = repmat(binnedU, 1,cmac.c) + offsets;

%hash conceptual addresses to get real addresses
ad = mod(s, cmac.N) + 1;


unique_ad = unique(ad(:));
qvals = zeros(1,app.robot.numActions);

for a = 1:1:app.robot.numActions
    cmac.sMatrix(a, :, :) = s;
    cmac.adMatrix(a, :, :) = ad;
    qvals(a) = sum(cmac.wMatrix(a, unique_ad)) / length(unique_ad);
end
end