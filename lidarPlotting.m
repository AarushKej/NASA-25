function [robot,plots]=lidarPlotting(robot,plots)


allObs = plots.poly.track;

nanRows = isnan(allObs.Vertices(:,1));
idx = [0; find(nanRows); size(allObs.Vertices,1)+1];
allShapes = {};

for i = 1:length(idx)-1
    startIdx = idx(i)+1;
    endIdx = idx(i+1)-1;
    if startIdx <= endIdx
        allShapes{end+1} = allObs.Vertices(startIdx:endIdx, :);
    end
end

edge_starts = [];
edge_ends = [];
for s = 1:length(allShapes)
    verts = allShapes{s};           
    edge_starts = [edge_starts; verts];
    edge_ends   = [edge_ends; verts([2:end,1], :)];  
end


mouseX = robot.center(1);
mouseY = robot.center(2);

N = robot.sensor.lidar.n;
distance = robot.sensor.lidar.distance;
theta = robot.kinematics.theta - pi/2;  % Robot's current heading angle
angles = mod(robot.sensor.lidar.angles' + theta, 2*pi);

M = size(edge_starts, 1);  
ray_dirs = [cos(angles), sin(angles)];  
ray_start = repmat([mouseX, mouseY], N*M, 1);
ray_end   = ray_start + repelem(ray_dirs * distance, M, 1);

segment_start = repmat(edge_starts, N, 1);
segment_end   = repmat(edge_ends, N, 1);


%ray casting math

r = ray_end - ray_start;
s = segment_end - segment_start;
cma = segment_start - ray_start;

rxs = r(:,1).*s(:,2) - r(:,2).*s(:,1);
cmaxs = cma(:,1).*s(:,2) - cma(:,2).*s(:,1);
cmar = cma(:,1).*r(:,2) - cma(:,2).*r(:,1);

t = cmaxs ./ rxs;
u = cmar ./ rxs;

hit = t >= 0 & t <= 1 & u >= 0 & u <= 1;

inter_pts = ray_start + t .* r;

validMat = reshape(hit, M, N);
inter_ptsMat_x = reshape(inter_pts(:,1), M, N);
inter_ptsMat_y = reshape(inter_pts(:,2), M, N);

distMat = sqrt((inter_ptsMat_x - mouseX).^2 + (inter_ptsMat_y - mouseY).^2);
distMat(~validMat) = NaN;

% normalize input so t value becomes the distance and no need to re NEED TO DO  

[minDists, minIdx] = min(distMat, [], 1);

closest_pts = [mouseX + distance * ray_dirs(:,1), mouseY + distance * ray_dirs(:,2)];

hitRays = ~isnan(minDists);
if any(hitRays)
    idxLinear = sub2ind(size(inter_ptsMat_x), minIdx(hitRays), find(hitRays));
    closest_pts(hitRays,1) = inter_ptsMat_x(idxLinear);
    closest_pts(hitRays,2) = inter_ptsMat_y(idxLinear);
end

robot.sensor.lidar.distances = minDists';
robot.sensor.lidar.endpoints = closest_pts;


validPts = ~isnan(robot.sensor.lidar.distances);
xHits = robot.sensor.lidar.endpoints(validPts, 1);
yHits = robot.sensor.lidar.endpoints(validPts, 2);

% Interleave rays with NaNs for clean line breaks in plotting
xLines = [repmat(mouseX, N, 1), closest_pts(:,1), nan(N,1)]';
yLines = [repmat(mouseY, N, 1), closest_pts(:,2), nan(N,1)]';

% Flatten for plotting (1D with NaNs in between)
robot.sensor.lidar.XData = reshape(xLines, 1, []);
robot.sensor.lidar.YData = reshape(yLines, 1, []);


% Interleave rays with NaNs for clean line breaks in plotting
xLines = [repmat(mouseX, N, 1), closest_pts(:,1), nan(N,1)]';
yLines = [repmat(mouseY, N, 1), closest_pts(:,2), nan(N,1)]';

% Flatten for plotting (1D with NaNs between rays)
xLinesFlat = reshape(xLines, 1, []);
yLinesFlat = reshape(yLines, 1, []);

% Save to robot struct (optional)
robot.sensor.lidar.XData = xLinesFlat;
robot.sensor.lidar.YData = yLinesFlat;



if isfield(robot.sensor.lidar, "hitPtsPlot") && isvalid(robot.sensor.lidar.hitPtsPlot)
    set(robot.sensor.lidar.hitPtsPlot, 'XData', xHits, 'YData', yHits);
else
    hold(plots.trackAx, 'on');
    robot.sensor.lidar.hitPtsPlot = plot(plots.trackAx, xHits, yHits, 'r.', 'MarkerSize', 8);
end


% Plot lines
if isfield(robot.sensor.lidar, "rayPlot") && isvalid(robot.sensor.lidar.rayPlot)
    set(robot.sensor.lidar.rayPlot, 'XData', xLinesFlat, 'YData', yLinesFlat);
else
    hold(plots.trackAx, 'on');
    robot.sensor.lidar.rayPlot = plot(plots.trackAx, xLinesFlat, yLinesFlat, 'r-', 'LineWidth', 0.000001);
end

