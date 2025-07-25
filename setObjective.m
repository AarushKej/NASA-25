function [robot, plots]=setObjective(robot,plots,xval,yval,isrand)
assert(islogical(isrand),'Random spawn must be ''true'' or ''false''.');

robot.obj.radius = 18;
radius = robot.obj.radius;

% don't spawn too close to track walls
buffer = 2 * radius;
objColor = [1 0.8 0]; % Yellow (RGB)

if isrand
    xval = (rand - 0.5) * (plots.trackWidth - buffer);
    yval = (rand - 0.5) * (plots.trackWidth - buffer);
end

% Keep objective inside track bounds
if xval > ((plots.trackWidth - plots.trackWidth/2) - buffer)
    xval = 0;
elseif xval < ((-1 * (plots.trackWidth - plots.trackWidth/2)) + buffer)
    xval = 0;
end
if yval > ((plots.trackHeight - plots.trackHeight/2) - buffer)
    yval = 0;
elseif yval < ((-1 * (plots.trackHeight - plots.trackHeight/2)) + buffer)
    yval = 0;
end

maxattempt = 10;
if isfield(robot.obj,'coords')
    xval_init = robot.obj.coords(1);
    yval_init = robot.obj.coords(2);
    imax = maxattempt + 1;
else
    imax = 1;
end

for i = 1:imax
    if i == maxattempt + 1
        xval = xval_init;
        yval = yval_init;
    end
    robot.obj.x = xval;
    robot.obj.y = yval;
    robot.obj.coords = [robot.obj.x, robot.obj.y];
end

% Remove previous objective if it exists
try
    delete(plots.objective);
catch e
    disp(e);
end

% ----- Create star shape -----
numPoints = 5;                % Star points
outerR = radius;              % Outer radius
innerR = radius * 0.4;        % Inner radius
angles = linspace(0, 2*pi, numPoints * 2 + 1); % 10 points + repeat first
r = outerR * ones(size(angles)); 
r(2:2:end) = innerR;          % Alternate outer/inner radius

xStar = r .* cos(angles);
yStar = r .* sin(angles);

% Store star points relative to center
plots.objpts0 = [xStar(:), yStar(:)];

% Draw the star
plots.objective = patch(plots.trackAx, ...
    plots.objpts0(:,1) + xval, ...
    plots.objpts0(:,2) + yval, ...
    objColor, 'EdgeColor', 'none');
end