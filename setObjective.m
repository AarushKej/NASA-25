function [robot, plots]=setObjective(robot,plots,xval,yval,isrand)
assert(islogical(isrand),'Random spawn must be ''true'' or ''false''.')

robot.obj.radius = 18;
radius = robot.obj.radius;


% don't spawn too close to track walls
buffer = 2*radius;
objColor = 'yellow';

if isrand
    xval =(rand-.5) * (plots.trackWidth - buffer);
    yval =(rand - .5) * (plots.trackWidth-buffer);
end

if xval > ((plots.trackWidth - plots.trackWidth/2) - buffer)
    xval = 0;
elseif xval < ((-1*(plots.trackWidth - plots.trackWidth/2)) + buffer)
    xval = 0;
end
if yval > ((plots.trackHeight - plots.trackHeight/2) - buffer)
    yval = 0;
elseif yval < ((-1*(plots.trackHeight - plots.trackHeight/2)) + buffer)
    yval = 0;
end

maxattempt=10;
if isfield(robot.obj,'coords')
    xval_init=robot.obj.coords(0);
    yval_init=robot.obj.coords(1);
    imax=maxattempt+1;
else
    imax=1;
end



for i=1:imax
    if i==maxattempt+1
        xval=xval_init;
        yval=yval_init;
    end
    % startingidx=round(interp1([0 1],[1 length(robot.spawn.allctrpts{spawnLane}(:,1))],mod(spawnFact,1)));
    robot.obj.x=xval;
    robot.obj.y=yval;
    robot.obj.coords = [robot.obj.x,robot.obj.y];
    %pos = [xval-radius, yval-radius, radius*2, radius*2];
    %robot.obj.poly = rectangle(plots.trackAx, 'Position', pos, 'Curvature', [1 1], 'FaceColor', objColor);
  
    % robot.spawn.origin=robot.spawn.allctrpts{spawnLane}(startingidx,1:2);
    % robot.spawn.heading=robot.spawn.allctrpts{spawnLane}(startingidx,3);
    % robot.obj.patchpts=cell2mat(arrayfun(@(px,py) rotz_fun(px,py,robot.obj.heading,robot.obj.origin),...
    %     robot.obj.patchpts0(:,1),robot.obj.patchpts0(:,2),'UniformOutput',false));
    % 
    % robot.obj.axle=rotz_fun(0,-(robot.dims(2)/2)+(robot.wheels.dims.back(2)/2),robot.obj.heading,robot.obj.origin);
    % 
    % if ~any(isinterior(plots.poly.track,robot.obj.patchpts))
    %     break
    % elseif i==1 && ~isrand
    %     warning('Obstacle location causes collision.')
    %     break
    % elseif i==maxattempt+1 && isrand
    %     warning('Could not locate random spawn location without collision. Defaulting to initial values.')
    % elseif isrand
    %     xval =(rand-.5) * (plots.trackWidth - buffer);
    %     yval =(rand - .5) * (plots.trackWidth-buffer);
    % end
end

try
    delete(plots.objective);
end

plots.objective=patch(plots.trackAx,nan(4,1),nan(4,1),objColor);
theta=linspace(0,2*pi,360)';
plots.objpts0=radius*[cos(theta) sin(theta)];

plots.objective.XData=plots.objpts0(:,1)+xval;
plots.objective.YData=plots.objpts0(:,2)+yval;

end
