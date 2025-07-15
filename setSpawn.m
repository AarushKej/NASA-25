function robot=setSpawn(robot,plots,spawnX,spawnY,isrand)
assert(islogical(isrand),'Random spawn must be ''true'' or ''false''.')
if isrand
    spawnX=rand * plots.trackWidth - plots.trackWidth/2;
    spawnY=rand * plots.trackHeight - plots.trackHeight/2;
else
    

end

if spawnX > (plots.trackWidth - plots.trackWidth/2)
    spawnX = 0;
elseif spawnX < (-1*(plots.trackWidth - plots.trackWidth/2))
    spawnX = 0;
end
if spawnY > (plots.trackHeight - plots.trackHeight/2)
    spawnY = 0;
elseif spawnY < (-1*(plots.trackHeight - plots.trackHeight/2))
    spawnY = 0;
end

rotz_fun=@(pt_x,pt_y,angle,ctr) ([cos(angle-pi/2) -sin(angle-pi/2);sin(angle-pi/2) cos(angle-pi/2)]*[pt_x;pt_y])'+ctr;

maxattempt=10;
if isfield(robot.spawn,'factor')
    spawnX_init=robot.spawn.X;
    spawnY_init=robot.spawn.Y;
    imax=maxattempt+1;
else
    imax=1;
end

for i=1:imax
    if i==maxattempt+1
        spawnX=spawnX_init;
        spawnY=spawnY_init;
    end
    % startingidx=round(interp1([0 1],[1 length(robot.spawn.allctrpts{spawnLane}(:,1))],mod(spawnFact,1)));
    robot.spawn.x=spawnX;
    robot.spawn.y=spawnY;
    robot.spawn.origin = [robot.spawn.x,robot.spawn.y];
 if robot.random == true
    theta = 2 * pi * rand;
    robot.spawn.heading = theta;
 else
     if class(robot.angie) == "char"
        robot.angie = str2double(robot.angie);
     end
     custom_theta = 2 * pi * (robot.angie/ 360);
     robot.spawn.heading = custom_theta;
 end

    % robot.spawn.origin=robot.spawn.allctrpts{spawnLane}(startingidx,1:2);
    % robot.spawn.heading=robot.spawn.allctrpts{spawnLane}(startingidx,3);
    robot.spawn.patchpts=cell2mat(arrayfun(@(px,py) rotz_fun(px,py,robot.spawn.heading,robot.spawn.origin),...
        robot.spawn.patchpts0(:,1),robot.spawn.patchpts0(:,2),'UniformOutput',false));
    
    robot.spawn.axle=rotz_fun(0,-(robot.dims(2)/2)+(robot.wheels.dims.back(2)/2),robot.spawn.heading,robot.spawn.origin);
    
    if ~any(isinterior(plots.poly.track,robot.spawn.patchpts))
        break
    elseif i==1 && ~isrand
        warning('New spawn location causes collision.')
        break
    elseif i==maxattempt+1 && isrand
        warning('Could not locate random spawn location without collision. Defaulting to initial values.')
    elseif isrand
        spawnX=rand * plots.trackWidth - plots.trackWidth/2;
        spawnY=rand * plots.trackHeight - plots.trackHeight/2;
    end
end
end