function robot=setSpawn(robot,plots,spawnLane,spawnFact,isrand)
    assert(islogical(isrand),'Random spawn must be ''true'' or ''false''.')
    if isrand
        spawnLane=randi(robot.spawn.NLanes);
        spawnFact=rand;
    else
        assert(spawnLane>=1 && spawnLane<=robot.spawn.NLanes && length(spawnLane)==1 && isnumeric(spawnLane),'Invalid new lane of robot spawn')
        assert(abs(spawnFact)>=0 && abs(spawnFact)<=1 && length(spawnFact)==1 && isnumeric(spawnFact),'Invalid new starting factor of robot.')
    end
    rotz_fun=@(pt_x,pt_y,angle,ctr) ([cos(angle-pi/2) -sin(angle-pi/2);sin(angle-pi/2) cos(angle-pi/2)]*[pt_x;pt_y])'+ctr;

    maxattempt=10;
    if isfield(robot.spawn,'factor')
        spawnFactor_init=robot.spawn.factor;
        spawnLane_init=robot.spawn.lane;
        imax=maxattempt+1;
    else
        imax=1;
    end
    for i=1:imax
        if i==maxattempt+1
            spawnFact=spawnFactor_init;
            spawnLane=spawnLane_init;
        end
        startingidx=round(interp1([0 1],[1 length(robot.spawn.allctrpts{spawnLane}(:,1))],mod(spawnFact,1)));
        robot.spawn.factor=spawnFact;
        robot.spawn.lane=spawnLane;
        robot.spawn.origin=robot.spawn.allctrpts{spawnLane}(startingidx,1:2);
        robot.spawn.heading=robot.spawn.allctrpts{spawnLane}(startingidx,3);
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
            spawnLane=randi(robot.spawn.NLanes);
            spawnFact=rand;
        end
    end
end