function plots=randObs(robot,plots,obsSet,shape,dims,buffer,Nlocs,NObs)
assert(all(NObs>0 & NObs<plots.Nlanes) && all(round(NObs)==NObs) && isnumeric(NObs) ,'Invalid max. number of obstacles.')
assert(Nlocs>=1 && round(Nlocs)==Nlocs && isnumeric(Nlocs),'Invalid number of random obstacle locations.')
assert(buffer(1)<=0 && buffer(2)>=0 && all(abs(buffer)>=0 & abs(buffer)<=1) && diff(buffer)~=0 && sum(buffer)<1,'Invalid random obstacle buffer.')
if length(plots.poly.obsSets)>=obsSet
    if ~isempty(plots.poly.obsSets{obsSet})
        assert(~isempty(plots.poly.obsSets{obsSet}(1).Vertices),'Obstacle set already established. Select a different obstacle set number.')
    end
end
switch shape
    case {'rect','poly'}
        offset=zeros(1,3);
    case 'circ'
        offset=zeros(1,2);
end
spawnFacts=sort(mod(linspace(0,1-diff(buffer),Nlocs)+buffer(2)+robot.spawn.factor,1));
for loc=1:Nlocs
    maxObs=NObs(randi(length(NObs)));
    allLanes=sort(randperm(plots.Nlanes,maxObs));
    if abs(spawnFacts(loc)-mod(robot.spawn.factor,1))<.05 && any(allLanes==robot.spawn.lane)
        openlanes_all=setdiff(1:plots.Nlanes,allLanes);
        openlane=openlanes_all(openlanes_all~=robot.spawn.lane);
        allLanes(allLanes==robot.spawn.lane)=openlane(randi(length(openlane)));
    end
    for obs=1:maxObs
        spawnLane=allLanes(obs);
        plots=createObs(robot,plots,obsSet,shape,dims,spawnFacts(loc),spawnLane,offset);
    end
end

end