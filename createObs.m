function plots=createObs(robot,plots,obsSet,shape,dims,spawnFact,spawnLane,offset)
assert(obsSet>=1 && round(obsSet)==obsSet && isnumeric(obsSet),'Invalid obstacle set.')
assert((strcmp(shape,'poly') && dims(1)>=3 && round(dims(1))==dims(1))|| ~strcmp(shape,'poly'),'Invalid number of sides for polygonal obstacle.')
assert(any(shape==["rect","circ","poly"]),'Invalid obstacle shape.')
assert(isnumeric(dims) && all(dims>0),'Invalid obstacle dimensions.')
assert((any(shape==["rect","poly"]) && length(offset)==3) || (strcmp(shape,'circ') && length(offset)==2),'Invalid obstacle offset.')
assert(isnumeric(spawnFact),'Invalid obstacle spawn location.')

switch shape
    case 'rect'
         interior = isinterior(plots.trackpoly, offset(1,1), offset(1,2));
          if interior
            poly=nsidedpoly(4,'sidelength',1);
            poly.Vertices=poly.Vertices.*dims;
            dtheta=offset(3);
            assert(isnumeric(offset) && length(offset)==3,'Invalid obstacle offset.')
        else
             disp("invalid")
         end
        
    case 'circ'
         interior = isinterior(plots.trackpoly, offset(1,1), offset(1,2));
          if interior
        nsides=round((2*pi)/(plots.ptspacing/dims));
        poly=nsidedpoly(nsides,'Radius',dims/2);
        dtheta=0;
        assert(isnumeric(offset) && length(offset)==2,'Invalid obstacle offset.')
         else
             disp("invalid")
         end
    case 'poly'
        interior = isinterior(plots.trackpoly, offset(1,1), offset(1,2));
          if interior
        poly=nsidedpoly(dims(1),'Radius',dims(2)/2);
        dtheta=offset(3);
        assert(isnumeric(offset) && length(offset)==3,'Invalid obstacle offset.')
        else
             disp("invalid")
         end
end

%spawnpts_all=robot.spawn.allctrpts{1};
%startingidx=round(interp1([0 1],[1 length(spawnpts_all(:,1))],mod(spawnFact,1)));
%heading=spawnpts_all(startingidx,3);
% ctrpt=spawnpts_all(startingidx,1:2)+(plots.laneWidth*(spawnLane-1)*[cos(heading-pi/2) sin(heading-pi/2)]);
polyout=translate(rotate(poly,0),offset(1:2));
if length(plots.poly.obsSets)<obsSet
    plots.poly.obsSets(obsSet)={polyshape()};
end
if isempty(plots.poly.obsSets{obsSet})
    plots.poly.obsSets(obsSet)={polyshape()};
end
if isempty(plots.poly.obsSets{obsSet}(1).Vertices)
    plots.poly.obsSets{obsSet}=polyout;
else
    plots.poly.obsSets{obsSet}=[plots.poly.obsSets{obsSet} polyout];
end

    % add to list
    obs = {shape, dims,spawnFact,spawnLane,offset};
    len = length(plots.obstacleArr);
    plots.obstacleArr{len+1} = obs;
end