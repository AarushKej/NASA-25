function plots = createObs(robot, plots, obsSet, shape, dims, spawnFact, spawnLane, offset)
    % Ensure basic fields exist
    if ~isfield(plots, 'ax') || isempty(plots.ax) || ~isgraphics(plots.ax)
        error('plots.ax must point to your UIAxes (e.g., app.UIAxes). Set it once, e.g., plots.ax = app.UIAxes;');
    end
    if ~isfield(plots, 'obstacleArr') || isempty(plots.obstacleArr)
        plots.obstacleArr = {};
    end
    if ~isfield(plots, 'poly')
        plots.poly = struct();
    end
    if ~isfield(plots.poly, 'obsSets') || isempty(plots.poly.obsSets)
        plots.poly.obsSets = {};
    end

    assert(obsSet>=1 && round(obsSet)==obsSet && isnumeric(obsSet),'Invalid obstacle set.')
    assert((strcmp(shape,'poly') && dims(1)>=3 && round(dims(1))==dims(1))|| ~strcmp(shape,'poly'),'Invalid number of sides for polygonal obstacle.')
    assert(any(shape==["rect","circ","poly"]),'Invalid obstacle shape.')
    assert(isnumeric(dims) && all(dims>0),'Invalid obstacle dimensions.')
    assert((any(shape==["rect","poly"]) && length(offset)==3) || (strcmp(shape,'circ') && length(offset)==2),'Invalid obstacle offset.')
    assert(isnumeric(spawnFact),'Invalid obstacle spawn location.')

    % ---- Build the primitive poly ----
    switch shape
        case 'rect'
            interior = isinterior(plots.trackpoly, offset(1), offset(2));
            if ~interior, disp("invalid"); return; end
            poly = nsidedpoly(4, 'sidelength', 1);
            poly.Vertices = poly.Vertices .* dims;
            dtheta = offset(3);

        case 'circ'
            interior = isinterior(plots.trackpoly, offset(1), offset(2));
            if ~interior, disp("invalid"); return; end
            nsides = round((2*pi)/(plots.ptspacing/dims));
            poly = nsidedpoly(nsides, 'Radius', dims/2);
            dtheta = 0;

        case 'poly'
            interior = isinterior(plots.trackpoly, offset(1), offset(2));
            if ~interior, disp("invalid"); return; end
            poly = nsidedpoly(dims(1), 'Radius', dims(2)/2);
            dtheta = offset(3);
    end

    % ---- Transform ----
    polyout = translate(rotate(poly, dtheta), offset(1:2));

    % ---- Ensure obsSets{obsSet} exists ----
    if length(plots.poly.obsSets) < obsSet || isempty(plots.poly.obsSets{obsSet})
        plots.poly.obsSets{obsSet} = polyshape.empty;
    end

    % ---- Append this obstacle to the set ----
    plots.poly.obsSets{obsSet} = [plots.poly.obsSets{obsSet}, polyout];

    % ---- Store obstacle record (NO per-obstacle graphics handle) ----
    % {1=type, 2=dims, 3=spawnFact, 4=spawnLane, 5=offset, 6=[], 7=vertices, 8=polyshape, 9=obsSet}
    obs = {shape, dims, spawnFact, spawnLane, offset, [], polyout.Vertices, polyout, obsSet};
    plots.obstacleArr{end+1} = obs;
end