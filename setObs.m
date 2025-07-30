function plots = setObs(plots, obsSet)
    % --------- sanity / existence checks ----------
    if ~isfield(plots, 'poly');            plots.poly = struct(); end
    if ~isfield(plots.poly, 'obsSets');    plots.poly.obsSets = {}; end
    if ~isfield(plots, 'ax') || isempty(plots.ax) || ~any(isgraphics(plots.ax))
        error('plots.ax must be a valid axes handle (e.g., app.UIAxes). Set plots.ax = app.UIAxes before calling setObs.');
    end
    if numel(plots.poly.obsSets) < obsSet
        plots.poly.obsSets{obsSet} = polyshape.empty;
    end

    % helper to check if graphics object is valid
    isValid = @(h) ~isempty(h) && isgraphics(h);

    % --------- empty set: clear safely and return ----------
    if isempty(plots.poly.obsSets{obsSet})
        if isfield(plots.poly, 'obsplt') && isValid(plots.poly.obsplt)
            delete(plots.poly.obsplt);
        end
        plots.poly.obsplt = [];
        if isfield(plots.poly, 'walls')
            plots.poly.track = plots.poly.walls;
        else
            plots.poly.track = polyshape();
        end
        drawnow;
        return;
    end

    % --------- normal path: union + (re)draw ----------
    allobs = union(plots.poly.obsSets{obsSet});

    h = [];
    if isfield(plots.poly, 'obsplt')
        h = plots.poly.obsplt;
    end

    if isValid(h)
        plots.poly.obsplt.Shape = allobs;
        plots.poly.obsplt.FaceColor = [140, 209, 255]/255;  % light blue
        plots.poly.obsplt.EdgeColor = 'none';               % no border
    else
        plots.poly.obsplt = plot(plots.ax, allobs, ...
            'FaceColor', [140, 209, 255]/255, ...
            'FaceAlpha', 0.4, ...
            'EdgeColor', 'none');
    end

    if isfield(plots.poly, 'walls')
        plots.poly.track = union(plots.poly.walls, allobs);
    else
        plots.poly.track = allobs;
    end

    drawnow;
end