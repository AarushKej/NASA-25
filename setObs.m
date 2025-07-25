function plots = setObs(plots, obsSet)
    % Ensure basic fields
    if ~isfield(plots, 'poly');            plots.poly = struct(); end
    if ~isfield(plots.poly, 'obsSets');    plots.poly.obsSets = {}; end
    if ~isfield(plots, 'ax') || isempty(plots.ax) || ~isgraphics(plots.ax)
        error('plots.ax must be a valid axes handle (e.g., app.UIAxes). Set plots.ax = app.UIAxes before calling setObs.');
    end
    if numel(plots.poly.obsSets) < obsSet
        plots.poly.obsSets{obsSet} = polyshape.empty;
    end

    % Helper to validate handle
    isValid = @(h) ~isempty(h) && isgraphics(h);

    % If no obstacles, clear plot
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

    % Merge all obstacles into a single shape for drawing
    allobs = union(plots.poly.obsSets{obsSet});
    if isfield(plots.poly, 'obsplt') && isValid(plots.poly.obsplt)
        % Update existing patch
        plots.poly.obsplt.Shape = allobs;
    else
        % Create a new patch
        plots.poly.obsplt = plot(plots.ax, allobs, ...
            'FaceColor', [1 0 0], 'FaceAlpha', 0.2, 'EdgeColor', 'k');
    end

    % Update track
    if isfield(plots.poly, 'walls')
        plots.poly.track = union(plots.poly.walls, allobs);
    else
        plots.poly.track = allobs;
    end

    drawnow;
end