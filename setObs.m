function plots = setObs(plots, obsSet)
    % Validate inputs
    assert(length(plots.poly.obsSets) >= obsSet, 'Obstacle set does not exist.');
    assert(~isempty(plots.poly.obsSets{obsSet}), 'Obstacle set does not exist.');
    assert(~isempty(vertcat(plots.poly.obsSets{obsSet}.Vertices)), 'Obstacle set does not exist.');

    % Union all obstacles in this set
    allobs = union(plots.poly.obsSets{obsSet});

    % If a plot handle already exists, update its shape
    if isfield(plots.poly, 'obsplt') && isgraphics(plots.poly.obsplt)
        plots.poly.obsplt.Shape = allobs;
    else
        % Create a new plot handle if none exists
        plots.poly.obsplt = plot(plots.ax, allobs, ...
            'FaceColor', [1 0 0], 'FaceAlpha', 0.2, 'EdgeColor', 'k');
    end

    % Update track (walls + obstacles)
    if isfield(plots.poly, 'walls')
        plots.poly.track = union(plots.poly.walls, allobs);
    else
        plots.poly.track = allobs;
    end

    drawnow;
end