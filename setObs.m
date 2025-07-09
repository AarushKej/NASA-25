function plots=setObs(plots,obsSet)
assert(length(plots.poly.obsSets)>=obsSet,'Obstacle set does not exist.')
assert(~isempty([plots.poly.obsSets{obsSet}]),'Obstacle set does not exist.')
assert(~isempty(vertcat(plots.poly.obsSets{1}.Vertices)),'Obstacle set does not exist.')
allobs=union(plots.poly.obsSets{obsSet});
plots.poly.obsplt.Shape=allobs;
plots.poly.track=union(plots.poly.walls,allobs);
drawnow
end