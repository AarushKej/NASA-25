function [robot,plots]=createTerrain(robot,plots,Inputs,UIAxes)
plots.trackAx = UIAxes;
axis(plots.trackAx,'square')
hold(plots.trackAx,'on')

longwall_length=Inputs.track.wall1_length; % vertical
shortwall_length=Inputs.track.wall2_length; % horizontal

plots.trackHeight = longwall_length;
plots.trackWidth = shortwall_length;

trackpts = cell(4, 1);  % only 4 needed

L = Inputs.track.wall1_length;
W = Inputs.track.wall2_length;
wt = Inputs.track.wallthickness;

L_half = L / 2;
W_half = W / 2;

% Inner rectangle (inside walls)
trackpts{1} = [-L_half, -W_half;
                L_half, -W_half;
                L_half,  W_half;
               -L_half,  W_half];

% Outer rectangle (outside walls)
trackpts{2} = [-L_half-wt, -W_half-wt;
                L_half+wt, -W_half-wt;
                L_half+wt,  W_half+wt;
               -L_half-wt,  W_half+wt];


robot.spawn.allctrpts={}; % NEED TO FILL IN THIS

robot.spawn.startpos_init=Inputs.track.startposition;



% Main track region
plots.trackpoly=simplify(polyshape(trackpts{1}(:,1), trackpts{1}(:,2),'Simplify',false));
% Styles and plots the track
plot(plots.trackAx,plots.trackpoly,'FaceColor',Inputs.colors.track.tracklane,'edgecolor','none','FaceAlpha',1);

% Unite track walls into one polyshape
outerpoly = simplify(polyshape(trackpts{2}(:,1), trackpts{2}(:,2), 'Simplify', false)); % Added this
wallpoly = subtract(outerpoly, plots.trackpoly);

plots.poly.walls=wallpoly; %union(innerringpoly,outerringpoly);
plots.poly.track=plots.poly.walls;
% Plot track walls
plot(plots.trackAx,plots.poly.walls,'FaceColor',Inputs.colors.track.trackwalls,'edgecolor','none','FaceAlpha',1);


plots.poly.lineseglen = hypot(L, W);  % max diagonal of the track


plots.poly.obsSets={};
plots.poly.obsplt=plot(plots.trackAx,polyshape(),'FaceColor',Inputs.colors.track.obs,'edgecolor','none','FaceAlpha',1);
    
% obstacles = plots.obstacleArr;
% [~,n] = size(obstacles);
% for i = 1:n
%     obs = obstacles{i};
%     disp(obs);
%     plots = createObs(robot,plots,1,obs{1},obs{2},obs{3},obs{4},obs{5});
%     plots= setObs(plots,1);
% end
end
