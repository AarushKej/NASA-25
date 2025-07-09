function [robot,plots]=createTerrain(robot,plots,Inputs,UIAxes)
plots.trackAx = UIAxes;
axis(plots.trackAx,'square')
hold(plots.trackAx,'on')

% Vestigial: Pre-open-world
% showlanelines=Inputs.track.lanelines;
% Nlanes=round(Inputs.track.Nlanes);
w=Inputs.track.width;
% plots.laneWidth=Inputs.track.width/Nlanes;
% plots.Nlanes=Nlanes;
% innerradius=Inputs.track.innerradius;
% LFlinethickness=Inputs.track.LFlinethickness;
% WTlinethickness=w;
% showLFline=Inputs.track.showLFline;

% wallthickness=Inputs.track.wallthickness;
longwall_length=Inputs.track.wall1_length; % vertical
shortwall_length=Inputs.track.wall2_length; % horizontal

plots.trackHeight = longwall_length;
plots.trackWidth = shortwall_length;

% Vestigial
% if LFlinethickness>w
%     close(plots.trackAx.Parent);
%     error('LFlinethickness must be < track width.')
% end


% outerradius=innerradius+w;
% rline=mean([innerradius outerradius]);
% rin_radii=innerradius;
% rout_radii=outerradius;


% ptspacing=Inputs.track.ptspacing;

% if showLFline
%     rmax=7;
% else
% rmax=8;
% end

% L_vert=longwall_length/2;
% L_horz=shortwall_length/2;
% 
% trackpts=cell(rmax,1);
% for r=1:rmax
%     switch r
%         case 1 % inner ring, inside
%             radii=rin_radii-wallthickness;
%         case 2 % inner ring, outside
%             radii=rin_radii;
%         case 3 % outer ring, inside
%             radii=rout_radii;
%         case 4 % outer ring, outside
%             radii=rout_radii+wallthickness;
%         case 5 % LF ring, inside
%             radii=rline-(LFlinethickness/2);
%         case 6 % LF ring, outside
%             radii=rline+(LFlinethickness/2);
%         case 7 
%             radii=rline-(WTlinethickness/2);
%         case 8 
%             radii=rline+(WTlinethickness/2);
%     end
%     theta=linspace(0,pi/2,(pi/2)/(ptspacing/radii))';
%     qtrring=radii.*[cos(theta) sin(theta)];
%     curveptsqtr=qtrring+[L_horz L_vert];
%     trackpts{r}=unique(vertcat(curveptsqtr,flipud([-curveptsqtr(:,1) curveptsqtr(:,2)]),[-curveptsqtr(:,1) -curveptsqtr(:,2)],...
%         flipud([curveptsqtr(:,1) -curveptsqtr(:,2)])),'rows','stable');
% end

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

% WT (optional)
% wt_half = WTlinethickness / 2;
% trackpts{3} = [-L_half-wt_half, -W_half-wt_half;
%                 L_half+wt_half, -W_half-wt_half;
%                 L_half+wt_half,  W_half+wt_half;
%                -L_half-wt_half,  W_half+wt_half];

% % LF (optional)
% lf_half = Inputs.track.LFlinethickness / 2;
% trackpts{4} = [-L_half-lf_half, -W_half-lf_half;
%                 L_half+lf_half, -W_half-lf_half;
%                 L_half+lf_half,  W_half+lf_half;
%                -L_half-lf_half,  W_half+lf_half];





% lanewidth=w/Nlanes;
% lanemids=(innerradius+lanewidth/2)+(lanewidth*(0:Nlanes-1));
% lanecount=1;
% lanemidpts=cell(Nlanes,1);
% for radius=lanemids
%     theta=linspace(0,pi/2,(pi/2)/(ptspacing/radius))';
%     qtrring=radius.*[cos(theta) sin(theta)];
%     horzpts=linspace(0,-L_horz,L_horz/ptspacing)';
%     horzpts(:,2)=radius;
%     vertpts(:,2)=linspace(-L_vert,0,L_vert/ptspacing)';
%     vertpts(:,1)=radius;
%     qtrring=unique(vertcat(vertpts,qtrring,horzpts),'rows','stable');
%     curveptsqtr=qtrring+[L_horz L_vert];
%     lanemidpts{lanecount}=unique(vertcat(curveptsqtr,flipud([-curveptsqtr(:,1) curveptsqtr(:,2)]),[-curveptsqtr(:,1) -curveptsqtr(:,2)],...
%         flipud([curveptsqtr(:,1) -curveptsqtr(:,2)])),'rows','stable');
%     angletmppts=[lanemidpts{lanecount};lanemidpts{lanecount}(1,:)];
%     lanemidpts{lanecount}(:,3)=mod(atan2(diff(angletmppts(:,2)),diff(angletmppts(:,1))),2*pi);
%     plot(plots.trackAx,lanemidpts{lanecount}(:,1),lanemidpts{lanecount}(:,2),'m.');
%     lanecount=lanecount+1;
% end
% 

robot.spawn.allctrpts={}; % NEED TO FILL IN THIS
% robot.spawn.lane_init=Inputs.track.startlane;

robot.spawn.startpos_init=Inputs.track.startposition;

% robot.spawn.NLanes=Nlanes;


% Main track region
plots.trackpoly=simplify(polyshape(trackpts{1}(:,1), trackpts{1}(:,2),'Simplify',false));
% Styles and plots the track
plot(plots.trackAx,plots.trackpoly,'FaceColor',Inputs.colors.track.tracklane,'edgecolor','none','FaceAlpha',1);

% plots.poly.WT=simplify(polyshape(trackpts{3}(:,1), trackpts{3}(:,2),'Simplify',false));
% plot(plots.trackAx,plots.poly.WT,'FaceColor',Inputs.colors.track.WT,'edgecolor','none','FaceAlpha',1);


% hidden lf
% if showLFline
%     plots.poly.LF=simplify(polyshape({trackpts{5}(:,1), trackpts{6}(:,1)},{trackpts{5}(:,2), trackpts{6}(:,2)},'Simplify',false));
%     plot(plots.trackAx,plots.poly.LF,'FaceColor',Inputs.colors.track.Lfline,'edgecolor','none','FaceAlpha',1);
% end

% Create track wall polyshapes
% innerringpoly=simplify(polyshape({trackpts{1}(:,1), trackpts{2}(:,1)},{trackpts{1}(:,2), trackpts{2}(:,2)},'Simplify',false));
% outerringpoly=simplify(polyshape({trackpts{3}(:,1), trackpts{4}(:,1)},{trackpts{3}(:,2), trackpts{4}(:,2)},'Simplify',false));

% Unite track walls into one polyshape
outerpoly = simplify(polyshape(trackpts{2}(:,1), trackpts{2}(:,2), 'Simplify', false)); % Added this
wallpoly = subtract(outerpoly, plots.trackpoly);

plots.poly.walls=wallpoly; %union(innerringpoly,outerringpoly);
plots.poly.track=plots.poly.walls;
% Plot track walls
plot(plots.trackAx,plots.poly.walls,'FaceColor',Inputs.colors.track.trackwalls,'edgecolor','none','FaceAlpha',1);


plots.poly.lineseglen = hypot(L, W);  % max diagonal of the track

% Create and plot lane lines (vestigial)

% rlanes=innerradius+((w/Nlanes).*(1:Nlanes-1));
% lanelines=cell(length(rlanes),1);
% lanelines_all(1:length(rlanes)*2,1)={nan(1,2)};
% for r=1:length(rlanes)
%     theta=linspace(0,pi/2,(pi/2)/(ptspacing/rlanes(r)))';
%     qtrring=kron(rlanes(r),[cos(theta) sin(theta)])+[L_horz L_vert];
%     lanelines{r}=vertcat(qtrring,flipud([-qtrring(:,1) qtrring(:,2)]),...
%         [-qtrring(:,1) -qtrring(:,2)],flipud([qtrring(:,1) -qtrring(:,2)]),qtrring(1,:));
% end
% lanelines_all(1:2:end-1)=lanelines;
% plots.lanelines=vertcat(lanelines_all{:});
% if showlanelines
%     plot(plots.trackAx,plots.lanelines(:,1),plots.lanelines(:,2),'Color',Inputs.colors.track.lanelines);
% end

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
