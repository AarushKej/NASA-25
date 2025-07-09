function [robot,plots]=createTerrain(robot,plots,Inputs,UIAxes)
plots.trackAx = UIAxes;
axis(plots.trackAx,'square')
hold(plots.trackAx,'on')
showlanelines=Inputs.track.lanelines;
Nlanes=round(Inputs.track.Nlanes);
w=Inputs.track.width;
plots.trackWidth = w;
plots.laneWidth=Inputs.track.width/Nlanes;
plots.Nlanes=Nlanes;
innerradius=Inputs.track.innerradius;
LFlinethickness=Inputs.track.LFlinethickness;
WTlinethickness=w;
showLFline=Inputs.track.showLFline;
wallthickness=Inputs.track.wallthickness;
longwall_length=Inputs.track.wall1_length;
shortwall_length=Inputs.track.wall2_length;
if LFlinethickness>w
    close(plots.trackAx.Parent);
    error('LFlinethickness must be < track width.')
end


outerradius=innerradius+w;
rline=mean([innerradius outerradius]);
rin_radii=innerradius;
rout_radii=outerradius;
ptspacing=Inputs.track.ptspacing;

if showLFline
    rmax=7;
else
    rmax=8;
end

L_vert=longwall_length/2;
L_horz=shortwall_length/2;

trackpts=cell(rmax,1);
for r=1:rmax
    switch r
        case 1 % inner ring, inside
            radii=rin_radii-wallthickness;
        case 2 % inner ring, outside
            radii=rin_radii;
        case 3 % outer ring, inside
            radii=rout_radii;
        case 4 % outer ring, outside
            radii=rout_radii+wallthickness;
        case 5 % LF ring, inside
            radii=rline-(LFlinethickness/2);
        case 6 % LF ring, outside
            radii=rline+(LFlinethickness/2);
        case 7 
            radii=rline-(WTlinethickness/2);
        case 8 
            radii=rline+(WTlinethickness/2);
    end
    theta=linspace(0,pi/2,(pi/2)/(ptspacing/radii))';
    qtrring=radii.*[cos(theta) sin(theta)];
    curveptsqtr=qtrring+[L_horz L_vert];
    trackpts{r}=unique(vertcat(curveptsqtr,flipud([-curveptsqtr(:,1) curveptsqtr(:,2)]),[-curveptsqtr(:,1) -curveptsqtr(:,2)],...
        flipud([curveptsqtr(:,1) -curveptsqtr(:,2)])),'rows','stable');
end

lanewidth=w/Nlanes;
lanemids=(innerradius+lanewidth/2)+(lanewidth*(0:Nlanes-1));
lanecount=1;
lanemidpts=cell(Nlanes,1);
for radius=lanemids
    theta=linspace(0,pi/2,(pi/2)/(ptspacing/radius))';
    qtrring=radius.*[cos(theta) sin(theta)];
    horzpts=linspace(0,-L_horz,L_horz/ptspacing)';
    horzpts(:,2)=radius;
    vertpts(:,2)=linspace(-L_vert,0,L_vert/ptspacing)';
    vertpts(:,1)=radius;
    qtrring=unique(vertcat(vertpts,qtrring,horzpts),'rows','stable');
    curveptsqtr=qtrring+[L_horz L_vert];
    lanemidpts{lanecount}=unique(vertcat(curveptsqtr,flipud([-curveptsqtr(:,1) curveptsqtr(:,2)]),[-curveptsqtr(:,1) -curveptsqtr(:,2)],...
        flipud([curveptsqtr(:,1) -curveptsqtr(:,2)])),'rows','stable');
    angletmppts=[lanemidpts{lanecount};lanemidpts{lanecount}(1,:)];
    lanemidpts{lanecount}(:,3)=mod(atan2(diff(angletmppts(:,2)),diff(angletmppts(:,1))),2*pi);
    plot(plots.trackAx,lanemidpts{lanecount}(:,1),lanemidpts{lanecount}(:,2),'m.');
    lanecount=lanecount+1;
end

robot.spawn.allctrpts=lanemidpts;
robot.spawn.lane_init=Inputs.track.startlane;
robot.spawn.startpos_init=Inputs.track.startposition;
robot.spawn.NLanes=Nlanes;



trackpoly=simplify(polyshape({trackpts{2}(:,1), trackpts{3}(:,1)},{trackpts{2}(:,2), trackpts{3}(:,2)},'Simplify',false));
plot(plots.trackAx,trackpoly,'FaceColor',Inputs.colors.track.tracklane,'edgecolor','none','FaceAlpha',1);

innerringpoly=simplify(polyshape({trackpts{1}(:,1), trackpts{2}(:,1)},{trackpts{1}(:,2), trackpts{2}(:,2)},'Simplify',false));
outerringpoly=simplify(polyshape({trackpts{3}(:,1), trackpts{4}(:,1)},{trackpts{3}(:,2), trackpts{4}(:,2)},'Simplify',false));

plots.poly.WT=simplify(polyshape({trackpts{7}(:,1), trackpts{8}(:,1)},{trackpts{7}(:,2), trackpts{8}(:,2)},'Simplify',false));
plot(plots.trackAx,plots.poly.WT,'FaceColor',Inputs.colors.track.WT,'edgecolor','none','FaceAlpha',1);

if showLFline
    plots.poly.LF=simplify(polyshape({trackpts{5}(:,1), trackpts{6}(:,1)},{trackpts{5}(:,2), trackpts{6}(:,2)},'Simplify',false));
    plot(plots.trackAx,plots.poly.LF,'FaceColor',Inputs.colors.track.Lfline,'edgecolor','none','FaceAlpha',1);
end
plots.poly.walls=union(innerringpoly,outerringpoly);
plots.poly.track=plots.poly.walls;
plot(plots.trackAx,plots.poly.walls,'FaceColor',Inputs.colors.track.trackwalls,'edgecolor','none','FaceAlpha',1);
plots.poly.lineseglen=2*sqrt(((max([Inputs.track.wall1_length Inputs.track.wall2_length])+(2*rin_radii)+(2*w)+(2*wallthickness))^2)+(w^2));
rlanes=innerradius+((w/Nlanes).*(1:Nlanes-1));
lanelines=cell(length(rlanes),1);
lanelines_all(1:length(rlanes)*2,1)={nan(1,2)};
for r=1:length(rlanes)
    theta=linspace(0,pi/2,(pi/2)/(ptspacing/rlanes(r)))';
    qtrring=kron(rlanes(r),[cos(theta) sin(theta)])+[L_horz L_vert];
    lanelines{r}=vertcat(qtrring,flipud([-qtrring(:,1) qtrring(:,2)]),...
        [-qtrring(:,1) -qtrring(:,2)],flipud([qtrring(:,1) -qtrring(:,2)]),qtrring(1,:));
end
lanelines_all(1:2:end-1)=lanelines;
plots.lanelines=vertcat(lanelines_all{:});
if showlanelines
    plot(plots.trackAx,plots.lanelines(:,1),plots.lanelines(:,2),'Color',Inputs.colors.track.lanelines);
end

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
