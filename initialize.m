function [robot,plots,sim]=initialize(app)
robot = app.robot;
plots = app.plots;
sim = app.sim;
inputs = app.inputs;
sim.episode = 0;
X = app.X;
Y = app.Y;
Rand = app.Rand;




% Create state combinations dictionary: state -> stateId, e.g. [1 3 2 4 1] -> 17  
robot.numSensors = 5;


robot.actions = ["forward", "left", "right"];
robot.numActions = length(robot.actions);


robot.dims=[9 17];
robot.wheels.dims.back=[1.6 6.4];
% if initial
    robot.dims(1,1) = inputs.robot.width;
    robot.dims(1,2) = inputs.robot.length;
    errorcheck(inputs,robot);
    robot.sensor.ultrasonic.sensorcolors=inputs.colors.ultrasonic.sensorcolors;
    robot.reset=inputs.resetRobot;
    robot.sim.realtime=inputs.realtime;
    plots.plotsensordata=inputs.display.showsensordata;
    plots.plotLFline=inputs.track.showLFline;
    plots.display.HUD.show=inputs.display.HUD.show;
    plots.display.zoom=inputs.display.Zoom;
    robot.sensor.ultrasonic.thetas=inputs.ultrasonic.define;
    robot.sensor.ultrasonic.Nsensors=length(robot.sensor.ultrasonic.thetas);
    robot.sensor.ultrasonic.sensorpts=zeros(length(robot.sensor.ultrasonic.thetas),4);
    robot.sensor.ultrasonic.collisiondist=inputs.ultrasonic_collisiondist;

    robot.obj=struct();

    plots.trackAx = app.UIAxes;
    [robot,plots]=createTerrain(robot,plots,inputs,app.UIAxes);
    [robot, plots] = setObjective(robot,plots,X,Y,Rand);

    plots.ptspacing=inputs.track.ptspacing;
    robot.crashed=false;
    robot.kinematics.dt=inputs.dt;


    % if plots.display.HUD.show
    %     plots.display.sensor.ultrasonic.txt=text(plots.trackAx,1,1,sprintf('\n\n\n\n\n\n\n\n\n'),'Units',...
    %         'normalized','HorizontalAlignment','left','VerticalAlignment','top');
    %     plots.display.timeelapsed.trueT.title=text(plots.trackAx,1,1,sprintf('\n\n Real Time'),'Units',...
    %         'normalized','HorizontalAlignment','left','VerticalAlignment','top','fontweight','bold');
    %     plots.display.timeelapsed.simT.title=text(plots.trackAx,1,1,sprintf('\n\n\n\n\n Sim. Time'),'Units',...
    %         'normalized','HorizontalAlignment','left','VerticalAlignment','top','fontweight','bold');
    % end
    

    [robot,plots] = initializeRobot(robot,plots,inputs);

    robot.objDist = norm(robot.obj.coords - robot.center);
    robot.prevObjDist = robot.objDist;

    % if plots.plotsensordata
    %     plots.mapping.plts.sensordata=plot(plots.mappingAx,plots.mapping.sensordata(1:robot.sensor.ultrasonic.Nsensors,1),...
    %         plots.mapping.sensordata(1:robot.sensor.ultrasonic.Nsensors,2),'k.');
    %     for i=1:robot.sensor.ultrasonic.Nsensors
    %         plots.mapping.plts.sensors(i)=plot(plots.mappingAx,plots.mapping.sensordata(i,1),plots.mapping.sensordata(i,2),...
    %             'Color',robot.sensor.ultrasonic.sensorcolors{i},'Marker','x','linewidth',1.5);
    %     end
    %     plots.mapping.plts.ctr=plot(plots.mappingAx,robot.center(1),robot.center(2),'r*');
    %     plots.mappingdata=[];
    %     if plots.plotLFline
    %         plots.mapping.plts.LF=plot(plots.mappingAx,nan,nan,'g.');
    %     end
    % end



    
    plots.display.framecount=1;
    plots.display.framecountmax=5;
    plots.display.fpsvec=zeros(plots.display.framecountmax,1);
    % 
    plots.display.timeelapsed.true=0;
    plots.display.timeelapsed.sim=0;
    plots.display.avgframetime=.01;
    % 
    % robot=motor(robot,'right','power',5);
    % robot=motor(robot,'left','power',5);
    % 
    % Ntimebuffer=5;
    % for n=1:Ntimebuffer
    %     [robot,plots]=executeMove(robot,plots,true);
    %     drawnow
    %     if robot.crashed
    %         [robot,plots]=resetRobot(robot,plots);
    %         break;
    %     end
    % end
    % for n=1:plots.display.framecountmax
    %     [robot,plots]=executeMove(robot,plots,true);
    %     if robot.crashed
    %         break
    %     end
    % end
    % robot.kinematics.V.right=0;
    % robot.kinematics.V.left=0;
    % [robot,plots]=resetRobot(robot,plots);
    % plots.display.timeelapsed.true=0;
    % plots.display.timeelapsed.sim=0;
    % plots.display.framecount=1;
    % plots.display.avgframetime=mean(1./plots.display.fpsvec(~isoutlier(plots.display.fpsvec)));
    % plots.display.framecountmax=50;
    % plots.display.fpsvec=zeros(plots.display.framecountmax,1);
    % if robot.sim.realtime
    %     robot.kinematics.dt=plots.display.avgframetime;
    % end
    % if plots.display.HUD.show
    %     plots.display.fps.text=text(plots.trackAx,1,1,[' ' num2str(round(1/plots.display.avgframetime)) ' FPS'],'Units',...
    %         'normalized','HorizontalAlignment','left','VerticalAlignment','top');
    %     plots.display.timeelapsed.trueT.txt=text(plots.trackAx,1,1,sprintf(['\n\n\n ' datestr(0,'MM:SS.FFF')])...
    %         ,'Units','normalized','HorizontalAlignment','left','VerticalAlignment','top');
    %     plots.display.timeelapsed.simT.txt=text(plots.trackAx,1,1,sprintf(['\n\n\n\n\n\n ' datestr(0,'MM:SS.FFF')])...
    %         ,'Units','normalized','HorizontalAlignment','left','VerticalAlignment','top');
    % end
    % plots.trackAx.Parent.Visible='on';
    % pause(1);
    
% else
    % robot.crashed=false;
    % 
    % robot.center=robot.spawn.origin;
    % robot.kinematics.axle=robot.spawn.axle;
    % robot.kinematics.theta=robot.spawn.heading;
    % plots.robotpatch.patch.Vertices=robot.spawn.patchpts;
    % 
    % plots.display.timeelapsed.true=0;
    % plots.display.timeelapsed.sim=0;
    % if plots.plotsensordata
    %     plots.mapping.plts.sensordata.XData=[];
    %     plots.mapping.plts.sensordata.YData=[];
    %     plots.mapping.plts.LF.XData=[];
    %     plots.mapping.plts.LF.YData=[];
    %     plots.mapping.plts.ctr.XData=robot.center(1);
    %     plots.mapping.plts.ctr.YData=robot.center(2);
    % end
    % robot=updateLineFollower(robot,plots);
    % [robot,plots]=sensorPlotting(robot,plots);
    % plots=updatePlot(robot,plots);
    % plots.robotpatch.patch.CData(1,1,:)=plots.robotpatch.colvec(1,1,:);
    % if plots.display.HUD.show
    %     plots.display.timeelapsed.trueT.txt.String=sprintf(['\n\n\n ' datestr(plots.display.timeelapsed.true/(24*60*60),...
    %         'MM:SS.FFF')]);
    %     plots.display.timeelapsed.simT.txt.String=sprintf(['\n\n\n\n\n\n ' datestr(plots.display.timeelapsed.sim/(24*60*60),...
    %         'MM:SS.FFF')]);
    % end
% end


end


function errorcheck(inputs,robot)
assert(inputs.ultrasonic_collisiondist>=0,'''inputs.ultrasonic_collision'' must be >=0')
assert(inputs.track.ptspacing>0,'''inputs.track.ptspacing'' must be >0');


assert(inputs.track.wall1_length>=0 && inputs.track.wall2_length>=0,'''wall1_length'' and ''wall2_length'' must be >0.')


assert(inputs.display.Zoom>0,'''Zoom'' must be >0')
assert(inputs.realtime || inputs.dt>0,'''dt'' must be >0')

assert(islogical(inputs.track.showLFline),'''showLFline'' must be true or false.')

assert(islogical(inputs.display.showsensordata),'''showsensordata'' must be true or false.')
assert(islogical(inputs.display.HUD.show),'''HUD.show'' must be true or false.')
assert(islogical(inputs.resetRobot),'''resetRobot'' must be true or false.')
assert(islogical(inputs.realtime),'''realtime'' must be true or false.')
colsin=[{inputs.colors.track.trackwalls, inputs.colors.track.tracklane, inputs.colors.track.lanelines, inputs.colors.track.Lfline,inputs.colors.track.obs,...
    inputs.colors.robot.crash} inputs.colors.ultrasonic.sensorcolors];
for i=1:length(colsin)
    checkcolor(colsin{i});
end
end


