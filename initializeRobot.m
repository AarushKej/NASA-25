function [robot,plots] = initializeRobot(robot,plots, Inputs)
robot.plot.closed=false;
robot.motor.power.right=0;
robot.motor.power.left=0;
robot.motor.power.max=5;
robot.kinematics.V.right=0;
robot.kinematics.V.left=0;
robot.motor.dir.right=1;
robot.motor.dir.left=1;
robot.motor.wheeldia=robot.wheels.dims.back(2);
robot.motor.VtoRPM=[3 120; 4.5 185; 6 250];
robot.motor.loadfactor=.8;
robot.motor.poly=polyfit(robot.motor.VtoRPM(:,1),robot.motor.loadfactor*robot.motor.VtoRPM(:,2),1);
robot.loop = 0;
robot.random = true;
robot.set.spawn.x = 0;
robot.set.spawn.y = 0;
robot.angie = 0;

robot.kinematics.L=robot.dims(1)+(2*(robot.wheels.dims.back(1)/2));
robot.kinematics.R=0;
robot.kinematics.omega=0;

robot.body.x=[-robot.dims(1)/2 +robot.dims(1)/2 ...
    robot.dims(1)/2 -robot.dims(1)/2];
robot.body.y=[-robot.dims(2)/2 -robot.dims(2)/2 ...
    robot.dims(2)/2 robot.dims(2)/2];

robot.wheels.backright.x=[robot.body.x(2) robot.body.x(2)+robot.wheels.dims.back(1) ...
    robot.body.x(2)+robot.wheels.dims.back(1) robot.body.x(2)];
robot.wheels.backright.y=[robot.body.y(1) robot.body.y(1) robot.body.y(1)+robot.wheels.dims.back(2)...
    robot.body.y(1)+robot.wheels.dims.back(2)];
robot.wheels.backleft.x=[robot.body.x(1)-robot.wheels.dims.back(1)...
    robot.body.x(1) robot.body.x(1) robot.body.x(1)-robot.wheels.dims.back(1)];
robot.wheels.backleft.y=robot.wheels.backright.y;

robot.sensor.linefollower.dims=[1.25 1];
robot.sensor.linefollower.sensorpts=zeros(2,2);
robot.sensor.linefollower.right.x=[robot.body.x(2) robot.body.x(2)-robot.sensor.linefollower.dims(1)...
    robot.body.x(2)-robot.sensor.linefollower.dims(1) robot.body.x(2)]-(robot.dims(1)/2-robot.sensor.linefollower.dims(1));
robot.sensor.linefollower.right.y=[robot.body.y(3)+robot.sensor.linefollower.dims(2) robot.body.y(3)+robot.sensor.linefollower.dims(2)...
    robot.body.y(3) robot.body.y(3)];
robot.sensor.linefollower.left.x=[robot.body.x(1)+robot.sensor.linefollower.dims(1) robot.body.x(1)...
    robot.body.x(1) robot.body.x(1)+robot.sensor.linefollower.dims(1)]+(robot.dims(1)/2-robot.sensor.linefollower.dims(1));
robot.sensor.linefollower.left.y=robot.sensor.linefollower.right.y;

robot.sensor.linefollower.right.state=false;
robot.sensor.linefollower.left.state=false;

robot.sensor.ultrasonic.plts=gobjects(1,length(robot.sensor.ultrasonic.sensorpts(:,1)));
if isempty(find(cellfun(@(x) isequal(x,Inputs.colors.ultrasonic.linestyle),{'-','--',':','-.'}),1))
    error('Invalid ultrasonic line style.')
end
for i=1:robot.sensor.ultrasonic.Nsensors
    robot.sensor.ultrasonic.plts(i)=plot(plots.trackAx,robot.sensor.ultrasonic.sensorpts(i,[1 3]),...
        robot.sensor.ultrasonic.sensorpts(i,[2 4]),'Color',robot.sensor.ultrasonic.sensorcolors{i},...
        'LineStyle',Inputs.colors.ultrasonic.linestyle, 'Visible', Inputs.ultrasonic.visible);
end

robot.sensor.lidar.hitPtsPlot = plot(plots.trackAx, nan, nan, 'r.', 'linewidth', 0.1);


robot_body=[robot.body.x' robot.body.y'];
robot_body=vertcat(robot_body,robot_body(1,:));
ptdist=5;
robot_body_all=zeros(sum(round([robot.dims robot.dims]/ptdist)),2);
nsideptstotal=1;
for i=1:4
    nsidepts=round(norm(robot_body(i+1,:)-robot_body(i,:))/ptdist)+1;
    robot_body_all(nsideptstotal:nsideptstotal+nsidepts-1,:)=[linspace(robot_body(i,1),robot_body(i+1,1),nsidepts)'...
        linspace(robot_body(i,2),robot_body(i+1,2),nsidepts)'];
    nsideptstotal=nsideptstotal+nsidepts;
end
robot.spawn.patchpts0=[[robot_body_all(:,1)' robot.wheels.backleft.x robot.wheels.backright.x robot.sensor.linefollower.left.x...
    robot.sensor.linefollower.right.x]'...
    [robot_body_all(:,2)' robot.wheels.backleft.y robot.wheels.backright.y  robot.sensor.linefollower.left.y...
    robot.sensor.linefollower.right.y]'];

if robot.random == true
    robot=setSpawn(robot,plots,0,0,true);
else
    robot=setSpawn(robot, plots, robot.set.spawn.x, robot.set.spawn.y, false);
end

robot.center=robot.spawn.origin;
robot.kinematics.axle=robot.spawn.axle;
robot.kinematics.theta=robot.spawn.heading;

colsin={Inputs.colors.robot.body,Inputs.colors.robot.wheels,Inputs.colors.robot.LF_Idle, Inputs.colors.robot.LF_Trigger,...
    Inputs.colors.robot.crash};
col=zeros(length(colsin),3);
for i=1:length(colsin)
    col(i,:)=checkcolor(colsin{i});
end
plots.robotpatch.colvec(1,1,:)=col(1,:);
plots.robotpatch.colvec(1,2,:)=col(2,:);
plots.robotpatch.colvec(1,3,:)=col(2,:);
plots.robotpatch.colvec(1,4,:)=col(3,:);
plots.robotpatch.colvec(1,5,:)=col(3,:);
plots.robotpatch.colvec(1,6,:)=col(4,:);
plots.robotpatch.colvec(1,7,:)=col(4,:);
plots.robotpatch.colvec(1,8,:)=col(5,:);

plots.robotpatch.faces=nan(5,length(robot_body_all(:,1)));
plots.robotpatch.faces(1,:)=1:length(robot_body_all(:,1));
plots.robotpatch.faces(2:5,1:4)=reshape(1:16,[4,4])'+length(robot_body_all(:,1));
plots.robotpatch.patch = patch(plots.trackAx, 'Faces', plots.robotpatch.faces, ...
    'Vertices', robot.spawn.patchpts, ...
    'FaceColor', 'flat', ...
    'EdgeColor', 'k', ...
    'CData', plots.robotpatch.colvec(1,1:5,:));
robot.plotHandle = plots.robotpatch.patch;
plots.mapping.LFdata=[];
robot=updateLineFollower(robot,plots);
robot.sensor.ultrasonic.distances=[];
[robot,plots]=sensorPlotting(robot,plots);
[robot,plots]=lidarPlotting(robot,plots);
plots.mapping.plts.sensors=gobjects(1,robot.sensor.ultrasonic.Nsensors+1);
end
