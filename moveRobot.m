function [robot,plots]=moveRobot(robot,plots)
VrightDir=robot.motor.dir.right*robot.kinematics.V.right;
VleftDir=robot.motor.dir.left*robot.kinematics.V.left;
robotverttemp=vertcat(plots.robotpatch.patch.Vertices,robot.center,robot.kinematics.axle);
if round(robot.kinematics.V.left,4)==0 && round(robot.kinematics.V.right,4)==0
    return
elseif round(VrightDir,4)~=round(VleftDir,4)
    robot.kinematics.R=(robot.kinematics.L/2)*(VrightDir+VleftDir)/(VrightDir-VleftDir);
    robot.kinematics.omega=(VrightDir-VleftDir)/robot.kinematics.L;
    ICC=[robot.kinematics.axle(1)-(robot.kinematics.R*sin(robot.kinematics.theta))...
        robot.kinematics.axle(2)+(robot.kinematics.R*cos(robot.kinematics.theta))];
    deltatheta=robot.kinematics.omega*robot.kinematics.dt;
    thetatemp=robot.kinematics.theta+deltatheta;
    robotverttemp=robotverttemp-ICC;
    robotverttemp=[sum(robotverttemp.*[cos(deltatheta) -sin(deltatheta)],2) sum(robotverttemp.*[sin(deltatheta) cos(deltatheta)],2)]+ICC;
else
    thetatemp=robot.kinematics.theta;
    movedelta=robot.kinematics.V.right*robot.motor.dir.right*robot.kinematics.dt*[cos(thetatemp) sin(thetatemp)];
    robotverttemp=robotverttemp+movedelta;
end

verts = plots.robotpatch.patch.Vertices;

if any(isinterior(plots.poly.track,verts)) ||...
        any(robot.sensor.ultrasonic.distances<=robot.sensor.ultrasonic.collisiondist)
    robot.crashed = true;
end

robot.objDist = norm(robot.obj.coords - robot.center);

% find the minimum distance any point of the robot is from the objective
% diffs = verts - repmat(robot.obj.coords, size(verts, 1), 1);
% dists = diffs(:,1).^2 + diffs(:, 2).^2;
% minDist = sqrt(min(dists));

%vecnorm(verts-robot.obj.coords,2,2)

% if the minimum distance is less than the radius of the obj, it intersects
robot.arrived = false;
if robot.objDist < robot.obj.radius * 1.2
    robot.arrived = true;
end

if robot.crashed
    plots.robotpatch.patch.CData(1,1,:)=plots.robotpatch.colvec(1,8,:);
    robot.kinematics.V.left=0;
    robot.kinematics.V.right=0;
elseif robot.arrived
    plots.robotpatch.patch.CData(1,1,:)=[0, 1, 0];
    robot.kinematics.V.left=0;
    robot.kinematics.V.right=0;
else
    robot.kinematics.theta=thetatemp;
    robot.center=robotverttemp(end-1,:);
    robot.kinematics.axle=robotverttemp(end,:);
    plots.robotpatch.patch.Vertices=robotverttemp(1:end-2,:);
    plots.robotpatch.patch.CData(1,1,:)=plots.robotpatch.colvec(1,1,:);
    robot=updateLineFollower(robot,plots);
    [robot,plots]=sensorPlotting(robot,plots);
    [robot,plots]=lidarPlotting(robot,plots);
end

end