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

if ~(any(isinterior(plots.poly.track,plots.robotpatch.patch.Vertices)) ||...
        any(robot.sensor.ultrasonic.distances<=robot.sensor.ultrasonic.collisiondist))
    robot.kinematics.theta=thetatemp;
    robot.center=robotverttemp(end-1,:);
    robot.kinematics.axle=robotverttemp(end,:);
    plots.robotpatch.patch.Vertices=robotverttemp(1:end-2,:);
    plots.robotpatch.patch.CData(1,1,:)=plots.robotpatch.colvec(1,1,:);
    robot=updateLineFollower(robot,plots);
    [robot,plots]=sensorPlotting(robot,plots);
else
    plots.robotpatch.patch.CData(1,1,:)=plots.robotpatch.colvec(1,8,:);
    robot.kinematics.V.left=0;
    robot.kinematics.V.right=0;
    robot.crashed=true;
end
end