function robot=updateLineFollower(robot,plots)
if ~plots.plotLFline
    return
end
robot.sensor.linefollower.sensorpts=[mean(plots.robotpatch.patch.Vertices(plots.robotpatch.patch.Faces(4,1:4),1))...
    mean(plots.robotpatch.patch.Vertices(plots.robotpatch.patch.Faces(4,1:4),2));...
    mean(plots.robotpatch.patch.Vertices(plots.robotpatch.patch.Faces(5,1:4),1))...
    mean(plots.robotpatch.patch.Vertices(plots.robotpatch.patch.Faces(5,1:4),2))];
sensor_in=isinterior(plots.poly.LF,robot.sensor.linefollower.sensorpts);
if sensor_in(1)
    plots.robotpatch.patch.CData(4,:,:)= plots.robotpatch.colvec(1,6,:);
    robot.sensor.linefollower.left.state=true;
else
    plots.robotpatch.patch.CData(4,:,:)=plots.robotpatch.colvec(1,4,:);
    robot.sensor.linefollower.left.state=false;
end
if sensor_in(2)
    plots.robotpatch.patch.CData(5,:,:)= plots.robotpatch.colvec(1,7,:);
    robot.sensor.linefollower.right.state=true;
else
    plots.robotpatch.patch.CData(5,:,:)=plots.robotpatch.colvec(1,4,:);
    robot.sensor.linefollower.right.state=false;
end
end