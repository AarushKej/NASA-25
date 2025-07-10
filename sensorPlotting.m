function [robot,plots]=sensorPlotting(robot,plots)
    sensorangles=robot.sensor.ultrasonic.thetas'+robot.kinematics.theta-pi/2;
    vecs=[cos(sensorangles) sin(sensorangles)];
    linesegs=(plots.poly.lineseglen*vecs)+robot.center;
    robot.sensor.ultrasonic.sensorpts(:,1:2)=robot.center.*ones(robot.sensor.ultrasonic.Nsensors,2);
    for i=1:robot.sensor.ultrasonic.Nsensors
        LS_in=intersect(plots.poly.track,[robot.center;linesegs(i,:)]);
        [~,minloc]=min(vecnorm(LS_in-robot.center,2,2));
        robot.sensor.ultrasonic.sensorpts(i,3:4)=LS_in(minloc,:);
    end
    set(robot.sensor.ultrasonic.plts,{'XData'},num2cell(robot.sensor.ultrasonic.sensorpts(:,[1 3]),2),...
        {'YData'},num2cell(robot.sensor.ultrasonic.sensorpts(:,[2 4]),2));
    [robot,plots]=calcSensorDist(robot,plots);
    end

    function [robot,plots]=calcSensorDist(robot,plots)
    robot.sensor.ultrasonic.distances=vecnorm(robot.sensor.ultrasonic.sensorpts(:,3:4)-robot.sensor.ultrasonic.sensorpts(:,1:2),2,2);
    plots.mapping.sensordata=robot.sensor.ultrasonic.sensorpts(:,3:4);
    plots.mapping.LFdata=[];
    if robot.sensor.linefollower.left.state
        plots.mapping.LFdata=[plots.mapping.LFdata; robot.sensor.linefollower.sensorpts(1,:)];
    end
    if robot.sensor.linefollower.right.state
        plots.mapping.LFdata=[plots.mapping.LFdata; robot.sensor.linefollower.sensorpts(2,:)];
    end
end