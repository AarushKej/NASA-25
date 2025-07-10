function [robot,plots,crash]=executeMove(robot,plots,init)
    try
        crash=false;
        plots.display.timer=tic;
        if robot.sim.realtime && plots.display.timeelapsed.true>plots.display.timeelapsed.sim
            robot.kinematics.dt=plots.display.avgframetime+(plots.display.timeelapsed.true-plots.display.timeelapsed.sim);
            if robot.kinematics.dt>2*plots.display.avgframetime
                robot.kinematics.dt=2*plots.display.avgframetime;
            end
        end
        if plots.display.framecount>plots.display.framecountmax
            plots.display.framecount=1;
            if plots.display.HUD.show
                plots.display.fps.text.String=[' ' num2str(round(mean(plots.display.fpsvec))) ' FPS'];
            end
        else
            plots.display.fpsvec(plots.display.framecount)=1/plots.display.avgframetime;
        end
        [robot,plots]=moveRobot(robot,plots);
        %plots=updatePlot(robot,plots);
        if ~init
            robot=checklap(robot);
        end
        plots.display.timeelapsed.sim=plots.display.timeelapsed.sim+robot.kinematics.dt;
        drawnow
        plots.display.avgframetime=((plots.display.avgframetime*(plots.display.framecount-1))+toc(plots.display.timer))/plots.display.framecount;
        while plots.display.timeelapsed.true+toc(plots.display.timer)<plots.display.timeelapsed.sim && robot.sim.realtime
        end
        plots.display.framecount=plots.display.framecount+1;
        plots.display.timeelapsed.true=plots.display.timeelapsed.true+toc(plots.display.timer);
    catch e
        if ~isempty(strfind(e.message,'deleted'))
            robot.plot.closed=true;
            return
        else
            error(['error: ' e.message]);
        end
    end
    crash=robot.crashed;
    if robot.crashed && ~init
        if robot.reset
            pause(.5);
            [robot,plots]=resetRobot(robot,plots);
            pause(.5);
        else
            %robot.plot.closed=true;
            return
        end
    end
    end

    function robot=checklap(robot)
    [robot.lap.theta.cur,~]=cart2pol(robot.kinematics.axle(1),robot.kinematics.axle(2));
    if robot.lap.theta.cur>robot.lap.theta.prev
        robot.lap.theta.total=robot.lap.theta.total+mod(robot.lap.theta.cur-robot.lap.theta.prev,2*pi);
    end
    if robot.lap.theta.total>=2*pi && robot.motor.dir.right==1 && robot.motor.dir.left==1
        robot.lap.theta.total=0;
        robot.lap.Nlap=robot.lap.Nlap+1;
    end
    robot.lap.theta.prev=robot.lap.theta.cur;
end