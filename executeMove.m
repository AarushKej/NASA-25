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
if robot.crashed && ~init || robot.arrived && ~init
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