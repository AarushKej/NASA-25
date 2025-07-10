function [robot,plots]=resetRobot(robot,plots)
    try
        %[robot,plots]=initialize(false,robot,plots,[]);
        robot.crashed=false;
        robot.center=robot.spawn.origin;
        robot.kinematics.axle=robot.spawn.axle;
        robot.kinematics.theta=robot.spawn.heading;
        plots.robotpatch.patch.Vertices=robot.spawn.patchpts;
        plots.display.timeelapsed.true=0;
        plots.display.timeelapsed.sim=0;
        if plots.plotsensordata
            plots.mapping.plts.sensordata.XData=[];
            plots.mapping.plts.sensordata.YData=[];
            plots.mapping.plts.LF.XData=[];
            plots.mapping.plts.LF.YData=[];
            plots.mapping.plts.ctr.XData=robot.center(1);
            plots.mapping.plts.ctr.YData=robot.center(2);
        end
        robot=updateLineFollower(robot,plots);
        [robot,plots]=sensorPlotting(robot,plots);
        plots=updatePlot(robot,plots);
        plots.robotpatch.patch.CData(1,1,:)=plots.robotpatch.colvec(1,1,:);
        if plots.display.HUD.show
            plots.display.timeelapsed.trueT.txt.String=sprintf(['\n\n\n ' datestr(plots.display.timeelapsed.true/(24*60*60),...
                'MM:SS.FFF')]);
            plots.display.timeelapsed.simT.txt.String=sprintf(['\n\n\n\n\n\n ' datestr(plots.display.timeelapsed.sim/(24*60*60),...
                'MM:SS.FFF')]);
        end
        [robot.lap.theta.prev,~]=cart2pol(robot.kinematics.axle(1),robot.kinematics.axle(2));
        robot.lap.theta.prev=mod(robot.lap.theta.prev,2*pi);
        robot.lap.theta.cur=robot.lap.theta.prev;
        robot.lap.Nlap=0;
        robot.lap.theta.total=0;
        %plots=updatePlot(robot,plots);
    catch e
        if ~isempty(strfind(e.message,'deleted'))
            robot.plot.closed=true;
            return
        else
            close(plots.trackAx.Parent);
            error(['error: ' e.message]);
        end
    end
end