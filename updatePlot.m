function plots=updatePlot(robot,plots)
limitoffset=(plots.trackWidth/2)*(1+plots.display.zoom);
if plots.plotsensordata
    if ~isempty(plots.mapping.sensordata)
        datatemp=unique([[plots.mapping.plts.sensordata.XData plots.mapping.sensordata(:,1)']'...
            [plots.mapping.plts.sensordata.YData plots.mapping.sensordata(:,2)']'],'rows');
        plots.mapping.plts.sensordata.XData=datatemp(:,1);
        plots.mapping.plts.sensordata.YData=datatemp(:,2);
        for i=1:robot.sensor.ultrasonic.Nsensors
            plots.mapping.plts.sensors(i).XData=plots.mapping.sensordata(i,1);
            plots.mapping.plts.sensors(i).YData=plots.mapping.sensordata(i,2);
        end
        if plots.plotLFline && ~isempty(plots.mapping.LFdata)
            datatempLF=unique([[plots.mapping.plts.LF.XData plots.mapping.LFdata(:,1)']'...
                [plots.mapping.plts.LF.YData plots.mapping.LFdata(:,2)']'],'rows');
            plots.mapping.plts.LF.XData=datatempLF(:,1);
            plots.mapping.plts.LF.YData=datatempLF(:,2);
            plots.mapping.LFdata=[];
        end
        plots.mapping.sensordata=[];
    end
    plots.mapping.plts.ctr.XData=robot.center(1);
    plots.mapping.plts.ctr.YData=robot.center(2);
    plots.trackAx.XLim=robot.center(1)+[-limitoffset limitoffset];
    plots.trackAx.YLim=robot.center(2)+[-limitoffset limitoffset];
    plots.mappingAx.XLim=robot.center(1)+[-limitoffset limitoffset];
    plots.mappingAx.YLim=robot.center(2)+[-limitoffset limitoffset];
else
    % plots.trackAx.XLim=robot.center(1)+[-limitoffset limitoffset];
    % plots.trackAx.YLim=robot.center(2)+[-limitoffset limitoffset];
end
end