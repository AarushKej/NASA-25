function stateID = getState(robot, tracksize)
    sensors = robot.sensor.ultrasonic.distances;
    state = zeros(1,length(sensors));
    % Set these as needed
    BLOCKED_THRESHOLD = tracksize / 3;
    MEDIUM_THRESHOLD = tracksize / 2.5;

    FWD_BLOCKED_THRESHOLD = tracksize / 2.65;
    FWD_MEDIUM_THRESHOLD = tracksize / 1.5;

    FWD_BLOCKED_THRESHOLD = robot.dims(1,2)/2;
    FWD_CLEAR_THRESHOLD = robot.dims(1,2)*2;

    thresholds = linspace(FWD_BLOCKED_THRESHOLD,FWD_CLEAR_THRESHOLD, robot.numStates-1);
    for i = 1:length(sensors)
        status = -1;
        for j = 1:length(thresholds);
            if(sensors(i) <= thresholds(j))
                if(status == -1)
                    status = j;
                end
            end
        end
        if(status == -1)
            status = robot.numStates;
        end
        state(i) = status;
    end   
    
   
    % for i = 1:length(sensors)
    %   if i == 3
    %       if sensors(i) <= FWD_BLOCKED_THRESHOLD
    %           state(i) = 1; % 1 state means blocked
    %       elseif sensors(i) <= FWD_MEDIUM_THRESHOLD && sensors(i) > FWD_BLOCKED_THRESHOLD
    %           state(i) = 2; % 2 state means the obstacle is medium distance away
    %       else
    %           state(i) = 3; % 2 state means the area is clear
    %           end
    %       else
    %           if sensors(i) <= BLOCKED_THRESHOLD
    %               state(i) = 1; % 1 state means blocked
    %           elseif sensors(i) <= MEDIUM_THRESHOLD && sensors(i) > BLOCKED_THRESHOLD
    %               state(i) = 2; % 2 state means the obstacle is medium distance away
    %           else
    %               state(i) = 3; % 2 state means the area is clear
    %           end
    %       end
    %   end
    % end
    
    stateID = robot.statesDict({state});
end