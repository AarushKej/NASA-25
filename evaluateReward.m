function rewards = evaluateReward(robot, action)
    rewards = 0;
    if (robot.crashed)
        rewards = -1000;
        return
    end
    distances = robot.sensor.ultrasonic.distances;
    prev = robot.sensor.prevDistances;
    
    % function transforms distances to be better balanced rewards
    f=@(x) .5*log(x);
    weight = [1 2 5 2 1];
    distReward = dot(f(min(distances,100)), weight);
    % disp(distReward);
    rewards = rewards+ distReward;


    % distFromObj = sqrt((robot.obj.coords(0)-robot.center(0))^2 + (robot.obj.coords(1) - robot.center(1))^2);

    % rewards = rewards + -10*log(distFromObj);
    
    % array contains min side distance, min diagonal distance, front distance
    %minDistances = [min(distances(1),distances(5)),min(distances(2),distances(4)),distances(3)];
    %altF = @(x) 20*log(x);
    %altWeight = [1 1.5 3];
    %rewards = rewards + dot(altF(minDistances),altWeight);

    rewards = rewards + 5 * dot(weight, (distances - prev));

    rewards = rewards - 500/(.2*min(distances(1),distances(5)));

    % Give rewards based on the suggested action
    if(action == "forward" && distances(3) > 20)
        % was -50
        rewards = rewards +40;
    end

    if robot.sensor.prevDistances(3)<distances(3)
        rewards= rewards+20;
    end
    if robot.sensor.prevDistances(3) > distances(3) && distances(3)<20
            rewards = rewards - 20;
    end
    if action == "left" || action == "right"
        rewards = rewards - 20;
    end
end