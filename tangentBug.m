function [leftSpeed, rightSpeed] = tangentBug(app)
    persistent state minDistToGoalOnHit

    if isempty(state)
        state = "GO_TO_GOAL";
        minDistToGoalOnHit = inf;
    end

    % Robot & goal positions
    robotX = app.robot.center(1);
    robotY = app.robot.center(2);
    theta = app.robot.kinematics.theta;
    goalX = app.robot.obj.coords(1);
    goalY = app.robot.obj.coords(2);

    % Vector to goal
    toGoalX = goalX - robotX;
    toGoalY = goalY - robotY;
    distToGoal = hypot(toGoalX, toGoalY);
    goalAngle = atan2(toGoalY, toGoalX);
    lidar = app.robot.sensor.lidar.distances;

    % Path clearance check over ±10°
    checkAngles = -10:10;
    indices = mod(round(rad2deg(goalAngle - theta)) + checkAngles, 360) + 1;
    pathClear = all(lidar(indices) > distToGoal + 0.2);

    switch state
        case "GO_TO_GOAL"
            if pathClear
                [leftSpeed, rightSpeed] = moveToGoal(theta, goalAngle, distToGoal);
            else
                % Switch to FOLLOW_WALL
                state = "FOLLOW_WALL";
                minDistToGoalOnHit = distToGoal;
                [leftSpeed, rightSpeed] = followObstacle(lidar, theta);
            end

        case "FOLLOW_WALL"
            [leftSpeed, rightSpeed] = followObstacle(lidar, theta);

            % Recheck goal visibility and proximity
            if pathClear && distToGoal < minDistToGoalOnHit - 0.1
                state = "GO_TO_GOAL";
            end
    end

    % Stop if very close
    if distToGoal < 0.1
        leftSpeed = 0;
        rightSpeed = 0;
        state = "GO_TO_GOAL";
    end
end

function [leftSpeed, rightSpeed] = moveToGoal(theta, goalAngle, distToGoal)
    angleError = atan2(sin(goalAngle - theta), cos(goalAngle - theta));
    Kp_dist = 1.0;
    Kp_angle = 2.0;
    
    forwardSpeed = Kp_dist * min(distToGoal, 3);
    angleCorrection = Kp_angle * angleError;

    maxSpeed = 5;
    forwardSpeed = max(0, min(forwardSpeed, maxSpeed));
    angleCorrection = max(-1.5, min(angleCorrection, 1.5));
    
    leftSpeed = forwardSpeed - angleCorrection;
    rightSpeed = forwardSpeed + angleCorrection;
    
    leftSpeed = max(0, min(leftSpeed, maxSpeed));
    rightSpeed = max(0, min(rightSpeed, maxSpeed));
end

function [leftSpeed, rightSpeed] = followObstacle(lidar, theta)
    % Detect the closest obstacle
    [minDist, minIdx] = min(lidar);

    % Wall-following: right-hand rule
    angleToObstacle = deg2rad(mod(minIdx - 1, 360));
    angleGlobal = theta + angleToObstacle;
    desiredAngle = angleGlobal + pi/2;  % clockwise follow

    angleError = atan2(sin(desiredAngle - theta), cos(desiredAngle - theta));

    forwardSpeed = 1.0;
    angleCorrection = 1.5 * angleError;

    leftSpeed = forwardSpeed - angleCorrection;
    rightSpeed = forwardSpeed + angleCorrection;

    maxSpeed = 4;
    leftSpeed = max(0, min(leftSpeed, maxSpeed));
    rightSpeed = max(0, min(rightSpeed, maxSpeed));
end
