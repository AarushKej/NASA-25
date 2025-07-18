function [robot, rewards] = evaluateReward(robot, action)
rewards = 0;

% State Rewards
if (robot.crashed)
    rewards = -1200;
    return
elseif (robot.arrived)
    rewards = 1200;
    return
end

% Obstacle Distance Rewards
distances = robot.sensor.ultrasonic.distances;
prevDistances = robot.sensor.prevDistances;


% when uncomfortably close, reward/punish changes in buffer
if distances(3) < 25
    bufferReward = 6*(distances(3) - prevDistances(3));
    rewards = rewards + max(min(bufferReward, 50),-50);
end

% reward distance from obstacles
% minDists = [min(distances(1), distances(5)), min(distances(2), distances(4)), distances(3)];
% minDists = min(minDists, 70);
% weights = [1 2 5];
% distReward = 20 * log(.01*(dot(weights, minDists)));
% rewards = rewards+ distReward;

%distance reward is in the 40s normally, and it gets small/negative if
%too close to obstacles

% --------Objective Distance Rewards----------

distanceReward = 7*(robot.prevObjDist - robot.objDist);
%varies from about -45 to 45
%disp(['Distance Reward: ', num2str(distanceReward)]);
rewards = rewards + distanceReward;

robot.prevObjDist = robot.objDist;


% --------Objective Direction Rewards---------
headingReward = 60*(robot.headingError(1));  %<- cos(angle from objective)
% -40 punishment for heading away, 50 reward for heading toward obj
% 0 reward for going directly perpendicular to objective

rewards = rewards + headingReward;

if(isfield(robot, 'prevHeadingDiff'))
    headingChange = 40*(abs(robot.prevHeadingDiff) - abs(robot.headingDiff));
    % basically +20 for turning closer, -20 for turning farther
    %disp(['Heading Change: ', num2str(headingChange)]);
    rewards = rewards + headingChange;
end
robot.prevHeadingDiff = robot.headingDiff;


rewards = rewards - 60;
end