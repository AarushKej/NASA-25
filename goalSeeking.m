function [leftSpeed, rightSpeed] = goalSeeking(app)

% Current and goal positions
robotX = app.robot.center(1);
robotY = app.robot.center(2);
objX = app.robot.obj.coords(1);
objY = app.robot.obj.coords(2);

% PID gains
Kp = 0.5;
Ki = 0.0;
Kd = 0.2;

% Persistent variables
persistent prevError integralError prevX prevY
if isempty(prevError)
    prevError = 0;
    integralError = 0;
    prevX = robotX;
    prevY = robotY;
end

dt = 0.1;

% Estimate heading from movement
dxPos = robotX - prevX;
dyPos = robotY - prevY;
if hypot(dxPos, dyPos) > 1e-4
    theta = atan2(dyPos, dxPos);
else
    theta = 0;  % Default heading
end

% Goal vector
dxGoal = objX - robotX;
dyGoal = objY - robotY;
goalAngle = atan2(dyGoal, dxGoal);

% PID angle control 
error = atan2(sin(goalAngle - theta), cos(goalAngle - theta));
integralError = integralError + error * dt;
derivativeError = (error - prevError) / dt;
correction = Kp * error + Ki * integralError + Kd * derivativeError;
prevError = error;

% Forward speed and angular correction
distance = hypot(dxGoal, dyGoal);
baseSpeed = min(0.5, distance);  % always forward

% Scale angular correction so both speeds stay ≥ 0
correctionScale = 0.5;  % how much to adjust based on angle error
leftSpeed = baseSpeed * (1 - correctionScale * correction);
rightSpeed = baseSpeed * (1 + correctionScale * correction);

% Prevent negative speeds
leftSpeed = max(leftSpeed, 0);
rightSpeed = max(rightSpeed, 0);

maxSpeed = 3.0;
leftSpeed = min(leftSpeed, maxSpeed);
rightSpeed = min(rightSpeed, maxSpeed);

prevX = robotX;
prevY = robotY;

end
