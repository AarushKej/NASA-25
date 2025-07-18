function [leftSpeed, rightSpeed] = goalSeeking(app)

% positions
robotX = app.robot.center(1);
robotY = app.robot.center(2);
objX = app.robot.obj.coords(1);
objY = app.robot.obj.coords(2);

% dist & angle to goal
distX = objX - robotX;
distY = objY - robotY;
dist = hypot(distX, distY);
goalAngle = atan2(distY, distX);


theta = app.robot.kinematics.theta;
angleError = atan2(sin(goalAngle - theta), cos(goalAngle - theta));

% PID Gains (angle)
Kp_angle = 0.7;
Ki_angle = 0.002;
Kd_angle = 0.08;

% PID Gains (dist)
Kp_dist = 0.5;
Ki_dist = 0.001;
Kd_dist = 0.04;

% persistent variables
persistent intAngleError prevAngleError
if isempty(prevAngleError)
    intAngleError = 0;
    prevAngleError = 0;
end

persistent intDistError prevDistError
if isempty (prevDistError)
    intDistError = 0;
    prevDistError = 0;
end

% time step
dt = 0.1;

% angle PID controller
intAngleError = intAngleError + angleError * dt;
derivativeAngleError = (angleError - prevAngleError) / dt;
angleCorrection = Kp_angle * angleError + Ki_angle * intAngleError + Kd_angle * derivativeAngleError;
prevAngleError = angleError;

% dist PID controller
distError = dist - 0;
intDistError = intDistError + distError * dt;
derivativeDistError = (distError - prevDistError) / dt;
forwardSpeed = Kp_dist * distError + Ki_dist * intDistError + Kd_dist * derivativeDistError;
prevDistError = distError;

% limits to prevent overshoot
maxForwardSpeed = 4.0;
maxAngleCorrection = 1.5;

forwardSpeed = max(0, min(forwardSpeed, maxForwardSpeed));
angleCorrection = max(-maxAngleCorrection, min(angleCorrection, maxAngleCorrection));

% damping factor
if dist < 0.8
    dampingFactor = 0.3 + 0.7 * (dist / 0.8);
    forwardSpeed = forwardSpeed * dampingFactor;
    angleCorrection = angleCorrection * dampingFactor;
end

if abs(angleError) > pi/3 && dist < 1.5
    forwardSpeed = forwardSpeed * 0.5;
end

% differential drive
leftSpeed = forwardSpeed - angleCorrection;
rightSpeed = forwardSpeed + angleCorrection;

% speeds are non-negative and within limit
maxSpeed = 5.0;
leftSpeed = max(0, min(leftSpeed, maxSpeed));
rightSpeed = max(0, min(rightSpeed, maxSpeed));

% stop if very close to goal
if dist < 0.1
    leftSpeed = 0;
    rightSpeed = 0;
end
end