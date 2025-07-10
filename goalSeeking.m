function [leftSpeed, rightSpeed] = goalSeeking(app)

objX = app.robot.obj.coords(1) - app.robot.center(1);
objY = app.robot.obj.coords(2) - app.robot.center(2);


dist = sqrt((objX).^2 + (objY).^2);
theta = rad2deg(atan2(objY, objX));

Kp_dist = 0.01;
Ki_dist = 0.0;
Kd_dist = 0.0;

% PID parameters for angle
Kp_theta = 0.01;
Ki_theta = 0.0;
Kd_theta = 0.0;

% Persistent errors
persistent intErrDist intErrTheta prevErrDist prevErrTheta
if isempty(intErrDist)
    intErrDist = 0;
    intErrTheta = 0;
    prevErrDist = 0;
    prevErrTheta = 0;
end

% Time step (you can make this dynamic)
dt = 0.1;

% Distance PID
errDist = dist;
intErrDist = intErrDist + errDist * dt;
derErrDist = (errDist - prevErrDist) / dt;
linearSpeed = Kp_dist * errDist + Ki_dist * intErrDist + Kd_dist * derErrDist;
prevErrDist = errDist;

% Theta PID
errTheta = theta;
intErrTheta = intErrTheta + errTheta * dt;
derErrTheta = (errTheta - prevErrTheta) / dt;
angularSpeed = Kp_theta * errTheta + Ki_theta * intErrTheta + Kd_theta * derErrTheta;
prevErrTheta = errTheta;

% Convert to left/right speeds (simple differential drive model)
wheelBase = 1;  % Adjust based on robot’s actual wheelbase
leftSpeed = linearSpeed - (angularSpeed * wheelBase / 2);
rightSpeed = linearSpeed + (angularSpeed * wheelBase / 2);

% Clamp speeds between 0 and 3
leftSpeed = max(0, min(3, leftSpeed));
rightSpeed = max(0, min(3, rightSpeed));
end
