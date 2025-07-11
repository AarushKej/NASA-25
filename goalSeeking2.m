function [leftSpeed, rightSpeed] = goalSeeking2(app);

robotX = app.robot.center(1);
robotY = app.robot.center(2);
objX = app.robot.obj.coords(1);
objY = app.robot.obj.coords(2);

distX = objX - robotX;
distY = objY - robotY;


dist = hypot((distX),(distY));

theta = atan2(distY, distX) - app.robot.kinematics.theta;

% PID gains
Kp = 0.09;
Ki = 0.0001;
Kd = 0.008;


