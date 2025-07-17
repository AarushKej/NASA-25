function [robot,plots,sim]=initializeObjective(app)
robot = app.robot;
plots = app.plots;
sim = app.sim;
inputs = app.inputs;
sim.episode = 0;
X = app.X;
Y = app.Y;
% app.plots.obj.XData=app.plots.objpts0(:,1)+X;
% app.plots.obj.YData=app.plots.objpts0(:,2)+Y;
Rand = app.Rand;
%robot.obj=struct();
[robot, plots] = setObjective(robot,plots,X,Y,Rand);
robot.objDist = norm(robot.obj.coords - robot.center);
robot.prevObjDist = robot.objDist;
    
end