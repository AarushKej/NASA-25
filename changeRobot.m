function [robot,plots,sim]=changeRobot(app)
robot = app.robot;
plots = app.plots;
sim = app.sim;
inputs = app.inputs;
sim.episode = 0;




robot.actions = ["forward", "left", "right"];
robot.numActions = length(robot.actions);


robot.dims=[inputs.robot.width inputs.robot.length];
robot.wheels.dims.back=[1.6 6.4];


    robot.reset=inputs.resetRobot;


    



   

    plots.ptspacing=inputs.track.ptspacing;
    robot.crashed=false;
    robot.kinematics.dt=inputs.dt;


    

    [robot,plots] = initializeRobot(robot,plots,inputs);

    
    robot.objDist = norm(robot.obj.coords - robot.center);
    robot.prevObjDist = robot.objDist;
   



end


function errorcheck(inputs,robot)
assert(inputs.ultrasonic_collisiondist>=0,'''inputs.ultrasonic_collision'' must be >=0')
assert(inputs.track.ptspacing>0,'''inputs.track.ptspacing'' must be >0');


assert(inputs.track.wall1_length>=0 && inputs.track.wall2_length>=0,'''wall1_length'' and ''wall2_length'' must be >0.')


assert(inputs.display.Zoom>0,'''Zoom'' must be >0')
assert(inputs.realtime || inputs.dt>0,'''dt'' must be >0')

assert(islogical(inputs.track.showLFline),'''showLFline'' must be true or false.')

assert(islogical(inputs.display.showsensordata),'''showsensordata'' must be true or false.')
assert(islogical(inputs.display.HUD.show),'''HUD.show'' must be true or false.')
assert(islogical(inputs.resetRobot),'''resetRobot'' must be true or false.')
assert(islogical(inputs.realtime),'''realtime'' must be true or false.')
colsin=[{inputs.colors.track.trackwalls, inputs.colors.track.tracklane, inputs.colors.track.lanelines, inputs.colors.track.Lfline,inputs.colors.track.obs,...
    inputs.colors.robot.crash} inputs.colors.ultrasonic.sensorcolors];
for i=1:length(colsin)
    checkcolor(colsin{i});
end
end
