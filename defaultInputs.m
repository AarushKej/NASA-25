function inputs = defaultInputs()
    inputs.ultrasonic.define=[pi/5 3*pi/8 pi/2 5*pi/8 4*pi/5]; % define angles (in radians) of ultrasonic sensors
    inputs.ultrasonic_collisiondist=3; % collision occurs when any sensor distance is <=inputs.ultrasonic_collisiondist
    % when inputs.ultrasonic_collisiondist=0, collision occurs when robot collides with track or obstacles
    inputs.track.startposition=-.03; % starting position of robot (0<=x<=1)
    inputs.track.startlane=2; % starting lane of the robot; lanes counted from inside to outside of track
    
   
    inputs.running = false;
    % --- robot --- %
    inputs.rot.length = 17;
    inputs.rot.width = 9;
    % --- q learning --- %
    inputs.qlearning.learningrate = 0.1;
    inputs.qlearning.discountrate = 0.3;
    inputs.qlearning.maxepisodes = 5000;
    inputs.qlearning.episodes_to_q = 500;
    inputs.qlearning.epsilonbias = 1;
    inputs.qlearning.maxsteps = 1000;


    % Robot values
    inputs.pow = 3.0;
    inputs.turnPow = 2.0;

    % --- lanes --- %
    inputs.track.width=60; % track width (cm)
    inputs.track.Nlanes=3; % number of lanes
    inputs.track.lanelines=true; % show/hide lines between lanes
   
    inputs.colors.track.WT=[237 242 245]/255;
    
    
    % --- track design --- %
    % (all spatial dimensions in cm)
    inputs.track.innerradius=40; % inner radius of track
    inputs.track.LFlinethickness=2.5; % thickness of line-follower line
    inputs.track.showLFline=false; % show/hide line-follower line
    inputs.track.wallthickness=3; % wall thickness
    inputs.track.wall1_length=300; % length of wall 1 (y-axis wall)
    inputs.track.wall2_length=300; % length of wall 2 (x-axis wall)
    inputs.track.ptspacing=1; % linear dist. between points along track curve (also applied to circ. obstacles)
    % smaller values will result in a higher-resolution track and may cause slower track generation and decreased FPS
    
    %%%%% Display inputs %%%%%
    inputs.display.showsensordata=false; % show/hide plot of data from sensors (hiding plot may increase FPS)
    inputs.display.HUD.show=true; % show/hide sim. information and instantaneous sensor data
    inputs.display.Zoom=8; % zoom in/out of plot
    inputs.display.FullScreen=false; % open simulation window at full screen
    
    %%%%% Simulation inputs %%%%%
    inputs.resetRobot=false; % reset robot after collision
    inputs.realtime=false; % runs the simulation in real time
    inputs.dt=.2; % constant dt for kinematics (if realtime=false)
    
    %%%%% Aesthetics %%%%%
    % enter RGB Triplet (e.g. [0 0 1], in which all values are 0<=x<=1), color name (e.g. 'red', 'blue'), or short name (e.g. 'r', 'b')
    inputs.colors.track.tracklane=[237 242 245]/255; % track-lane color
    inputs.colors.track.trackwalls=[92 125 144]/255; % track-walls color
    inputs.colors.track.lanelines=[173 180 184]/255; % lane-line color
    inputs.colors.track.Lfline='k'; % line-follower-line color
    inputs.colors.track.obs=[140, 209, 255]/255; % obstacle color
    inputs.colors.robot.body=[180 242 250]/255; % robot body color
    inputs.colors.robot.crash=[255 209 238]/255; % robot body crash color
    inputs.colors.robot.wheels=[252 245 141]/255; % robot wheels color
    inputs.colors.robot.LF_Idle=[200 200 200]/255; % line-follower idle color
    inputs.colors.robot.LF_Trigger='g'; % line-follower triggered color
    inputs.colors.ultrasonic.sensorcolors={[162 232 160]/255,[255 74 71]/255,[196 106 252]/255,[255 198 92]/255,[160 232 221]/255};
    inputs.colors.ultrasonic.linestyle='-';

end