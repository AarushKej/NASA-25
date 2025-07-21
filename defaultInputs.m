function inputs = defaultInputs()
inputs.ultrasonic.define=[pi/5 3*pi/8 pi/2 5*pi/8 4*pi/5]; % define angles (in radians) of ultrasonic sensors
inputs.ultrasonic_collisiondist=3; % collision occurs when any sensor distance is <=inputs.ultrasonic_collisiondist
inputs.ultrasonic.visible = 'off';
% when inputs.ultrasonic_collisiondist=0, collision occurs when robot collides with track or obstacles
inputs.track.startposition=-.03; % starting position of robot (0<=x<=1) 
inputs.running = false;

inputs.lidar.enable = true;
inputs.lidar.showrays = true;
inputs.lidar.raycolor = 'r';
inputs.lidar.raylinewidth = 0.5;
inputs.lidar.raystyle = '-';
inputs.lidar.n = 10;
inputs.lidar.distance = 1000;
inputs.lidar.startAngle = pi/4;
inputs.lidar.endAngle = 2*pi;

inputs.lidar.angles = linspace(inputs.lidar.startAngle, inputs.lidar.endAngle, inputs.lidar.n);

% Robot Parameters
inputs.robot.length = 17;
inputs.robot.width = 9;
inputs.robot.pow = 3.0;
inputs.robot.turnPow = 2.0;


% --- track design --- %
% (all spatial dimensions in cm)
inputs.track.innerradius=40; % inner radius of track
inputs.track.LFlinethickness=2.5; % thickness of line-follower line
inputs.track.showLFline=false; % show/hide line-follower line
inputs.track.wallthickness=3; % wall thickness
inputs.track.wall1_length=400; % length of wall 1 (y-axis wall)
inputs.track.wall2_length=400; % length of wall 2 (x-axis wall)
inputs.track.ptspacing=3; % linear dist. between points along track curve (also applied to circ. obstacles)
% smaller values will result in a higher-resolution track and may cause slower track generation and decreased FPS


% Simulation Parameters
inputs.sim.maxepisodes = 5000;
inputs.sim.episodesUntilQ = 250; % episodes until all decisions are greedy
inputs.sim.epsilonbias = 1;
inputs.sim.maxsteps = 1000;


% Learning parameters
inputs.cmac.alpha = 0.07;
inputs.cmac.gamma = 0.92;
inputs.cmac.beta = 0.3;
inputs.cmac.c = 3; % inputs are generalized to c adjacent cells
inputs.cmac.numInputs = inputs.lidar.n + 3;

numActions = 3;

% set number of bins for each type of input
inputs.cmac.sensorRes = 5;
inputs.cmac.headingRes = 12; 
inputs.cmac.distanceRes = 4; 

inputs.cmac.inputBins = zeros(1, inputs.cmac.numInputs);
inputs.cmac.inputBins(1:inputs.lidar.n) = inputs.cmac.sensorRes;
inputs.cmac.inputBins(inputs.lidar.n + 1) = inputs.cmac.distanceRes;
inputs.cmac.inputBins(inputs.lidar.n + 2: inputs.lidar.n + 3) = inputs.cmac.headingRes;

% sensor range limit
inputs.cmac.rangeMax = 70;

inputs.cmac.inputRanges = repmat([0 inputs.cmac.rangeMax], inputs.lidar.n, 1);
% max distance from objective is norm of track dims
inputs.cmac.inputRanges = [inputs.cmac.inputRanges;[0, norm([inputs.track.wall1_length, inputs.track.wall2_length])]];
% min/max difference in heading from robot to obstacle
inputs.cmac.inputRanges= [inputs.cmac.inputRanges;[-1, 1]];
inputs.cmac.inputRanges= [inputs.cmac.inputRanges;[-1, 1]];

% number of actual memory addresses
inputs.cmac.N = 50000; %round(0.05 * inputs.cmac.sensorRes ^ inputs.cmac.numInputs);

numActions = 3;


inputs.cmac.wMatrix = zeros(numActions, inputs.cmac.N);
inputs.cmac.adMatrix = zeros(numActions, inputs.cmac.numInputs, inputs.cmac.c);
inputs.cmac.sMatrix = zeros(numActions, inputs.cmac.numInputs, inputs.cmac.c);

inputs.cmac.w_1 = zeros(numActions, inputs.cmac.N);
inputs.cmac.w_2= zeros(numActions, inputs.cmac.N);
inputs.cmac.d_w= zeros(numActions, inputs.cmac.N);


inputs.cmac.y_1 = zeros(numActions,1);
inputs.cmac.u_1 = zeros(numActions,1);







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
inputs.colors.track.WT=[237 242 245]/255;

end