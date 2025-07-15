classdef Application < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        GridLayout                 matlab.ui.container.GridLayout
        LeftPanel                  matlab.ui.container.Panel
        CustomizationTabs          matlab.ui.container.TabGroup
        QLearningTab               matlab.ui.container.Tab
        EpisodesUntilQLabel        matlab.ui.control.Label
        DecisionMakingLabel        matlab.ui.control.Label
        TrainingParametersLabel    matlab.ui.control.Label
        DecisionMakingSwitch       matlab.ui.control.Switch
        DeltaTimeEditField         matlab.ui.control.NumericEditField
        DeltaTimeLabel             matlab.ui.control.Label
        EpisodesUntilQEditField    matlab.ui.control.NumericEditField
        NumberofStatesEditField    matlab.ui.control.NumericEditField
        NumberOfStatesLabel        matlab.ui.control.Label
        DiscountFactorEditField    matlab.ui.control.NumericEditField
        DiscountFactorLabel        matlab.ui.control.Label
        LearningRateEditField      matlab.ui.control.NumericEditField
        LearningRateLabel          matlab.ui.control.Label
        MaxStepsEditField          matlab.ui.control.NumericEditField
        MaximumStepsLabel          matlab.ui.control.Label
        MaxEpisodesEditField       matlab.ui.control.NumericEditField
        MaxEpisodesEditFieldLabel  matlab.ui.control.Label
        RoverTab                   matlab.ui.container.Tab
        SpawnPositionSwitch        matlab.ui.control.Switch
        SpawnPositionSwitchLabel   matlab.ui.control.Label
        RoverParametersLabel       matlab.ui.control.Label
        TurnPowerEditField         matlab.ui.control.NumericEditField
        TurnPowerEditFieldLabel    matlab.ui.control.Label
        PowerEditField             matlab.ui.control.NumericEditField
        PowerEditFieldLabel        matlab.ui.control.Label
        roverLengthEditField       matlab.ui.control.NumericEditField
        LengthLabel                matlab.ui.control.Label
        RoverWidthEditField        matlab.ui.control.NumericEditField
        WidthLabel                 matlab.ui.control.Label
        TrackTab                   matlab.ui.container.Tab
        TrackParametersLabel       matlab.ui.control.Label
        TrackInnerRadiusSlider     matlab.ui.control.Slider
        TrackInnerRadiusEditField  matlab.ui.control.NumericEditField
        TrackInnerRadiusLabel      matlab.ui.control.Label
        TrackWidthSlider           matlab.ui.control.Slider
        TrackWidthEditField        matlab.ui.control.NumericEditField
        TrackWidthLabel            matlab.ui.control.Label
        TrackLengthSlider          matlab.ui.control.Slider
        TrackLengthEditField       matlab.ui.control.NumericEditField
        TrackLengthLabel           matlab.ui.control.Label
        TrackHeightEditField       matlab.ui.control.NumericEditField
        TrackHeightLabel           matlab.ui.control.Label
        TrackHeightSlider          matlab.ui.control.Slider
        ObstaclesTab               matlab.ui.container.Tab
        ObstacleLibraryLabel       matlab.ui.control.Label
        polygonimage_2             matlab.ui.control.Image
        PolygonalObstacleLabel     matlab.ui.control.Label
        SquareObstacleLabel        matlab.ui.control.Label
        CircularObstacleLabel      matlab.ui.control.Label
        SideNumberLabel            matlab.ui.control.Label
        polyradiuslabel            matlab.ui.control.Label
        PolyRadiusEditField        matlab.ui.control.NumericEditField
        PlotButton_3               matlab.ui.control.Button
        PlotButton_2               matlab.ui.control.Button
        PlotButton                 matlab.ui.control.Button
        SideNumberEditField        matlab.ui.control.NumericEditField
        CircleRadiusLabel          matlab.ui.control.Label
        CircleRadiusEditField      matlab.ui.control.NumericEditField
        RectHeightLabel            matlab.ui.control.Label
        RectHeightEditField        matlab.ui.control.NumericEditField
        circleImage                matlab.ui.control.Image
        polygonimage               matlab.ui.control.Image
        CenterPanel                matlab.ui.container.Panel
        UIAxes                     matlab.ui.control.UIAxes
        RightPanel                 matlab.ui.container.Panel
        Panel_2                    matlab.ui.container.Panel
        EpisodeCountEditField      matlab.ui.control.NumericEditField
        EpisodeCountLabel          matlab.ui.control.Label
        FramesPerSecondEditField   matlab.ui.control.NumericEditField
        SimulationTimeEditField    matlab.ui.control.NumericEditField
        FPSLabel                   matlab.ui.control.Label
        SimulationTimeLabel        matlab.ui.control.Label
        LapCountEditField          matlab.ui.control.NumericEditField
        LapCountLabel              matlab.ui.control.Label
        EpsilonGreedyGraph         matlab.ui.control.UIAxes
        LossGraph                  matlab.ui.control.UIAxes
        ConvergenceGraph           matlab.ui.control.UIAxes
        RewardsGraph               matlab.ui.control.UIAxes
        ConfigPanel                matlab.ui.container.Panel
        ResetConfigurationButton   matlab.ui.control.Button
        LoadTrackButton            matlab.ui.control.Button
        SaveTrackButton            matlab.ui.control.Button
        LoadQMatrixButton          matlab.ui.control.Button
        SaveQMatrixButton          matlab.ui.control.Button
        StopButton                 matlab.ui.control.Button
        StartButton                matlab.ui.control.Button
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
        twoPanelWidth = 768;
    end

    properties (Access = public)
        robot; % Robot in sim and app
        plots; % used for creating polygons and calling other files
        sim; % contains important simulation parameters such as learning rate
        inputs; % used for calling Intialize and some of the other files
        dims; % dimensions, used for dimensions of polygons. 
        RadiusArray; % the variable used for setting the two possibilties for minimum radius. 
        minradius; % the variable that is equal to the greater of the two possibilities.
        stopSim; % Booleon to check if stop button is pressed 
        maxradius; %Max radius of circle obstacles are circumscribed in 
        outputs;% Field used in graphing 
        counterplot=0;
        maxrover; % Max width of the rover 
        mintrack; % Minimum width of track
        cmac;
    end
    
    methods (Access = public)
        
        function updateInputs(app)% read parameters from user inputs
            app.sim.alpha = app.LearningRateEditField.Value;
            app.sim.gamma = app.DiscountFactorEditField.Value;
            app.sim.maxEpisodes = app.MaxEpisodesEditField.Value;
            app.sim.episodesUntilQ = app.EpisodesUntilQEditField.Value;
            app.sim.maxSteps = app.MaxStepsEditField.Value;
            app.robot.numStates = app.NumberofStatesEditField.Value;
            app.robot.pow = app.PowerEditField.Value;
            app.robot.turnPow = app.TurnPowerEditField.Value;
            
        end

        function updateTrack(app)
            app.inputs.track.width = app.TrackWidthEditField.Value;
            app.inputs.track.wall1_length = app.TrackLengthEditField.Value;
            app.inputs.track.wall2_length = app.TrackHeightEditField.Value;
            app.inputs.track.innerradius = app.TrackInnerRadiusEditField.Value;
            cla(app.UIAxes);
            [app.robot,app.plots,app.sim]=initialize(app); 
        end

        function updateRobot(app)
            app.robot.turnPow = app.TurnPowerEditField.Value;
            app.robot.pow = app.PowerEditField.Value;
            app.inputs.rot.width = app.RoverWidthEditField.Value;
            app.inputs.rot.length = app.roverLengthEditField.Value;
            cla(app.UIAxes);
            [app.robot,app.plots,app.sim]=initialize(app); 
        end
      function displayOutputs(app) % output fps, time, laps, etc.
            app.FramesPerSecondEditField.Value = round(mean(app.plots.display.fpsvec));
            app.SimulationTimeEditField.Value = app.plots.display.timeelapsed.sim;
            app.LapCountEditField.Value = app.robot.lap.Nlap;
            app.EpisodeCountEditField.Value = app.sim.episode;

            drawnow;
            % x=linspace(1,app.sim.maxSteps);
            % yVals(end+1) =mean(app.sim.lastTenRewards);
        end

        function runSimulation(app) % simulation top level
            app.stopSim = false;
            cla(app.RewardsGraph);
            cla(app.ConvergenceGraph);
            cla(app.LossGraph);
            cla(app.EpsilonGreedyGraph);

            app.sim.status = "running";
            updateInputs(app);
            app.outputs.rewardsGraph.y = zeros(1,app.sim.maxEpisodes);
            app.outputs.lossCurve.y = zeros(1, app.sim.maxEpisodes);
            app.outputs.convergenceGraph.y = zeros(1,app.sim.maxEpisodes);
            app.outputs.lossGraph.y = zeros(1,app.sim.maxEpisodes);
            app.outputs.crossValidationGraph.y = zeros(1,app.sim.maxEpisodes);
            episoderewardvector=1:1:app.sim.maxEpisodes;

            % load Qtable.mat Qtable
            epsilonVec = linspace(1,0,app.sim.episodesUntilQ);

            if(length(app.sim.qmatrix) < length(app.robot.statesDict.keys)) % Basically if not already created/imported
                app.sim.qmatrix = rand(length(app.robot.statesDict.keys),3);
            end
            app.sim.totalReward = 0;
            qChange = 0;

            % ---------- new -------------
            app.cmac.c = 4;

            numActions = length(app.robot.actions);

            app.cmac.numInputs = 5;
            app.cmac.sensorRes = 10;
            app.cmac.N = round(0.05 * app.cmac.sensorRes ^ app.cmac.numInputs);


            app.cmac.rangeMax = 70;
            app.cmac.inputRanges = repmat([0 app.cmac.rangeMax], 5, 1);



            app.cmac.wMatrix = zeros(numActions, app.cmac.N);
            app.cmac.adMatrix = zeros(numActions, app.cmac.numInputs, app.cmac.c);
            app.cmac.sMatrix = zeros(numActions, app.cmac.numInputs, app.cmac.c);
            
            app.cmac.w_1=app.cmac.wMatrix;app.cmac.w_2=app.cmac.wMatrix;app.cmac.d_w=app.cmac.wMatrix;

            app.cmac.xite=0.1;% Learning efficiency

            app.cmac.alfa=0.05;% Momentum factor

            app.cmac.y_1 = zeros(numActions,1);
            app.cmac.u_1 = zeros(numActions,1);
            qPreds = zeros(numActions,1);



            % -----------------------------

            while ~app.stopSim
                try
                    app.sim.episode = app.sim.episode + 1;
                    app.sim.episodeReward=0;
                    app.sim.episodeChange = 0;
                    stepCount = 0;
                    while stepCount<app.sim.maxSteps
                        stepCount = stepCount + 1;

                        for a = 1:1:length(app.robot.actions)
                            [app.cmac, qPreds(a)] = cmacModel(app, a);
                        end

                        % Decide on action
                        if app.DecisionMakingSwitch.Value == "Epsilon-Greedy" && app.sim.episode<app.sim.episodesUntilQ && rand < epsilonVec(app.sim.episode)
                            action = randi([1 3]);
                        else
                            [~, action] = max(qPreds);
                        end
                        % disp((qPreds));

                        app.robot.sensor.prevDistances = app.robot.sensor.ultrasonic.distances;

                        switch app.robot.actions(action)
                            case "forward"
                                app.robot=motor(app.robot,'right','power',app.robot.pow);
                                app.robot=motor(app.robot,'left','power',app.robot.pow);
                            case "left"
                                app.robot=motor(app.robot,'right','power',app.robot.turnPow);
                                app.robot=motor(app.robot,'left','power',app.robot.turnPow * 0.2);
                            case "right"
                                app.robot=motor(app.robot,'right','power',app.robot.turnPow * 0.2);
                                app.robot=motor(app.robot,'left','power',app.robot.turnPow);
                        end
                        
                        %*********************** DO NOT CHANGE ************************%
                            [app.robot,app.plots]=executeMove(app.robot,app.plots,false);
                        if app.robot.plot.closed
                            % save Qtable.mat
                        end
                        %**************************************************************%
                        rewards = evaluateReward(app.robot, app.robot.actions(action));
                        

                        % Update weights
                        try
                            % Before updating weights
                            oldWeights = app.cmac.wMatrix;
                            
                            % Update weights
                            app.cmac = updateCmac(app, action, qPreds(action), rewards);
                            
                            % After updating weights
                            newWeights = app.cmac.wMatrix;
                            
                            % Calculate and accumulate Q-value (weight) change
                            qChange = sum(abs(newWeights(:) - oldWeights(:)));
                            app.sim.episodeChange = app.sim.episodeChange + qChange;
                            
                        catch err
                            disp(getReport(err));
                            uialert(app.UIFigure, 'Error updating weights', getReport(err));
                        end
                        if app.robot.crashed || app.robot.arrived || app.stopSim
                            break
                        end
                        drawnow limitrate;
                        app.sim.episodeReward=app.sim.episodeReward+rewards;
                        app.sim.totalreward=app.sim.totalReward+rewards;
                        % fix this line, qChange is never used after
                        % initialization
                        app.sim.episodeChange = app.sim.episodeChange + abs(qChange);
                        
                        displayOutputs(app);
    
                        episoderewardvector(app.sim.episode)=app.sim.episodeReward;
                        
    
                          % c2=app.MaxEpisodesEditField.Value;
                         % xlim(app.graph1,[0 20])
                         %  ylim(app.graph1,[-100,10000000000]);
                    end
                    if app.robot.arrived
                        uialert(app.UIFigure, 'Arrived at objective', 'Arrived');
                    end

                    % Reset Robot and Spawn
                    app.robot=setSpawn(app.robot,app.plots,3,rand,true); % randomly select new spawn
                    [app.robot,app.plots]=resetRobot(app.robot,app.plots);

                    % Display Episode Reward
                    app.outputs.rewardsGraph.y(app.sim.episode) = app.sim.episodeReward/stepCount;
                    plot(app.RewardsGraph, 1:app.sim.episode, app.outputs.rewardsGraph.y(1:app.sim.episode), 'b', 'LineWidth', 1.5);

                    app.outputs.convergenceGraph.y(app.sim.episode) = app.sim.episodeChange/stepCount;
                    plot(app.ConvergenceGraph, 1:app.sim.episode, app.outputs.convergenceGraph.y(1:app.sim.episode), 'r', 'LineWidth', 1.5);

                    %change the code below for the new graphs

                    numActions = length(app.robot.actions);
                    qPredsNext = zeros(numActions, 1); % Preallocate
                    for a = 1:1:length(app.robot.actions)
                        [~, qPredsNext(a)] = cmacModel(app, a); % Use updated state in cmacModel
                    end
                    qTarget = rewards + app.sim.gamma * max(qPredsNext);
                    loss = app.computeLoss(qPreds, qTarget);
                    app.outputs.lossGraph.y(app.sim.episode) = loss;
                    plot(app.LossGraph, 1:app.sim.episode, app.outputs.lossGraph.y(1:app.sim.episode), 'g', 'LineWidth', 1.5);

                    % --- Track and plot epsilon (ε-greedy exploration) ---
                    epsilonVec = linspace(1,0,app.sim.episodesUntilQ); % Already in your code
                    if app.sim.episode <= length(epsilonVec)
                        epsilon = epsilonVec(app.sim.episode);
                    else
                        epsilon = 0;
                    end
                    app.outputs.EpsilonGreedyGraph.y(app.sim.episode) = epsilon;
                    plot(app.EpsilonGreedyGraph, 1:app.sim.episode, app.outputs.EpsilonGreedyGraph.y(1:app.sim.episode), 'm', 'LineWidth', 1.5);

                catch err
                    if app.stopSim
                        return;
                    else
                        uialert(app.UIFigure,'Error during simulation iteration',getReport(err));
                        disp(err);
                        disp(getReport(err));
                        break;
                    end
                end
            end
            %cla(app.UIAxes);
            %[app.robot,app.plots,app.sim]=initialize(app);  % Call Intialize and take new inputs, user inputs, and make them simulation inputs.
            displayOutputs(app)
        end

        function loss = computeLoss(~, qPreds, qTarget)
            loss = mean((qPreds - qTarget).^2);
        end
        
         function deleteObs(app)  
for i=1:length(app.plots.obstacleArr)
     interior = isinterior(app.plots.obstacleArr(i),ax.CurrentPoint(1,1:2));
      if interior
        

      end

end




        
          if interior
           





         end
        end
    end
    
   
 

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app, Alpha)
            %app.graph1.Visible = 'off';
            %addpath('V6_Ian\SimulationCode\');
            %addpath('V6_Ian\Images\');

           %set(app.graph1,'position',[0.005,5,300,0.00001])

            % ---------------- Initialize Certain Variables ---------------------%
            app.inputs = defaultInputs(); % default inputs are app inputs, it HAS to be at top!!!!
            app.sim.qmatrix = [];
            app.plots.obstacleArr = {};
             
            % ----------------Setting up the app-----------------%
            cla(app.UIAxes)   % clear the app axes to avoid overlaying axes. 
            cla(app.RewardsGraph);
            cla(app.ConvergenceGraph);
            cla(app.EpsilonGreedyGraph)
            cla(app.LossGraph)

            [app.robot,app.plots,app.sim]=initialize(app);   % creates everything on main axis. 
            axis(app.UIAxes,'Equal')
            axis(app.UIAxes,'off')
            app.UIAxes.Interactions=[];

            % ----------------Positioning Components-----------------%
            pos = app.ConfigPanel.Position;
            pos(2) = 5;
            %set(app.ConfigPanel,"Position", pos);
            %movegui(app.ConfigPanel,pos);
            
            % ----------------Setting Default Values as the Values that Appear-----------------%
            app.TrackLengthEditField.Value = app.inputs.track.wall1_length;   % sets the default track length in the edit field to display to user
            app.TrackLengthSlider.Value = app.inputs.track.wall1_length;  % sets the default track length, to display to the user in the slider
            app.TrackHeightEditField.Value = app.inputs.track.wall2_length;   % sets the default track width in the edit field to display to user
            app.TrackHeightSlider.Value = app.inputs.track.wall2_length;  % sets the default track width,to display to the user in the slider
            app.RoverWidthEditField.Value = app.inputs.rot.width;   % sets the default rover width in the edit field to display to user
            app.roverLengthEditField.Value = app.inputs.rot.length;   % sets the default rover length in the edit field to display to user


            % ----------------Setting the min radius for sensors------------------%
            app.RadiusArray=[((app.roverLengthEditField.Value/2)*0.68) (app.roverLengthEditField.Value)];  % Two possibilities for minimum radius, of an object, that a robot will be able to sense depending on the robot length.
            app.minradius = max(app.RadiusArray);  % Setting and choosing the possibility that works for the robot length choosen to the variable "minradius". 
            app.maxradius=((app.TrackWidthEditField.Value-(app.RoverWidthEditField.Value+(app.RoverWidthEditField.Value/3))));
            app.PolyRadiusEditField.Limits= [app.minradius app.maxradius];  % Setting the polygon's radius edit field box to unlimited values.             app.RectHeightEditField.Limits= [17 inf];  % Setting the Rectangles side length edit field box to unlimited values.
            app.CircleRadiusEditField.Limits= [app.minradius app.maxradius]; % Setting the Circle's radius edit field to unlimited values. 
            app.RectHeightEditField.Limits= [app.minradius app.maxradius];

            % ----------------Setting the min and max for robot width------------------%
            app.maxrover=app.TrackWidthEditField.Value/2;
            app.RoverWidthEditField.Limits=[2 app.maxrover];
            % ----------------Setting the min and max for track width------------------%
            app.mintrack=app.RoverWidthEditField.Value*2;
            app.TrackWidthEditField.Limits=[app.mintrack 110];
        end

        % Callback function
        function CustomButtonPushed(app, event)
            app.CoordinatesEditField.Visible = 'on';  % Making the edit field boxes for the user to input the coordinates of the robot visable. 

        end

        % Value changed function: TrackHeightSlider
        function TrackHeightSliderValueChanged(app, event)
            app.TrackHeightEditField.Value = app.TrackHeightSlider.Value;  % The value the user's input in the width of the track edit field, is equal to the number shown in  the slider.  
            updateTrack(app);
        end

        % Value changed function: TrackHeightEditField
        function TrackHeightEditFieldValueChanged(app, event)
            app.TrackHeightSlider.Value = app.TrackHeightEditField.Value;  % The value the user's input in the width of the track slider, is equal to the number shown in  the edit field.  
            updateTrack(app);
        end

        % Value changed function: TrackLengthEditField
        function TrackLengthEditFieldValueChanged(app, event)
           app.TrackLengthSlider.Value = app.TrackLengthEditField.Value;
           updateTrack(app);
        end

        % Value changed function: TrackLengthSlider
        function TrackLengthSliderValueChanged(app, event)
            app.TrackLengthEditField.Value = app.TrackLengthSlider.Value;  % The value the user's input in the length of the track slider, is equal to the number shown in  the edit field.  
           updateTrack(app);
        end

        % Value changed function: RoverWidthEditField
        function RoverWidthEditFieldValueChanged(app, event)
            app.maxrover=app.TrackWidthEditField.Value/2;
            app.RoverWidthEditField.Limits=[2 app.maxrover];
            app.mintrack=app.RoverWidthEditField.Value*2;
            app.TrackWidthEditField.Limits=[app.mintrack 110];
            if app.RoverWidthEditField.Value < (3*app.TrackWidthEditField.Value-(3*app.roverLengthEditField.Value))/4  
            app.RadiusArray=[((app.roverLengthEditField.Value/2)*0.68) (app.roverLengthEditField.Value)];  % Two possibilities for minimum radius, of an object, that a robot will be able to sense depending on the robot length.
            app.minradius = max(app.RadiusArray);  % Setting and choosing the possibility that works for the robot length choosen to the variable "minradius". 
            app.maxradius=((app.TrackWidthEditField.Value-(app.RoverWidthEditField.Value+(app.RoverWidthEditField.Value/3))));
            app.PolyRadiusEditField.Limits= [app.minradius app.maxradius];  % Setting the polygon's radius edit field box to unlimited values.             app.RectHeightEditField.Limits= [17 inf];  % Setting the Rectangles side length edit field box to unlimited values.
            app.CircleRadiusEditField.Limits= [app.minradius app.maxradius]; % Setting the Circle's radius edit field to unlimited values. 
            app.RectHeightEditField.Limits= [app.minradius app.maxradius];    
            else
                disp('invalid');
           end
            % app.inputs.rot.width=app.RoverWidthEditField.Value; % Set the user's edit field input of the rover width to the actual rover width in the simulation.
            % cla(app.UIAxes);  % clear the app axes to avoid overlaying axes. 
            % [app.robot,app.plots]=initializeRobot(app.robot,app.plots,app.inputs);
            updateRobot(app);
          
        end

        % Value changed function: roverLengthEditField
        function roverLengthEditFieldValueChanged(app, event)
            
            app.inputs.rot.length=app.roverLengthEditField.Value;  % Set the user's edit field input of the rover length to the actual rover length in the simulation.
           if app.roverLengthEditField > ((app.TrackWidthEditField.Value-(app.RoverWidthEditField.Value+(app.RoverWidthEditField.Value/3))))
            app.RadiusArray=[((app.roverLengthEditField.Value/2)*0.68) (app.roverLengthEditField.Value)];  % Two possibilities for minimum radius, of an object, that a robot will be able to sense depending on the robot length.
            app.minradius = max(app.RadiusArray);  % Setting and choosing the possibility that works for the robot length choosen to the variable "minradius". 
            app.maxradius=((app.TrackWidthEditField.Value-(app.RoverWidthEditField.Value+(app.RoverWidthEditField.Value/3))));
            app.PolyRadiusEditField.Limits= [app.minradius app.maxradius];  % Setting the polygon's radius edit field box to unlimited values.             app.RectHeightEditField.Limits= [17 inf];  % Setting the Rectangles side length edit field box to unlimited values.
            app.CircleRadiusEditField.Limits= [app.minradius app.maxradius]; % Setting the Circle's radius edit field to unlimited values. 
            app.RectHeightEditField.Limits= [app.minradius app.maxradius]; 
           end
            % cla(app.UIAxes);  % clear the app axes to avoid overlaying axes. 
            %   [app.robot,app.plots]=initializeRobot(app.robot,app.plots,app.inputs);
                        updateRobot(app);
        end

        % Button pushed function: StartButton
        function StartButtonPushed(app, event)
            %[app.robot,app.plots] = initialize(app); 
            runSimulation(app)
        end

        % Button pushed function: PlotButton
        function PlotButtonPushed(app, event)
           
         
app.counterplot=0;
            plot = true;
          sides=app.SideNumberEditField.Value;

            % app.dims=(2*app.PolyRadiusEditField.Value*sin((pi)/(sides)));
            % poly=nsidedpoly(sides,'SideLength', (2*app.PolyRadiusEditField.Value*sin((pi)/(sides))));
            
           % ella's version
            app.dims=(app.PolyRadiusEditField.Value*sin((pi)/(sides)));
            poly=nsidedpoly(sides,'SideLength', (app.PolyRadiusEditField.Value*sin((pi)/(sides))));

            
            p=patch(app.UIAxes,'Faces', 1:sides,'Vertices',poly.Vertices,'FaceColor','none','FaceAlpha',1,'EdgeColor',[140, 209, 255]/255);
            app.UIFigure.WindowButtonMotionFcn={@mousemove,app.UIAxes,p,poly};
            app.UIFigure.WindowButtonDownFcn={@mouseclick,sides};
                        
           
            function mousemove(~,~,ax,p,poly) 
               try
                p.Vertices=poly.Vertices+ax.CurrentPoint(1,1:2);
                plot = true;
               catch
                 plot = false;
               end 
  
            end

            function mouseclick(~,~,sides)
                app.counterplot=app.counterplot+1;
                if plot
                if app.counterplot<2
                        app.plots= createObsNicoleTest(app.robot,app.plots,1,'poly',[sides (app.PolyRadiusEditField.Value)],1,3,[app.UIAxes.CurrentPoint(1,1) app.UIAxes.CurrentPoint(1,2) 0]);
                        app.plots= setObs(app.plots,1);
                        p.Visible='off';
                else
                   p.Visible='off';
                end
                end 
            end

        end

        % Button pushed function: PlotButton_3
        function PlotButton_3Pushed(app, event)

            plot=true;
           app.counterplot=0;
            sides=10000;
            poly=nsidedpoly(sides,'SideLength', (app.CircleRadiusEditField.Value*sin((pi)/(sides))));
            
            p=patch(app.UIAxes,'Faces', 1:sides,'Vertices',poly.Vertices,'FaceColor','none','FaceAlpha',1,'EdgeColor',[174 252 157]/255);
            app.UIFigure.WindowButtonMotionFcn={@mousemove,app.UIAxes,p,poly};
            app.UIFigure.WindowButtonDownFcn={@mouseclick,app.UIAxes,p, sides};
            
            app.dims=(app.CircleRadiusEditField.Value);
            function mousemove(~,~,ax,p,poly)
             try
                p.Vertices=poly.Vertices+ax.CurrentPoint(1,1:2);
                plot = true;
             catch
                 plot = false;
             end
            end
            
            function mouseclick(~,~,ax,p,sides)
                app.counterplot=app.counterplot+1;
                if plot
                    if app.counterplot<2
                        app.plots=createObs(app.robot,app.plots,1,'circ',app.dims,1,3,[app.UIAxes.CurrentPoint(1,1) app.UIAxes.CurrentPoint(1,2)]);
                        app.plots=setObs(app.plots,1);
                        %patch(ax,'Faces',1:sides,'Vertices',p.Vertices,'FaceColor',[174 252 157]/255,'EdgeColor','none');
                        p.Visible='off';
                    else 
                         p.Visible='off';
                    end
                end
            end


        end

        % Button pushed function: PlotButton_2
        function PlotButton_2Pushed(app, event)
            plot=true;
            app.counterplot=0;
            sides = 4;           
            poly=nsidedpoly(4, 'SideLength', (app.RectHeightEditField.Value*sin((pi)/(sides))));
             p=patch(app.UIAxes,'Faces', 1:sides,'Vertices',poly.Vertices,'FaceColor','none','FaceAlpha',1,'EdgeColor',[231 173 247]/255);
             app.UIFigure.WindowButtonMotionFcn={@mousemove,app.UIAxes,p,poly};
             app.UIFigure.WindowButtonDownFcn={@mouseclick, sides};
             app.dims=(app.RectHeightEditField.Value*sin((pi)/(sides)));
            function mousemove(~,~,ax,p,poly)
                try
                    p.Vertices=poly.Vertices+ax.CurrentPoint(1,1:2);
                    plot = true;
                catch
                    plot = false;
                end 
            end

            function mouseclick(~,~,sides)
               app.counterplot=app.counterplot+1;
                if plot 
                    % app.plots.obstacleArr{length(app.plots.obstacleArr) + 1} = {'rect',[app.dims app.dims],1,3,[xy app.UIAxes.CurrentPoint(1,2) 0]};
                    % updateTrack(app);
                if app.counterplot<2
                app.plots = createObs(app.robot,app.plots,1,'rect',[app.dims app.dims],1,3,[app.UIAxes.CurrentPoint(1,1) app.UIAxes.CurrentPoint(1,2) 0]);
                app.plots=setObs(app.plots,1);
                %patch(ax,'Faces',1:sides,'Vertices',p.Vertices,'FaceColor',[231 173 247]/255,'EdgeColor','none');
                p.Visible='off';
                else 
                     p.Visible='off';
                end
                end
            end

        end

        % Button pushed function: StopButton
        function StopButtonPushed(app, event)
            app.stopSim=true;
        end

        % Value changed function: NumberofStatesEditField
        function NumberofStatesEditFieldValueChanged(app, event)
            app.robot.numStates = app.NumberofStatesEditField.Value;
            n = app.robot.numStates^app.robot.numSensors;
            states = 1:app.robot.numStates;
            combos = table2array(combinations(states,states,states,states,states));
            app.robot.statesDict = dictionary;
            for i = 1:n
                app.robot.statesDict({[combos(i, :)]}) = i;
            end
        end

        % Callback function
        function ImageClicked(app, event)
             cla(app.UIAxes)  % clear the app axes to avoid overlaying axes. 
             [app.robot,app.plots,app.sim]=initialize(app);  % Call Intialize and take new inputs, user inputs, and make them simulation inputs.
        end

        % Value changed function: PolyRadiusEditField
        function PolyRadiusEditFieldValueChanged(app, event)
         if app.TrackWidthEditField.Value < (app.roverLengthEditField.Value+(app.RoverWidthEditField.Value-(app.RoverWidthEditField.Value/3)))  
          app.RadiusArray=[((app.roverLengthEditField.Value/2)*0.68) (app.roverLengthEditField.Value)];  % Two possibilities for minimum radius, of an object, that a robot will be able to sense depending on the robot length.
            app.minradius = max(app.RadiusArray);  % Setting and choosing the possibility that works for the robot length choosen to the variable "minradius". 
            app.maxradius=((app.TrackWidthEditField.Value-(app.RoverWidthEditField.Value+(app.RoverWidthEditField.Value/3))));
            app.PolyRadiusEditField.Limits= [app.minradius app.maxradius];  % Setting the polygon's radius edit field box to unlimited values.             app.RectHeightEditField.Limits= [17 inf];  % Setting the Rectangles side length edit field box to unlimited values.
            app.CircleRadiusEditField.Limits= [app.minradius app.maxradius]; % Setting the Circle's radius edit field to unlimited values. 
            app.RectHeightEditField.Limits= [app.minradius app.maxradius];
         end 
        end

        % Value changed function: RectHeightEditField
        function RectHeightEditFieldValueChanged(app, event)
           if app.TrackWidthEditField.Value < (app.roverLengthEditField.Value+(app.RoverWidthEditField.Value-(app.RoverWidthEditField.Value/3)))  
            app.RadiusArray=[((app.roverLengthEditField.Value/2)*0.68) (app.roverLengthEditField.Value)];  % Two possibilities for minimum radius, of an object, that a robot will be able to sense depending on the robot length.
            app.minradius = max(app.RadiusArray);  % Setting and choosing the possibility that works for the robot length choosen to the variable "minradius". 
            app.maxradius=((app.TrackWidthEditField.Value-(app.RoverWidthEditField.Value+(app.RoverWidthEditField.Value))));
            app.PolyRadiusEditField.Limits= [app.minradius app.maxradius];  % Setting the polygon's radius edit field box to unlimited values.             app.RectHeightEditField.Limits= [17 inf];  % Setting the Rectangles side length edit field box to unlimited values.
            app.CircleRadiusEditField.Limits= [app.minradius app.maxradius]; % Setting the Circle's radius edit field to unlimited values. 
            app.RectHeightEditField.Limits= [app.minradius app.maxradius];
           end 
        end

        % Value changed function: CircleRadiusEditField
        function CircleRadiusEditFieldValueChanged(app, event)
          if app.TrackWidthEditField.Value < (app.roverLengthEditField.Value+(app.RoverWidthEditField.Value-(app.RoverWidthEditField.Value/3)))  
            app.RadiusArray=[((app.roverLengthEditField.Value/2)*0.68) (app.roverLengthEditField.Value)];  % Two possibilities for minimum radius, of an object, that a robot will be able to sense depending on the robot length.
            app.minradius = max(app.RadiusArray);  % Setting and choosing the possibility that works for the robot length choosen to the variable "minradius". 
            app.maxradius=((app.TrackWidthEditField.Value-(app.RoverWidthEditField.Value+(app.RoverWidthEditField.Value))));
            app.PolyRadiusEditField.Limits= [app.minradius app.maxradius];  % Setting the polygon's radius edit field box to unlimited values.             app.RectHeightEditField.Limits= [17 inf];  % Setting the Rectangles side length edit field box to unlimited values.
            app.CircleRadiusEditField.Limits= [app.minradius app.maxradius]; % Setting the Circle's radius edit field to unlimited values. 
            app.RectHeightEditField.Limits= [app.minradius app.maxradius];
           end
        end

        % Button pushed function: SaveQMatrixButton
        function downloadQMatrix(app, event)
            qmatrix = app.sim.qmatrix;
            uisave('qmatrix','q_matrix.mat');
        end

        % Button pushed function: LoadQMatrixButton
        function uploadQMatrix(app, event)
            try
                [filename, filepath] = uigetfile("*.mat");
                app.sim.qmatrix = load(fullfile(filepath, filename), 'qmatrix');
                
            catch err
                uialert(app.UIFigure,'Error uploading q-matrix; please check file and try again.','Upload Error');
                disp(err);
            end
        end

        % Button pushed function: LoadTrackButton
        function uploadTrackConfig(app, event)
            %try
                [filename, filepath] = uigetfile("*.mat");
                load(fullfile(filepath, filename),'-mat', 'width','height','length','innerRadius','obstacles');
                [~,n] = size(obstacles);
                app.TrackWidthEditField.Value = width;
                app.TrackHeightEditField.Value = height;
                app.TrackLengthEditField.Value = length;
                app.TrackInnerRadiusEditField.Value = innerRadius;
                app.plots.obstacleArr = obstacles;
                
                updateTrack(app);
                % draw obstacles
                for i = 1:n
                    obs = app.plots.obstacleArr{i};
                    app.plots = createObsNicoleTest(app.robot,app.plots,1,obs{1},obs{2},obs{3},obs{4},obs{5});
                    app.plots=setObs(app.plots,1);
                end

            
            % catch err
            %     uialert(app.UIFigure,'Error uploading track configuration; please check file and try again.','Upload Error');
            %     disp(err);
            % end
        end

        % Button pushed function: SaveTrackButton
        function downloadTrackConfig(app, event)
            width = app.TrackWidthEditField.Value;
            height = app.TrackHeightEditField.Value;
            length = app.TrackLengthEditField.Value;
            innerRadius = app.TrackInnerRadiusEditField.Value;
            obstacles = app.plots.obstacleArr;
            uisave({'width','height','length','innerRadius','obstacles'},'trackConfig.mat');
        end

        % Value changed function: TrackInnerRadiusSlider
        function innerRadiusSliderChanged(app, event)
            app.TrackInnerRadiusEditField.Value = app.TrackInnerRadiusSlider.Value;
            updateTrack(app);
        end

        % Value changed function: TrackInnerRadiusEditField
        function innerRadiusEditFieldChanged(app, event)
            updateTrack(app);
        end

        % Value changed function: TrackWidthSlider
        function trackWidthSliderChanged(app, event)
            app.maxrover=app.TrackWidthEditField.Value/2;
            app.RoverWidthEditField.Limits=[2 app.maxrover];
            app.mintrack=app.RoverWidthEditField.Value*2;
            app.TrackWidthEditField.Limits=[app.mintrack 110];
            app.TrackWidthEditField.Value = app.TrackWidthSlider.Value;
            updateTrack(app);
            if app.TrackWidthEditField.Value < (app.roverLengthEditField.Value+(app.RoverWidthEditField.Value-(app.RoverWidthEditField.Value/3)))  
            app.RadiusArray=[((app.roverLengthEditField.Value/2)*0.68) (app.roverLengthEditField.Value)];  % Two possibilities for minimum radius, of an object, that a robot will be able to sense depending on the robot length.
            app.minradius = max(app.RadiusArray);  % Setting and choosing the possibility that works for the robot length choosen to the variable "minradius". 
            app.maxradius=((app.TrackWidthEditField.Value-(app.RoverWidthEditField.Value+(app.RoverWidthEditField.Value/3))));
            app.PolyRadiusEditField.Limits= [app.minradius app.maxradius];  % Setting the polygon's radius edit field box to unlimited values.             app.RectHeightEditField.Limits= [17 inf];  % Setting the Rectangles side length edit field box to unlimited values.
            app.CircleRadiusEditField.Limits= [app.minradius app.maxradius]; % Setting the Circle's radius edit field to unlimited values. 
            app.RectHeightEditField.Limits= [app.minradius app.maxradius];
            

            end
        end

        % Value changed function: TrackWidthEditField
        function trackWidthEditFieldChanged(app, event)
           app.maxrover=app.TrackWidthEditField.Value/2;
            app.RoverWidthEditField.Limits=[2 app.maxrover];
            app.mintrack=app.RoverWidthEditField.Value*2;
            app.TrackWidthEditField.Limits=[app.mintrack 110];
            updateTrack(app);
            if app.TrackWidthEditField.Value < (app.roverLengthEditField.Value+(app.RoverWidthEditField.Value-(app.RoverWidthEditField.Value/3)))  
            app.RadiusArray=[((app.roverLengthEditField.Value/2)*0.68) (app.roverLengthEditField.Value)];  % Two possibilities for minimum radius, of an object, that a robot will be able to sense depending on the robot length.
            app.minradius = max(app.RadiusArray);  % Setting and choosing the possibility that works for the robot length choosen to the variable "minradius". 
            app.maxradius=((app.TrackWidthEditField.Value-(app.RoverWidthEditField.Value+(app.RoverWidthEditField.Value/3))));

            app.PolyRadiusEditField.Limits= [app.minradius app.maxradius];  % Setting the polygon's radius edit field box to unlimited values.             app.RectHeightEditField.Limits= [17 inf];  % Setting the Rectangles side length edit field box to unlimited values.
            app.CircleRadiusEditField.Limits= [app.minradius app.maxradius]; % Setting the Circle's radius edit field to unlimited values. 
            app.RectHeightEditField.Limits= [app.minradius app.maxradius];
            end
        end

        % Button pushed function: ResetConfigurationButton
        function reset(app, event)
            clearvars app.plots app.sim app.robot app.inputs;
            app.stopSim = true;
            startupFcn(app);
            % cla(app.UIAxes);
            % app.plots.obstacleArr = {};
            % app.inputs = defaultInputs();
            % [app.robot,app.plots,app.sim] = initialize(app);
        end

        % Close request function: UIFigure
        function CloseWindow(app, event)
            app.delete;
        end

        % Value changed function: DecisionMakingSwitch
        function DecisionMakingChanged(app, event)
            value = app.DecisionMakingSwitch.Value;
            switch value
                case "Epsilon-Greedy"
                    app.EpisodesUntilQEditField.Visible = "on";
                    app.EpisodesUntilQLabel.Visible = "on";
                case "Value-Based"
                    app.EpisodesUntilQEditField.Visible = "off";
                    app.EpisodesUntilQLabel.Visible = "off";
            end
        end

        % Value changed function: SpawnPositionSwitch
        function SpawnPositionChanged(app, event)
            value = app.SpawnPositionSwitch.Value;
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 3x1 grid
                app.GridLayout.RowHeight = {630, 630, 630};
                app.GridLayout.ColumnWidth = {'1x'};
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = 1;
                app.LeftPanel.Layout.Row = 2;
                app.LeftPanel.Layout.Column = 1;
                app.RightPanel.Layout.Row = 3;
                app.RightPanel.Layout.Column = 1;
            elseif (currentFigureWidth > app.onePanelWidth && currentFigureWidth <= app.twoPanelWidth)
                % Change to a 2x2 grid
                app.GridLayout.RowHeight = {630, 630};
                app.GridLayout.ColumnWidth = {'1x', '1x'};
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = [1,2];
                app.LeftPanel.Layout.Row = 2;
                app.LeftPanel.Layout.Column = 1;
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 2;
            else
                % Change to a 1x3 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {232, '1x', 278};
                app.LeftPanel.Layout.Row = 1;
                app.LeftPanel.Layout.Column = 1;
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = 2;
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 3;
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Get the file path for locating images
            pathToMLAPP = fileparts(mfilename('fullpath'));

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 1035 630];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @CloseWindow, true);
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {232, '1x', 278};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.TitlePosition = 'centertop';
            app.LeftPanel.Title = 'Customization';
            app.LeftPanel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;
            app.LeftPanel.FontName = 'Cambria Math';
            app.LeftPanel.FontWeight = 'bold';
            app.LeftPanel.FontSize = 18;

            % Create CustomizationTabs
            app.CustomizationTabs = uitabgroup(app.LeftPanel);
            app.CustomizationTabs.Position = [6 151 221 447];

            % Create QLearningTab
            app.QLearningTab = uitab(app.CustomizationTabs);
            app.QLearningTab.Title = 'Q-Learning';
            app.QLearningTab.BackgroundColor = [0.9412 0.9412 0.9412];

            % Create MaxEpisodesEditFieldLabel
            app.MaxEpisodesEditFieldLabel = uilabel(app.QLearningTab);
            app.MaxEpisodesEditFieldLabel.FontName = 'Cambria Math';
            app.MaxEpisodesEditFieldLabel.Position = [8 241 107 22];
            app.MaxEpisodesEditFieldLabel.Text = 'Maximum Episodes :';

            % Create MaxEpisodesEditField
            app.MaxEpisodesEditField = uieditfield(app.QLearningTab, 'numeric');
            app.MaxEpisodesEditField.FontName = 'Cambria Math';
            app.MaxEpisodesEditField.Position = [120 241 94 22];
            app.MaxEpisodesEditField.Value = 5000;

            % Create MaximumStepsLabel
            app.MaximumStepsLabel = uilabel(app.QLearningTab);
            app.MaximumStepsLabel.FontName = 'Cambria Math';
            app.MaximumStepsLabel.Position = [8 212 101 22];
            app.MaximumStepsLabel.Text = 'Maximum Steps:';

            % Create MaxStepsEditField
            app.MaxStepsEditField = uieditfield(app.QLearningTab, 'numeric');
            app.MaxStepsEditField.FontName = 'Cambria Math';
            app.MaxStepsEditField.Position = [99 211 115 22];
            app.MaxStepsEditField.Value = 1000;

            % Create LearningRateLabel
            app.LearningRateLabel = uilabel(app.QLearningTab);
            app.LearningRateLabel.FontName = 'Cambria Math';
            app.LearningRateLabel.Position = [9 366 77 22];
            app.LearningRateLabel.Text = 'Learning Rate:';

            % Create LearningRateEditField
            app.LearningRateEditField = uieditfield(app.QLearningTab, 'numeric');
            app.LearningRateEditField.FontName = 'Cambria Math';
            app.LearningRateEditField.Position = [92 366 122 22];
            app.LearningRateEditField.Value = 0.2;

            % Create DiscountFactorLabel
            app.DiscountFactorLabel = uilabel(app.QLearningTab);
            app.DiscountFactorLabel.FontName = 'Cambria Math';
            app.DiscountFactorLabel.Position = [9 334 88 22];
            app.DiscountFactorLabel.Text = 'Discount Factor:';

            % Create DiscountFactorEditField
            app.DiscountFactorEditField = uieditfield(app.QLearningTab, 'numeric');
            app.DiscountFactorEditField.FontName = 'Cambria Math';
            app.DiscountFactorEditField.Position = [104 334 110 22];
            app.DiscountFactorEditField.Value = 0.8;

            % Create NumberOfStatesLabel
            app.NumberOfStatesLabel = uilabel(app.QLearningTab);
            app.NumberOfStatesLabel.FontName = 'Cambria Math';
            app.NumberOfStatesLabel.Position = [9 302 99 22];
            app.NumberOfStatesLabel.Text = 'Sensor Segments:';

            % Create NumberofStatesEditField
            app.NumberofStatesEditField = uieditfield(app.QLearningTab, 'numeric');
            app.NumberofStatesEditField.Limits = [0 100];
            app.NumberofStatesEditField.RoundFractionalValues = 'on';
            app.NumberofStatesEditField.ValueChangedFcn = createCallbackFcn(app, @NumberofStatesEditFieldValueChanged, true);
            app.NumberofStatesEditField.FontName = 'Cambria Math';
            app.NumberofStatesEditField.Position = [104 302 110 22];
            app.NumberofStatesEditField.Value = 3;

            % Create EpisodesUntilQEditField
            app.EpisodesUntilQEditField = uieditfield(app.QLearningTab, 'numeric');
            app.EpisodesUntilQEditField.FontName = 'Cambria Math';
            app.EpisodesUntilQEditField.Position = [136 120 74 22];
            app.EpisodesUntilQEditField.Value = 50;

            % Create DeltaTimeLabel
            app.DeltaTimeLabel = uilabel(app.QLearningTab);
            app.DeltaTimeLabel.FontName = 'Cambria Math';
            app.DeltaTimeLabel.Position = [10 272 100 22];
            app.DeltaTimeLabel.Text = 'Delta Time:';

            % Create DeltaTimeEditField
            app.DeltaTimeEditField = uieditfield(app.QLearningTab, 'numeric');
            app.DeltaTimeEditField.FontName = 'Cambria Math';
            app.DeltaTimeEditField.Position = [73 272 142 22];
            app.DeltaTimeEditField.Value = 0.2;

            % Create DecisionMakingSwitch
            app.DecisionMakingSwitch = uiswitch(app.QLearningTab, 'slider');
            app.DecisionMakingSwitch.Items = {'Epsilon-Greedy', 'Value-Based'};
            app.DecisionMakingSwitch.ValueChangedFcn = createCallbackFcn(app, @DecisionMakingChanged, true);
            app.DecisionMakingSwitch.FontName = 'cambria math';
            app.DecisionMakingSwitch.Position = [99 156 36 16];
            app.DecisionMakingSwitch.Value = 'Epsilon-Greedy';

            % Create TrainingParametersLabel
            app.TrainingParametersLabel = uilabel(app.QLearningTab);
            app.TrainingParametersLabel.HorizontalAlignment = 'center';
            app.TrainingParametersLabel.FontName = 'Cambria Math';
            app.TrainingParametersLabel.FontSize = 18;
            app.TrainingParametersLabel.FontWeight = 'bold';
            app.TrainingParametersLabel.Position = [0 393 219 23];
            app.TrainingParametersLabel.Text = 'Training Parameters';

            % Create DecisionMakingLabel
            app.DecisionMakingLabel = uilabel(app.QLearningTab);
            app.DecisionMakingLabel.HorizontalAlignment = 'center';
            app.DecisionMakingLabel.FontWeight = 'bold';
            app.DecisionMakingLabel.Position = [18 178 180 22];
            app.DecisionMakingLabel.Text = 'Decision-Making';

            % Create EpisodesUntilQLabel
            app.EpisodesUntilQLabel = uilabel(app.QLearningTab);
            app.EpisodesUntilQLabel.WordWrap = 'on';
            app.EpisodesUntilQLabel.FontName = 'Cambria Math';
            app.EpisodesUntilQLabel.Position = [11 106 119 44];
            app.EpisodesUntilQLabel.Text = 'Episodes Until Fully Value-Based Decisions';

            % Create RoverTab
            app.RoverTab = uitab(app.CustomizationTabs);
            app.RoverTab.Title = 'Rover';
            app.RoverTab.BackgroundColor = [0.9412 0.9412 0.9412];

            % Create WidthLabel
            app.WidthLabel = uilabel(app.RoverTab);
            app.WidthLabel.FontName = 'Cambria Math';
            app.WidthLabel.Position = [10 369 39 22];
            app.WidthLabel.Text = 'Width:';

            % Create RoverWidthEditField
            app.RoverWidthEditField = uieditfield(app.RoverTab, 'numeric');
            app.RoverWidthEditField.ValueChangedFcn = createCallbackFcn(app, @RoverWidthEditFieldValueChanged, true);
            app.RoverWidthEditField.FontName = 'Cambria Math';
            app.RoverWidthEditField.Position = [55 368 153 22];

            % Create LengthLabel
            app.LengthLabel = uilabel(app.RoverTab);
            app.LengthLabel.FontName = 'Cambria Math';
            app.LengthLabel.Position = [9 339 43 22];
            app.LengthLabel.Text = 'Length:';

            % Create roverLengthEditField
            app.roverLengthEditField = uieditfield(app.RoverTab, 'numeric');
            app.roverLengthEditField.ValueChangedFcn = createCallbackFcn(app, @roverLengthEditFieldValueChanged, true);
            app.roverLengthEditField.FontName = 'Cambria Math';
            app.roverLengthEditField.Position = [55 339 153 22];

            % Create PowerEditFieldLabel
            app.PowerEditFieldLabel = uilabel(app.RoverTab);
            app.PowerEditFieldLabel.FontName = 'Cambria Math';
            app.PowerEditFieldLabel.Position = [8 310 41 22];
            app.PowerEditFieldLabel.Text = 'Power:';

            % Create PowerEditField
            app.PowerEditField = uieditfield(app.RoverTab, 'numeric');
            app.PowerEditField.Limits = [0 5];
            app.PowerEditField.FontName = 'Cambria Math';
            app.PowerEditField.Position = [55 310 153 22];
            app.PowerEditField.Value = 3;

            % Create TurnPowerEditFieldLabel
            app.TurnPowerEditFieldLabel = uilabel(app.RoverTab);
            app.TurnPowerEditFieldLabel.FontName = 'Cambria Math';
            app.TurnPowerEditFieldLabel.Position = [9 279 71 22];
            app.TurnPowerEditFieldLabel.Text = 'Turn Power: ';

            % Create TurnPowerEditField
            app.TurnPowerEditField = uieditfield(app.RoverTab, 'numeric');
            app.TurnPowerEditField.Limits = [0 5];
            app.TurnPowerEditField.FontName = 'Cambria Math';
            app.TurnPowerEditField.Position = [80 279 128 22];
            app.TurnPowerEditField.Value = 2;

            % Create RoverParametersLabel
            app.RoverParametersLabel = uilabel(app.RoverTab);
            app.RoverParametersLabel.HorizontalAlignment = 'center';
            app.RoverParametersLabel.FontName = 'Cambria Math';
            app.RoverParametersLabel.FontSize = 18;
            app.RoverParametersLabel.FontWeight = 'bold';
            app.RoverParametersLabel.Position = [0 393 219 23];
            app.RoverParametersLabel.Text = 'Rover Parameters';

            % Create SpawnPositionSwitchLabel
            app.SpawnPositionSwitchLabel = uilabel(app.RoverTab);
            app.SpawnPositionSwitchLabel.HorizontalAlignment = 'center';
            app.SpawnPositionSwitchLabel.FontName = 'Cambria Math';
            app.SpawnPositionSwitchLabel.FontSize = 14;
            app.SpawnPositionSwitchLabel.FontWeight = 'bold';
            app.SpawnPositionSwitchLabel.Position = [1 243 214 22];
            app.SpawnPositionSwitchLabel.Text = 'Spawn Position';

            % Create SpawnPositionSwitch
            app.SpawnPositionSwitch = uiswitch(app.RoverTab, 'slider');
            app.SpawnPositionSwitch.Items = {'Randomized', 'Custom'};
            app.SpawnPositionSwitch.ValueChangedFcn = createCallbackFcn(app, @SpawnPositionChanged, true);
            app.SpawnPositionSwitch.FontName = 'Cambria Math';
            app.SpawnPositionSwitch.Position = [92 222 37 16];
            app.SpawnPositionSwitch.Value = 'Randomized';

            % Create TrackTab
            app.TrackTab = uitab(app.CustomizationTabs);
            app.TrackTab.Title = 'Track';
            app.TrackTab.BackgroundColor = [0.9412 0.9412 0.9412];

            % Create TrackHeightSlider
            app.TrackHeightSlider = uislider(app.TrackTab);
            app.TrackHeightSlider.Limits = [0 300];
            app.TrackHeightSlider.ValueChangedFcn = createCallbackFcn(app, @TrackHeightSliderValueChanged, true);
            app.TrackHeightSlider.FontName = 'Cambria Math';
            app.TrackHeightSlider.Position = [13 351 189 3];
            app.TrackHeightSlider.Value = 60;

            % Create TrackHeightLabel
            app.TrackHeightLabel = uilabel(app.TrackTab);
            app.TrackHeightLabel.FontName = 'Cambria Math';
            app.TrackHeightLabel.Position = [13 365 41 22];
            app.TrackHeightLabel.Text = 'Height:';

            % Create TrackHeightEditField
            app.TrackHeightEditField = uieditfield(app.TrackTab, 'numeric');
            app.TrackHeightEditField.ValueChangedFcn = createCallbackFcn(app, @TrackHeightEditFieldValueChanged, true);
            app.TrackHeightEditField.FontName = 'Cambria Math';
            app.TrackHeightEditField.Position = [59 365 132 22];
            app.TrackHeightEditField.Value = 100;

            % Create TrackLengthLabel
            app.TrackLengthLabel = uilabel(app.TrackTab);
            app.TrackLengthLabel.FontName = 'Cambria Math';
            app.TrackLengthLabel.Position = [11 287 45 22];
            app.TrackLengthLabel.Text = ' Length:';

            % Create TrackLengthEditField
            app.TrackLengthEditField = uieditfield(app.TrackTab, 'numeric');
            app.TrackLengthEditField.ValueChangedFcn = createCallbackFcn(app, @TrackLengthEditFieldValueChanged, true);
            app.TrackLengthEditField.FontName = 'Cambria Math';
            app.TrackLengthEditField.Position = [59 286 145 22];
            app.TrackLengthEditField.Value = 200;

            % Create TrackLengthSlider
            app.TrackLengthSlider = uislider(app.TrackTab);
            app.TrackLengthSlider.Limits = [0 300];
            app.TrackLengthSlider.ValueChangedFcn = createCallbackFcn(app, @TrackLengthSliderValueChanged, true);
            app.TrackLengthSlider.FontName = 'Cambria Math';
            app.TrackLengthSlider.Position = [19 269 184 3];

            % Create TrackWidthLabel
            app.TrackWidthLabel = uilabel(app.TrackTab);
            app.TrackWidthLabel.FontName = 'Cambria Math';
            app.TrackWidthLabel.Position = [15 205 39 22];
            app.TrackWidthLabel.Text = 'Width:';

            % Create TrackWidthEditField
            app.TrackWidthEditField = uieditfield(app.TrackTab, 'numeric');
            app.TrackWidthEditField.ValueChangedFcn = createCallbackFcn(app, @trackWidthEditFieldChanged, true);
            app.TrackWidthEditField.FontName = 'Cambria Math';
            app.TrackWidthEditField.Position = [59 205 132 22];
            app.TrackWidthEditField.Value = 60;

            % Create TrackWidthSlider
            app.TrackWidthSlider = uislider(app.TrackTab);
            app.TrackWidthSlider.Limits = [0 110];
            app.TrackWidthSlider.MajorTicks = [0 10 30 50 70 90 110];
            app.TrackWidthSlider.ValueChangedFcn = createCallbackFcn(app, @trackWidthSliderChanged, true);
            app.TrackWidthSlider.FontName = 'Cambria Math';
            app.TrackWidthSlider.Position = [19 186 183 3];
            app.TrackWidthSlider.Value = 60;

            % Create TrackInnerRadiusLabel
            app.TrackInnerRadiusLabel = uilabel(app.TrackTab);
            app.TrackInnerRadiusLabel.FontName = 'Cambria Math';
            app.TrackInnerRadiusLabel.Position = [13 128 71 22];
            app.TrackInnerRadiusLabel.Text = 'Inner Radius:';

            % Create TrackInnerRadiusEditField
            app.TrackInnerRadiusEditField = uieditfield(app.TrackTab, 'numeric');
            app.TrackInnerRadiusEditField.ValueChangedFcn = createCallbackFcn(app, @innerRadiusEditFieldChanged, true);
            app.TrackInnerRadiusEditField.FontName = 'Cambria Math';
            app.TrackInnerRadiusEditField.Position = [85 127 108 22];
            app.TrackInnerRadiusEditField.Value = 40;

            % Create TrackInnerRadiusSlider
            app.TrackInnerRadiusSlider = uislider(app.TrackTab);
            app.TrackInnerRadiusSlider.Limits = [0 200];
            app.TrackInnerRadiusSlider.ValueChangedFcn = createCallbackFcn(app, @innerRadiusSliderChanged, true);
            app.TrackInnerRadiusSlider.FontName = 'Cambria Math';
            app.TrackInnerRadiusSlider.Position = [21 108 182 3];

            % Create TrackParametersLabel
            app.TrackParametersLabel = uilabel(app.TrackTab);
            app.TrackParametersLabel.HorizontalAlignment = 'center';
            app.TrackParametersLabel.FontName = 'Cambria Math';
            app.TrackParametersLabel.FontSize = 18;
            app.TrackParametersLabel.FontWeight = 'bold';
            app.TrackParametersLabel.Position = [0 392 219 23];
            app.TrackParametersLabel.Text = 'Track Parameters';

            % Create ObstaclesTab
            app.ObstaclesTab = uitab(app.CustomizationTabs);
            app.ObstaclesTab.Title = 'Obstacles';
            app.ObstaclesTab.BackgroundColor = [0.9412 0.9412 0.9412];

            % Create polygonimage
            app.polygonimage = uiimage(app.ObstaclesTab);
            app.polygonimage.Position = [144 325 71 67];
            app.polygonimage.ImageSource = fullfile(pathToMLAPP, 'Images', 'polynewnew.png');

            % Create circleImage
            app.circleImage = uiimage(app.ObstaclesTab);
            app.circleImage.Position = [136 120 79 72];
            app.circleImage.ImageSource = fullfile(pathToMLAPP, 'Images', 'GreenCircle-removebg-preview.png');

            % Create RectHeightEditField
            app.RectHeightEditField = uieditfield(app.ObstaclesTab, 'numeric');
            app.RectHeightEditField.ValueChangedFcn = createCallbackFcn(app, @RectHeightEditFieldValueChanged, true);
            app.RectHeightEditField.FontName = 'Cambria Math';
            app.RectHeightEditField.Position = [81 233 54 22];
            app.RectHeightEditField.Value = 6;

            % Create RectHeightLabel
            app.RectHeightLabel = uilabel(app.ObstaclesTab);
            app.RectHeightLabel.FontName = 'Cambria Math';
            app.RectHeightLabel.Position = [13 231 66 22];
            app.RectHeightLabel.Text = 'Side Length:';

            % Create CircleRadiusEditField
            app.CircleRadiusEditField = uieditfield(app.ObstaclesTab, 'numeric');
            app.CircleRadiusEditField.ValueChangedFcn = createCallbackFcn(app, @CircleRadiusEditFieldValueChanged, true);
            app.CircleRadiusEditField.FontName = 'Cambria Math';
            app.CircleRadiusEditField.Position = [55 136 80 22];
            app.CircleRadiusEditField.Value = 3;

            % Create CircleRadiusLabel
            app.CircleRadiusLabel = uilabel(app.ObstaclesTab);
            app.CircleRadiusLabel.FontName = 'Cambria Math';
            app.CircleRadiusLabel.Position = [12 136 82 22];
            app.CircleRadiusLabel.Text = 'Radius:';

            % Create SideNumberEditField
            app.SideNumberEditField = uieditfield(app.ObstaclesTab, 'numeric');
            app.SideNumberEditField.FontName = 'Cambria Math';
            app.SideNumberEditField.Position = [79 334 56 22];
            app.SideNumberEditField.Value = 3;

            % Create PlotButton
            app.PlotButton = uibutton(app.ObstaclesTab, 'push');
            app.PlotButton.ButtonPushedFcn = createCallbackFcn(app, @PlotButtonPushed, true);
            app.PlotButton.BackgroundColor = [0.902 0.902 0.902];
            app.PlotButton.FontName = 'Cambria Math';
            app.PlotButton.FontColor = [0.149 0.149 0.149];
            app.PlotButton.Position = [153 307 55 22];
            app.PlotButton.Text = 'Plot ';

            % Create PlotButton_2
            app.PlotButton_2 = uibutton(app.ObstaclesTab, 'push');
            app.PlotButton_2.ButtonPushedFcn = createCallbackFcn(app, @PlotButton_2Pushed, true);
            app.PlotButton_2.BackgroundColor = [0.902 0.902 0.902];
            app.PlotButton_2.FontName = 'Cambria Math';
            app.PlotButton_2.FontColor = [0.149 0.149 0.149];
            app.PlotButton_2.Position = [159 200 44 22];
            app.PlotButton_2.Text = 'Plot';

            % Create PlotButton_3
            app.PlotButton_3 = uibutton(app.ObstaclesTab, 'push');
            app.PlotButton_3.ButtonPushedFcn = createCallbackFcn(app, @PlotButton_3Pushed, true);
            app.PlotButton_3.BackgroundColor = [0.902 0.902 0.902];
            app.PlotButton_3.FontName = 'Cambria Math';
            app.PlotButton_3.FontColor = [0.149 0.149 0.149];
            app.PlotButton_3.Position = [158 108 45 22];
            app.PlotButton_3.Text = 'Plot';

            % Create PolyRadiusEditField
            app.PolyRadiusEditField = uieditfield(app.ObstaclesTab, 'numeric');
            app.PolyRadiusEditField.ValueChangedFcn = createCallbackFcn(app, @PolyRadiusEditFieldValueChanged, true);
            app.PolyRadiusEditField.FontName = 'Cambria Math';
            app.PolyRadiusEditField.Position = [87 303 48 22];
            app.PolyRadiusEditField.Value = 6;

            % Create polyradiuslabel
            app.polyradiuslabel = uilabel(app.ObstaclesTab);
            app.polyradiuslabel.FontName = 'Cambria Math';
            app.polyradiuslabel.Position = [11 303 75 22];
            app.polyradiuslabel.Text = 'Circumradius:';

            % Create SideNumberLabel
            app.SideNumberLabel = uilabel(app.ObstaclesTab);
            app.SideNumberLabel.FontName = 'Cambria Math';
            app.SideNumberLabel.Position = [13 334 61 22];
            app.SideNumberLabel.Text = 'Side Count:';

            % Create CircularObstacleLabel
            app.CircularObstacleLabel = uilabel(app.ObstaclesTab);
            app.CircularObstacleLabel.FontName = 'Cambria Math';
            app.CircularObstacleLabel.FontSize = 14;
            app.CircularObstacleLabel.Position = [11 167 110 22];
            app.CircularObstacleLabel.Text = 'Circular Obstacle';

            % Create SquareObstacleLabel
            app.SquareObstacleLabel = uilabel(app.ObstaclesTab);
            app.SquareObstacleLabel.FontName = 'Cambria Math';
            app.SquareObstacleLabel.FontSize = 14;
            app.SquareObstacleLabel.Position = [13 262 105 22];
            app.SquareObstacleLabel.Text = 'Square Obstacle';

            % Create PolygonalObstacleLabel
            app.PolygonalObstacleLabel = uilabel(app.ObstaclesTab);
            app.PolygonalObstacleLabel.FontName = 'Cambria Math';
            app.PolygonalObstacleLabel.FontSize = 14;
            app.PolygonalObstacleLabel.Position = [12 363 123 22];
            app.PolygonalObstacleLabel.Text = 'Polygonal Obstacle';

            % Create polygonimage_2
            app.polygonimage_2 = uiimage(app.ObstaclesTab);
            app.polygonimage_2.Position = [153 230 57 50];
            app.polygonimage_2.ImageSource = fullfile(pathToMLAPP, 'Images', 'premier-paints-t60-5-pale-sunflower-paint-color-match-2.jpg');

            % Create ObstacleLibraryLabel
            app.ObstacleLibraryLabel = uilabel(app.ObstaclesTab);
            app.ObstacleLibraryLabel.HorizontalAlignment = 'center';
            app.ObstacleLibraryLabel.FontName = 'Cambria Math';
            app.ObstacleLibraryLabel.FontSize = 18;
            app.ObstacleLibraryLabel.FontWeight = 'bold';
            app.ObstacleLibraryLabel.Position = [0 392 219 23];
            app.ObstacleLibraryLabel.Text = 'Obstacle Library';

            % Create CenterPanel
            app.CenterPanel = uipanel(app.GridLayout);
            app.CenterPanel.BackgroundColor = [0.9804 0.9804 0.9804];
            app.CenterPanel.Layout.Row = 1;
            app.CenterPanel.Layout.Column = 2;

            % Create UIAxes
            app.UIAxes = uiaxes(app.CenterPanel);
            app.UIAxes.Toolbar.Visible = 'off';
            app.UIAxes.PlotBoxAspectRatio = [1 1 1];
            app.UIAxes.FontName = 'Cambria Math';
            app.UIAxes.XLim = [-200 200];
            app.UIAxes.YLim = [-250 250];
            app.UIAxes.GridLineWidth = 0.05;
            app.UIAxes.GridLineStyle = 'none';
            app.UIAxes.BoxStyle = 'full';
            app.UIAxes.LineWidth = 0.1;
            app.UIAxes.Color = [0.9804 0.9804 0.9804];
            app.UIAxes.Box = 'on';
            app.UIAxes.Position = [7 12 513 611];

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.TitlePosition = 'centertop';
            app.RightPanel.Title = 'Simulation';
            app.RightPanel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 3;
            app.RightPanel.FontName = 'Cambria';
            app.RightPanel.FontWeight = 'bold';
            app.RightPanel.FontSize = 18;

            % Create StartButton
            app.StartButton = uibutton(app.RightPanel, 'push');
            app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @StartButtonPushed, true);
            app.StartButton.BackgroundColor = [0.6941 0.9882 0.7569];
            app.StartButton.FontName = 'Cambria Math';
            app.StartButton.FontSize = 14;
            app.StartButton.Position = [30 565 86 25];
            app.StartButton.Text = 'Start';

            % Create StopButton
            app.StopButton = uibutton(app.RightPanel, 'push');
            app.StopButton.ButtonPushedFcn = createCallbackFcn(app, @StopButtonPushed, true);
            app.StopButton.BackgroundColor = [1 0.3216 0.3216];
            app.StopButton.FontName = 'Cambria Math';
            app.StopButton.FontSize = 14;
            app.StopButton.Position = [147 566 100 25];
            app.StopButton.Text = 'Stop';

            % Create ConfigPanel
            app.ConfigPanel = uipanel(app.RightPanel);
            app.ConfigPanel.BorderType = 'none';
            app.ConfigPanel.Position = [19 12 243 109];

            % Create SaveQMatrixButton
            app.SaveQMatrixButton = uibutton(app.ConfigPanel, 'push');
            app.SaveQMatrixButton.ButtonPushedFcn = createCallbackFcn(app, @downloadQMatrix, true);
            app.SaveQMatrixButton.BackgroundColor = [0.902 0.902 0.902];
            app.SaveQMatrixButton.FontName = 'Cambria Math';
            app.SaveQMatrixButton.Position = [136 44 90 22];
            app.SaveQMatrixButton.Text = 'Save Q-Matrix';

            % Create LoadQMatrixButton
            app.LoadQMatrixButton = uibutton(app.ConfigPanel, 'push');
            app.LoadQMatrixButton.ButtonPushedFcn = createCallbackFcn(app, @uploadQMatrix, true);
            app.LoadQMatrixButton.BackgroundColor = [0.902 0.902 0.902];
            app.LoadQMatrixButton.FontName = 'Cambria Math';
            app.LoadQMatrixButton.Position = [30 44 93 22];
            app.LoadQMatrixButton.Text = 'Load Q-Matrix';

            % Create SaveTrackButton
            app.SaveTrackButton = uibutton(app.ConfigPanel, 'push');
            app.SaveTrackButton.ButtonPushedFcn = createCallbackFcn(app, @downloadTrackConfig, true);
            app.SaveTrackButton.BackgroundColor = [0.902 0.902 0.902];
            app.SaveTrackButton.FontName = 'Cambria Math';
            app.SaveTrackButton.Position = [136 77 90 22];
            app.SaveTrackButton.Text = 'Save Track';

            % Create LoadTrackButton
            app.LoadTrackButton = uibutton(app.ConfigPanel, 'push');
            app.LoadTrackButton.ButtonPushedFcn = createCallbackFcn(app, @uploadTrackConfig, true);
            app.LoadTrackButton.BackgroundColor = [0.902 0.902 0.902];
            app.LoadTrackButton.FontName = 'Cambria Math';
            app.LoadTrackButton.Position = [30 77 93 22];
            app.LoadTrackButton.Text = 'Load Track';

            % Create ResetConfigurationButton
            app.ResetConfigurationButton = uibutton(app.ConfigPanel, 'push');
            app.ResetConfigurationButton.ButtonPushedFcn = createCallbackFcn(app, @reset, true);
            app.ResetConfigurationButton.BackgroundColor = [0.902 0.902 0.902];
            app.ResetConfigurationButton.FontName = 'Cambria Math';
            app.ResetConfigurationButton.Position = [30 12 197 22];
            app.ResetConfigurationButton.Text = 'Reset Configuration';

            % Create Panel_2
            app.Panel_2 = uipanel(app.RightPanel);
            app.Panel_2.BorderType = 'none';
            app.Panel_2.Position = [7 130 266 423];

            % Create RewardsGraph
            app.RewardsGraph = uiaxes(app.Panel_2);
            xlabel(app.RewardsGraph, 'Episode')
            ylabel(app.RewardsGraph, 'Average Reward')
            zlabel(app.RewardsGraph, 'Z')
            subtitle(app.RewardsGraph, ' ')
            app.RewardsGraph.PlotBoxAspectRatio = [1 1 1];
            app.RewardsGraph.FontName = 'Cambria Math';
            app.RewardsGraph.Position = [2 156 129 137];

            % Create ConvergenceGraph
            app.ConvergenceGraph = uiaxes(app.Panel_2);
            xlabel(app.ConvergenceGraph, 'Episode')
            ylabel(app.ConvergenceGraph, 'Average Q Change')
            zlabel(app.ConvergenceGraph, 'Z')
            subtitle(app.ConvergenceGraph, ' ')
            app.ConvergenceGraph.PlotBoxAspectRatio = [1 1 1];
            app.ConvergenceGraph.FontName = 'Cambria Math';
            app.ConvergenceGraph.Position = [2 3 133 140];

            % Create LossGraph
            app.LossGraph = uiaxes(app.Panel_2);
            xlabel(app.LossGraph, 'Episode')
            ylabel(app.LossGraph, 'Learning Curve Loss')
            zlabel(app.LossGraph, 'Z')
            subtitle(app.LossGraph, ' ')
            app.LossGraph.PlotBoxAspectRatio = [1 1 1];
            app.LossGraph.FontName = 'Cambria Math';
            app.LossGraph.Position = [134 155 129 137];

            % Create EpsilonGreedyGraph
            app.EpsilonGreedyGraph = uiaxes(app.Panel_2);
            xlabel(app.EpsilonGreedyGraph, 'Episode')
            ylabel(app.EpsilonGreedyGraph, 'ε-Greedy Exploration')
            zlabel(app.EpsilonGreedyGraph, 'Z')
            subtitle(app.EpsilonGreedyGraph, ' ')
            app.EpsilonGreedyGraph.PlotBoxAspectRatio = [1 1 1];
            app.EpsilonGreedyGraph.FontName = 'Cambria Math';
            app.EpsilonGreedyGraph.Position = [134 5 129 137];

            % Create LapCountLabel
            app.LapCountLabel = uilabel(app.Panel_2);
            app.LapCountLabel.FontName = 'Cambria Math';
            app.LapCountLabel.Position = [41 369 59 22];
            app.LapCountLabel.Text = 'Lap Count:';

            % Create LapCountEditField
            app.LapCountEditField = uieditfield(app.Panel_2, 'numeric');
            app.LapCountEditField.RoundFractionalValues = 'on';
            app.LapCountEditField.Editable = 'off';
            app.LapCountEditField.FontName = 'Cambria Math';
            app.LapCountEditField.Position = [134 371 90 22];

            % Create SimulationTimeLabel
            app.SimulationTimeLabel = uilabel(app.Panel_2);
            app.SimulationTimeLabel.FontName = 'Cambria Math';
            app.SimulationTimeLabel.Position = [41 338 90 22];
            app.SimulationTimeLabel.Text = 'Simulation Time:';

            % Create FPSLabel
            app.FPSLabel = uilabel(app.Panel_2);
            app.FPSLabel.FontName = 'Cambria Math';
            app.FPSLabel.Position = [41 304 27 22];
            app.FPSLabel.Text = 'FPS:';

            % Create SimulationTimeEditField
            app.SimulationTimeEditField = uieditfield(app.Panel_2, 'numeric');
            app.SimulationTimeEditField.RoundFractionalValues = 'on';
            app.SimulationTimeEditField.Editable = 'off';
            app.SimulationTimeEditField.FontName = 'Cambria Math';
            app.SimulationTimeEditField.Position = [134 339 90 22];

            % Create FramesPerSecondEditField
            app.FramesPerSecondEditField = uieditfield(app.Panel_2, 'numeric');
            app.FramesPerSecondEditField.RoundFractionalValues = 'on';
            app.FramesPerSecondEditField.Editable = 'off';
            app.FramesPerSecondEditField.FontName = 'Cambria Math';
            app.FramesPerSecondEditField.Position = [134 306 90 22];

            % Create EpisodeCountLabel
            app.EpisodeCountLabel = uilabel(app.Panel_2);
            app.EpisodeCountLabel.FontName = 'Cambria Math';
            app.EpisodeCountLabel.Position = [41 402 80 22];
            app.EpisodeCountLabel.Text = 'Episode Count:';

            % Create EpisodeCountEditField
            app.EpisodeCountEditField = uieditfield(app.Panel_2, 'numeric');
            app.EpisodeCountEditField.RoundFractionalValues = 'on';
            app.EpisodeCountEditField.Editable = 'off';
            app.EpisodeCountEditField.FontName = 'Cambria Math';
            app.EpisodeCountEditField.Position = [134 402 90 22];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Application(varargin)

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @(app)startupFcn(app, varargin{:}))

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end