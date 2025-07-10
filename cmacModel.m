% -------------------------------------------------------------------------
% CMAC Approximation of a q function
% -------------------------------------------------------------------------
function [cmac, qval] = cmacModel(app, a)
    cmac= app.cmac;
    % Initialize arrays with zeros

    % conceptual address
    s = squeeze(cmac.sMatrix(a,:,:));
    % actual address in memory
    ad = squeeze(cmac.adMatrix(a,:,:));

    % System Input (Input layer)
    cmac.pred = 0;

    % Create vector from robot to objective
    % objX = app.robot.obj.coords(0) - app.robot.center(0);
    % objY = app.robot.obj.coords(1) - app.robot.center(1);

    % predict q-value of  action
    u = app.robot.sensor.ultrasonic.distances;

    for i =1:1:length(u)
        if u(i)<cmac.inputRanges(i,1)
            u(i) = cmac.inputRanges(i,1);
        end

        if u(i)>cmac.inputRanges(i,2)
            u(i) = cmac.inputRanges(i,2);
        end
    end

    % concept mapping and actual mapping
    % Mapping U --> A
    % Mapping A --> P (hashing)

    for d=1:1:cmac.numInputs
        uMin = cmac.inputRanges(d, 1);
        uMax = cmac.inputRanges(d, 2);

        % iteratates from 1 to the generalization width, meaning input is
        % mapped to 3 different nearby cells for generalization
        for i=1:1:cmac.c
            %Normalizes the input, scales it to a table of size M and rounds
            s(d,i) = round((u(d)-uMin)*cmac.sensorRes/(uMax-uMin))+i;    % Quantity:U-->A
            ad(d, i) = mod(s(d,i),cmac.N)+1;                        % Hash transfer:A--> P
        end
        % Update the prediction
end
qval = 0;



% Output calculation
% for j = 1:1:cmac.numInputs
%     for i=1:1:cmac.c
%         qval = qval + cmac.wMatrix(a, ad(j,i));
%     end
% end
% qval = qval / (cmac.numInputs * cmac.c);

% replaced the above with a calculation that removes duplicate ad's
unique_ad = unique(ad(:));
for i = 1:length(unique_ad)
    qval = qval + cmac.wMatrix(a, unique_ad(i));
end
qval = qval / length(unique_ad);

cmac.adMatrix(a,:,:) = ad;
cmac.sMatrix(a,:,:) = s;
end
