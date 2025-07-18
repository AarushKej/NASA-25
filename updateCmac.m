% -------------------------------------------------------------------------
% CMAC Approximation of a q function
% -------------------------------------------------------------------------
function cmac =updateCmac(app, a, pred, reward)
cmac = app.cmac;

% get future expected q values based on the action just taken
[~, futureQs] = cmacModel(app);

                % discount factor
reward = reward + app.cmac.gamma * max(futureQs);

% tracking error
error=reward-pred;

% ad = squeeze(cmac.adMatrix(a,:,:));
% s = squeeze(cmac.sMatrix(a,:,:));


unique_ad = unique(cmac.adMatrix(a,:,:)); % turn into 1d array of unique addresses
delta = cmac.alpha*error/length(unique_ad);

% weight update using gradient descent w/ momentum

j = unique_ad(:);
cmac.d_w(a,j) = delta;
cmac.wMatrix(a,j) =cmac.w_1(a,j)+cmac.d_w(a,j) + cmac.beta*(cmac.w_1(a,j) - cmac.w_2(a,j));

% iteration update
% Parameters Update %
cmac.w_2=cmac.w_1;cmac.w_1=cmac.wMatrix;
cmac.y_1(a)=reward;
cmac.u_1(a) = pred;

% cmac.adMatrix(a,:,:) = ad;
% cmac.sMatrix(a,:,:) = s;
end