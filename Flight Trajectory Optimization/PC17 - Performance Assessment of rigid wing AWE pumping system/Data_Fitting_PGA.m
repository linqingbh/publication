clc;clear all;close all;

%% algorithm verification
% T          = 4000.*rand(100,1); % generate torque meas
% w          = 700.*rand(100,1);  % generate omega meas
% ptrue      = [1 0.1 -0.3 0.4];  % assume know parameter
% Ploss_true = ptrue(1).*T.*w + ptrue(2).*T.^2.*w.^2 + ptrue(3).*T.*w.^2 + ptrue(4).*T.^2.*w; % Generate Ploss
% Ploss_meas = Ploss_true + randn(100,1); % Add gaussian noise
% 
% %% perford data fitting
% PHI = [T.*w, T.^2.*w.^2,T.*w.^2, T.^2.*w]; % chose model fitting
% theta_est = pinv(PHI)*Ploss_meas;           % solve Least squares
% Ploss_est = PHI*theta_est;                  % get Ploss usinng estimated parameters
% 
% disp(theta_est-ptrue')
% 
% figure;hold on;
% plot(Ploss_true,'bo');
% plot(Ploss_meas,'rx');
% plot(Ploss_est,'g');
% figure; title('residual');
% plot(Ploss_meas - Ploss_est)

%% ========================================================================

data.Speed  = [60 120    300    450    600    750];
data.Torque = [220 440 1100    2200 3300    4400];
data.map = [...
   86        86        85        82        82        83        ;...
   90        92        91        91        90        90        ;...
   90        94        95        95        95        95        ;...
   85        92        95        96        96        97        ;...
   80        89        95        96        96        97        ;...
   75        86        93        95        96        97        ]/100;
 
n = reshape(data.map,numel(data.map),1);
T = repmat(data.Torque,1,size(data.map,2))';
w = reshape(repmat(data.Speed,size(data.map,1),1),numel(data.map),1);

% Ploss = (1-n).*T.*w;
% PHI = [T , w, T.*w, T.^2.*w.^2];
% theta = pinv(PHI)*Ploss

PHI = [ones(length(T),1), -T./(T.*w) , -w./(T.*w), -T.*w./(T.*w), (-T.^2.*w.^2)./(T.*w)]
theta = pinv(PHI)*n

cond(PHI)

n_est = PHI*theta;

figure;
subplot(2,1,1);
plot(n    ,'go');hold on;grid on;
plot(n_est,'r');hold on;grid on;
subplot(2,1,2);
plot(n-n_est,'b');hold on;grid on;
plot(mean(n-n_est).*ones(length(T),1),'r');
plot( std(n-n_est).*ones(length(T),1),'g-.');
plot(-std(n-n_est).*ones(length(T),1),'g-.');
legend('residual',['\mu =',num2str(mean(n-n_est))],['\sigma =',num2str(std(n-n_est))])





