clc;clear all;close all;

global np Nm Nexp Exp1 Exp2 Exp3 Exp4

IndexIn  = 1;
IndexOut = 1200; 
load('FC101.3-01 sysID elevator.mat')

Exp1     = getExpreriment(2,systemID_Data,IndexIn,IndexOut,'No'); 
Exp2     = getExpreriment(3,systemID_Data,IndexIn,IndexOut,'No'); 
Exp3     = getExpreriment(4,systemID_Data,IndexIn,IndexOut,'No');

load('FC101.1-01 sysID elevator.mat')
Exp4     = getExpreriment(2,systemID_Data,IndexIn,IndexOut,'No'); 

load('PhysicalProperties.mat')
%% SETTINGS =============================================================
ts    = 0.01;       % sample time
fs    = 1/ts;       % Sampling frequency [hz]
N_RK4 = 4;          % order of Runge-Kutta
dt    = 1/fs/N_RK4; % integration step for ode   

%% Model generation via casADi =============================
import casadi.*
[theta,x,u,ode] = LongitudinalDynamic(PhysicalProperties);
  
%% build integrator: RK4 ================================================
k1 = ode(x          ,u,theta);
k2 = ode(x+dt/2.0*k1,u,theta);
k3 = ode(x+dt/2.0*k2,u,theta);
k4 = ode(x+dt*k3    ,u,theta);
xf = x + dt/6.0*(k1+2*k2+2*k3+k4);
% Create a function that simulates one step propagation in a sample
one_step = Function('one_step',{x, u, theta},{xf});
X = x;
for i=1:N_RK4
  X = one_step(X, u, theta);
end
% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{x, u, theta}, {X});
% speedup trick: expand into scalar operations
one_sample = one_sample.expand();

%% Prepare Experiment for data fitting ====================================
Nexp            = 4;
% Experiement 1  
Exp1.Udata      = [Exp1.p,Exp1.r,Exp1.dde,Exp1.alpha,Exp1.V,Exp1.qbar]';
Exp1.y_data     =  Exp1.q;
% Experiement 2 
Exp2.Udata      = [Exp2.p,Exp2.r,Exp2.dde,Exp2.alpha,Exp2.V,Exp2.qbar]';
Exp2.y_data     =  Exp2.q;
% Experiement 3
Exp3.Udata      = [Exp3.p,Exp3.r,Exp3.dde,Exp3.alpha,Exp3.V,Exp3.qbar]';
Exp3.y_data     =  Exp3.q;
% Experiement 4
Exp4.Udata      = [Exp4.p,Exp4.r,Exp4.dde,Exp4.alpha,Exp4.V,Exp4.qbar]';
Exp4.y_data     =  Exp4.q;

%% Set Identification Algorithm =========================================== 
disp('Multiple shooting: Multiple Experiment')
nx           = 1;             % p = roll rate
np           = length(theta);
scale        = ones(np,1);
theta_guess  = zeros(np,1);

X1 = MX.sym('X1',nx, Exp1.N); % continuity for exp1
X2 = MX.sym('X2',nx, Exp2.N); % continuity for exp2
X3 = MX.sym('X3',nx, Exp3.N); % continuity for exp3
X4 = MX.sym('X4',nx, Exp4.N); % continuity for exp4

theta_scale = theta.*scale;   % params contains my design variable 

Xn1 = one_sample.map({X1, Exp1.Udata, theta_scale});Xn1 = Xn1{1};
Xn2 = one_sample.map({X2, Exp2.Udata, theta_scale});Xn2 = Xn2{1};
Xn3 = one_sample.map({X3, Exp3.Udata, theta_scale});Xn3 = Xn3{1};
Xn4 = one_sample.map({X4, Exp4.Udata, theta_scale});Xn4 = Xn4{1};

% gap-closing constraints
gaps1 = Xn1(:,1:end-1) - X1(:,2:end);
gaps2 = Xn2(:,1:end-1) - X2(:,2:end);
gaps3 = Xn3(:,1:end-1) - X3(:,2:end);
gaps4 = Xn4(:,1:end-1) - X4(:,2:end);

% error = y_data - x_symbolic  ============================================ 
e_exp1 = vec(Exp1.y_data'-X1);
e_exp2 = vec(Exp2.y_data'-X2);
e_exp3 = vec(Exp3.y_data'-X3);
e_exp4 = vec(Exp4.y_data'-X4);

% multiple shooting casadi ================================= 4 experiments
w      = {theta;vec(X1);vec(X2);vec(X3);vec(X4)};
w0     = [theta_guess; vec(Exp1.y_data'); 
                       vec(Exp2.y_data');
                       vec(Exp3.y_data');
                       vec(Exp4.y_data')];
                     
lbw    = -inf*ones(size(w0));
ubw    =  inf*ones(size(w0));
J      = 0.5*(e_exp1'*e_exp1)+...
         0.5*(e_exp2'*e_exp2)+...
         0.5*(e_exp3'*e_exp3)+...
         0.5*(e_exp4'*e_exp4);
g      = {vec(gaps1);vec(gaps2);vec(gaps3);vec(gaps4)};
lbg    = zeros(nx*(Exp1.N-1)+nx*(Exp2.N-1)+nx*(Exp3.N-1)+nx*(Exp4.N-1),1);
ubg    = zeros(nx*(Exp1.N-1)+nx*(Exp2.N-1)+nx*(Exp3.N-1)+nx*(Exp4.N-1),1);

%% Create Callback
Nm   = Exp1.N;                        % we assume Nm equal for each meas
pc   = MX.sym('p',0,1);               % no parametric variable
nw   = size(vertcat(w{:}),1);         % number of decision variables
ng   = size(vertcat(g{:}),1);         % number of constraints
npc   = size(pc,1);                   % number of parametric variable
Call = MyCallback('f', nw, ng, npc);

%% Create an NLP solver
prob   = struct('x', vertcat(w{:}), 'p', pc, 'f', J, 'g', vertcat(g{:}));
% option IPOPT
opts                             = struct;
opts.ipopt.linear_solver         = 'ma27';
opts.ipopt.max_iter              = 2;
opts.ipopt.hessian_approximation = 'exact';  %  iteration
opts.iteration_callback          = Call;

% Solve the NLP
figure('units','normalized','outerposition',[0 0 1 1])  
solver  = nlpsol('solver', 'ipopt', prob,opts);
w_sol   = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);

theta_est  = scale.*full(w_sol.x(1:np));  % retrieve parameters
disp('                Estimates                ')
disp('     Cm_0    Cm_alpha   Cm_q    Cm_deltaE')
disp(theta_est')

%% compute estimates covariance matrix
%       Cm_0 + Cm_alpha*alpha + Cm_q*qhat + Cm_deltaE*deltaE
% cref = PhysicalProperties.Value.chord;
% Phi1 = [ones(Exp1.N,1) , Exp1.alpha , (cref.*Exp1.q)./(2.*Exp1.V) , Exp1.dde];
% Phi2 = [ones(Exp2.N,1) , Exp2.alpha , (cref.*Exp2.q)./(2.*Exp2.V) , Exp2.dde];
% Phi3 = [ones(Exp3.N,1) , Exp3.alpha , (cref.*Exp3.q)./(2.*Exp3.V) , Exp3.dde];
% Phi4 = [ones(Exp4.N,1) , Exp4.alpha , (cref.*Exp4.q)./(2.*Exp4.V) , Exp4.dde];
% 
% Phi  = [Phi1;Phi2;Phi3;Phi4];
% sigma2_est      = full(w_sol.f)/(Exp1.N+Exp2.N+Exp3.N+Exp4.N-np);
% covariance_est = sigma2_est.*inv(Phi'*Phi);

Jac_Xn1Fun = Function('Xn1Fun',{X1,theta_scale},{jacobian(Xn1,theta_scale)});
Jac_Xn1Fun.printDimensions();
Jac_Xn2Fun = Function('Xn2Fun',{X2,theta_scale},{jacobian(Xn2,theta_scale)});
Jac_Xn2Fun.printDimensions();
Jac_Xn3Fun = Function('Xn3Fun',{X3,theta_scale},{jacobian(Xn3,theta_scale)});
Jac_Xn3Fun.printDimensions();
Jac_Xn4Fun = Function('Xn4Fun',{X4,theta_scale},{jacobian(Xn4,theta_scale)});
Jac_Xn4Fun.printDimensions();

JacExp1 = full(Jac_Xn1Fun(Exp1.q,theta_est));
JacExp2 = full(Jac_Xn2Fun(Exp2.q,theta_est));
JacExp3 = full(Jac_Xn2Fun(Exp3.q,theta_est));
JacExp4 = full(Jac_Xn2Fun(Exp4.q,theta_est));

sigma_q       = deg2rad(0.1);
covariance_est = sigma_q^2.*inv(JacExp1'*JacExp1 + ...
                                JacExp2'*JacExp2 + ...
                                JacExp3'*JacExp3 + ...
                                JacExp4'*JacExp4);

plotConfidenceEllipsoids(theta_est,covariance_est)

%% fitting
X_est = full(w_sol.x(np+1:end));     % retrieve states
X_est = reshape(X_est,Exp1.N,Nexp);
plotFitting(Exp1,Exp2,Exp3,Exp4,X_est)

%%
%X_est5 = X_est(:,4);
%ExpP   = plotFittingPaper(3,systemID_Data,80720,83630,X_est5,'Yes'); 

%% validation
load('FC101.3-01 sysID elevator.mat')
index        = 5;
IndexIn      = 1;
IndexOut     = 700;

ExpV         = getExpreriment(index,systemID_Data,IndexIn,IndexOut,'No');
ExpV.Udata   = [ExpV.p,ExpV.r,ExpV.dde,ExpV.alpha,ExpV.V,ExpV.qbar]';
all_samples  = one_sample.mapaccum('all_samples', ExpV.N);

ExpV.X_truth  = all_samples(ExpV.q(1), ExpV.Udata, repmat(theta_est,1,ExpV.N));
ExpV.X_truth  = full(ExpV.X_truth);
ExpV.residual = rad2deg(ExpV.q)-rad2deg(ExpV.X_truth');

y     = -6:0.1:6;
mu    = mean(ExpV.residual)
sigma = std(ExpV.residual)
f     = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi));

figure;
subplot(2,1,1);hold on;grid on;
title('Model Validation')
plot(ExpV.time,rad2deg(ExpV.q)      ,'LineWidth',1.5,'Color','b');
plot(ExpV.time,rad2deg(ExpV.X_truth),'LineWidth',1.5,'Color','r');             
ylabel('q [deg/s]');xlabel('time [s]')
subplot(2,1,2);hold on;grid on;title('Residual Distribution');
histogram(ExpV.residual,20,'Normalization','pdf');xlim([-6,6]);
plot(y,f,'LineWidth',1.5);xlabel('\epsilon [deg/s]');

% figure;hold on;grid on;
% plot(ExpV.time,rad2deg(ExpV.dde),'LineWidth',1.5,'Color','b');
% ylabel('\delta_{e} [deg]');xlabel('time [s]');

% Measuring the Goodness of Fit using R-Squared
R2 = 1 - norm(ExpV.q-ExpV.X_truth',2)^2/norm(ExpV.q,2)^2