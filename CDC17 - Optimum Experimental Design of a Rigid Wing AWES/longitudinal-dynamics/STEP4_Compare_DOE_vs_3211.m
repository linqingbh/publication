%% Sys ID Longitudinal Dynamics AP2 Vtrim = 20 m/s
clc;clear all;close all;
global N ts fs nx np time
global Vmax alpha_max theta_max theta_min q_max q_min de_max
global theta_maxTol theta_minTol q_maxTol q_minTol
     
cd DOE
load('p_truth.mat')
% load('DOE_initialization_v20ms_5%.mat')
% load('DOE_optimization_v20ms_5%.mat')
% load('DOE_initialization_v20ms_20%.mat')
% load('DOE_optimization_v20ms_20%.mat')
load('DOE_initialization_v20ms_100%.mat')
load('DOE_optimization_v20ms_100%.mat')
cd ..

%% SETTINGS ===============================================================
%Option = 'Load_Results';
Option = 'Get_Results';
number_of_experiments = 1000;       % chose the number of experiments

N    = 1000;                        % Number of samples
ts   = 0.01;                        % sample time [s]
fs   = 1/ts;                        % Sampling frequency [hz]
nx   = 4;
np   = 12;                          % n parameters
time = linspace(0,(N-1)*(1/fs),N);  % time array

a11 = p_truth(1);   
a12 = p_truth(2);    
a14 = p_truth(3);
a21 = p_truth(4);  
a22 = p_truth(5);  
a24 = p_truth(6);
a41 = p_truth(7);  
a42 = p_truth(8); 
a44 = p_truth(9);

b1 = p_truth(10);   
b2 = p_truth(11);   
b4 = p_truth(12);  

import casadi.*

%% Load Flight Envelope ===================================================
cd Data
load('FlightEnvelope.mat')
cd ..

Vmax         = 3;
alpha_max    = deg2rad(4);
theta_max    = FlightEnvelope.pitchAngleMaximum; 
theta_min    = FlightEnvelope.pitchAngleMinimum; 
theta_maxTol = theta_max - 0.1*theta_max; 
theta_minTol = theta_min - 0.1*theta_min; 
q_max        =  FlightEnvelope.pitchRateMaximum; 
q_min        = -FlightEnvelope.pitchRateMaximum; 
q_maxTol     = q_max - 0.1*q_max; 
q_minTol     = q_min - 0.1*q_min; 
de_max       = 5;

% Model generation via casADi
[ode,states,controls,theta] = symbolic_LongDyn_AP2;

%% build integrator: RK4 ==================================================
[RK4_sim1,RK4_simN,Jac_RK4_1] = Runge_Kutta4(ode,states,controls,theta);
 
x0           = DM.zeros(nx,1);    % Initial Condition 
Udata_init   = de_init;           % 
X_truth_init = RK4_simN(x0, Udata_init, repmat(p_truth,1,N));
X_truth_init = full(X_truth_init);
X_truth_init = [x0,X_truth_init]; % add the initial condition
X_truth_init = X_truth_init(:,1:end-1);

theta_vec_init     = [];
deviation_vec_init = [];
plotSignalExcitacion(de_init,Udata_init)

%% perform forward simulation ============================== Optimized Input
Udata_opt   = de_opt;          % 
X_truth_opt = RK4_simN(x0, Udata_opt, repmat(p_truth,1,N));
X_truth_opt = full(X_truth_opt);
X_truth_opt = [x0,X_truth_opt]; % add the initial condition
X_truth_opt = X_truth_opt(:,1:end-1);

theta_vec_opt     = [];
deviation_vec_opt = [];
plotSignalExcitacion(de_opt,Udata_opt)

computeFourieTranform(Udata_init,Udata_opt)

sigma_V     =  Sigma_ny(1);
sigma_a     =  Sigma_ny(2);
sigma_theta =  Sigma_ny(3);
sigma_q     =  Sigma_ny(4);

%% data fitting ===========================================================
if strcmp(Option,'Get_Results')
  for i = 1:number_of_experiments
  disp(['Data Fitting Experiment number = ',num2str(i)])
  % Generate noise measurements 
  ny     = [sigma_V*randn(N,1),sigma_a*randn(N,1),sigma_theta*randn(N,1),sigma_q*randn(N,1)];
  
  y_data_init = X_truth_init' + ny;
                   
  disp('data fitting: Initial Signal Excitacion')
  X       = MX.sym('X',nx, N); % continuity 
  Xn_init = RK4_sim1.map({X, Udata_init, theta});
  Xn_init = Xn_init{1};
  gaps    = Xn_init(:,1:end-1) - X(:,2:end); % gap-closing constraints
  e = [];
  for i = 1:N
    e = [e,(1./Sigma_ny).*(y_data_init(i,:)'-X(:,i))]; 
  end
  e = vec(e);

  w      = {theta;vec(X)};
  w0     = [p_guess; vec(y_data_init')];
  lbw    = -inf*ones(size(w0));
  ubw    =  inf*ones(size(w0));
  J      = 0.5*dot(e,e);
  g      = {vec(gaps)};
  lbg    = zeros(nx*(N-1),1);
  ubg    = zeros(nx*(N-1),1);
  
  [theta_est_init,X_est_init] = computeDataFitting(w,w0,lbw,ubw,J,g,lbg,ubg);
  deviationPercentage_init    = abs((p_truth - theta_est_init)./p_truth)*100;
  theta_vec_init              = [theta_vec_init    ,theta_est_init];
  deviation_vec_init          = [deviation_vec_init,deviationPercentage_init];
   
  %% Data Fitting ========================================= Optimized input
  y_data_opt = X_truth_opt' + ny;
                       
  disp('data fitting: Optimized Signal Excitacion')
  X      = MX.sym('X',nx, N); % continuity 
  Xn_Opt = RK4_sim1.map({X, Udata_opt, theta});
  Xn_Opt = Xn_Opt{1};
  gaps   = Xn_Opt(:,1:end-1) - X(:,2:end); % gap-closing constraints

  e = [];
  for i = 1:N
    e = [e,(1./Sigma_ny).*(y_data_opt(i,:)'-X(:,i))]; 
  end
  e = vec(e);

  % multiple shooting casadi ===============================================
  w      = {theta;vec(X)};
  w0     = [p_guess; vec(y_data_opt')];
  lbw    = -inf*ones(size(w0));
  ubw    =  inf*ones(size(w0));
  J      = 0.5*dot(e,e);
  g      = {vec(gaps)};
  lbg    = zeros(nx*(N-1),1);
  ubg    = zeros(nx*(N-1),1);
  
  [theta_est_opt,X_est_opt] = computeDataFitting(w,w0,lbw,ubw,J,g,lbg,ubg);
  deviationPercentage_opt   = abs((p_truth - theta_est_opt)./p_truth)*100;
  theta_vec_opt             = [theta_vec_opt    ,theta_est_opt];
  deviation_vec_opt         = [deviation_vec_opt,deviationPercentage_opt];  
  end
  % Plot response 
  PlotDataFitting(y_data_init,X_truth_init,X_est_init)
  PlotDataFitting(y_data_opt ,X_truth_opt ,X_est_opt)
  % save estimation 
  cd Results
  save('Estimation_results.mat','theta_vec_init','theta_vec_opt')
  cd ..
elseif strcmp(Option,'Load_Results')
  cd Results
  load('Estimation_results.mat')
  cd ..
end

%% Compute Cramer Rao Lower Bound -----------------------------------------
Jac_vec_init = [];
Jac_vec_opt  = [];

for i = 1:N
  temp = full(Jac_RK4_1(X_truth_init(:,i) ,Udata_init(i) ,p_guess));
  Jac_vec_init = [Jac_vec_init;temp];
   temp = full(Jac_RK4_1(X_truth_opt(:,i) ,Udata_opt(i) ,p_guess));
  Jac_vec_opt = [Jac_vec_opt;temp];
end


cov_noise = blkdiag(sigma_V*eye(N),sigma_a*eye(N),sigma_theta*eye(N),sigma_q*eye(N));

W = inv(cov_noise);
J_init = diag(Jac_vec_init'*W*Jac_vec_init);
J_opt  = diag(Jac_vec_opt'*W*Jac_vec_opt);

disp('')
disp([J_init,J_opt])



a11_init = theta_vec_init( 1,:);   
a12_init = theta_vec_init( 2,:);    
a14_init = theta_vec_init( 3,:);
a41_init = theta_vec_init( 7,:);  
b1_init  = theta_vec_init(10,:);  
b2_init  = theta_vec_init(11,:);  

covariance_init  = cov(theta_vec_init');
correlation_init = corr(theta_vec_init');

a11_opt = theta_vec_opt( 1,:);   
a12_opt = theta_vec_opt( 2,:);    
a14_opt = theta_vec_opt( 3,:);
a41_opt = theta_vec_opt( 7,:);  
b1_opt  = theta_vec_opt(10,:);  
b2_opt  = theta_vec_opt(11,:);  

covariance_opt  = cov(theta_vec_opt');
correlation_opt = corr(theta_vec_opt');

%% Assess where we have improvment
Eval = diag(covariance_init) - diag(covariance_opt);
disp('')
disp('Improvment')
disp(Eval')

%% Generate Condidence Ellipsoids
plot2sigmaCondidenceEllipsoid(a11,a12,a11_init,a12_init,a11_opt,a12_opt)
plot2sigmaCondidenceEllipsoid(a11,a14,a11_init,a14_init,a11_opt,a14_opt)
plot2sigmaCondidenceEllipsoid(a11,a41,a11_init,a41_init,a11_opt,a41_opt)
plot2sigmaCondidenceEllipsoid(a11,b1 ,a11_init, b1_init,a11_opt, b1_opt)
plot2sigmaCondidenceEllipsoid(a11,b1 ,a11_init, b2_init,a11_opt, b2_opt)
plot2sigmaCondidenceEllipsoid(a11,b1 ,a12_init, b1_init,a12_opt, b1_opt)