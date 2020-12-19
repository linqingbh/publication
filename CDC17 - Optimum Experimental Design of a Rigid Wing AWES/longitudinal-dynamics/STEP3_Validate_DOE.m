clc;clear all;close all;

global N ts fs nx np time
global Vmax alpha_max theta_max theta_min q_max q_min de_max
global theta_maxTol theta_minTol q_maxTol q_minTol

cd DOE
% Inaccuracy = 0.05; % 20% w.r.t. true parameters
% load('DOE_optimization_v20ms_5%.mat')
Inaccuracy = 0.2; % 20% w.r.t. true parameters
load('DOE_optimization_v20ms_20%.mat')
% Inaccuracy = 1.0; % 100% w.r.t. true parameters
% load('DOE_optimization_v20ms_100%.mat')
cd ..

%% resample time and input -------------------------------------------------
figure;
plot(time(1:end-1),u_opt,'rx');hold on;grid on;
resample  = 10;
u_DOE_opt = restoresSampling(u_opt,resample); % DOE initial input
N    = length(u_DOE_opt);
time = linspace(0,time(end),N)';
plot(time,u_DOE_opt)
  
cd Data
load('FlightEnvelope.mat')
%load('LongitudinalDynamic_AP2_20ms.mat') % nominal flight
%load('LongitudinalDynamic_AP2_14ms.mat') % max AoA
load('LongitudinalDynamic_AP2_28ms.mat') % min AoA
cd ..

%% SETTINGS ===============================================================
ts   = 0.01;                          % sample time [s]
fs   = 1/ts;                          % Sampling frequency [hz]

%% add noise ==============================================================
% real IMU: noise expressed in standard deviation
sigma_V     = 0.4;%sqrt(2*n_qbar/rho);
sigma_a     = deg2rad(1);  %deg2rad(0.5);   
sigma_theta = deg2rad(2);  %deg2rad(0.1);
sigma_q     = deg2rad(1);%deg2rad(0.1); 
Sigma_ny    = [sigma_V,sigma_a,sigma_theta,sigma_q]'; % Weighted Data fitting

%% Load Flight Envelope ===================================================
Vmax         = 3;                                % [m/s]
alpha_max    = deg2rad(4);                       % [rad]
theta_max    = FlightEnvelope.pitchAngleMaximum; % [rad]
theta_min    = FlightEnvelope.pitchAngleMinimum; % [rad]
theta_maxTol = theta_max - 0.1*theta_max;        % [rad]
theta_minTol = theta_min - 0.1*theta_min;        % [rad]
q_max        =  FlightEnvelope.pitchRateMaximum; % [rad/s]
q_min        = -FlightEnvelope.pitchRateMaximum; % [rad/s]
q_maxTol     = q_max - 0.1*q_max;                % [rad/s]
q_minTol     = q_min - 0.1*q_min;                % [rad/s]
de_max       = 5;                                % deg

%% Longitudinal Dynamic ===================================================
B       = B(:,1); % discard thrust component
[nx,nu] = size(B);
np      = 12;     % n parameters

% Model parameters [true]
a11 = A(1,1);   
a12 = A(1,2);    
a14 = A(1,4);
a21 = A(2,1);  
a22 = A(2,2);  
a24 = A(2,4);
a41 = A(4,1);  
a42 = A(4,2); 
a44 = A(4,4);

b1 = B(1);   
b2 = B(2);   
b4 = B(4);    
  
p_truth = [a11;a12;a14;a21;a22;a24;a41;a42;a44;b1;b2;b4]; % True parameters
p_guess = p_truth + Inaccuracy*p_truth; % Inject inaccuracy to the priori model

checkStability(p_truth)
checkStability(p_guess)

% Model generation via casADi
[ode,states,controls,theta] = symbolic_LongDyn_AP2_dde_input;

%% build integrator: RK4 ==================================================
[RK4_sim1,RK4_simN,Jac_RK4_1] = Runge_Kutta4(ode,states,controls,theta);
 
%% build signal excitacion 3-2-1-1 ========================================
dde_bound     = 3.25;              % FCC [rad/s]
x0            = zeros(nx+1,1);     % Initial Condition 

Udata  = u_DOE_opt';                      % 
X_sim  = RK4_simN(x0, Udata, repmat(p_guess,1,N));
X_sim  = full(X_sim);
X_sim  = [x0,X_sim]; % add the initial condition
X_sim  = X_sim(:,1:end-1);
de_opt = full(X_sim(5,:));
X_sim(5,:) = [];

ny    = [sigma_V*randn(N,1),sigma_a*randn(N,1),sigma_theta*randn(N,1),sigma_q*randn(N,1)];
y_sim = X_sim' + ny;

PlotDataFitting(y_sim,X_sim,X_sim')

%% Plot with true parameters
Udata       = u_DOE_opt';                    % 
X_sim       = RK4_simN(x0, Udata, repmat(p_truth,1,N));
X_sim       = full(X_sim);
X_sim       = [x0,X_sim]; % add the initial condition
X_sim       = X_sim(:,1:end-1);
X_sim(5,:)  = [];
y_sim       = X_sim' + ny;

PlotDataFitting(y_sim,X_sim,X_sim')



