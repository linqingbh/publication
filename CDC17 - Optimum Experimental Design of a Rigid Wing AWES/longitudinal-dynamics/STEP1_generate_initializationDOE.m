%% Generate Step Responde for DoE 
% clc;clear all;close all;
global N ts fs nx np time
global Vmax alpha_max theta_max theta_min q_max q_min de_max
global theta_maxTol theta_minTol q_maxTol q_minTol
     
import casadi.*

cd Data
load('FlightEnvelope.mat')
load('LongitudinalDynamic_AP2_20ms.mat')
cd ..
%% SETTINGS ===============================================================
Inaccuracy = 0.05;                    % Inaccuracy to the P_guess
N    = 1000;                          % Number of samples
ts   = 0.01;                          % sample time [s]
fs   = 1/ts;                          % Sampling frequency [hz]
time = linspace(0,(N-1)*(1/fs),N)';   % time array

%% add noise ==============================================================
% real IMU: noise expressed in standard deviation
sigma_V     = 0.4;%sqrt(2*n_qbar/rho);
sigma_a     = deg2rad(1);  %deg2rad(0.5);   
sigma_theta = deg2rad(2);  %deg2rad(0.1);
sigma_q     = deg2rad(1);%deg2rad(0.1); 

Sigma_ny  = [sigma_V,sigma_a,sigma_theta,sigma_q]'; % Weighted Data fitting

%% Load Flight Envelope ===================================================
% angleOfAttackMinimum: -0.1745 [rad] = -10 [deg]
% angleOfAttackMaximum: 0.3491  [rad] = +20 [deg]
%          EAS_Minimum: 12
%          EAS_Maximum: 30
               
Vmax         = 3;                        % [m/s]
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

% Model generation via casADi
[ode,states,controls,theta] = symbolic_LongDyn_AP2_dde_input;

%% build integrator: RK4 ==================================================
[RK4_sim1,RK4_simN,Jac_RK4_1] = Runge_Kutta4(ode,states,controls,theta);
 
%% build signal excitacion 3-2-1-1 ========================================
dde_bound     = 3.25;              % FCC [rad/s]
tau           = 0.02;
ServoSys      = tf( 1 ,[tau 1]);   % ServoDynamics 1^st order system
tauD          = 0.02;              % constant time for derivative
derivativeSys = tf([1 0],[tauD 1]);
x0            = zeros(nx+1,1);     % Initial Condition 

%% perform forward simulation ================================ Initial Input
A_3211       = 5;                                % Amplitude 3-2-1-1 [deg]
Dt_3211      = 0.3;     % pulse width 3-2-1-1 [s] |maxAoA = 0.32|minAoA = 

u_3211       = generate_3211(A_3211,Dt_3211);     % 3-2-1-1 demand
de_3211      = lsim(ServoSys     ,u_3211,time);   % 3-2-1-1 deflection
dde_3211     = lsim(derivativeSys,de_3211,time);  % 3-2-1-1 deflection

%% Generate Data for DOE algorithm ----------------------------------------
rescale      = 0.1;
u_DOE_init   = resampleInput(dde_3211,rescale);          % DOE initial input
t_DOE        = linspace(0,(N-1)*(1/fs)+ts,N*rescale+1)'; % DOE time

Udata_init  = dde_3211';                      % 
X_sim = RK4_simN(x0, Udata_init, repmat(p_guess,1,N));
X_sim = full(X_sim);
X_sim = [x0,X_sim]; % add the initial condition
X_sim = X_sim(:,1:end-1);

de_init = full(X_sim(5,:));

ny    = [sigma_V*randn(N,1),sigma_a*randn(N,1),sigma_theta*randn(N,1),sigma_q*randn(N,1),0.001*randn(N,1)];
y_sim = X_sim' + ny;

figure;hold on;grid on;
subplot(2,1,1);hold on;grid on;
title('3-2-1-1 signal excitacion');
plot(time , rad2deg(u_3211) ,'LineWidth',1,'Color','g','LineStyle','-.');
plot(time , rad2deg(de_3211),'LineWidth',1.5,'Color','b','LineStyle','-' );
plot(time , rad2deg(full(X_sim(5,:))),'LineWidth',1.5,'Color','r','LineStyle','-')
plot(time , de_max.*ones(N,1),'r-.');
plot(time ,-de_max.*ones(N,1),'r-.');
ylabel('[deg]');legend('u_{e}','d_{e}','d_{e} sim');
ylim([-de_max-0.5, de_max+0.5]);
subplot(2,1,2);hold on;grid on;
title('d/dt 3-2-1-1 signal excitacion');
plot(time , dde_3211 ,'LineWidth',1,'Color','b','LineStyle','-.');
plot(t_DOE(1:end-1),u_DOE_init,'rx')
plot(time , dde_bound.*ones(N,1),'r-.');
plot(time ,-dde_bound.*ones(N,1),'r-.');
ylabel('[rad/s]');xlabel('time [s]');legend('dde [rad/s]');

PlotDataFitting(y_sim,X_sim,X_sim')
%% store everything in DOE folder
cd DOE
save('DOE_initialization.mat','t_DOE','u_DOE_init','de_init','x0','p_guess','Sigma_ny')
cd ..

%% Plot with true parameters
Udata_init  = dde_3211';                      % 
X_sim = RK4_simN(x0, Udata_init, repmat(p_truth,1,N));
X_sim = full(X_sim);
X_sim = [x0,X_sim]; % add the initial condition
X_sim = X_sim(:,1:end-1);

ny    = [sigma_V*randn(N,1),sigma_a*randn(N,1),sigma_theta*randn(N,1),sigma_q*randn(N,1),0.001*randn(N,1)];
y_sim = X_sim' + ny;

PlotDataFitting(y_sim,X_sim,X_sim')




