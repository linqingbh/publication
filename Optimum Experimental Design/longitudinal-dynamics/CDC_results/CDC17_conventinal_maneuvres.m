%clc;clear all;close all;
global N ts fs nx np
cd ..    
cd DOE
load('p_truth.mat')

load('DOE_initialization_v20ms_20%.mat')
load('DOE_optimization_v20ms_20%.mat')
cd ..

%% SETTINGS ===============================================================
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

%% Model generation via casADi
[ode,states,controls,theta] = symbolic_LongDyn_AP2;

% build integrator: RK4
[RK4_sim1,RK4_simN,Jac_RK4_1] = Runge_Kutta4(ode,states,controls,theta);
x0           = DM.zeros(nx,1);    % Initial Condition 

%% perform forward simulation =============================== Initial Input
Udata_init   = de_init;           

Xsim_prioriModel = RK4_simN(x0, Udata_init, repmat(p_guess,1,N));
Xsim_prioriModel = full(Xsim_prioriModel);
Xsim_prioriModel = [x0,Xsim_prioriModel]; % add the initial condition
Xsim_prioriModel = Xsim_prioriModel(:,1:end-1);

Xsim_trueModel = RK4_simN(x0, Udata_init, repmat(p_truth,1,N));
Xsim_trueModel = full(Xsim_trueModel);
Xsim_trueModel = [x0,Xsim_trueModel]; % add the initial condition
Xsim_trueModel = Xsim_trueModel(:,1:end-1);

figure;
subplot(4,1,1);hold on;grid on;
plot(time, full(Xsim_prioriModel(1,:)) ,'LineWidth',2,'Color','b');
plot(time, full(Xsim_trueModel(1,:)),'LineWidth',2,'Color','g');
plot(time, Vmax.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time,-Vmax.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
ylabel('V [m/s]');
ylim([-Vmax-0.5,Vmax+0.5])

subplot(4,1,2);hold on;grid on;
plot(time,rad2deg(full(Xsim_prioriModel(2,:))) ,'LineWidth',2,'Color','b');
plot(time,rad2deg(full(Xsim_trueModel(2,:))),'LineWidth',2,'Color','g');
plot(time,rad2deg( alpha_max).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time,rad2deg(-alpha_max).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
xlabel('time [s]');ylabel(' \alpha [deg]');
ylim([rad2deg(-alpha_max-0.1), rad2deg(alpha_max+0.1)])

subplot(4,1,3);hold on;grid on;
plot(time, rad2deg(full(Xsim_prioriModel(3,:))),'LineWidth',2,'Color','b');
plot(time, rad2deg(full(Xsim_trueModel(3,:)))  ,'LineWidth',2,'Color','g');
plot(time, rad2deg(theta_max).*ones(N,1)   ,'LineWidth',1.25,'Color','r','LineStyle','-.');
plot(time, rad2deg(theta_maxTol).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time, rad2deg(theta_min).*ones(N,1)   ,'LineWidth',1.25,'Color','r','LineStyle','-.');
plot(time, rad2deg(theta_minTol).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
ylabel('\phi [deg]');
ylim([rad2deg(theta_min-0.1), rad2deg(theta_max+0.1)])

subplot(4,1,4);hold on;grid on;
plot(time, rad2deg(full(Xsim_prioriModel(4,:))),'LineWidth',2,'Color','b');
plot(time, rad2deg(full(Xsim_trueModel(4,:))),'LineWidth',2,'Color','g');
plot(time,rad2deg(q_max).*ones(N,1)   ,'LineWidth',1.25,'Color','r','LineStyle','-.');
plot(time,rad2deg(q_maxTol).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time,rad2deg(q_min).*ones(N,1)   ,'LineWidth',1.25,'Color','r','LineStyle','-.');
plot(time,rad2deg(q_minTol).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
xlabel('time [s]');ylabel('q [deg/s]');
legend('Priori Model Response','True Model Response','Flight Envelope','Constrains');
ylim([rad2deg(q_min-0.1), rad2deg(q_max+0.1)])

%% perform forward simulation ============================== Optimized Input
Udata_opt   = de_opt;          % 

Xsim_prioriModel = RK4_simN(x0, Udata_opt, repmat(p_guess,1,N));
Xsim_prioriModel = full(Xsim_prioriModel);
Xsim_prioriModel = [x0,Xsim_prioriModel]; % add the initial condition
Xsim_prioriModel = Xsim_prioriModel(:,1:end-1);

Xsim_trueModel = RK4_simN(x0, Udata_opt, repmat(p_truth,1,N));
Xsim_trueModel = full(Xsim_trueModel);
Xsim_trueModel = [x0,Xsim_trueModel]; % add the initial condition
Xsim_trueModel = Xsim_trueModel(:,1:end-1);

figure;
subplot(4,1,1);hold on;grid on;
plot(time, full(Xsim_prioriModel(1,:)) ,'LineWidth',2,'Color','b');
plot(time, full(Xsim_trueModel(1,:)),'LineWidth',2,'Color','g');
plot(time, Vmax.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time,-Vmax.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
ylabel('V [m/s]');
ylim([-Vmax-0.5,Vmax+0.5])

subplot(4,1,2);hold on;grid on;
plot(time,rad2deg(full(Xsim_prioriModel(2,:))) ,'LineWidth',2,'Color','b');
plot(time,rad2deg(full(Xsim_trueModel(2,:))),'LineWidth',2,'Color','g');
plot(time,rad2deg( alpha_max).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time,rad2deg(-alpha_max).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
xlabel('time [s]');ylabel(' \alpha [deg]');
ylim([rad2deg(-alpha_max-0.1), rad2deg(alpha_max+0.1)])

subplot(4,1,3);hold on;grid on;
plot(time, rad2deg(full(Xsim_prioriModel(3,:))),'LineWidth',2,'Color','b');
plot(time, rad2deg(full(Xsim_trueModel(3,:)))  ,'LineWidth',2,'Color','g');
plot(time, rad2deg(theta_max).*ones(N,1)   ,'LineWidth',1.25,'Color','r','LineStyle','-.');
plot(time, rad2deg(theta_maxTol).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time, rad2deg(theta_min).*ones(N,1)   ,'LineWidth',1.25,'Color','r','LineStyle','-.');
plot(time, rad2deg(theta_minTol).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
ylabel('\phi [deg]');
ylim([rad2deg(theta_min-0.1), rad2deg(theta_max+0.1)])

subplot(4,1,4);hold on;grid on;
plot(time, rad2deg(full(Xsim_prioriModel(4,:))),'LineWidth',2,'Color','b');
plot(time, rad2deg(full(Xsim_trueModel(4,:))),'LineWidth',2,'Color','g');
plot(time,rad2deg(q_max).*ones(N,1)   ,'LineWidth',1.25,'Color','r','LineStyle','-.');
plot(time,rad2deg(q_maxTol).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time,rad2deg(q_min).*ones(N,1)   ,'LineWidth',1.25,'Color','r','LineStyle','-.');
plot(time,rad2deg(q_minTol).*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
xlabel('time [s]');ylabel('q [deg/s]');
legend('Priori Model Response','True Model Response','Flight Envelope','Constrains');
ylim([rad2deg(q_min-0.1), rad2deg(q_max+0.1)])

%% Plot input
dde_bound     = 3.25;              % FCC [rad/s]
tau           = 0.02;
ServoSys      = tf( 1 ,[tau 1]);   % ServoDynamics 1^st order system
tauD          = 0.002;              % constant time for derivative
derivativeSys = tf([1 0],[tauD 1]);

dde_init     = lsim(derivativeSys,de_init,time);  % 3-2-1-1 deflection
dde_opt     = lsim(derivativeSys,de_opt,time);  % 3-2-1-1 deflection

figure;hold on;grid on;
subplot(2,1,1);hold on;grid on;
plot(time , rad2deg(de_init),'LineWidth',2,'Color','b');
plot(time , de_max.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time ,-de_max.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
ylabel('[deg]');legend('de','Constrains');
ylim([-de_max-0.5, de_max+0.5]);
subplot(2,1,2);hold on;grid on;
plot(time , dde_init ,'LineWidth',2,'Color','b');
plot(time ,  dde_bound.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time , -dde_bound.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
ylabel('[rad/s]');legend('dde','Constrains');
xlabel('time [s]');

figure;hold on;grid on;
subplot(2,1,1);hold on;grid on;
plot(time , rad2deg(de_opt),'LineWidth',2,'Color','b');
plot(time , de_max.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time ,-de_max.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
ylabel('[deg]');legend('de','Constrains');
ylim([-de_max-0.5, de_max+0.5]);
subplot(2,1,2);hold on;grid on;
plot(time , dde_opt ,'LineWidth',2,'Color','b');
plot(time ,  dde_bound.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
plot(time , -dde_bound.*ones(N,1),'LineWidth',1.25,'Color','r','LineStyle',':');
ylabel('[rad/s]');legend('dde','Constrains');
xlabel('time [s]');

%%
cd CDC_results