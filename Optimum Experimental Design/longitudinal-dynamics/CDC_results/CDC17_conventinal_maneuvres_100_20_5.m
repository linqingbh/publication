%% Title: Generate 3-2-1-1 response using conventional inputs
% clc;clear all;close all;
global N ts fs nx np
cd ..
cd DOE
load('p_truth.mat')
cd ..

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

import casadi.*


for i = [100,20,5]
switch i
    case 100
        cd DOE
            load('DOE_initialization_v20ms_100%.mat')
            load('DOE_optimization_v20ms_100%.mat')
        cd ..
        %% SETTINGS =======================================================
        N    = 1000;                        % Number of samples
        ts   = 0.01;                        % sample time [s]
        fs   = 1/ts;                        % Sampling frequency [hz]
        nx   = 4;
        np   = 12;                          % n parameters
        time = linspace(0,(N-1)*(1/fs),N);  % time array

        %% Model generation via casADi
        [ode,states,controls,theta] = symbolic_LongDyn_AP2;

        % build integrator: RK4
        [RK4_sim1,RK4_simN,Jac_RK4_1] = Runge_Kutta4(ode,states,controls,theta);
        x0           = DM.zeros(nx,1);    % Initial Condition 

        %% perform forward simulation =============================== Initial Input
        Res100 = struct;
        Udata_init   = de_init;           

        Xsim_prioriModel = RK4_simN(x0, Udata_init, repmat(p_guess,1,N));
        Xsim_prioriModel = full(Xsim_prioriModel);
        Xsim_prioriModel = [x0,Xsim_prioriModel]; % add the initial condition
        Res100.init.Xsim_prioriModel = Xsim_prioriModel(:,1:end-1);

        Xsim_trueModel = RK4_simN(x0, Udata_init, repmat(p_truth,1,N));
        Xsim_trueModel = full(Xsim_trueModel);
        Xsim_trueModel = [x0,Xsim_trueModel]; % add the initial condition
        Res100.init.Xsim_trueModel = Xsim_trueModel(:,1:end-1);

        %% perform forward simulation ============================== Optimized Input
        Udata_opt   = de_opt;          % 

        Xsim_prioriModel = RK4_simN(x0, Udata_opt, repmat(p_guess,1,N));
        Xsim_prioriModel = full(Xsim_prioriModel);
        Xsim_prioriModel = [x0,Xsim_prioriModel]; % add the initial condition
        Res100.opt.Xsim_prioriModel = Xsim_prioriModel(:,1:end-1);

        Xsim_trueModel = RK4_simN(x0, Udata_opt, repmat(p_truth,1,N));
        Xsim_trueModel = full(Xsim_trueModel);
        Xsim_trueModel = [x0,Xsim_trueModel]; % add the initial condition
        Res100.opt.Xsim_trueModel = Xsim_trueModel(:,1:end-1);
        
        %%
        dde_bound     = 3.25;              % FCC [rad/s]
        tau           = 0.02;
        ServoSys      = tf( 1 ,[tau 1]);   % ServoDynamics 1^st order system
        tauD          = 0.002;              % constant time for derivative
        derivativeSys = tf([1 0],[tauD 1]);
        
        dde_init     = lsim(derivativeSys,de_init,time);  % 3-2-1-1 deflection
        dde_opt      = lsim(derivativeSys,de_opt,time);  % 3-2-1-1 deflection
        
        Res100.de_init  = de_init;
        Res100.dde_init = dde_init;
        Res100.de_opt   = de_opt;
        Res100.dde_opt  = dde_opt;
        
    case 20
        cd DOE
        load('DOE_initialization_v20ms_20%.mat')
        load('DOE_optimization_v20ms_20%.mat')
        cd ..
                %% SETTINGS =======================================================
        N    = 1000;                        % Number of samples
        ts   = 0.01;                        % sample time [s]
        fs   = 1/ts;                        % Sampling frequency [hz]
        nx   = 4;
        np   = 12;                          % n parameters
        time = linspace(0,(N-1)*(1/fs),N);  % time array

        %% Model generation via casADi
        [ode,states,controls,theta] = symbolic_LongDyn_AP2;

        % build integrator: RK4
        [RK4_sim1,RK4_simN,Jac_RK4_1] = Runge_Kutta4(ode,states,controls,theta);
        x0           = DM.zeros(nx,1);    % Initial Condition 

        %% perform forward simulation =============================== Initial Input
        Res20 = struct;
        Udata_init   = de_init;           

        Xsim_prioriModel = RK4_simN(x0, Udata_init, repmat(p_guess,1,N));
        Xsim_prioriModel = full(Xsim_prioriModel);
        Xsim_prioriModel = [x0,Xsim_prioriModel]; % add the initial condition
        Res20.init.Xsim_prioriModel = Xsim_prioriModel(:,1:end-1);

        Xsim_trueModel = RK4_simN(x0, Udata_init, repmat(p_truth,1,N));
        Xsim_trueModel = full(Xsim_trueModel);
        Xsim_trueModel = [x0,Xsim_trueModel]; % add the initial condition
        Res20.init.Xsim_trueModel = Xsim_trueModel(:,1:end-1);

        %% perform forward simulation ============================== Optimized Input
        Udata_opt   = de_opt;          % 

        Xsim_prioriModel = RK4_simN(x0, Udata_opt, repmat(p_guess,1,N));
        Xsim_prioriModel = full(Xsim_prioriModel);
        Xsim_prioriModel = [x0,Xsim_prioriModel]; % add the initial condition
        Res20.opt.Xsim_prioriModel = Xsim_prioriModel(:,1:end-1);

        Xsim_trueModel = RK4_simN(x0, Udata_opt, repmat(p_truth,1,N));
        Xsim_trueModel = full(Xsim_trueModel);
        Xsim_trueModel = [x0,Xsim_trueModel]; % add the initial condition
        Res20.opt.Xsim_trueModel = Xsim_trueModel(:,1:end-1);
        
        %%
        dde_bound     = 3.25;              % FCC [rad/s]
        tau           = 0.02;
        ServoSys      = tf( 1 ,[tau 1]);   % ServoDynamics 1^st order system
        tauD          = 0.002;              % constant time for derivative
        derivativeSys = tf([1 0],[tauD 1]);
        
        dde_init     = lsim(derivativeSys,de_init,time);  % 3-2-1-1 deflection
        dde_opt      = lsim(derivativeSys,de_opt,time);  % 3-2-1-1 deflection
        
        Res20.de_init  = de_init;
        Res20.dde_init = dde_init;
        Res20.de_opt   = de_opt;
        Res20.dde_opt  = dde_opt;

    case 5
        cd DOE
        load('DOE_initialization_v20ms_5%.mat')
        load('DOE_optimization_v20ms_5%.mat')
        cd ..
                        %% SETTINGS =======================================================
        N    = 1000;                        % Number of samples
        ts   = 0.01;                        % sample time [s]
        fs   = 1/ts;                        % Sampling frequency [hz]
        nx   = 4;
        np   = 12;                          % n parameters
        time = linspace(0,(N-1)*(1/fs),N);  % time array

        %% Model generation via casADi
        [ode,states,controls,theta] = symbolic_LongDyn_AP2;

        % build integrator: RK4
        [RK4_sim1,RK4_simN,Jac_RK4_1] = Runge_Kutta4(ode,states,controls,theta);
        x0           = DM.zeros(nx,1);    % Initial Condition 

        %% perform forward simulation =============================== Initial Input
        Res5 = struct;
        Udata_init   = de_init;           

        Xsim_prioriModel = RK4_simN(x0, Udata_init, repmat(p_guess,1,N));
        Xsim_prioriModel = full(Xsim_prioriModel);
        Xsim_prioriModel = [x0,Xsim_prioriModel]; % add the initial condition
        Res5.init.Xsim_prioriModel = Xsim_prioriModel(:,1:end-1);

        Xsim_trueModel = RK4_simN(x0, Udata_init, repmat(p_truth,1,N));
        Xsim_trueModel = full(Xsim_trueModel);
        Xsim_trueModel = [x0,Xsim_trueModel]; % add the initial condition
        Res5.init.Xsim_trueModel = Xsim_trueModel(:,1:end-1);

        %% perform forward simulation ============================== Optimized Input
        Udata_opt   = de_opt;          % 

        Xsim_prioriModel = RK4_simN(x0, Udata_opt, repmat(p_guess,1,N));
        Xsim_prioriModel = full(Xsim_prioriModel);
        Xsim_prioriModel = [x0,Xsim_prioriModel]; % add the initial condition
        Res5.opt.Xsim_prioriModel = Xsim_prioriModel(:,1:end-1);

        Xsim_trueModel = RK4_simN(x0, Udata_opt, repmat(p_truth,1,N));
        Xsim_trueModel = full(Xsim_trueModel);
        Xsim_trueModel = [x0,Xsim_trueModel]; % add the initial condition
        Res5.opt.Xsim_trueModel = Xsim_trueModel(:,1:end-1);
        
        %%
        dde_bound     = 3.25;              % FCC [rad/s]
        tau           = 0.02;
        ServoSys      = tf( 1 ,[tau 1]);   % ServoDynamics 1^st order system
        tauD          = 0.002;              % constant time for derivative
        derivativeSys = tf([1 0],[tauD 1]);
        
        dde_init     = lsim(derivativeSys,de_init,time);  % 3-2-1-1 deflection
        dde_opt      = lsim(derivativeSys,de_opt,time);  % 3-2-1-1 deflection
        
        Res5.de_init  = de_init;
        Res5.dde_init = dde_init;
        Res5.de_opt   = de_opt;
        Res5.dde_opt  = dde_opt;

end

end

%% Initial Input
figure;
subplot(4,1,1);hold on;grid on;
plot(time, Vmax.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time,-Vmax.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
ylabel('V [m/s]');
ylim([-Vmax-1,Vmax+1])
plot(time, full(Res100.init.Xsim_prioriModel(1,:)),'LineWidth',1.2,'Color','b','LineStyle','-.');
plot(time, full(Res100.init.Xsim_trueModel(1,:))  ,'LineWidth',1.2,'Color','b','LineStyle','-');
plot(time, full(Res20.init.Xsim_prioriModel(1,:)) ,'LineWidth',1.2,'Color','g','LineStyle','-.');
plot(time, full(Res20.init.Xsim_trueModel(1,:))   ,'LineWidth',1.2,'Color','g','LineStyle','-');
plot(time, full(Res5.init.Xsim_prioriModel(1,:))  ,'LineWidth',1.2,'Color','r','LineStyle','-.');
plot(time, full(Res5.init.Xsim_trueModel(1,:))    ,'LineWidth',1.2,'Color','r','LineStyle','-');

subplot(4,1,2);hold on;grid on;
plot(time,rad2deg( alpha_max).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time,rad2deg(-alpha_max).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
xlabel('time [s]');ylabel(' \alpha [deg]');
ylim([rad2deg(-alpha_max-0.1), rad2deg(alpha_max+0.1)])
plot(time, rad2deg(full(Res100.init.Xsim_prioriModel(2,:))),'LineWidth',1.2,'Color','b','LineStyle','-.');
plot(time, rad2deg(full(Res100.init.Xsim_trueModel(2,:)))  ,'LineWidth',1.2,'Color','b','LineStyle','-');
plot(time, rad2deg(full(Res20.init.Xsim_prioriModel(2,:))) ,'LineWidth',1.2,'Color','g','LineStyle','-.');
plot(time, rad2deg(full(Res20.init.Xsim_trueModel(2,:)))   ,'LineWidth',1.2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.init.Xsim_prioriModel(2,:)))  ,'LineWidth',1.2,'Color','r','LineStyle','-.');
plot(time, rad2deg(full(Res5.init.Xsim_trueModel(2,:)))    ,'LineWidth',1.2,'Color','r','LineStyle','-');

subplot(4,1,3);hold on;grid on;
plot(time, rad2deg(theta_max).*ones(N,1)   ,'LineWidth',1.25,'Color','k','LineStyle','-.');
plot(time, rad2deg(theta_maxTol).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time, rad2deg(theta_min).*ones(N,1)   ,'LineWidth',1.25,'Color','k','LineStyle','-.');
plot(time, rad2deg(theta_minTol).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
ylabel('\phi [deg]');
ylim([rad2deg(theta_min-0.1), rad2deg(theta_max+0.1)])
plot(time, rad2deg(full(Res100.init.Xsim_prioriModel(3,:))),'LineWidth',1.2,'Color','b','LineStyle','-.');
plot(time, rad2deg(full(Res100.init.Xsim_trueModel(3,:)))  ,'LineWidth',1.2,'Color','b','LineStyle','-');
plot(time, rad2deg(full(Res20.init.Xsim_prioriModel(3,:))) ,'LineWidth',1.2,'Color','g','LineStyle','-.');
plot(time, rad2deg(full(Res20.init.Xsim_trueModel(3,:)))   ,'LineWidth',1.2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.init.Xsim_prioriModel(3,:)))  ,'LineWidth',1.2,'Color','r','LineStyle','-.');
plot(time, rad2deg(full(Res5.init.Xsim_trueModel(3,:)))    ,'LineWidth',1.2,'Color','r','LineStyle','-');

subplot(4,1,4);hold on;grid on;
plot(time,rad2deg(q_max).*ones(N,1)   ,'LineWidth',1.25,'Color','k','LineStyle','-.');
plot(time,rad2deg(q_maxTol).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time,rad2deg(q_min).*ones(N,1)   ,'LineWidth',1.25,'Color','k','LineStyle','-.');
plot(time,rad2deg(q_minTol).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
xlabel('time [s]');ylabel('q [deg/s]');
legend('Priori Model Response','True Model Response','Flight Envelope','Constrains');
ylim([rad2deg(q_min-0.1), rad2deg(q_max+0.1)])
plot(time, rad2deg(full(Res100.init.Xsim_prioriModel(4,:))),'LineWidth',1.2,'Color','b','LineStyle','-.');
plot(time, rad2deg(full(Res100.init.Xsim_trueModel(4,:)))  ,'LineWidth',1.2,'Color','b','LineStyle','-');
plot(time, rad2deg(full(Res20.init.Xsim_prioriModel(4,:))) ,'LineWidth',1.2,'Color','g','LineStyle','-.');
plot(time, rad2deg(full(Res20.init.Xsim_trueModel(4,:)))   ,'LineWidth',1.2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.init.Xsim_prioriModel(4,:)))  ,'LineWidth',1.2,'Color','r','LineStyle','-.');
plot(time, rad2deg(full(Res5.init.Xsim_trueModel(4,:)))    ,'LineWidth',1.2,'Color','r','LineStyle','-');

%% Optimal Input
figure;
subplot(4,1,1);hold on;grid on;
plot(time, Vmax.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time,-Vmax.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
ylabel('V [m/s]');
ylim([-Vmax-1,Vmax+1])
plot(time, full(Res100.opt.Xsim_prioriModel(1,:)),'LineWidth',1.2,'Color','b','LineStyle','-.');
plot(time, full(Res100.opt.Xsim_trueModel(1,:))  ,'LineWidth',1.2,'Color','b','LineStyle','-');
plot(time, full(Res20.opt.Xsim_prioriModel(1,:)) ,'LineWidth',1.2,'Color','g','LineStyle','-.');
plot(time, full(Res20.opt.Xsim_trueModel(1,:))   ,'LineWidth',1.2,'Color','g','LineStyle','-');
plot(time, full(Res5.opt.Xsim_prioriModel(1,:))  ,'LineWidth',1.2,'Color','r','LineStyle','-.');
plot(time, full(Res5.opt.Xsim_trueModel(1,:))    ,'LineWidth',1.2,'Color','r','LineStyle','-');

subplot(4,1,2);hold on;grid on;
plot(time,rad2deg( alpha_max).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time,rad2deg(-alpha_max).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
xlabel('time [s]');ylabel(' \alpha [deg]');
ylim([rad2deg(-alpha_max-0.1), rad2deg(alpha_max+0.1)])
plot(time, rad2deg(full(Res100.opt.Xsim_prioriModel(2,:))),'LineWidth',1.2,'Color','b','LineStyle','-.');
plot(time, rad2deg(full(Res100.opt.Xsim_trueModel(2,:)))  ,'LineWidth',1.2,'Color','b','LineStyle','-');
plot(time, rad2deg(full(Res20.opt.Xsim_prioriModel(2,:))) ,'LineWidth',1.2,'Color','g','LineStyle','-.');
plot(time, rad2deg(full(Res20.opt.Xsim_trueModel(2,:)))   ,'LineWidth',1.2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.opt.Xsim_prioriModel(2,:)))  ,'LineWidth',1.2,'Color','r','LineStyle','-.');
plot(time, rad2deg(full(Res5.opt.Xsim_trueModel(2,:)))    ,'LineWidth',1.2,'Color','r','LineStyle','-');

subplot(4,1,3);hold on;grid on;
plot(time, rad2deg(theta_max).*ones(N,1)   ,'LineWidth',1.25,'Color','k','LineStyle','-.');
plot(time, rad2deg(theta_maxTol).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time, rad2deg(theta_min).*ones(N,1)   ,'LineWidth',1.25,'Color','k','LineStyle','-.');
plot(time, rad2deg(theta_minTol).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
ylabel('\phi [deg]');
ylim([rad2deg(theta_min-0.1), rad2deg(theta_max+0.1)])
plot(time, rad2deg(full(Res100.opt.Xsim_prioriModel(3,:))),'LineWidth',1.2,'Color','b','LineStyle','-.');
plot(time, rad2deg(full(Res100.opt.Xsim_trueModel(3,:)))  ,'LineWidth',1.2,'Color','b','LineStyle','-');
plot(time, rad2deg(full(Res20.opt.Xsim_prioriModel(3,:))) ,'LineWidth',1.2,'Color','g','LineStyle','-.');
plot(time, rad2deg(full(Res20.opt.Xsim_trueModel(3,:)))   ,'LineWidth',1.2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.opt.Xsim_prioriModel(3,:)))  ,'LineWidth',1.2,'Color','r','LineStyle','-.');
plot(time, rad2deg(full(Res5.opt.Xsim_trueModel(3,:)))    ,'LineWidth',1.2,'Color','r','LineStyle','-');


subplot(4,1,4);hold on;grid on;
plot(time,rad2deg(q_max).*ones(N,1)   ,'LineWidth',1.25,'Color','k','LineStyle','-.');
plot(time,rad2deg(q_maxTol).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time,rad2deg(q_min).*ones(N,1)   ,'LineWidth',1.25,'Color','k','LineStyle','-.');
plot(time,rad2deg(q_minTol).*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
xlabel('time [s]');ylabel('q [deg/s]');
legend('Priori Model Response','True Model Response','Flight Envelope','Constrains');
ylim([rad2deg(q_min-0.1), rad2deg(q_max+0.1)])
plot(time, rad2deg(full(Res100.opt.Xsim_prioriModel(4,:))),'LineWidth',1.2,'Color','b','LineStyle','-.');
plot(time, rad2deg(full(Res100.opt.Xsim_trueModel(4,:)))  ,'LineWidth',1.2,'Color','b','LineStyle','-');
plot(time, rad2deg(full(Res20.opt.Xsim_prioriModel(4,:))) ,'LineWidth',1.2,'Color','g','LineStyle','-.');
plot(time, rad2deg(full(Res20.opt.Xsim_trueModel(4,:)))   ,'LineWidth',1.2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.opt.Xsim_prioriModel(4,:)))  ,'LineWidth',1.2,'Color','r','LineStyle','-.');
plot(time, rad2deg(full(Res5.opt.Xsim_trueModel(4,:)))    ,'LineWidth',1.2,'Color','r','LineStyle','-');

%% Input
figure;hold on;grid on;
subplot(2,1,1);hold on;grid on;
plot(time , rad2deg(Res100.de_init),'LineWidth',1,'Color','b');
plot(time , rad2deg(Res20.de_init),'LineWidth',1,'Color','g');
plot(time , rad2deg(Res5.de_init),'LineWidth',1,'Color','r');

plot(time , de_max.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time ,-de_max.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
ylabel('[deg]');legend('de','Constrains');
ylim([-de_max-0.5, de_max+0.5]);

subplot(2,1,2);hold on;grid on;
plot(time , Res100.dde_init ,'LineWidth',1,'Color','b');
plot(time , Res20.dde_init ,'LineWidth',1,'Color','g');
plot(time , Res5.dde_init ,'LineWidth',1,'Color','r');
plot(time ,  dde_bound.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time , -dde_bound.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
ylabel('[rad/s]');legend('dde','Constrains');
xlabel('time [s]');

figure;hold on;grid on;
subplot(4,1,1);hold on;grid on;
plot(time , rad2deg(Res100.de_opt),'LineWidth',1,'Color','b');
plot(time , de_max.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time ,-de_max.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
ylim([-de_max-0.5, de_max+0.5]);
subplot(4,1,2);hold on;grid on;
plot(time , rad2deg(Res20.de_opt),'LineWidth',1,'Color','g');
plot(time , de_max.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time ,-de_max.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
ylim([-de_max-0.5, de_max+0.5]);
subplot(4,1,3);hold on;grid on;
plot(time , rad2deg(Res5.de_opt),'LineWidth',1,'Color','r');
plot(time , de_max.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time ,-de_max.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
ylabel('[deg]');legend('de','Constrains');
ylim([-de_max-0.5, de_max+0.5]);
subplot(4,1,4);hold on;grid on;
plot(time , Res100.dde_opt ,'LineWidth',1,'Color','b');
plot(time , Res20.dde_opt ,'LineWidth',1,'Color','g');
plot(time , Res5.dde_opt ,'LineWidth',1,'Color','r');
plot(time ,  dde_bound.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
plot(time , -dde_bound.*ones(N,1),'LineWidth',1.25,'Color','k','LineStyle',':');
ylabel('[rad/s]');legend('dde','Constrains');
xlabel('time [s]');

%%
cd CDC_results

%% PhD thesis ============================================================= 
%% Initial Input
figure;
subplot(5,1,1);hold on;grid on;
plot(time, Vmax.*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
plot(time,-Vmax.*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
ylabel('V [m/s]');
ylim([-Vmax-1,Vmax+1])
plot(time, full(Res100.init.Xsim_prioriModel(1,:)),'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle',':');
plot(time, full(Res100.init.Xsim_trueModel(1,:))  ,'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle','-');
plot(time, full(Res20.init.Xsim_prioriModel(1,:)) ,'LineWidth',2,'Color','g'          ,'LineStyle',':');
plot(time, full(Res20.init.Xsim_trueModel(1,:))   ,'LineWidth',2,'Color','g'          ,'LineStyle','-');
plot(time, full(Res5.init.Xsim_prioriModel(1,:))  ,'LineWidth',2,'Color','r'          ,'LineStyle',':');
plot(time, full(Res5.init.Xsim_trueModel(1,:))    ,'LineWidth',2,'Color','r'          ,'LineStyle','-');

subplot(5,1,2);hold on;grid on;
plot(time,rad2deg( alpha_max).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
plot(time,rad2deg(-alpha_max).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
xlabel('time [s]');ylabel(' \alpha [deg]');
ylim([-5, 5]);
plot(time, rad2deg(full(Res100.init.Xsim_prioriModel(2,:))),'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle',':');
plot(time, rad2deg(full(Res100.init.Xsim_trueModel(2,:)))  ,'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle','-');
plot(time, rad2deg(full(Res20.init.Xsim_prioriModel(2,:))) ,'LineWidth',2,'Color','g','LineStyle',':');
plot(time, rad2deg(full(Res20.init.Xsim_trueModel(2,:)))   ,'LineWidth',2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.init.Xsim_prioriModel(2,:)))  ,'LineWidth',2,'Color','r','LineStyle',':');
plot(time, rad2deg(full(Res5.init.Xsim_trueModel(2,:)))    ,'LineWidth',2,'Color','r','LineStyle','-');

subplot(5,1,3);hold on;grid on;
plot(time, rad2deg(theta_max).*ones(N,1)   ,'LineWidth',1.3,'Color',[0.64,0.08,0.18],'LineStyle','-.');
plot(time, rad2deg(theta_maxTol).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
plot(time, rad2deg(theta_min).*ones(N,1)   ,'LineWidth',1.3,'Color',[0.64,0.08,0.18],'LineStyle','-.');
plot(time, rad2deg(theta_minTol).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
ylabel('\theta [deg]');
ylim([rad2deg(theta_min-0.1), rad2deg(theta_max+0.1)])
plot(time, rad2deg(full(Res100.init.Xsim_prioriModel(3,:))),'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle',':');
plot(time, rad2deg(full(Res100.init.Xsim_trueModel(3,:)))  ,'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle','-');
plot(time, rad2deg(full(Res20.init.Xsim_prioriModel(3,:))) ,'LineWidth',2,'Color','g','LineStyle',':');
plot(time, rad2deg(full(Res20.init.Xsim_trueModel(3,:)))   ,'LineWidth',2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.init.Xsim_prioriModel(3,:)))  ,'LineWidth',2,'Color','r','LineStyle',':');
plot(time, rad2deg(full(Res5.init.Xsim_trueModel(3,:)))    ,'LineWidth',2,'Color','r','LineStyle','-');

subplot(5,1,4);hold on;grid on;
plot(time,rad2deg(q_max).*ones(N,1)   ,'LineWidth',1.3,'Color',[0.64,0.08,0.18],'LineStyle','-.');
plot(time,rad2deg(q_maxTol).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5]   ,'LineStyle','--');
plot(time,rad2deg(q_min).*ones(N,1)   ,'LineWidth',1.3,'Color',[0.64,0.08,0.18],'LineStyle','-.');
plot(time,rad2deg(q_minTol).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5]   ,'LineStyle','--');
xlabel('time [s]');ylabel('q [deg/s]');

ylim([rad2deg(q_min-0.1), rad2deg(q_max+0.1)])
plot(time, rad2deg(full(Res100.init.Xsim_prioriModel(4,:))),'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle',':');
plot(time, rad2deg(full(Res100.init.Xsim_trueModel(4,:)))  ,'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle','-');
plot(time, rad2deg(full(Res20.init.Xsim_prioriModel(4,:))) ,'LineWidth',2,'Color','g','LineStyle',':');
plot(time, rad2deg(full(Res20.init.Xsim_trueModel(4,:)))   ,'LineWidth',2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.init.Xsim_prioriModel(4,:)))  ,'LineWidth',2,'Color','r','LineStyle',':');
plot(time, rad2deg(full(Res5.init.Xsim_trueModel(4,:)))    ,'LineWidth',2,'Color','r','LineStyle','-');

subplot(5,1,5);hold on;grid on;
plot(time , rad2deg(Res100.de_init),'LineWidth',2,'Color',[0,0.45,0.74]);
plot(time , rad2deg(Res20.de_init),'LineWidth',2,'Color','g');
plot(time , rad2deg(Res5.de_init),'LineWidth',2,'Color','r');
plot(time , de_max.*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5] ,'LineStyle','--');
plot(time ,-de_max.*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5] ,'LineStyle','--');
ylabel('[deg]');
ylim([-de_max-0.5, de_max+0.5]);
xlabel('time [s]');

%% Optimized Maneuvers
figure;
subplot(5,1,1);hold on;grid on;
plot(time, Vmax.*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
plot(time,-Vmax.*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
ylabel('V [m/s]');
ylim([-Vmax-1,Vmax+1])
plot(time, full(Res100.opt.Xsim_prioriModel(1,:)),'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle',':');
plot(time, full(Res100.opt.Xsim_trueModel(1,:))  ,'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle','-');
plot(time, full(Res20.opt.Xsim_prioriModel(1,:)) ,'LineWidth',2,'Color','g','LineStyle',':');
plot(time, full(Res20.opt.Xsim_trueModel(1,:))   ,'LineWidth',2,'Color','g','LineStyle','-');
plot(time, full(Res5.opt.Xsim_prioriModel(1,:))  ,'LineWidth',2,'Color','r','LineStyle',':');
plot(time, full(Res5.opt.Xsim_trueModel(1,:))    ,'LineWidth',2,'Color','r','LineStyle','-');

subplot(5,1,2);hold on;grid on;
plot(time,rad2deg( alpha_max).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
plot(time,rad2deg(-alpha_max).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
ylabel(' \alpha [deg]');
ylim([-5, 5])
plot(time, rad2deg(full(Res100.opt.Xsim_prioriModel(2,:))),'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle',':');
plot(time, rad2deg(full(Res100.opt.Xsim_trueModel(2,:)))  ,'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle','-');
plot(time, rad2deg(full(Res20.opt.Xsim_prioriModel(2,:))) ,'LineWidth',2,'Color','g','LineStyle',':');
plot(time, rad2deg(full(Res20.opt.Xsim_trueModel(2,:)))   ,'LineWidth',2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.opt.Xsim_prioriModel(2,:)))  ,'LineWidth',2,'Color','r','LineStyle',':');
plot(time, rad2deg(full(Res5.opt.Xsim_trueModel(2,:)))    ,'LineWidth',2,'Color','r','LineStyle','-');

subplot(5,1,3);hold on;grid on;
plot(time, rad2deg(theta_max).*ones(N,1)   ,'LineWidth',1.3,'Color',[0.64,0.08,0.18],'LineStyle','-.');
plot(time, rad2deg(theta_maxTol).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
plot(time, rad2deg(theta_min).*ones(N,1)   ,'LineWidth',1.3,'Color',[0.64,0.08,0.18],'LineStyle','-.');
plot(time, rad2deg(theta_minTol).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
ylabel('\theta [deg]');
ylim([rad2deg(theta_min-0.1), rad2deg(theta_max+0.1)])
plot(time, rad2deg(full(Res100.opt.Xsim_prioriModel(3,:))),'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle',':');
plot(time, rad2deg(full(Res100.opt.Xsim_trueModel(3,:)))  ,'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle','-');
plot(time, rad2deg(full(Res20.opt.Xsim_prioriModel(3,:))) ,'LineWidth',2,'Color','g','LineStyle',':');
plot(time, rad2deg(full(Res20.opt.Xsim_trueModel(3,:)))   ,'LineWidth',2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.opt.Xsim_prioriModel(3,:)))  ,'LineWidth',2,'Color','r','LineStyle',':');
plot(time, rad2deg(full(Res5.opt.Xsim_trueModel(3,:)))    ,'LineWidth',2,'Color','r','LineStyle','-');

subplot(5,1,4);hold on;grid on;
plot(time,rad2deg(q_max).*ones(N,1)   ,'LineWidth',1.3,'Color',[0.64,0.08,0.18],'LineStyle','-.');
plot(time,rad2deg(q_maxTol).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5]   ,'LineStyle','--');
plot(time,rad2deg(q_min).*ones(N,1)   ,'LineWidth',1.3,'Color',[0.64,0.08,0.18],'LineStyle','-.');
plot(time,rad2deg(q_minTol).*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
ylabel('q [deg/s]');
ylim([rad2deg(q_min-0.1), rad2deg(q_max+0.1)])
plot(time, rad2deg(full(Res100.opt.Xsim_prioriModel(4,:))),'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle',':');
plot(time, rad2deg(full(Res100.opt.Xsim_trueModel(4,:)))  ,'LineWidth',2,'Color',[0,0.45,0.74],'LineStyle','-');
plot(time, rad2deg(full(Res20.opt.Xsim_prioriModel(4,:))) ,'LineWidth',2,'Color','g','LineStyle',':');
plot(time, rad2deg(full(Res20.opt.Xsim_trueModel(4,:)))   ,'LineWidth',2,'Color','g','LineStyle','-');
plot(time, rad2deg(full(Res5.opt.Xsim_prioriModel(4,:)))  ,'LineWidth',2,'Color','r','LineStyle',':');
plot(time, rad2deg(full(Res5.opt.Xsim_trueModel(4,:)))    ,'LineWidth',2,'Color','r','LineStyle','-');

subplot(5,1,5);hold on;grid on;
plot(time , rad2deg(Res100.de_opt),'LineWidth',2);
plot(time , rad2deg(Res20.de_opt),'LineWidth',2,'Color','g');
plot(time , rad2deg(Res5.de_opt),'LineWidth',2,'Color','r');
plot(time , de_max.*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
plot(time ,-de_max.*ones(N,1),'LineWidth',1.5,'Color',[0.5,0.5,0.5],'LineStyle','--');
ylim([-de_max-0.5, de_max+0.5]);ylabel('\delta_{e} [deg]');
xlabel('time [s]');

