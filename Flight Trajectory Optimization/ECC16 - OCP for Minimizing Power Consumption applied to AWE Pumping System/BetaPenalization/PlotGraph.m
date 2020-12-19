clc;clear all;close all;
load crosswind_opt_beta_Penalization
%load crosswind_opt_mechanical_beta
crosswind_opt_mechanical_3_loops

% EXIT: Optimal Solution Found. iterations: 413
% time spent in eval_f: 0.7169 s. (575 calls, 1.24678 ms. average)
% time spent in eval_grad_f: 0.550601 s. (415 calls, 1.32675 ms. average)
% time spent in eval_g: 1.44367 s. (575 calls, 2.51073 ms. average)
% time spent in eval_jac_g: 2.43929 s. (419 calls, 5.8217 ms. average)
% time spent in eval_h: 119.472 s. (415 calls, 287.883 ms. average)
% time spent in main loop: 535.989 s.
% time spent in callback function: 0 s.
% time spent in callback preparation: 0.063012 s.
% optimal cost:  -13890.8808383
% optimizing mechanical
% optimal mechanical power: -13922.5282816
% optimal electrical power: -5881.48625864
% endTime: 22.5


%% Plot angle of attack and sideslip angle
t_ab = crosswind_opt_mechanical_3_loops.alpha_deg.time;

alpha = cleanStruct(crosswind_opt_mechanical_3_loops.alpha_deg.value);
beta  = cleanStruct(crosswind_opt_mechanical_3_loops.beta_deg.value);

alphaMin = -8*ones(length(t_ab)); 
alphaMax = 22*ones(length(t_ab));
betaMin  = -8*ones(length(t_ab)); 
betaMax  = 8*ones(length(t_ab));

figure(1);
subplot(2,1,1);                           
plot(t_ab,alpha,'LineWidth',2);hold on;plot(t_ab,alphaMin,'r--',t_ab,alphaMax,'r--');
axis([0 30 -8.5 23]);legend('angle of attack','bounds');ylabel('[deg]');grid on;
subplot(2,1,2);
plot(t_ab,beta,'LineWidth',2);hold on;plot(t_ab,betaMin,'r--',t_ab,betaMax,'r--');
axis([0 30 -16 20]);legend('side slip angle','bounds');ylabel('[deg]');grid on;
xlabel('time [s]');

%% r = length tether | dr = speed tether | tether tension 

t_r = crosswind_opt_mechanical_3_loops.r.time;
t_t = crosswind_opt_mechanical_3_loops.tether_tension.time;

r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);
tether_tension = cleanStruct(crosswind_opt_mechanical_3_loops.tether_tension.value); 

rMin = 1*ones(length(t_r));
rMax = 1000*ones(length(t_r));
drMin = -9.5*ones(length(t_r));
drMax = +9.5*ones(length(t_r));
t_tension_min = 0*ones(length(t_t));

figure(2);

subplot(3,1,1);                           
plot(t_r,r,'LineWidth',2);legend('tether length');ylabel('[m]');grid on;hold on;
%axis([0 22.5 360 540]);

% [rmax,imax] = max(r);[rmin,imin] = min(r);
% text(9.2,(max(r)+min(r))/2,' \Delta l = 121 [m]');
% text(8,385,'reel-in phase');
% 
% yfit = line([9,9],[rmin,rmax]);
% set(yfit,'LineWidth',1,'Color',[0,0.5,0]);
% yfit2 = line([8.8,9.2],[rmin,rmin]);
% set(yfit2,'LineWidth',1,'Color',[0,0.5,0]);
% yfit3 = line([8.8,9.2],[rmax,rmax]);
% set(yfit3,'LineWidth',1,'Color',[0,0.5,0]);
% 
% xfit = line([5.85,12],[380,380]);
% set(xfit,'LineWidth',1,'Color',[0.5,0,0]);
% xfit2 = line([5.85,5.85],[375,385]);
% set(xfit2,'LineWidth',1,'Color',[0.5,0,0]);
% xfit3 = line([12,12],[375,385]);
% set(xfit3,'LineWidth',1,'Color',[0,0.5,0]);

subplot(3,1,2);
plot(t_r,dr,'LineWidth',2);hold on;plot(t_r,drMin,'r--',t_r,drMax,'r--');
axis([0 30 -32 32]);legend('tether speed','bounds');ylabel('[m/s]');grid on;

subplot(3,1,3);
plot(t_t,tether_tension,'LineWidth',2);hold on;plot(t_t,t_tension_min,'r--');
axis([0 30 -100 3100]);legend('tether tension','bound');ylabel('[N]');grid on;

%% rpm = Revolutions per minute | torque

t_rpm = crosswind_opt_mechanical_3_loops.rpm.time;
t_torque = crosswind_opt_mechanical_3_loops.torque.time; 

rpm = cleanStruct(crosswind_opt_mechanical_3_loops.rpm.value);
torque = cleanStruct(crosswind_opt_mechanical_3_loops.torque.value);

torqueMin = 0*ones(length(t_torque));
torqueMax = 750*ones(length(t_torque));

figure(3);
subplot(2,1,1);                           
plot(t_rpm,rpm,'LineWidth',2);hold on;axis([0 30 -1500 3000]);
legend('Revolutions per minute');ylabel('[rpm]');grid on;

subplot(2,1,2);
plot(t_torque,torque,'LineWidth',2);hold on;plot(t_torque,torqueMin,'r--',t_torque,torqueMax,'r--');
legend('Torque','bounds');xlabel('time [s]');ylabel('[N*m]');grid on;axis([0 30 -10 800]);

%% aileron | rudder | elevator |
t_cs  = crosswind_opt_mechanical_3_loops.aileron.time;
% t_dcs = crosswind_opt_mechanical_3_loops.daileron.time;

aileron  = cleanStruct(crosswind_opt_mechanical_3_loops.aileron.value);
rudder   = cleanStruct(crosswind_opt_mechanical_3_loops.rudder.value);
elevator = cleanStruct(crosswind_opt_mechanical_3_loops.elevator.value);

% daileron  = cleanStruct(crosswind_opt_mechanical_3_loops.daileron.value);
% drudder   = cleanStruct(crosswind_opt_mechanical_3_loops.drudder.value);
% delevator = cleanStruct(crosswind_opt_mechanical_3_loops.delevator.value);


ailMin = -20*ones(length(t_cs));
ailMax = 20*ones(length(t_cs));
rudMin = -30*ones(length(t_cs));
rudMax = 30*ones(length(t_cs));

% dcontrolsurfMin = -2*ones(1,length(t_dcs));
% dcontrolsurfMax = +2*ones(1,length(t_dcs));

figure(4);

subplot(3,1,1);                           
plot(t_cs,rad2deg(aileron),'LineWidth',2);hold on;plot(t_cs,ailMin,'r--',t_cs,ailMax,'r--');
legend('aileron','bounds');ylabel('[deg]');grid on;axis([0 30 -22 22]);

subplot(3,1,2);
plot(t_cs,rad2deg(rudder),'LineWidth',2);hold on;plot(t_cs,rudMin,'r--',t_cs,rudMax,'r--');
legend('rudder','bounds');ylabel('[deg]');grid on;axis([0 30 -32 32]);

subplot(3,1,3);
plot(t_cs,rad2deg(elevator),'LineWidth',2);hold on;plot(t_cs,rudMin,'r--',t_cs,rudMax,'r--');
legend('elevator','bounds');ylabel('[deg]');grid on;axis([0 30 -32 32]);
xlabel('time [s]');

% subplot(3,1,2);                           
% plot(t_dcs,daileron,'LineWidth',2);hold on;plot(t_dcs,dcontrolsurfMin,'r--',t_dcs,dcontrolsurfMin,'r--');
% legend('daileron');ylabel('[deg]');grid on;%axis([0 22.5 -22 22]);
% subplot(2,3,4);
% plot(t_cs,rad2deg(drudder),'LineWidth',2);hold on;plot(t_cs,dcontrolsurfMin,'r--',t_cs,dcontrolsurfMin,'r--');
% legend('drudder');ylabel('[deg]');grid on;%axis([0 22.5 -32 32]);
% subplot(2,3,6);
% plot(t_cs,rad2deg(elevator),'LineWidth',2);hold on;plot(t_cs,dcontrolsurfMin,'r--',t_cs,dcontrolsurfMin,'r--');
% legend('delevator');ylabel('[deg]');grid on;%axis([0 22.5 -32 32]);
% xlabel('time [s]');




%% Loyd Limit
t_Loyd = crosswind_opt_mechanical_3_loops.loyds_limit_exact.time;
t_pow = crosswind_opt_mechanical_3_loops.neg_mechanical_winch_power.time;

Loyd_Exact = cleanStruct(crosswind_opt_mechanical_3_loops.loyds_limit_exact.value);
Loyd_Sim = cleanStruct(crosswind_opt_mechanical_3_loops.loyds_limit.value);
NegWinchPow = cleanStruct(crosswind_opt_mechanical_3_loops.neg_mechanical_winch_power.value);

figure(5);
plot(t_Loyd,Loyd_Exact,'LineWidth',1.5);hold on;
plot(t_Loyd,Loyd_Sim,'LineWidth',1.5);hold on;
area(t_pow,NegWinchPow,'FaceColor','green','LineWidth',1.5);
legend('Loyd s limit exact','Loyd s limit sim','winch power');
ylabel('power [w]');xlabel('time [s]');grid on;
title('optimal mechanical power= 11.141 kW')
axis([0 30 -40000 80000]);%text(15,35000,'Optimal mechanical power = 11.4 KW');

%% Airspeed

t_air = crosswind_opt_mechanical_3_loops.airspeed.time;
airspeed = cleanStruct(crosswind_opt_mechanical_3_loops.airspeed.value);
airMin = 15*ones(length(airspeed));
figure(6);
                        
plot(t_air,airspeed,'LineWidth',2);hold on;plot(t_air,airMin,'r--');
legend('airspeed','bound');ylabel('[m/s]');xlabel('time [s]')
grid on;axis([0 30 14.5 50]);
