clc;clear all;close all;
load Traj10windTether

%% Time vector
t_r   = crosswind_opt_mechanical_3_loops.r.time;
t_ab  = crosswind_opt_mechanical_3_loops.alpha_deg.time;
t_pow = crosswind_opt_mechanical_3_loops.neg_mechanical_winch_power.time;
t_air = crosswind_opt_mechanical_3_loops.airspeed.time;

%% Quantities
r        = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr       = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);
alpha    = cleanStruct(crosswind_opt_mechanical_3_loops.alpha_deg.value);
airspeed = cleanStruct(crosswind_opt_mechanical_3_loops.airspeed.value);
Pmech    = cleanStruct(crosswind_opt_mechanical_3_loops.neg_mechanical_winch_power.value);

Pelec    = [];
for i = 1:length(Pmech)
    if Pmech > 0
        Pelec = [Pelec Pmech(i)*0.94];
    else
        Pelec = [Pelec Pmech(i)*1.04];
    end
end

Pelec = Pelec./1000; 

efficency = 0.94;
Pavg      = abs(efficency*crosswind_opt_mechanical_3_loops.mechanical_energy.value(end-1)/30).*ones(length(Pelec),1)/1000;

%% bounds
drMin    = -9.5*ones(length(t_r));
drMax    = +9.5*ones(length(t_r));
alphaMin = -8*ones(length(t_ab)); 
alphaMax = 22*ones(length(t_ab));
airMin = 15*ones(length(airspeed));

%% Plots
figure;
subplot(5,1,1);                           
plot(t_r,r,'LineWidth',2,'Color','b');
ylabel('l [m]');grid on;hold on;axis([0 30 385 470]);
legend('tether length');
subplot(5,1,2);
plot(t_r,dr,'LineWidth',2,'Color','b');hold on;plot(t_r,drMin,'k--',t_r,drMax,'k--');
ylabel(' dl [m/s]');grid on;axis([0 30 -10 10]);legend('tether speed');
subplot(5,1,3);
plot(t_ab,alpha,'LineWidth',2,'Color','b');hold on;plot(t_ab,alphaMin,'k--',t_ab,alphaMax,'k--');
axis([0 30 -8.5 23]);ylabel('\alpha [deg]');grid on;
legend('angle of attack');
subplot(5,1,4);
plot(t_air,airspeed,'LineWidth',2,'Color','b');hold on;plot(t_air,airMin,'k--');
ylabel('V [m/s]');legend('Airspeed');
grid on;axis([0 30 14.5 35]);
subplot(5,1,5);
plot(t_pow,Pelec,'Color','b','LineWidth',1.5);hold on;grid on;
plot(t_pow,Pavg,'Color','green','LineWidth',1.5,'LineStyle','-.');grid on;
plot(t_pow,zeros(length(Pelec),1),'Color','k','LineWidth',1,'LineStyle',':');grid on;
axis([0 30 -12 28]);legend('P_{elec}','P_{av}');legend('Electrical Power')
ylabel('P_{elec} [KW]'); xlabel('time [s]');