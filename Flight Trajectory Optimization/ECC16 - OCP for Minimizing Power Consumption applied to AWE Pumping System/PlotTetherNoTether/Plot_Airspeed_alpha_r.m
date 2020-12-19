clc;clear all;close all;

load Traj10windTether

t1 = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;         % time for trajecotory
t_r = crosswind_opt_mechanical_3_loops.r.time;               % time for cable
t_t = crosswind_opt_mechanical_3_loops.tether_tension.time;  % time fof tension

x1 = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;        % x-coordinate
y1 = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;       % y-coordinate
z1 = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;       % z-coordinate

r1   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value); % tether length
dr1  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);% tether speed

%% Plot 3D trajecotory
 fig3D = figure;hold on;grid on;                    
% Plot ground around winch
for k = 0:1:30
    fig3D = subplot(5,1,[1:2]);cylinder(0.5*k);            % plot winch
end
hold on;
% Plot Trajecotory
for k = 2:2:length(t1)
    if r1(k)-r1(k-1)>=0
             fig3D = subplot(5,1,[1:2]);plot3(x1(k),y1(k),z1(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',8);             
         else
             fig3D = subplot(5,1,[1:2]);plot3(x1(k),y1(k),z1(k),'Marker','o','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',8);             
    end   
end
% Plot cable
for k = 1:10:length(t1)
    fig3D = subplot(5,1,[1:2]);line([0,x1(k)],[0,y1(k)],[0,z1(k)],'LineWidth',0.5,'Color',[0.8,0.8,0.8],'LineStyle',':');
end
%view(325,3);
axis equal;                            % adjust view
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');

t_a   = crosswind_opt_mechanical_3_loops.alpha_deg.time;
t_r   = crosswind_opt_mechanical_3_loops.r.time;
t_air = crosswind_opt_mechanical_3_loops.airspeed.time;

alpha    = cleanStruct(crosswind_opt_mechanical_3_loops.alpha_deg.value);
alphaMin = -8*ones(length(t_a)); 
alphaMax = 22*ones(length(t_a));

r        = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
rMin     = 1*ones(length(t_r));
rMax     = 1000*ones(length(t_r));

airspeed = cleanStruct(crosswind_opt_mechanical_3_loops.airspeed.value);
airMin   = 15*ones(length(airspeed));

figure(1);
subplot(5,1,3);  
plot(t_r,r,'LineWidth',2);legend('tether length');ylabel('[m]');grid on;hold on;

subplot(5,1,4);                           
plot(t_a,alpha,'LineWidth',2);hold on;plot(t_a,alphaMin,'r--',t_a,alphaMax,'r--');
axis([0 30 -8.5 23]);legend('angle of attack','bounds');ylabel('[deg]');grid on;

subplot(5,1,5);
plot(t_air,airspeed,'LineWidth',2);hold on;plot(t_air,airMin,'r--');
legend('airspeed','bound');ylabel('[m/s]');xlabel('time [s]')
grid on;axis([0 30 14.5 35]);
