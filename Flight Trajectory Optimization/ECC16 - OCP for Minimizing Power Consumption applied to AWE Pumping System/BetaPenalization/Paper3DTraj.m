% | title: plot 3D off line trajectory       |
% | Author: Giovanni Licitra                 |
% | date: 03/05/2015                         |
% ============================================

clc; clear all; close all;
%load('Traj10windTether');
load('crosswind_opt_beta_Penalization');
%
%% import trajectory with Tether
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;
t_r = crosswind_opt_mechanical_3_loops.r.time;
t_t = crosswind_opt_mechanical_3_loops.tether_tension.time;

x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;

r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);
tether_tension = cleanStruct(crosswind_opt_mechanical_3_loops.tether_tension.value); 

%% Plot 3D trajecotory
fig3D = figure(1);                                              % select area in figure
Xground = [-30,-30,30,30];Yground = [30,-30,-30,30];Zground = [0,0,0,0];
fig3D = patch(Xground,Yground,Zground,'k');hold on;grid on;     % plot ground
fig3D = cylinder(5);                                            % plot winch

for k = 2:1:length(t)
    if r(k)-r(k-1)>=0
             fig3D = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor','g');
             fig3D = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.5,'Color','k','LineStyle',':');
         else
             fig3D = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor','r');
             fig3D = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.5,'Color','k','LineStyle',':');
    end    
end
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');


%load('Traj10windNoTether');
% load('crosswind_opt_mechanical_beta');
% t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;
% t_r = crosswind_opt_mechanical_3_loops.r.time;
% t_t = crosswind_opt_mechanical_3_loops.tether_tension.time;
% 
% x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;
% y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;
% z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;
% 
% r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
% dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);
% tether_tension = cleanStruct(crosswind_opt_mechanical_3_loops.tether_tension.value); 
% 
% for k = 2:6:length(t)
%     if r(k)-r(k-1)>=0
%              fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','b','MarkerFaceColor','b');
%              fig = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.5,'Color','k','LineStyle',':');
%          else
%              fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','y','MarkerFaceColor','y');
%              fig = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.5,'Color','k','LineStyle',':');
%     end    
% end
axis equal
% axis([-50 600 -325 325 -50 550])
view(325,3);                                   % adjust view

%% r = length tether | dr = speed tether | tether tension 
% rMin = 1*ones(length(t_r));
% rMax = 1000*ones(length(t_r));
% drMin = -30*ones(length(t_r));
% drMax = +30*ones(length(t_r));
% t_tension_min = 0*ones(length(t_t));
% 
% figure(2);
% 
% subplot(3,1,1);                           
% plot(t_r,r,'LineWidth',2);legend('tether length');ylabel('[m]');grid on;hold on;
% axis([0 22.5 360 540]);
% 
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
% 
% subplot(3,1,2);
% plot(t_r,dr,'LineWidth',2);hold on;plot(t_r,drMin,'r--',t_r,drMax,'r--');
% axis([0 22.5 -32 32]);legend('tether speed','bounds');ylabel('[m/s]');grid on;
% 
% subplot(3,1,3);
% plot(t_t,tether_tension,'LineWidth',2);hold on;plot(t_t,t_tension_min,'r--');
% axis([0 22.5 -100 3100]);legend('tether tension','bound');ylabel('[N]');grid on;

