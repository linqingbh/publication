
% ============================================
% | title: plot 3D off line trajectory       |
% | Author: Giovanni Licitra                 |
% | date: 03/05/2015                         |
% ============================================
%% Legend
% 1: Case with Tether drag included
% 2: Case without Tether drag included
function PaperXtether
clear all;close all;clc;
[t1,t_r1,t_T1,x1,y1,z1,r1,dr1,Pmech1,wind_1,cD1,cD_tether1] = loadTetherCase();
[~,t_r2,t_T2,x2,y2,z2,r2,dr2,Pmech2,wind_2,cD2,cD_tether2] = loadNoTetherCase();

%% Plot 3D trajecotory
fig3D = figure(1);hold on;grid on;                     
% Plot ground around winch
for k = 0:1:30
    Xground=[-30,-30,30,30];Yground=[30,-30,-30,30];Zground=[-k,-k,-k,-k];
    fig3D = patch(Xground,Yground,Zground,'c');              % plot ground
    cylinder(0.5*k);                                         % plot winch
end
% Plot Trajecotory
for k = 2:2:length(t1)
    if r1(k)-r1(k-1)>=0
             fig3D = plot3(x1(k),y1(k),z1(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',8);             
         else
             fig3D = plot3(x1(k),y1(k),z1(k),'Marker','o','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',8);             
    end   
    if r2(k)-r2(k-1)>=0
        fig3D = plot3(x2(k),y2(k),z2(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor','b','MarkerSize',8);
    else
        fig3D = plot3(x2(k),y2(k),z2(k),'Marker','o','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor','y','MarkerSize',8);
    end
end
% Plot cable
for k = 1:10:length(t1)
    fig3D = line([0,x1(k)],[0,y1(k)],[0,z1(k)],'LineWidth',0.5,'Color',[0.8,0.8,0.8],'LineStyle',':');
    fig3D = line([0,x2(k)],[0,y2(k)],[0,z2(k)],'LineWidth',0.5,'Color',[0.8,0.8,0.8],'LineStyle',':');
end
view(325,3);axis equal;                            % adjust view
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');

%% r = length tether | dr = speed tether |T = tether tension 
tw = t_r1(1:end-1);
r_max =  1000*ones(1,length(t_r1));
figure(2);
% plot tether length
subplot(4,1,1);hold on;grid on; ylabel('tether length [m]');
xlim([0,30]);ylim([350,1020]); % tether length Graph                          
plot(t_r1,r1,'LineWidth',3,'Color','g','LineStyle','--');
plot(t_r2,r2,'LineWidth',3,'Color','b','LineStyle','-.');
plot(t_r1,r_max,'LineWidth',1.5,'Color','r','LineStyle',':');
% text for tether drag
% [r_max1,~] = max(r1);[r_min1,~] = min(r1);     % get values and index
% text1 = strcat('\Delta l = ',num2str(max(r1)-min(r1)),' [m]');
% text(6.6,(max(r1)+min(r1))/2+65,text1);

% yfit = line([6.5,6.5],[r_min1,r_max1]);
% set(yfit,'LineWidth',1,'Color','k');
% yfit2 = line([6.3,6.7],[r_min1,r_min1]);
% set(yfit2,'LineWidth',1,'Color','k');
% yfit3 = line([6.3,6.7],[r_max1,r_max1]);
% set(yfit3,'LineWidth',1,'Color','k');

% text1 = strcat('\Delta l = 39.86 [m]');
% text(23,(max(r1)+min(r1))/2+65,text1);

% yfit = line([22.7,22.7],[r_min1+5,r_max1-17]);
% set(yfit,'LineWidth',1,'Color','k');
% yfit2 = line([22.5,22.9],[r_min1+5,r_min1+5]);
% set(yfit2,'LineWidth',1,'Color','k');
% yfit3 = line([22.5,22.9],[r_max1-17,r_max1-17]);
% set(yfit3,'LineWidth',1,'Color','k');

% [r_max2,~] = max(r2);[r_min2,~] = min(r2);     % get values and index
% text2 = strcat('\Delta l= ',num2str(max(r2)-min(r2)),' [m]');
% text(17,(max(r2)+min(r2))/2,text2);

% yfit = line([16.5,16.5],[r_min2,r_max2]);
% set(yfit,'LineWidth',1,'Color','k');
% yfit2 = line([16.3,16.8],[r_min2,r_min2]);
% set(yfit2,'LineWidth',1,'Color','k');
% yfit3 = line([16.3,16.8],[r_max2,r_max2]);
% set(yfit3,'LineWidth',1,'Color','k');

%% tether speed
% bounds tether
dr_min = -9.5*ones(length(t_r1));
dr_max =  9.5*ones(length(t_r1));
subplot(4,1,2);hold on;grid on; ylabel('tether speed [m/s]');xlim([0,30]);           
plot(t_r1,dr1,'LineWidth',3,'Color','g','LineStyle','--');
plot(t_r2,dr2,'LineWidth',3,'Color','b','LineStyle','-.');
plot(t_r1,dr_min,'r:',t_r1,dr_max,'r:','LineWidth',1.5);
axis([0 30 -11 11]);
%legend('Tether drag included','Tether Drag not included','bounds');

% print('r_dr', '-dpng', '-r300');   %<-Save as PNG with 300 DPI
% print('r_dr', '-depsc ', '-r300'); %<-Save as eps3 with 300 DPI

%% wind at altitude
%figure(3);
subplot(4,1,3);hold on;ylabel('wind at altitude [m/s]');
xlim([0,30]);
plot(tw,wind_1,'LineWidth',3,'Color','g','LineStyle','--');
plot(tw,wind_2,'LineWidth',3,'Color','b','LineStyle','-.');
grid on; 
%% plot Mechanical power
subplot(4,1,4);hold on;grid on; ylabel('mechanical power [kW]');xlim([0,30]);            
P = area(t_T2,Pmech2./1e3);
P.FaceColor = [0,0,1];
P = area(t_T1,Pmech1./1e3);
P.FaceColor = [0,1,0];
ylim([-12 32]);
xlabel('time [s]'); 

%%
% print('wind_and_power', '-dpng', '-r300');   %<-Save as PNG with 300 DPI
% print('r_dr_w_P', '-depsc ', '-r300');       %<-Save as eps3 with 300 DPI

figure(5);grid on;hold on;
plot(tw,cD1,'LineWidth',3,'Color','k','LineStyle','-.');
plot(tw,cD_tether1,'LineWidth',3,'Color','k','LineStyle',':');
plot(tw,cD1+cD_tether1,'LineWidth',3,'Color','k');
xlabel('time [s]');ylabel('Drag Coefficients');
legend('C_{D}','C_{D_{t}}','C_{D} + C_{D_{t}}');
%print('cD', '-depsc ', '-r300');       %<-Save as eps3 with 300 DPI
end

function [t,t_r,t_t,x,y,z,r,dr,Pmech,wind,cD,cD_tether] = loadTetherCase()
%% import trajectory with Tether
load('Traj10windTether');
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;         % time for trajecotory
t_r = crosswind_opt_mechanical_3_loops.r.time;               % time for cable
t_t = crosswind_opt_mechanical_3_loops.tether_tension.time;  % time fof tension

x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;        % x-coordinate
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;       % y-coordinate
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;       % z-coordinate

r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value); % tether length
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);% tether speed

Pmech = cleanStruct(crosswind_opt_mechanical_3_loops.neg_mechanical_winch_power.value);
wind = cleanStruct(crosswind_opt_mechanical_3_loops.wind_at_altitude.value);

cD = cleanStruct(crosswind_opt_mechanical_3_loops.cD.value);
cD_tether = cleanStruct(crosswind_opt_mechanical_3_loops.cD_tether.value);
end

function [t,t_r,t_t,x,y,z,r,dr,Pmech,wind,cD,cD_tether] = loadNoTetherCase()
%% import trajectory with Tether
load('Traj10windNoTether');
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;         % time for trajecotory
t_r = crosswind_opt_mechanical_3_loops.r.time;               % time for cable
t_t = crosswind_opt_mechanical_3_loops.tether_tension.time;  % time fof tension

x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;        % x-coordinate
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;       % y-coordinate
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;       % z-coordinate

r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value); % tether length
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);% tether speed

Pmech = cleanStruct(crosswind_opt_mechanical_3_loops.neg_mechanical_winch_power.value);
wind = cleanStruct(crosswind_opt_mechanical_3_loops.wind_at_altitude.value);

cD = cleanStruct(crosswind_opt_mechanical_3_loops.cD.value);
cD_tether = cleanStruct(crosswind_opt_mechanical_3_loops.cD_tether.value);
end
