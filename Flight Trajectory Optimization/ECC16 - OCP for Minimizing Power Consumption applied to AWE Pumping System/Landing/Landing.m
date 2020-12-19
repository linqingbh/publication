% | title: plot 3D off line trajectory       |
% | Author: Giovanni Licitra                 |
% | date: 03/05/2015                         |
% ============================================

clc; clear all; close all;

green = [0    0.4  0;
         0    0.6  0; 
         0    0.8  0;
         0    1    0;
         0.2  1    0.2;
         0.4  1    0.4;
         0.8  1    0.8];
red =   [0.4  0   0; 
         0.6  0   0;
         0.8  0   0;
         1    0     0;
         1    0.2   0.2;
         1    0.4   0.4;
         1    0.8   0.8];
grey = [0.125 0.125 0.125;
        0.25 0.25 0.25;
        0.325 0.325 0.325;
        0.5 0.5 0.5;
        0.625 0.625 0.625;
        0.75 0.75 0.75;
        0.825 0.825 0.825];
     
     
figure(1);hold on;                                % select area in figure
for k = 0:1:30
Xground=[-30,-30,200,200];
Yground=[60,-60,-60,60];
Zground=[-k,-k,-k,-k];
patch(Xground,Yground,Zground,'c');        % plot ground
end

grid on;
cylinder(5);                                   % plot winch

load('crosswind_opt_mechanical_5_wind');
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;
x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;
r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);

for k = 2:1:length(t)
    if r(k)-r(k-1)>=0
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor','g');
         else
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor','r');          
    end    
    fig = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.25,'Color',grey(1,:),'LineStyle',':');
end

%% plot parte elicoidale
t = 3*pi:0.1:34*pi;
st = 70*sin(0.3*t);
ct = 70*cos(0.3*t);
fig = plot3(st,ct,t,'Marker','>','LineStyle','-','MarkerEdgeColor','b','MarkerFaceColor','g');
for k = 2:1:length(t)
    fig = line([0,st(k)],[0,ct(k)],[0,t(k)],'LineWidth',0.25,'Color','b','LineStyle',':');
end

%% plot intersection line-elicoidale
fig = line([x(end),st(end)],[y(end),ct(end)],[z(end),t(end)],'Color','b','LineWidth',3,'LineStyle','-')
fig = line([150,st(1)],[0,ct(1)],[0,t(1)],'Color','b','LineWidth',3,'LineStyle','-')


%axis([-50 450 -250 250 -50 450])
view(325,3);                                   % adjust view

