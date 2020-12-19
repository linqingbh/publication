% | title: plot 3D off line trajectory       |
% | Author: Giovanni Licitra                 |
% | date: 03/05/2015                         |
% ============================================

clc; clear all; close all;

orange = [0.9608  0.6118  0;  
          0.9608  0.6118  0;
          0.9608  0.6118  0;
          0.9608  0.6118  0;
          0.9608  0.6118  0;
          0.9608  0.6118  0;
          0.9608  0.6118  0];
       
blue =   [0   0.5569   0.8118; 
          0   0.5669   0.8218;
          0   0.5769   0.8318;
          0   0.5869   0.8418;
          0   0.5969   0.8518;
          0   0.6069   0.8618;
          0   0.6169   0.8718];

% green = [0    0.4  0;
%          0    0.6  0; 
%          0    0.8  0;
%          0    1    0;
%          0.2  1    0.2;
%          0.4  1    0.4;
%          0.8  1    0.8];
% red =   [0.4  0   0; 
%          0.6  0   0;
%          0.8  0   0;
%          1    0     0;
%          1    0.2   0.2;
%          1    0.4   0.4;
%          1    0.8   0.8];
grey = [0.125 0.125 0.125;
        0.25 0.25 0.25;
        0.325 0.325 0.325;
        0.5 0.5 0.5;
        0.625 0.625 0.625;
        0.75 0.75 0.75;
        0.825 0.825 0.825];

     
figure(1);hold on;                                % select area in figure
for k = 0:1:30
Xground=[-30,-30,30,30];
Yground=[30,-30,-30,30];
Zground=[-k,-k,-k,-k];
patch(Xground,Yground,Zground,'c');        % plot ground
end

grid on;
cylinder(5);                                   % plot winch

% Plot Wind Direction
% w0 = [90 0 60;120 0 70;150 0 80;180 0 90;210 0 100];
% wp = 50*[1 0 0;1 0 0;1 0 0;1 0 0;1 0 0];
% quiver3(w0(:,1), w0(:,2), w0(:,3), wp(:,1), wp(:,2), wp(:,3),'Color','b','LineWidth',2,'LineStyle',':');   
% 
% w0 = [90 50 120;120 50 130;150 50 140;180 50 150;210 50 160];
% wp = 30*[1 0 0;1 0 0;1 0 0;1 0 0;1 0 0];
% quiver3(w0(:,1), w0(:,2), w0(:,3), wp(:,1), wp(:,2), wp(:,3),'Color','b','LineWidth',2,'LineStyle',':');  
% 
% w0 = [90 -50 120;120 -50 130;150 -50 140;180 -50 150;210 -50 160];
% wp = 50*[1 0 0;1 0 0;1 0 0;1 0 0;1 0 0];
% quiver3(w0(:,1), w0(:,2), w0(:,3), wp(:,1), wp(:,2), wp(:,3),'Color','b','LineWidth',2,'LineStyle',':');  


load('wind0_Pmech-589_49');
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;
x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;
r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);

for k = 2:3:length(t)
    if r(k)-r(k-1)>=0
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',orange(4,:));
         else
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',blue(4,:));          
    end    
    fig = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.25,'Color',grey(1,:),'LineStyle',':');
end

load('wind0-5_Pmech-561_33');
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;
x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;
r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);

for k = 2:3:length(t)
    if r(k)-r(k-1)>=0
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',orange(4,:));
         else
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',blue(4,:));
    end    
    fig = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.25,'Color',grey(2,:),'LineStyle',':');
end

load('wind1_Pmech-524-46');
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;
x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;
r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);

for k = 2:3:length(t)
    if r(k)-r(k-1)>=0
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',orange(4,:));
         else
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',blue(4,:));
    end  
    fig = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.25,'Color',grey(3,:),'LineStyle',':');
end

load('wind1-5_Pmech-481-08');
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;
x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;

r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);

for k = 2:3:length(t)
    if r(k)-r(k-1)>=0
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',orange(4,:));
         else
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',blue(4,:));
    end
    fig = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.25,'Color',grey(4,:),'LineStyle',':');
end

load('wind2_Pmech-367-01');
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;
x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;
r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);

for k = 2:3:length(t)
    if r(k)-r(k-1)>=0
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',orange(4,:));
         else
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',blue(4,:));
    end 
    fig = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.25,'Color',grey(5,:),'LineStyle',':');
end

load('wind2-5_Pmech-153-63');
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;
x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;
r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);

for k = 2:3:length(t)
    if r(k)-r(k-1)>=0
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',orange(4,:));
         else
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',blue(4,:));
    end 
    fig = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.25,'Color',grey(6,:),'LineStyle',':');
end

load('wind3_Pmech+128-71');
t = crosswind_opt_mechanical_3_loops.r_n2b_n_x.time;
x = crosswind_opt_mechanical_3_loops.r_n2b_n_x.value;
y = -crosswind_opt_mechanical_3_loops.r_n2b_n_y.value;
z = -crosswind_opt_mechanical_3_loops.r_n2b_n_z.value;
r   = cleanStruct(crosswind_opt_mechanical_3_loops.r.value);
dr  = cleanStruct(crosswind_opt_mechanical_3_loops.dr.value);

for k = 2:3:length(t)
    if r(k)-r(k-1)>=0
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',orange(4,:));
         else
             fig = plot3(x(k),y(k),z(k),'Marker','>','LineStyle','-','MarkerEdgeColor','k','MarkerFaceColor',blue(4,:));
    end 
    fig = line([0,x(k)],[0,y(k)],[0,z(k)],'LineWidth',0.25,'Color',grey(7,:),'LineStyle',':');
end

xlabel('x[m]');ylabel('y[m]');zlabel('z[m]');
%axis([-50 450 -250 250 -50 200])
%view(-11,15);                                   % adjust view




