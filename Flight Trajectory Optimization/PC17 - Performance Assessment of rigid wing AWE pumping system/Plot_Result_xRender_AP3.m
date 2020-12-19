clc;clear all;close all;
load('crosswind_opt_mechanical_AP3_54KW.mat')
AP   = crosswind_opt_mechanical_3_AP3;
id1  = 6;id2 = 5;
time = AP.r_n2b_n_x.time';%time(id1:id1:end) = [];%time(id2:id2:end) = [];
N    = length(time); 

%% retrieve PowerPlane trajectory -----------------------------------------
px = AP.r_n2b_n_x.value';%px(id1:id1:end) = [];%px(id2:id2:end) = [];
py = AP.r_n2b_n_y.value';%py(id1:id1:end) = [];%py(id2:id2:end) = [];
pz = AP.r_n2b_n_z.value';%pz(id1:id1:end) = [];%pz(id2:id2:end) = [];
h  = -pz; % altitude

ptraj = [px,py,pz];

dl    = AP.dr.value;%dl(id1:id1:end) = [];%dl(id2:id2:end) = [];

reel_in  = [];
reel_out = [];

for i = 1:N
  if dl(i) > 0      % reel-out <=> power generation 
    reel_out = [ptraj(i,:);reel_out];
  elseif dl(i) < 0  % reel-in <=> power consumption
    reel_in  = [ptraj(i,:);reel_in];
  end   
end

N_reel_in  = length(reel_in);
N_reel_out = length(reel_out);

%imshow(imread('landscape.jpg'))
figure;hold on;grid on;
Trajectory    = plot3(px,py,-pz,'k');
traj_reel_out = plot3(reel_out(:,1),reel_out(:,2),-reel_out(:,3),...
      'Marker','o',...
      'MarkerEdgeColor','k',...
      'MarkerFaceColor',[0.1176,0.5804,0.8235],...
      'MarkerSize',5,...
      'Color','none');
traj_reel_in = plot3(reel_in(:,1),reel_in(:,2),-reel_in(:,3),...
      'Marker','o',...
      'MarkerEdgeColor','k',...
      'MarkerFaceColor',[1 0.6196 0.1059],...
      'MarkerSize',5,...
      'Color','none');

%% retrieve orientation ---------------------------------------------------
e11 = AP.e11.value';%e11(id1:id1:end) = [];%e11(id2:id2:end) = [];
e21 = AP.e21.value';%e21(id1:id1:end) = [];%e21(id2:id2:end) = [];
e31 = AP.e31.value';%e31(id1:id1:end) = [];%e31(id2:id2:end) = [];
ex  = [e11,e21,e31];

e12 = AP.e12.value';%e12(id1:id1:end) = [];%e12(id2:id2:end) = [];
e22 = AP.e22.value';%e22(id1:id1:end) = [];%e22(id2:id2:end) = [];
e32 = AP.e32.value';%e32(id1:id1:end) = [];%e32(id2:id2:end) = [];
ey  = [e12,e22,e32];

e13 = AP.e13.value';%e13(id1:id1:end) = [];%e13(id2:id2:end) = [];
e23 = AP.e23.value';%e23(id1:id1:end) = [];%e23(id2:id2:end) = [];
e33 = AP.e33.value';%e33(id1:id1:end) = [];%e33(id2:id2:end) = [];
ez  = [e13,e23,e33];

%Plot wing ---------------------------------------------------------------
factor = 5;

for i = 1:1:N_reel_out
Wing_Ey = line([ reel_out(i,1)- factor*ex(i,1), reel_out(i,1) + factor*ex(i,1)],...
               [ reel_out(i,2)- factor*ex(i,2), reel_out(i,2) + factor*ex(i,2)],...
              -[ reel_out(i,3)- factor*ex(i,3), reel_out(i,3) + factor*ex(i,3)],...
               'LineWidth',5,'Color',[0.1176,0.5804,0.8235]);          
Tether = line([0,reel_out(i,1)],...
              [0,reel_out(i,2)],...
             -[0,reel_out(i,3)],'LineWidth',0.25,'Color',[0.8 0.8 0.8],'LineStyle','-');
end

for i = 1:1:N_reel_in 
Wing_Ey = line([ reel_in(i,1)- factor*ex(i,1), reel_in(i,1) + factor*ex(i,1)],...
               [ reel_in(i,2)- factor*ex(i,2), reel_in(i,2) + factor*ex(i,2)],...
              -[ reel_in(i,3)- factor*ex(i,3), reel_in(i,3) + factor*ex(i,3)],...
                'LineWidth',5,'Color',[1 0.6196 0.1059]);
              
Tether = line([0,reel_in(i,1)],...
              [0,reel_in(i,2)],...
             -[0,reel_in(i,3)],'LineWidth',0.25,'Color',[0.8 0.8 0.8],'LineStyle','-');
end
xlabel('x[m]');ylabel('y[m]');zlabel('altitude[m]');
%zlim([-20,250])
%xlim([-250,250])
set(gca, 'Projection', 'perspective'); % Sets axes background
set(gca, 'Color', 'none'); % Sets axes background
axis equal

%export_fig Trajectory3D_AP3.png -transparent

