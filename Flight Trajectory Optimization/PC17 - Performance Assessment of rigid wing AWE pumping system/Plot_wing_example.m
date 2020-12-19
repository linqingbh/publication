clc;clear all;close all;
% Plot Wing Example
% build wing
c = 0.55; % chord [m]
b = 5.5;  % wing span [m]

%  p1=(-b/2,c/2)          p2=(b/2,1)                
%     o-----------------o 
%     |                 |   ^ y
%     |        o        |   |
%     |         p0=(0,0)|   |
%     o-----------------o   ---> x
%  p4=(-b/2,-c/2)         p3=(b/2,-c/2)

figure;hold on;grid on;
%% position aircraft
px = 100;
py = 0;
h  = 100;

%% build wing
%             |p1|       |p2|       |p3|       |p4|
Xground = [ (px-c/2) , (px+c/2) , (px+c/2) , (px-c/2)];
Yground = [ (py+b/2) , (py+b/2) , (py-b/2) , (py-b/2)];
Zground = [    h     ,    h     ,    h     ,    h    ];
wing    = fill3(Xground,Yground,Zground,'b');

cg  = plot3(px,py,h,...
      'Color',[0.9,0.6,0],...
      'Marker','o',...
      'MarkerFaceColor',...
      [0.9,0.6,0],'MarkerSize',4);
%% cable 
Tether = line([0,px],[0,py],[0,h],'LineWidth',2,'Color','y','LineStyle',':');

% Plot Drum -------------------------------------------------------------
Rdrum = 0.35; % radious drum
[Zwinch,Xwinch,~] = cylinder(Rdrum);
Ldrum  = 2; % length drum [m]
Ywinch = [-(Ldrum/2).*ones(21,1),(Ldrum/2).*ones(21,1)]';
Drum   = surf(Xwinch,Ywinch,Rdrum+Zwinch);

xlabel('x [m]');
ylabel('y [m]');
zlabel('h [m]');

axis([-100, 100, -100 ,100 , 0 , 100])
