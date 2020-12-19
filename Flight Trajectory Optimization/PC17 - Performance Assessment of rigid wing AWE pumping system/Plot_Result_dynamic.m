clc;clear all;close all;
% load('crosswind_opt_mechanical_AP2_accurate_drag.mat')
% AP = crosswind_opt_mechanical_3_AP2;

% load('crosswind_opt_mechanical_AP3_54KW.mat')
% AP = crosswind_opt_mechanical_3_AP3;

load('crosswind_opt_mechanical_3_loops.mat')
AP = crosswind_opt_mechanical_3_loops;

time = AP.r_n2b_n_x.time';time(6:6:end) = [];time(5:5:end) = [];
%% retrieve PowerPlane trajectory -----------------------------------------
px = AP.r_n2b_n_x.value';px(6:6:end) = [];px(5:5:end) = [];
py = AP.r_n2b_n_y.value';py(6:6:end) = [];py(5:5:end) = [];
pz = AP.r_n2b_n_z.value';pz(6:6:end) = [];pz(5:5:end) = [];
h  = -pz; % altitude

ptraj = [px,py,pz];
%% retrieve orientation ---------------------------------------------------
e11 = AP.e11.value';e11(6:6:end) = [];e11(5:5:end) = [];
e21 = AP.e21.value';e21(6:6:end) = [];e21(5:5:end) = [];
e31 = AP.e31.value';e31(6:6:end) = [];e31(5:5:end) = [];
ex  = [e11,e21,e31];

e12 = AP.e12.value';e12(6:6:end) = [];e12(5:5:end) = [];
e22 = AP.e22.value';e22(6:6:end) = [];e22(5:5:end) = [];
e32 = AP.e32.value';e32(6:6:end) = [];e32(5:5:end) = [];
ey  = [e12,e22,e32];

e13 = AP.e13.value';e13(6:6:end) = [];e13(5:5:end) = [];
e23 = AP.e23.value';e23(6:6:end) = [];e23(5:5:end) = [];
e33 = AP.e33.value';e33(6:6:end) = [];e33(5:5:end) = [];
ez  = [e13,e23,e33];

% R  = eye(3,3);
% ex = R(:,1)';
% ey = R(:,2)';
% ez = R(:,3)';
% 
% dir = -1;

%% Plot trajectory ========================================================
N = length(px);
figure;hold on;grid on;
% Plot Drum -------------------------------------------------------------
Rdrum             = 2; % radious drum
[Zwinch,Xwinch,~] = cylinder(Rdrum);
Ldrum  = 3; % length drum [m]
Ywinch = [-(Ldrum/2).*ones(21,1),(Ldrum/2).*ones(21,1)]';
Drum   = surf(Xwinch,Ywinch,Rdrum+Zwinch);

% Plot Ground -----------------------------------------------------------
Xground = [ max(ptraj(:,1)) max(ptraj(:,1))     -50                   -50];
Yground = [ min(ptraj(:,2)) max(ptraj(:,2))  max(ptraj(:,2))  min(ptraj(:,2))];
for i = 0:0.2:5
  Zground = [   -i   -i    -i     -i];
  Ground  = fill3(Xground,Yground,Zground,'g');
end

%% Plot PowerPlane Trajectory --------------------------------------------
Trajecotry = plot3(ptraj(:,1),ptraj(:,2),-ptraj(:,3),'b.');

%% Plot tether ------------------------------------------------------------
for k = 1:1:N
Tether = line([0,ptraj(k,1)],[0,ptraj(k,2)],-[0,ptraj(k,3)],'LineWidth',0.25,'Color','y','LineStyle',':');
end

% Plot wing ---------------------------------------------------------------
for i = 1:5:N
Wing_Ex = line([ ptraj(i,1), ptraj(i,1) + ey(i,1)],...
             [ ptraj(i,2), ptraj(i,2) + ex(i,2)],...
             -[ ptraj(i,3), ptraj(i,3) + ex(i,3)],'LineWidth',2,'Color','r');

Wing_Ey = line([ ptraj(i,1), ptraj(i,1) + ey(i,1)],...
             [ ptraj(i,2), ptraj(i,2) + ey(i,2)],...
             -[ ptraj(i,3), ptraj(i,3) + ey(i,3)],'LineWidth',2,'Color','g');

Wing_Ez = line([ ptraj(i,1), ptraj(i,1) + ez(i,1)],...
             [ ptraj(i,2), ptraj(i,2) + ez(i,2)],...
            -[ ptraj(i,3), ptraj(i,3) + ez(i,3)],'LineWidth',2,'Color','b');
end

xlabel('x[m]');ylabel('y[m]');zlabel('altitude[m]');





