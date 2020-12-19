clc;clear all;close all;
load('crosswind_opt_mechanical_AP3_54KW.mat')
AP   = crosswind_opt_mechanical_3_AP3;
id1  = 6;id2 = 5;
time = AP.r_n2b_n_x.time';%time(id1:id1:end) = [];%time(id2:id2:end) = [];

%% retrieve PowerPlane trajectory -----------------------------------------
px   = AP.r_n2b_n_x.value';px(id1:id1:end) = [];px(id2:id2:end) = [];
py   = AP.r_n2b_n_y.value';py(id1:id1:end) = [];py(id2:id2:end) = [];
pz   = AP.r_n2b_n_z.value';pz(id1:id1:end) = [];pz(id2:id2:end) = [];
l    = AP.r.value'       ;  l(id1:id1:end) = []; l(id2:id2:end) = [];
dl   = AP.dr.value';       dl(id1:id1:end) = [];dl(id2:id2:end) = [];
wind = AP.wind_at_altitude.value';wind(id1:id1:end) = [];wind(id2:id2:end) = [];
Pe   = AP.mechanical_winch_power.value';Pe(id2:id2:end) = [];Pe = -Pe./1000;

h  = -pz; % altitude

ptraj = [px,py,pz];
N     = length(h);
T     = time(end);
time  = linspace(0,T,N)';
ts    = T/N;

% retrieve orientation ----------------------------------------------------
e11 = AP.e11.value';e11(id1:id1:end) = [];e11(id2:id2:end) = [];
e21 = AP.e21.value';e21(id1:id1:end) = [];e21(id2:id2:end) = [];
e31 = AP.e31.value';e31(id1:id1:end) = [];e31(id2:id2:end) = [];
ex  = [e11,e21,e31];

e12 = AP.e12.value';e12(id1:id1:end) = [];e12(id2:id2:end) = [];
e22 = AP.e22.value';e22(id1:id1:end) = [];e22(id2:id2:end) = [];
e32 = AP.e32.value';e32(id1:id1:end) = [];e32(id2:id2:end) = [];
ey  = [e12,e22,e32];

e13 = AP.e13.value';e13(id1:id1:end) = [];e13(id2:id2:end) = [];
e23 = AP.e23.value';e23(id1:id1:end) = [];e23(id2:id2:end) = [];
e33 = AP.e33.value';e33(id1:id1:end) = [];e33(id2:id2:end) = [];
ez  = [e13,e23,e33];

eWing = [e11,e12,e13];

factor = 1;
%%
figure;
subplot(4,4,[1,2,3,5,6,7,9,10,11,13,14,15]);hold on;grid on;

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
  Ground  = fill3(Xground,Yground,Zground,[0.4 0.6 0.2]);
end

xlabel('x[m]');ylabel('y[m]');zlabel('altitude[m]');
set(gca, 'Projection', 'perspective'); % Sets axes background
Trajectory = plot3(px,py,-pz,'LineWidth',1,'Color','k');
AP         = plot3(px(1),py(1),h(1),...
             'Marker','o','MarkerSize',5,'MarkerEdgeColor','k','MarkerFaceColor',[0.1176,0.5804,0.8235]);
Tether     = line([0,px(1)],[0,py(1)],[0,h(1)],...
             'LineWidth',2,'Color',[0.8 0.8 0.8],'LineStyle','-');
Wing       = line([px(1) - factor*ey(1,1), px(1) + factor*ey(1,1)],...
                  [py(1) - factor*ey(1,2), py(1) + factor*ey(1,2)],...
                  [ h(1) - factor*ey(1,3),  h(1) + factor*ey(1,3)],...
                  'LineWidth',10,'Color',[0.1176,0.5804,0.8235]);    

subplot(4,4,4);hold on;grid on;
plot(time   ,l   ,'LineWidth',1.5,'Color','k');
pl     = plot(time(1),l(1),'Marker','o','MarkerEdgeColor','k');
legend('Tether Length [m]')
xlim([0,T]);

subplot(4,4,8);hold on;grid on;
plot(time   ,dl   ,'LineWidth',1.5,'Color','k');
pdl      = plot(time(1),dl(1),'Marker','o','MarkerEdgeColor','k');
dl_M     = plot(time   ,  8.*ones(N,1),'r-.');
dl_m     = plot(time   ,-10.*ones(N,1),'r-.');
legend('Tether speed [m]')
xlim([0,T]);ylim([-12,10]);

subplot(4,4,12);hold on;grid on;
plot(time(1:end-1) ,wind   ,'LineWidth',1.5,'Color','k');
pwind      = plot(time(1),wind(1),'Marker','o','MarkerEdgeColor','k');
legend('wind at altitude [m/s]')
xlim([0,T]);

subplot(4,4,16);hold on;grid on;
area(time(1:end-1)   ,Pe,'FaceColor',[0.4 0.6 0.6],'FaceAlpha',.5,'EdgeAlpha',.5 );
pPe  = plot(time(1)  ,Pe(1),'Marker','o','MarkerEdgeColor','k');
legend('Energy [kJ]');xlabel('time [s]');
xlim([0,T]);

nloop = 5;
for k = 1:nloop
    for i=2:N
        if     dl(i) > 0 % reel-out
            set(AP, 'XData', px(i),'MarkerFaceColor',[0.1176,0.5804,0.8235]);
            set(AP, 'YData', py(i),'MarkerFaceColor',[0.1176,0.5804,0.8235]);
            set(AP, 'ZData',  h(i),'MarkerFaceColor',[0.1176,0.5804,0.8235]);
            set(Wing, 'XData', [px(i) - factor*eWing(i,1), px(i) + factor*eWing(i,1)],'Color',[0.1176,0.5804,0.8235]);
            set(Wing, 'YData', [py(i) - factor*eWing(i,2), py(i) + factor*eWing(i,2)],'Color',[0.1176,0.5804,0.8235]);
            set(Wing, 'ZData', [ h(i) - factor*eWing(i,3),  h(i) + factor*eWing(i,3)],'Color',[0.1176,0.5804,0.8235]);
            set(pl  , 'XData', time(i),'MarkerFaceColor',[0.1176,0.5804,0.8235]);
            set(pl  , 'YData', l(i)   ,'MarkerFaceColor',[0.1176,0.5804,0.8235]); 
            set(pdl , 'XData', time(i),'MarkerFaceColor',[0.1176,0.5804,0.8235]);
            set(pdl , 'YData', dl(i)   ,'MarkerFaceColor',[0.1176,0.5804,0.8235]);
            set(Tether, 'XData', [0,px(i)],'Color',[0.1176,0.5804,0.8235]);
            set(Tether, 'YData', [0,py(i)],'Color',[0.1176,0.5804,0.8235]);
            set(Tether, 'ZData', [0, h(i)],'Color',[0.1176,0.5804,0.8235]);
            if i == N
                set(pwind, 'XData', time(N-1),'MarkerFaceColor',[0.1176,0.5804,0.8235]);
                set(pwind, 'YData', wind(N-1) ,'MarkerFaceColor',[0.1176,0.5804,0.8235]);
                set(pPe  , 'XData', time(N-1),'MarkerFaceColor',[0.1176,0.5804,0.8235]);
                set(pPe  , 'YData',   Pe(N-1) ,'MarkerFaceColor',[0.1176,0.5804,0.8235]);
            else
                set(pwind, 'XData', time(i),'MarkerFaceColor',[0.1176,0.5804,0.8235]);
                set(pwind, 'YData', wind(i) ,'MarkerFaceColor',[0.1176,0.5804,0.8235]); 
                set(pPe  , 'XData', time(i),'MarkerFaceColor',[0.1176,0.5804,0.8235]);
                set(pPe  , 'YData',   Pe(i) ,'MarkerFaceColor',[0.1176,0.5804,0.8235]);
            end   
        elseif dl(i) < 0 % reel-in
            set(AP, 'XData', px(i),'MarkerFaceColor',[1     ,0.6196,0.1059]);
            set(AP, 'YData', py(i),'MarkerFaceColor',[1     ,0.6196,0.1059]);
            set(AP, 'ZData',  h(i),'MarkerFaceColor',[1     ,0.6196,0.1059]);
            set(Wing, 'XData', [px(i) - factor*eWing(i,1), px(i) + factor*eWing(i,1)],'Color',[1     ,0.6196,0.1059]);
            set(Wing, 'YData', [py(i) - factor*eWing(i,2), py(i) + factor*eWing(i,2)],'Color',[1     ,0.6196,0.1059]);
            set(Wing, 'ZData', [ h(i) - factor*eWing(i,3),  h(i) + factor*eWing(i,3)],'Color',[1     ,0.6196,0.1059]);
            set(pl  , 'XData', time(i),'MarkerFaceColor',[1     ,0.6196,0.1059]);
            set(pl  , 'YData', l(i)   ,'MarkerFaceColor',[1     ,0.6196,0.1059]);    
            set(pdl  , 'XData',time(i),'MarkerFaceColor',[1     ,0.6196,0.1059]);
            set(pdl  , 'YData',dl(i)  ,'MarkerFaceColor',[1     ,0.6196,0.1059]);
            set(Tether, 'XData', [0,px(i)],'Color',[1     ,0.6196,0.1059]);
            set(Tether, 'YData', [0,py(i)],'Color',[1     ,0.6196,0.1059]);
            set(Tether, 'ZData', [0, h(i)],'Color',[1     ,0.6196,0.1059]);
            if i == N
            set(pwind, 'XData', time(N-1),'MarkerFaceColor',[1     ,0.6196,0.1059]);
            set(pwind, 'YData', wind(N-1),'MarkerFaceColor',[1     ,0.6196,0.1059]); 
            set(pPe  , 'XData', time(N-1),'MarkerFaceColor',[1     ,0.6196,0.1059]);
            set(pPe  , 'YData',   Pe(N-1),'MarkerFaceColor',[1     ,0.6196,0.1059]); 
            else
            set(pwind, 'XData', time(i),'MarkerFaceColor',[1     ,0.6196,0.1059]);
            set(pwind, 'YData', wind(i),'MarkerFaceColor',[1     ,0.6196,0.1059]); 
            set(pPe  , 'XData', time(i),'MarkerFaceColor',[1     ,0.6196,0.1059]);
            set(pPe  , 'YData',   Pe(i),'MarkerFaceColor',[1     ,0.6196,0.1059]); 
            end               
        end
        pause(ts)
        drawnow
    end
end
    
