function Exp = getExpreriment(i,systemID_Data,IndexIn,IndexOut,showPlots)
Exp = struct;
% Get flight test ======================================================  
sampleTime  = systemID_Data(1).ServoDemands.time(2) - systemID_Data(1).ServoDemands.time(1);  
Exp.N       = length(systemID_Data(i).ServoDemands.packetData.elevator(IndexIn:IndexOut));    
Exp.time    = [0:sampleTime:(Exp.N-1)*sampleTime]';  % time array  for plot

% get Control Surfaces ------------------------------------------- [rad]
Exp.dda_Port = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.portAileron(IndexIn:IndexOut);
Exp.dda_Stbd = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.stbdAileron(IndexIn:IndexOut);
Exp.dde      = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.portElevator(IndexIn:IndexOut);

Exp.dde(1)   = [];
Exp.dde = [Exp.dde;Exp.dde(end)];

Exp.ddr      = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.rudder(IndexIn:IndexOut);
Exp.ddt      = systemID_Data(i).processed.thrust(IndexIn:IndexOut);

% get aero States ------------------------------------------------------
Exp.alpha    = systemID_Data(i).processed.alphaFiltered(IndexIn:IndexOut);    %  
Exp.V        = systemID_Data(i).processed.TAS_Filtered(IndexIn:IndexOut);     % [m/s]   
Exp.qbar     = systemID_Data(i).processed.qbarFiltered(IndexIn:IndexOut);

% get body angural rate and accel--------------------------------------
Exp.p        = systemID_Data(i).processed.angularRate((IndexIn:IndexOut),1); % [wx,wy,wz] = [p,q,r] [rad/s]
Exp.q        = systemID_Data(i).processed.angularRate((IndexIn:IndexOut),2); % [wx,wy,wz] = [p,q,r] [rad/s]
Exp.r        = systemID_Data(i).processed.angularRate((IndexIn:IndexOut),3); % [wx,wy,wz] = [p,q,r] [rad/s]

wind         = sqrt(systemID_Data(i).processed.windNED((IndexIn:IndexOut),1).^2 + ...
                    systemID_Data(i).processed.windNED((IndexIn:IndexOut),2).^2 + ...
                    systemID_Data(i).processed.windNED((IndexIn:IndexOut),2).^2);

if strcmp(showPlots,'Yes')
figure;hold on;
subplot(5,1,1);title(['System Identification ',num2str(i)]);
               hold on;grid on;ylabel('\delta_{t} [%]');
               plot(Exp.time,Exp.ddt,'LineWidth',1.5,'Color','b');
subplot(6,1,2);hold on;grid on;ylabel('q [deg/s]');
               plot(Exp.time,rad2deg(Exp.q),'LineWidth',1.5,'Color','b');
subplot(6,1,3);hold on;grid on;ylabel('\delta_{e} [deg]');
               plot(Exp.time,rad2deg(Exp.dde),'LineWidth',1.5,'Color','b');
subplot(6,1,4);hold on;grid on;ylabel('\alpha [deg]');
               plot(Exp.time,rad2deg(Exp.alpha),'LineWidth',1.5,'Color','b');
subplot(6,1,5);hold on;grid on;ylabel('V [m/s]');
               plot(Exp.time,Exp.V,'LineWidth',1.5,'Color','b');
subplot(6,1,6);hold on;grid on;ylabel('W [m/s]');
               plot(Exp.time,wind,'LineWidth',1.5,'Color','b');
               xlabel('time [s]');
               
               
figure;
subplot(2,2,1);title(['System Identification ',num2str(i)]);
               hold on;grid on;ylabel('p [deg/s]');
               plot(Exp.time,rad2deg(Exp.p),'LineWidth',1.5,'Color','b');
subplot(2,2,2);hold on;grid on;ylabel('r [deg/s]');
               plot(Exp.time,rad2deg(Exp.r),'LineWidth',1.5,'Color','b');                           
subplot(2,2,3);hold on;grid on;ylabel('\delta_{a} [deg]');
               plot(Exp.time,rad2deg(Exp.dda_Port),'LineWidth',1.5,'Color','b');
               plot(Exp.time,rad2deg(Exp.dda_Stbd),'LineWidth',1.5,'Color','r');
               legend('\delta_{a} Port','\delta_{a} Stbd');
               xlabel('time [s]');
subplot(2,2,4);hold on;grid on;ylabel('\delta_{r} [deg]');
               plot(Exp.time,rad2deg(Exp.ddr),'LineWidth',1.5,'Color','b');
               xlabel('time [s]');
               
end


end