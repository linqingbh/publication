clc;clear all;close all;
%% Wind class specifications
windrange_wanted=[0:0.1:25]';
href_wanted_windclasses=50;%Height of available data (Weibull)
h0_wanted_windclasses=0.05;%Roughness of available data (Weibull)
Vavg_1=0.2*50;%class 1A according to IEC61400 doc
Vavg_2=0.2*40;%class 2A according to IEC61400 doc
F_1a=@(Vw,Vavg_1) 1-exp(-pi*(Vw/(2*Vavg_1))^2);
F_2a=@(Vw,Vavg_2) 1-exp(-pi*(Vw/(2*Vavg_2))^2);
stepsize=windrange_wanted(2)-windrange_wanted(1);

%Terowie @ 100 m height
 href_wanted_Terowie=100;%Height of available data (Weibull)
 h0_wanted_Terowie=0.21;%Roughness of available data see "Updated AP3 Prospects, wind data comparison"
 alpha_Terowie=9.28;
 betha_Terowie=2.41;
 F_Terowie=@(Vw,alpha,betha) 1-exp(-((Vw/alpha)^betha));
 
%Kraggenburg @ 80m! Estimate from wind guru
alpha_Kraggenburg=7.046;
betha_Kraggenburg=2.3844;
F_Kraggenburg=@(Vw,alpha,betha) 1-exp(-((Vw/alpha)^betha));

for i = 1:1:length(windrange_wanted) 
     windprob_1A(i,1)           = (F_1a(windrange_wanted(i)+stepsize/2,Vavg_1)-F_1a(windrange_wanted(i)-stepsize/2,Vavg_1));
     windprob_2A(i,1)           = (F_2a(windrange_wanted(i)+stepsize/2,Vavg_2)-F_2a(windrange_wanted(i)-stepsize/2,Vavg_2));
     windprob_Terowie(i,1)      = (F_Terowie(windrange_wanted(i)+stepsize/2,alpha_Terowie,betha_Terowie)-F_Terowie(windrange_wanted(i)-stepsize/2,alpha_Terowie,betha_Terowie));
     windprob_Kraggenburg(i,1)  = (F_Kraggenburg(windrange_wanted(i)+stepsize/2,alpha_Kraggenburg,betha_Kraggenburg)-F_Terowie(windrange_wanted(i)-stepsize/2,alpha_Kraggenburg,betha_Kraggenburg));
end

figure;
plot(windprob_2A);hold on;
plot(windprob_1A);

windInput.windrange   = windrange_wanted;
windInput.href        = href_wanted_windclasses;
windInput.h0          = h0_wanted_windclasses;
windInput.probability = windprob_2A;%;
Wind_prob = windInput.probability;%

%% Plot info mechanical power: wind speed | Pmech | Probability of occurence
Data = [0    -583.49    0.1227;% <--
        0.5  -561.33    1.2234;% <--
        1    -524.49    2.4244;% <--
        1.5  -481.08    3.5813;% <--
        2    -367.01    4.6736;% <--
        2.5  -153.63    5.6829;% <--
        2.7     0       6.0597;      % Pmech = 0 
        3     128.7     6.5931;% <--
        3.5   459.91    7.3912;% <--
        4     923.60    8.0672;% <--
        4.5   1376.06   8.6144;% <--
        5     2177.18           9.0296;% <--
        7     5708              9.4164;% <--
        7.5   6486.04           9.2302;% <--
        8     7040.2            8.9523;% <--
        9     8198.82           8.1749;% <--
        10    9610.73           7.1942;% <--
        11    10714.87          6.1158;% <--
        12    10898             5.0311;% <--
        12.25 0.5*(10899+10898) 4.7677;
        12.5  10899             4.5092;              % interp
        12.75 0.5*(10900+10899) 4.2566;
        13    10900             4.0104;
        14    10920             3.1008;
        15    10940             2.3274;
        16    10960             1.6970;
        17    11000             1.2027;
        18    11000             0.8288;
        19    11000             0.5555;
        20    10950             0.3624
        25    10920             0];


efficency = 0.94;
Data(1:6,2)   = (Data(1:6,2).*0.06 + Data(1:6,2)); 
Data(7:end,2) = Data(7:end,2).*efficency; 
%%
      
DataWind =  Data(:,1);             % [m/s]
DataPmech = Data(:,2)./1e3;        % [kW]
% interpolation of mechanical power in function of wind speed    
PmechInt = interp1(DataWind,DataPmech,windrange_wanted,'spline'); % kW       
PmechInt(226:end) = 0;                     % cut-off wind speed at 22.5 m/s.

CumSumPowerPercentage = sum(Wind_prob); % close to 1

PowerPercentage = Wind_prob.*PmechInt;  % [%]*[kW]

PowerProductionPerYear = PowerPercentage*365*24             % kWh
capacityfactor = sum(PowerPercentage)/(max(DataPmech))      %  

totalEnergyUsed      = sum(PowerPercentage(PowerPercentage<0))*365*24 % kWh
totalEnergyHarvested = sum(PowerPercentage(PowerPercentage>0))*365*24 % kWh
totalEnergyProduced  = sum(PowerPercentage)*365*24                    % kWh

test = totalEnergyProduced - (totalEnergyHarvested + totalEnergyUsed)

cost_kWh = 0.05 ; % euro

relativeCost = (-totalEnergyUsed/totalEnergyHarvested)*100  % relative cost in percentage
% relative cost means the ratio between the energy consumption to keep the
% system aloft (during low wind conditions) and the total energy that can be harvested with the system.
absoluteCost = -totalEnergyUsed*cost_kWh
profit = totalEnergyProduced*cost_kWh

AP4 = 0.4*2e3*365*24*0.005*0.05
AP4_money = 0.53*1800*365*24*0.05 % ??? 

%% Plotting
figure;subplot(3,1,1);hold on;grid on;
h = area(windrange_wanted,PmechInt);        % x-axis [m/s], y-axis [kW]
h.FaceColor = [0.8 0.8 0.8];
stem(Data(1,1),Data(1,2)./1e3,'LineWidth',1,'Color','k','LineStyle','none','Marker','s','MarkerFaceColor','k');
stem(Data(7,1),Data(7,2)./1e3,'LineWidth',1,'Color','k','LineStyle','none','Marker','s','MarkerFaceColor','k');
ylabel('Mechanical Power [kW]');ylim([-2 12]);xlim([0,23.5]);
xlabel('wind speed [m/s]');

subplot(3,1,2);hold on;grid on;
w = area(windrange_wanted,Wind_prob./(windrange_wanted(2)-windrange_wanted(1)).*100);
w.FaceColor = [0.8 0.8 0.8];
ylabel('Probability of occurrence in %');xlim([0,23.5]);
xlabel('wind speed [m/s]');

subplot(3,1,3);hold on;grid on;
P1 = area(windrange_wanted(1:28),PowerProductionPerYear(1:28));
P1.FaceColor = [0.1176,0.5804,0.8235];
P2 = area(windrange_wanted(29:end),PowerProductionPerYear(29:end));
P2.FaceColor = [0.9608  0.6118  0];
xlabel('wind speed [m/s]');ylabel('Power Production Per Year [kWh]');
ylim([-50 650]);xlim([0,23.5]);




