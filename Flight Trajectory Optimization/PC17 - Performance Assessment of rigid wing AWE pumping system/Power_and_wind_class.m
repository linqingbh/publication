clc;clear all;close all;
%% Wind class specifications
windrange_wanted=[0:0.1:25]';
href_wanted_windclasses = 50;   % Height of available data (Weibull)
h0_wanted_windclasses   = 0.05; % Roughness of available data (Weibull)
Vavg_1   = 0.2*50;%class 1A according to IEC61400 doc
Vavg_2   = 0.2*40;%class 2A according to IEC61400 doc
F_1a     = @(Vw,Vavg_1) 1-exp(-pi*(Vw/(2*Vavg_1))^2);
F_2a     = @(Vw,Vavg_2) 1-exp(-pi*(Vw/(2*Vavg_2))^2);
stepsize = windrange_wanted(2)-windrange_wanted(1);

for i = 1:1:length(windrange_wanted) 
     windprob_1A(i,1)           = (F_1a(windrange_wanted(i)+stepsize/2,Vavg_1)-F_1a(windrange_wanted(i)-stepsize/2,Vavg_1));
     windprob_2A(i,1)           = (F_2a(windrange_wanted(i)+stepsize/2,Vavg_2)-F_2a(windrange_wanted(i)-stepsize/2,Vavg_2));
end

windInput.windrange   = windrange_wanted;
windInput.href        = href_wanted_windclasses;
windInput.h0          = h0_wanted_windclasses;
% windInput.probability = windprob_1A;%;
% Wind_prob = windInput.probability;%


%% Plot info mechanical power: wind speed | Pmech | Probability of occurence 1A
Data = [0    -583.49;            
        0.5  -561.33;
        1    -524.49;
        1.5  -481.08;
        2    -367.01;
        2.5  -153.63;
        2.7      0   ;% Pmech = 0 
        3      128.7 ;% <--
        3.5    459.91;% <--
        4      923.60;% <--
        4.5   1376.06;% <--
        5     2177.18;% <--
        7     5708   ;% <--
        7.5   6486.04;% <--
        8     7040.2 ;% <--
        9     8198.82;% <--
        10    9610.73;% <--
        11    10714.87;% <--
        12    10898.00;% <--
        12.25 10898.5;
        12.5  10899  ;              % interp
        12.75 10898.5;
        13    10900  ;
        14    10920  ;
        15    10940  ;
        16    10960  ;
        17    11000  ;
        18    11000  ;
        19    11000  ;
        20    10950  ;
        25    10920  ];

% Pelec with constant efficency
efficency     = 0.94;
Data(1:6,2)   = (Data(1:6,2).*0.06 + Data(1:6,2)); 
Data(7:end,2) = Data(7:end,2).*efficency;

%%
DataWind =  Data(:,1);             % [m/s]
DataPmech = Data(:,2)./1e3;        % [kW]
% interpolation of mechanical power in function of wind speed    
PmechInt = interp1(DataWind,DataPmech,windrange_wanted,'spline'); % kW       
PmechInt(226:end) = 0;  % cut-off wind speed at 22.5 m/s.

CumSumPowerPercentage1A = sum(windprob_1A) % close to 1
CumSumPowerPercentage2A = sum(windprob_2A) % close to 1

PowerPercentage_1A      = windprob_1A.*PmechInt;  % [%]*[kW]
PowerPercentage_2A      = windprob_2A.*PmechInt;  % [%]*[kW]

PowerProductionPerYear_1A = PowerPercentage_1A*365*24        % kWh
PowerProductionPerYear_2A = PowerPercentage_2A*365*24        % kWh

capacityfactor_1A = sum(PowerPercentage_1A)/(max(DataPmech)) %  
capacityfactor_2A = sum(PowerPercentage_2A)/(max(DataPmech)) %  

totalEnergyUsed_1A = sum(PowerPercentage_1A(PowerPercentage_1A<0))*365*24 % kWh
totalEnergyUsed_2A = sum(PowerPercentage_2A(PowerPercentage_2A<0))*365*24 % kWh

totalEnergyHarvested_1A = sum(PowerPercentage_1A(PowerPercentage_1A>0))*365*24 % kWh
totalEnergyHarvested_2A = sum(PowerPercentage_2A(PowerPercentage_2A>0))*365*24 % kWh

totalEnergyProduced_1A  = sum(PowerPercentage_1A)*365*24                    % kWh
totalEnergyProduced_2A  = sum(PowerPercentage_2A)*365*24                    % kWh

test_1A = totalEnergyProduced_1A - (totalEnergyHarvested_1A + totalEnergyUsed_1A)
test_2A = totalEnergyProduced_2A - (totalEnergyHarvested_2A + totalEnergyUsed_2A)

cost_kWh = 0.05 ; % euro

relativeCost_1A = (-totalEnergyUsed_1A/totalEnergyHarvested_1A)*100  % relative cost in percentage
relativeCost_2A = (-totalEnergyUsed_2A/totalEnergyHarvested_2A)*100  % relative cost in percentage

% relative cost means the ratio between the energy consumption to keep the
% system aloft (during low wind conditions) and the total energy that can be harvested with the system.
absoluteCost_1A = -totalEnergyUsed_1A*cost_kWh
absoluteCost_2A = -totalEnergyUsed_2A*cost_kWh

profit_1A = totalEnergyProduced_1A*cost_kWh
profit_2A = totalEnergyProduced_2A*cost_kWh

AP4 = 0.4*2e3*365*24*0.005*0.05
AP4_money = 0.53*1800*365*24*0.05 % ??? 


%% Plotting
figure;
subplot(3,1,1);hold on;grid on;
h = area(windrange_wanted,PmechInt,...
    'FaceColor',[1 0.6196 0.1059],'FaceAlpha',.5,'EdgeAlpha',.5); 
h.FaceColor = [0.4 0.6 0.6];
ylabel('Electrical Power [kW]');ylim([-2 12]);
subplot(3,1,2);hold on;grid on;
w1A = area(windrange_wanted,windprob_1A./(windrange_wanted(2)-windrange_wanted(1)).*100,...
           'FaceColor',[1 0.6196 0.1059],'FaceAlpha',.3,'EdgeAlpha',.5);
w2A = area(windrange_wanted,windprob_2A./(windrange_wanted(2)-windrange_wanted(1)).*100,...
           'FaceColor',[0.1176,0.5804,0.8235],'FaceAlpha',.3,'EdgeAlpha',.5);
%w.FaceColor = [0,0.5,0.5];
ylabel('Probability of occurrence in %'); 
subplot(3,1,3);hold on;grid on;
P1A = area(windrange_wanted,PowerProductionPerYear_1A,...
           'FaceColor',[1 0.6196 0.1059],'FaceAlpha',.3,'EdgeAlpha',.5);
P2A = area(windrange_wanted,PowerProductionPerYear_2A,...
           'FaceColor',[0.1176,0.5804,0.8235],'FaceAlpha',.3,'EdgeAlpha',.5);
xlabel('wind speed [m/s]');ylabel('Power Production Per Year [kWh]');
ylim([-50 650]);


% RGB orange = [1 0.6196 0.1059];
% RGB blue = [0.1176,0.5804,0.8235];