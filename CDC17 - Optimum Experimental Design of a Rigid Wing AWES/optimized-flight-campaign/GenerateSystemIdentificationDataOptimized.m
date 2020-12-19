function systemIdentificationData = GenerateSystemIdentificationDataOptimized()

%% TO DO: fix aileron rudder and option last trim 

systemIdentificationData                 = eval(ConvertBusToMATLAB_Struct('SystemIdentificationData_BUS'));

systemIdentificationData.numberOfSystemIdentificationTests = 5;

% Optimized Elevator (Vtrim = 20 [m/s])
systemIdentificationData.systemIdentificationTests(1).elevator.testType   = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TIME_BASED);
systemIdentificationData.systemIdentificationTests(1).elevator.testIndex  = 1;
systemIdentificationData.systemIdentificationTests(1).aileron.testType    = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);
systemIdentificationData.systemIdentificationTests(1).rudder.testType     = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);
systemIdentificationData.systemIdentificationTests(1).throttle.testType   = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.ZERO);

% Optimized Aileron (Vtrim = 20 [m/s]) 
systemIdentificationData.systemIdentificationTests(2).elevator.testType   = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);
systemIdentificationData.systemIdentificationTests(2).aileron.testType    = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TIME_BASED);
systemIdentificationData.systemIdentificationTests(2).aileron.testIndex   = 2;
systemIdentificationData.systemIdentificationTests(2).rudder.testType     = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);
systemIdentificationData.systemIdentificationTests(2).throttle.testType   = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);

% Optimized Rudder (Vtrim = 20 [m/s])
systemIdentificationData.systemIdentificationTests(3).elevator.testType   = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);
systemIdentificationData.systemIdentificationTests(3).aileron.testType    = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);
systemIdentificationData.systemIdentificationTests(3).rudder.testType     = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TIME_BASED);
systemIdentificationData.systemIdentificationTests(3).rudder.testIndex    = 3;
systemIdentificationData.systemIdentificationTests(3).throttle.testType   = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.ZERO);

% Optimized Elevator (Vtrim = 14 [m/s] - max AoA)
systemIdentificationData.systemIdentificationTests(4).elevator.testType   = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TIME_BASED);
systemIdentificationData.systemIdentificationTests(4).elevator.testIndex  = 4;
systemIdentificationData.systemIdentificationTests(4).aileron.testType    = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);
systemIdentificationData.systemIdentificationTests(4).rudder.testType     = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);
systemIdentificationData.systemIdentificationTests(4).throttle.testType   = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.ZERO);

% Optimized Elevator (Vtrim = 28 [m/s] - min AoA)
systemIdentificationData.systemIdentificationTests(5).elevator.testType   = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TIME_BASED);
systemIdentificationData.systemIdentificationTests(5).elevator.testIndex  = 5;
systemIdentificationData.systemIdentificationTests(5).aileron.testType    = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);
systemIdentificationData.systemIdentificationTests(5).rudder.testType     = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.TRIM);
systemIdentificationData.systemIdentificationTests(5).throttle.testType   = CastEnumerate(E_SYSTEM_IDENTIFICATION_TYPE.ZERO);

%% ------------------------------------------------------------------------
systemIdentificationData.numberOfSystemIdentificationTimeTests = 0;

% add test for elevator, test index 1
systemIdentificationData = AddOptimalTest(systemIdentificationData, 1, 5*pi/180, 1.0, 1.0,'longitudinal');
% add test for aileron , test index 2
systemIdentificationData = AddOptimalTest(systemIdentificationData, 2, 5*pi/180, 2.0, 1.0,'lateral');
% add test for rudder  , test index 3
systemIdentificationData = AddOptimalTest(systemIdentificationData, 3, 5*pi/180, 1.0, 1.0,'lateral');
% add test for elevator, test index 4
systemIdentificationData = AddOptimalTest(systemIdentificationData, 5, 5*pi/180, 1.0, 1.0,'longitudinal');
% add test for elevator, test index 5
systemIdentificationData = AddOptimalTest(systemIdentificationData, 5, 5*pi/180, 1.0, 0.1,'longitudinal');

end

function timeTest = AddPointToTimeTest(time, deflection, timeTest)
timeTest.numberOfPoints = timeTest.numberOfPoints + 1;
timeTest.time(timeTest.numberOfPoints) = time;
timeTest.deflection(timeTest.numberOfPoints) = deflection;

end

function systemIdentificationData = AddOptimalTest(systemIdentificationData, testCase, amplitude, startTime, holdTime,dynamics)

[time, deflection] = GetOptimalTestInput(testCase, amplitude, startTime, holdTime);

systemIdentificationData.numberOfSystemIdentificationTimeTests = systemIdentificationData.numberOfSystemIdentificationTimeTests + 1;
n = systemIdentificationData.numberOfSystemIdentificationTimeTests;

if numel(time)~=numel(deflection)
  error('Dimensions of the time and deflection do not agree');
end

for i = 1:numel(time)
  systemIdentificationData.systemIdentificationTimeTests(n) = AddPointToTimeTest(time(i), deflection(i), systemIdentificationData.systemIdentificationTimeTests(n));
end

  switch dynamics;
    case 'longitudinal'
    systemIdentificationData.systemIdentificationTimeTests(n).meanType = CastEnumerate(E_MEAN_TYPE.LAST_TRIM); % Longitudinal Dynamics
    case 'lateral'
    systemIdentificationData.systemIdentificationTimeTests(n).meanType = CastEnumerate(E_MEAN_TYPE.ZERO); % lateral dynamic
  end
end

function [time, deflection] = GetOptimalTestInput(testCase, amplitude, startTime, holdTime)

switch testCase
  case 1
  %% Optimized maneuver \Delta\theta = 5% (from CD17: Optimum Experimental Design of a Rigid Wing AWE Pumping System)
  % Setting:
  % Vtrim = 20 [m/s]
  % df    = 0  [deg]
  % de    = input [-5 +5]

  time       = [ 0.0,  0.1, 0.6, 0.7, 1.2, 1.3, 1.7, 1.8, 2.7, 2.8, 3.0,...
                       3.1, 3.8, 3.9, 4.6, 4.7, 5.1, 5.2, 5.8, 5.9, 6.5,...
                       6.6, 7.0, 7.1, 7.8, 7.9, 8.3, 8.4, 9.0, 9.1, 9.5,...
                       9.6, 10.0, 10.1];
  N_bang_bang = 8;
  bang_bang   = [-1.0, -1.0, 1.0 , 1.0];
  deflection  = [];
  for i=1:N_bang_bang
    deflection = [deflection,bang_bang];
  end       
  deflection = [0,deflection,0];
       
  case 2
  %% Optimized maneuver \Delta\theta = 5% lateral dynamics
  % Setting:
  % Vtrim = 20 [m/s]
  % df    = 0  [deg]
  % da    = input  [-5 +5]
  % dr    = trimmed;
  % Original optimized maneuver 
  %   time       = [0.0, 0.1, 0.4, 0.5, 0.9, 1.0, 1.7, 1.8, 2.2, 2.3, 2.9, 3.0, 3.6,...
  %                 3.7, 4.5, 4.6, 5.1, 5.2, 5.9, 6.0, 6.7, 6.8, 7.5, 7.6, 8.1, 8.2,...
  %                 8.9, 9.0, 9.4, 9.5, 10.0, 10.1];    
  %   deflection = [0.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0,...
  %                 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0,...
  %                 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 0.0];
   time       = [0.0, 0.1, 0.4, 0.5, 0.9, 1.0, 1.7, 1.8, 2.2, 2.3];    
   deflection = [0.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0, 0.0];
    
  case 3
  %% Optimized maneuver \Delta\theta = 5% lateral dynamics
  % Setting:
  % Vtrim = 20 [m/s]
  % df    = 0  [deg]
  % da    = trimmed
  % dr    = input  [-5 +5]

  time = [0.0, 0.1, 2.7, 2.8, 3.5, 3.6, 5.2, 5.3, 6.0, 6.1, 7.7, 7.8,...
          8.3, 8.4, 8.6, 8.7, 9.0, 9.1, 9.7, 9.8, 10.0, 10.1];

  N_bang_bang = 5;
  bang_bang   = [-1.0, -1.0, 1.0, 1.0];
  deflection  = [];
  for i=1:N_bang_bang
    deflection = [deflection,bang_bang];
  end       
  deflection = [0,deflection,0];

  case 4
  %% Optimized maneuver \Delta\theta = 5% (max angle of attack)
  % Setting:
  % Vtrim = 14 [m/s]
  % df    = 0  [deg]
  % de    = input  [-5 +5]

  time       = [0.0, 0.1, 0.9, 1.0, 1.5, 1.6, 1.9, 2.0, 2.4, 2.5, 4.2, 4.3,...
                5.6, 5.7, 6.0, 6.1, 7.1, 7.2, 8.2, 8.3, 9.1, 9.3, 10.0, 10.1];
  deflection = [0.0,-1.0,-1.0,-0.4,-0.4, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,...
               -1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0, 0.0];  
  
  case 5
  %% Optimized maneuver \Delta\theta = 5% (min angle of attack)
  % Setting:
  % Vtrim = 28 [m/s]
  % df    = 0  [deg]
  % de    = input  [-5 +5]

%   time = [0.0, 0.1, 0.3, 0.4, 0.6, 0.7, 1.0, 1.1, 1.3, 1.4, 1.7, 1.8, 2.1,...
%           2.2, 2.6, 2.7, 3.0, 3.1, 3.4, 3.5, 3.8, 3.9, 4.2, 4.3, 4.6, 4.7,...
%           5.1, 5.2, 5.6, 5.7, 6.0, 6.1, 6.4, 6.5, 6.8, 6.9, 7.2, 7.3, 7.7,...
%           7.8, 8.0, 8.1, 9.3, 9.4, 9.6, 9.7, 10.0, 10.1];
% 
%   deflection = [ 0.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,...
%                 -1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,...
%                 -1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,...
%                 -1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0, 0.0];
  time = [0.0, 0.1, 0.3, 0.4, 0.6, 0.7, 1.0, 1.1, 1.3, 1.4, 1.7, 1.8, 2.1,...
          2.2, 2.6, 2.7, 3.0, 3.1, 3.4, 3.5, 3.8, 3.9, 4.2, 4.3, 4.6, 4.7,...
          5.1, 5.2, 5.6, 5.7, 6.0, 6.1, 6.4, 6.5, 6.8, 6.9, 7.2, 7.3, 7.7,...
          7.8, 8.0, 9.1];

  deflection = [ 0.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,...
                -1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,...
                -1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,-1.0, 1.0, 1.0,-1.0,...
                -1.0, 1.0, 1.0,-1.0,-1.0, 0.0];

  
  otherwise
    error('Invalid test case');
end

time = [0 time+startTime time(end)+holdTime+startTime];
deflection = [0 deflection 0] * amplitude;

end

