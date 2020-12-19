%%  Prepare AP2 Flight Test for System Identification
%   Author:      Marie Curie PhD student Giovanni Licitra

StatusUpdate('Initialising Untethered Mission');
SetSimulationForUntetheredMission;    

windConditions.Value.windSpeed.variableParameter.value                  = 0; % [m/s]
windConditions.Value.turbulence.turbulenceLevel.variableParameter.value = 0;
StatusUpdate(['Setting wind speed at = ',...
  num2str(windConditions.Value.windSpeed.variableParameter.value),' [m/s]']);

% innerLoopsUseLQR.Value = false;
% StatusUpdate('Disable LQR');

OnboardPowerParameters.Value.infinitePower;
StatusUpdate('Set infinity charge Battery');

IMU_ModelParameters.Value.useErrors = false;
StatusUpdate('Set measurements Noise free');

flightPlan = InitialiseFlightPlanAP2;
StatusUpdate('Flight Plane for AP2 loaded');

% StatusUpdate('Boundaries for flight tests during System Identification');
% innerLoopsParameters.Value.systemIdentificationLimits
% StatusUpdate('NB: If the aircraft hit the limits, the system ID test will abort');

% Compute flight test
% open SimPowerPlane
% run  SimPowerPlane
% StatusUpdate('Flight test Computed!!!');
% LogViewer
% Store Data in a Structure
%systemID_Data      = PrepareSystemID;