function flightPlan = InitialiseFlightPlanOptimized(varargin)

%% process optional arguments
options.untetheredWaypointMode                                                = 'sysID8';
options.systemID_Mode                                                         = 1;
options.flightPlanName                                                        = ['FLIGHT_PLAN_FCC_',SentenceToCamelCase(GetLastFCC_Tag)];
options.locationName                                                          = 'Kraggenburg';
options.tetheredMission.WP_Table                                              = [];
options.tetheredMission.tetherReleaseWP                                       = [];
options.tetheredMission.missionType                                           = 'CIRCUIT_FOR_APPROACH';
options.tetheredMission.EAS                                                   = [];
options.tetheredMission.launchHeight                                          = [];
options.tetheredMission.altitude                                              = [];
options.tetheredTransition.WP_Table                                           = [];
options.tetheredTransition.transitionType                                     = 'FIXED_ALTITUDE';
options.tetheredTransition.altitude                                           = 100;
options.untetheredCircuit.EAS                                                 = [];
options.tetheredCircuit.EAS                                                   = [];
options.tetheredCircuit.method                                                = 'ALT';
options.tetheredCircuit.WP_Table                                              = [];
options.landingParameters.untetheredLandingPositionLLH                        = [52.6781093839515; 5.8616315056296; 39.4];
options.landingParameters.winchStandbyDistance                                = 150;
options.tetheredPowerGen.launchHeight                                         = [];
options.landingOffset                                                         = 0;
options.landingParameters.maxNumberOfGoArounds                                = 3;
options.takeoffDirection                                                      = 0;   % for untethered runway takeoff
options.runwayLength                                                          = 300; % for untethered runway takeoff
options.tetheredTransition.turnBackDistance                                   = [];
options.tetheredTransition.entryDistanceOffset                                = [];
options.takeoffClimbAutothrottleHeight                                        = 50;
options.reelinSpeed                                                           = 10;
options.winchAccelerationMin                                                  = -6;
options.winchAccelerationMax                                                  = 6;
options.useFixedAngleOfAttackOffset                                           = true;
options.windPatternSchedule                                                   = {};

options                                                                       = ProcessInputArguments(options, 'extractAll', varargin{:});

%%
flightPlan                                                                    = eval(ConvertBusToMATLAB_Struct('FlightPlan_BUS'));

flightPlan.flightPlanName                                                     = GenerateFlightPlanName(options.flightPlanName);
flightPlan.locationName                                                       = GenerateLocationName(options.locationName);

waypointSchedule                                                              = GeneratePatternSchedule('launchHeight', options.tetheredPowerGen.launchHeight, 'windPatternSchedule', options.windPatternSchedule);
AssignToStruct('flightPlan.waypointSchedule',                                 waypointSchedule);

AssignToStruct('flightPlan.flightPlanParameters.engineInstalled',             true);
AssignToStruct('flightPlan.flightPlanParameters.anchorHeightAboveReference',  GetParameter('anchorHeightAboveReference'));
AssignToStruct('flightPlan.flightPlanParameters.takeoffClimbAutothrottleHeight', options.takeoffClimbAutothrottleHeight);
AssignToStruct('flightPlan.flightPlanParameters.useFixedAngleOfAttackOffset', options.useFixedAngleOfAttackOffset);

%%
flightArea                                                                    = GenerateFlightArea;
AssignToStruct('flightPlan.flightPlanFlightArea',                             flightArea);
%% 
RTB_Sequences                                                                 = GenerateRTB_Waypoints;
AssignToStruct('flightPlan.RTB_Sequences',                                    RTB_Sequences);
%%
loiterSequence                                                                = GenerateLoiterWaypoints;
AssignToStruct('flightPlan.loiterSequence',                                   loiterSequence);
%%
untetheredWaypointSequence                                                    = GenerateUntetheredWaypointsOptimized('untetheredWaypointMode', options.untetheredWaypointMode, 'systemID_Mode', options.systemID_Mode);
AssignToStruct('flightPlan.untetheredWaypointSequence',                       untetheredWaypointSequence);
%%
tetheredWaypointSequence                                                      = GenerateTetheredWaypoints('WP_Table', options.tetheredMission.WP_Table, 'tetherReleaseWP', options.tetheredMission.tetherReleaseWP, 'missionType', options.tetheredMission.missionType, 'EAS', options.tetheredMission.EAS, 'launchHeight', options.tetheredMission.launchHeight, 'altitude', options.tetheredMission.altitude);
AssignToStruct('flightPlan.tetheredWaypointSequence',                         tetheredWaypointSequence);
%%
tetheredTransitionSequence                                                    = GenerateTetheredTransitionWaypoints('WP_Table', options.tetheredTransition.WP_Table, 'transitionType', options.tetheredTransition.transitionType, 'turnBackDistance', options.tetheredTransition.turnBackDistance, 'entryDistanceOffset', options.tetheredTransition.entryDistanceOffset, 'altitude', options.tetheredTransition.altitude);
AssignToStruct('flightPlan.tetheredTransitionSequence',                       tetheredTransitionSequence);
%%
AssignToStruct('flightPlan.landingParameters.untetheredLandingPositionLLH',   options.landingParameters.untetheredLandingPositionLLH );
AssignToStruct('flightPlan.landingParameters.landingPositionRelativeToAnchor',[options.landingOffset; 0; 0] );
AssignToStruct('flightPlan.landingParameters.aimPointOffset',                 40 );
AssignToStruct('flightPlan.landingParameters.overrideManualCircuitEAS',       true );
AssignToStruct('flightPlan.landingParameters.circuitEAS',                     20 );
AssignToStruct('flightPlan.landingParameters.approachEAS',                    18 );
AssignToStruct('flightPlan.landingParameters.approachFlapsSetting',           30/35 );
AssignToStruct('flightPlan.landingParameters.flapsRetractHeight',             3 );
AssignToStruct('flightPlan.landingParameters.useApproachSpeedBrakes',         true );
AssignToStruct('flightPlan.landingParameters.flareMode',                      E_FLARE_MODE.PRECISION );
AssignToStruct('flightPlan.landingParameters.liftDumpSetting',                1 );
AssignToStruct('flightPlan.landingParameters.winchStandbyDistance',           options.landingParameters.winchStandbyDistance );
AssignToStruct('flightPlan.landingParameters.maxNumberOfGoArounds',           options.landingParameters.maxNumberOfGoArounds );

%%
landingArea                                                                   = GenerateLandingArea;
AssignToStruct('flightPlan.landingArea',                                      landingArea);
%%
circuitWaypointSequence                                                       = GenerateCircuitWaypoints('EAS', options.untetheredCircuit.EAS);
AssignToStruct('flightPlan.circuitWaypointSequence',                          circuitWaypointSequence);
%%
tetheredCircuitWaypointSequence                                               = GenerateTetheredCircuitWaypoints('WP_Table', options.tetheredCircuit.WP_Table, 'EAS', options.tetheredCircuit.EAS, 'landingOffset', flightPlan.landingParameters.landingPositionRelativeToAnchor(1), 'method', options.tetheredCircuit.method);
AssignToStruct('flightPlan.tetheredCircuitWaypointSequence',                  tetheredCircuitWaypointSequence);

systemIdentificationData                                                      = GenerateSystemIdentificationDataOptimized;
AssignToStruct('flightPlan.systemIdentificationData',                         systemIdentificationData);

untetheredTakeoffSequences                                                    = GenerateUntetheredTakeoffWaypoints('takeoffDirection', options.takeoffDirection, 'runwayLength', options.runwayLength);
AssignToStruct('flightPlan.untetheredTakeoffSequences',                       untetheredTakeoffSequences);

systemSyncData                                                                = GenerateSystemSyncData('reelinSpeed', options.reelinSpeed, 'winchAccelerationMin', options.winchAccelerationMin, 'winchAccelerationMax', options.winchAccelerationMax);
AssignToStruct('flightPlan.systemSyncData',                                   systemSyncData);

if nargout < 1
  assignin('base','flightPlan',flightPlan);
end

end

function flightPlanName = GenerateFlightPlanName(nameIn)

flightPlanName = uint8(zeros(255,1));
flightPlanName(1:numel(nameIn)) = uint8(nameIn);

end

function locationName = GenerateLocationName(nameIn)

locationName = uint8(zeros(32,1));
locationName(1:numel(nameIn)) = uint8(nameIn);

end
