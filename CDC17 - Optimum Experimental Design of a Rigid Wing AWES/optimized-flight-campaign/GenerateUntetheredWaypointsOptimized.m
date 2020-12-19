function untetheredWaypointSequence = GenerateUntetheredWaypointsOptimized(varargin)

options.untetheredWaypointMode             = 'normal';
options.systemID_Mode                      = 1;

options                                    = ProcessInputArguments(options, 'extractAll', varargin{:});

untetheredWaypointSequence                 = eval(ConvertBusToMATLAB_Struct('UntetheredWaypointSequence_BUS'));

switch options.untetheredWaypointMode
  case 'normal'
    speed = 20;
    radius = 100;
    rotationAngle = 60/180*pi;
    rotationMatrix = [cos(rotationAngle) -sin(rotationAngle) 0;
      sin(rotationAngle)   cos(rotationAngle) 0;
      0                   0                  1];
    
    wp = ConstructFigureEight(600, 0*pi/180, radius)';
    WP_Table = [0,0,0; wp(2:end,:); wp; wp; wp(1,:); 0,0,0];
    WP_Table = transpose(rotationMatrix * WP_Table')-ones(size(WP_Table,1),1)*[300 0 0];
    WP_Table(:,3) = -150;
    
    
  case 'large8'
    speed = 20;
    radius = 200;
    
    % Large Figure 8
    wp = ConstructFigureEight(1300, 0*pi/180, radius)';
    WP_Table = [0,0,0; wp(2:end,:); wp; wp; wp(1,:); 0,0,0];
    WP_Table(:,1) = WP_Table(:,1) - 250;
    WP_Table(:,2) = WP_Table(:,2) + 125;
    WP_Table(:,3) = -150;
    
  case 'racetrack'
    speed = 20;
    radius = 200;
    downPosition = -150;
    
    
    WP_Table = [0,        0,        downPosition; ...
      0,        3*radius, downPosition; ...
      -2*radius, 3*radius, downPosition; ...
      -2*radius, 0,        downPosition; ...
      0,        0,        downPosition; ...
      0,        3*radius, downPosition; ...
      -2*radius, 3*radius, downPosition; ...
      -2*radius, 0,        downPosition; ...
      ];
    
  case 'teardrop'
    speed = 20;
    radius = 100;
    downPosition = -150;
    WP_Table = ConstructTearDropReversal(radius,10);
    WP_Table(:,3) = downPosition;
    
  case 'sysID8'
    rotationAngle = 60/180*pi;
    rotationMatrix = [cos(rotationAngle) -sin(rotationAngle) 0;
      sin(rotationAngle)   cos(rotationAngle) 0;
      0                   0                  1];
    
  % Large Figure 8
  switch options.systemID_Mode
    case 1
      speed  = 20;
      radius = 100;
    case 2
      speed = 18;
      radius = 100;
    case 3
      speed = 25;
      radius = 140;
  end

  % Large Figure 8
  wp                            = ConstructFigureEight(700, 60*pi/180, radius, 80)';
  WP_Table                      = [0,0,0; wp(3:end,:); wp; wp; wp(1,:); 0,0,0];
  WP_Table(:,1)                 = WP_Table(:,1)-150;
  WP_Table(:,2)                 = WP_Table(:,2)-90;
  WP_Table(:,3)                 = -150;    
    
  otherwise
    error('Invalid untethered waypoint generation mode');
end

% convert from NED to LLH
WP_TableLLH = zeros(size(WP_Table,1),3);
for i = 1:size(WP_Table,1)
  WP_TableLLH(i,:) = NED2LLH_Spherical(WP_Table(i,1:3)', GetParameter('anchorPositionLLH'));
end

AssignToStruct('untetheredWaypointSequence.launchCompleteHeight', 150);
AssignToStruct('untetheredWaypointSequence.launchCompleteClimbRate', 2);

AssignToStruct('untetheredWaypointSequence.numberOfWaypoints', size(WP_TableLLH,1));
for i = 1:size(WP_Table,1)
  if size(WP_Table,2)>3
    radius = WP_Table(i,4);
  end
  AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(',num2str(i),').waypointType'], E_WAYPOINT_TYPE.NORMAL);
  AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(',num2str(i),').waypointPositionLLH.latitude'], WP_TableLLH(i,1));
  AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(',num2str(i),').waypointPositionLLH.longitude'], WP_TableLLH(i,2));
  AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(',num2str(i),').waypointPositionLLH.height'], WP_TableLLH(i,3));
  AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(',num2str(i),').turnRadius'], radius);
  AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(',num2str(i),').speedDemand'], speed);
end

% turn system identification on
switch options.systemID_Mode
  case 1 % Vtrim = 20 [m/s]
    AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(4).systemIdenticationTestIndex'] , 1); % Elevator optimized
    AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(7).systemIdenticationTestIndex'] , 2); % Aileron optimized
    AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(10).systemIdenticationTestIndex'], 3); % Rudder optimized
    %AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(13).systemIdenticationTestIndex'], 20); % elevator 3211 (2)
    %AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(16).systemIdenticationTestIndex'], 4); % elevator 3211 (1.5)
  case 2 % Vtrim = 14 [m/s] max AoA
    AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(4).systemIdenticationTestIndex'] , 4); % Elevator optimized
  case 3 % Vtrim = 28 [m/s] min AoA
    AssignToStruct(['untetheredWaypointSequence.untetheredWaypoints(4).systemIdenticationTestIndex'] , 5); % Elevator optimized
end

end