validationSet      = PrepareSystemID;

%%
validationSeries = {};
for k = 1:length(validationSet)-1
  data = validationSet(k);
  times = data.ServoDemands.time';
  elevator = data.ServoDemands.packetData.elevator';
  port_aileron = data.ServoDemands.packetData.port_aileron';
  stbd_aileron = data.ServoDemands.packetData.stbd_aileron';
  rudder = data.ServoDemands.packetData.rudder';
  aileron = (stbd_aileron - port_aileron)/2.;

  wind = data.NavigationData.packetData.windSpeed;
  wind = [wind';zeros(1,length(wind))];

  positionNED         = data.NavigationData.packetData.positionNED';
  velocityNED         = data.NavigationData.packetData.velocityNED';
  bodyAngularVelocity = data.NavigationData.packetData.bodyRate';
  cosMatBodyToNav     = data.NavigationData.packetData.cosMatBodyToNav;
  velocityBody = zeros(3,length(times));
  for l=1:length(times)
    velocityBody(:,l) = cosMatBodyToNav(:,:,l)' * velocityNED(:,l);
  end
  rollPitchYaw = data.NavigationData.packetData.rollPitchYaw';


  %% return simulation data
  initialState = struct;
  initialState.p = positionNED(:,1);
  initialState.v = velocityNED(:,1);
  initialState.R = cosMatBodyToNav(:,:,1);
  initialState.w = bodyAngularVelocity(:,1);

  controls = struct;
  controls.times      = times(1:end-1) - times(1);
  controls.aileron    = aileron(1:end-1);
  controls.elevator   = elevator(1:end-1);
  controls.rudder     = rudder(1:end-1);
  controls.wind       = wind(:,1:end-1);

  response = struct;
  response.p = positionNED(:,2:end);
  response.v = velocityNED(:,2:end);
  response.R = cosMatBodyToNav(:,:,2:end);
  response.w = bodyAngularVelocity(:,2:end);
  response.velocityBody = velocityBody(:,2:end);
  response.rollPitchYaw = rollPitchYaw(:,2:end);

  validationData = struct;
  validationData.initialState = initialState;
  validationData.controls = controls;
  validationData.response = response;
  
  validationSeries = [validationSeries, validationData];
  
end

%%
data = validationSeries{3};

p = data.response.p;
R = data.response.R;

rotationNav = zeros(9,size(R,3));
for k=1:size(R,3)
  rotationNav(:,k) = reshape(R(:,:,k),9,1);
end

ts = 0.01;
viewStruct = CreatePlotView(ts,0.9);
UpdatePlotView( viewStruct, p, rotationNav)