function u_resampled = restoresSampling(u,nTimes)

u_resampled = zeros(length(u)*nTimes,1);
N = length(u_resampled);
j = 1;
  for i=1:nTimes:N
      u_resampled(i:i+nTimes-1) = u(j);
      j = j+1;
  end
end