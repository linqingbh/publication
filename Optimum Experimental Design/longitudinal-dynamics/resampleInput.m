function u_resampled = resampleInput(u,nTimes)

u_resampled = zeros(length(u)*nTimes,1);
N = length(u);
j = 1;
  for i=1:1/nTimes:N
    u_resampled(j) = u(i);
    j = j+1;
  end

end