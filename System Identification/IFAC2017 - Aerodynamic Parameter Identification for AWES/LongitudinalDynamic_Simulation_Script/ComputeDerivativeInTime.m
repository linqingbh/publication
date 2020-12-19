function derivativeInTime = ComputeDerivativeInTime(derivativeTable,alphaTable,alphaInTime)
    % interpolate alphaTable for get a smoother graph
    numberOfNewGrid  = 250;
    oldGrid          = linspace(0,1,length(alphaTable));
    newGrid          = linspace(0,1,numberOfNewGrid);
    alphaTable       = interp1(oldGrid,alphaTable,newGrid);
    derivativeTable  = interp1(oldGrid,derivativeTable,newGrid);
    
    derivativeInTime = zeros(length(alphaInTime),1);

    for i=1:length(alphaInTime)
            index = find(alphaTable < alphaInTime(i));
            derivativeInTime(i) =  derivativeTable(index(end));
    end
end