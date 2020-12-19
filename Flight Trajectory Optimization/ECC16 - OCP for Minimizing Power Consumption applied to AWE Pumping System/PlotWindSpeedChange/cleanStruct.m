function arrayClean = cleanStruct(arrayDirty)
    % clear array from NaN replasing the middle value between the previous
    % and later value;
    L = length(arrayDirty);
    for i=1:L
        if isnan(arrayDirty(i))   % if the element contain NaN
            switch i              % evaluate in which part of the array we are
                case 1,           % case 1: first element of the array
                    arrayDirty(i) = arrayDirty(i+1);
                case L,           % case 2: last element of the array
                    arrayDirty(i) = arrayDirty(i-1);
                otherwise         % case Df: in the middle of the array
                    arrayDirty(i) = (arrayDirty(i-1)+arrayDirty(i+1))/2;
            end % end switch
        end  % end if
    end % end for
    arrayClean = arrayDirty;
end % end func