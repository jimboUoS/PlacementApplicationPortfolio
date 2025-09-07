function [transformedGuess, chi_f] = transformTrackAngleGuess(waypoints)
    %run an optimisation with no linkage constraints on track angle

    %then, figure out the smallest turn that can be taken between each
    %phase by adding multiples of 2pi

    %import cartesian coordinates
    chiArray = atan2(diff(waypoints.DistEast), diff(waypoints.DistNorth));
    chiArray=[chiArray;atan2(waypoints.DistEast(1)-waypoints.DistEast(end), waypoints.DistNorth(1)-waypoints.DistNorth(end))];
    
    %check delta between two entries
    %add 2pi, if greater delta then subtract 2pi from original delta
    %if greater, choose original delta else continue

    deltaArray = zeros(1, size(waypoints,1) - 1);
    for i = 1:size(waypoints,1)-1
        if i==size(waypoints,1)
            delta = chiArray(1) - chiArray(i);
        else
            delta = chiArray(i+1) - chiArray(i);
        end
        while abs(delta) > pi
            delta = delta - sign(delta) * 2 * pi;
        end
        deltaArray(1,i) = delta;
    end

    %transform deltas into a new guess for the track angle starting from
    %the original first track angle
    transformedGuess = zeros(1, size(waypoints,1));
    transformedGuess(1) = chiArray(1, 1); % Start with the first track angle
    transformedGuess(2:end) = transformedGuess(1) + cumsum(deltaArray);
    %transformedGuess is not ready to be run directly, have to turn into vector with
    %ones to get the correct dimension

    %determine optimal value for chif for final phase

    %chi_f - chiArray(1) = 2*pi*N where N is an integer
    %minimise |transformedGuess(nPhases) - chi_f|
    %choose N to minimise above delta
    %round function achieves this job by finding integer closest to setting
    %delta to 0

    %note chi_f isn't the exact value for the final value of chi, just an
    %approximation which narrows the region of values that satisfy the mod
    %linkage constraint
    chi_f = transformedGuess(1) - 2 * pi * round((transformedGuess(1) - transformedGuess(size(waypoints,1)))/(2*pi));
end