function chi = tangentialHeading(phaseNumber,waypoints)
    i=phaseNumber+1;

    if i == size(waypoints,1)
        deltaVector = -[waypoints.DistNorth(i-1) waypoints.DistEast(i-1)]+[waypoints.DistNorth(1) waypoints.DistEast(1)];
    else
        deltaVector = -[waypoints.DistNorth(i-1) waypoints.DistEast(i-1)]+[waypoints.DistNorth(i+1) waypoints.DistEast(i+1)];
    end
    chi = atan2(deltaVector(1),deltaVector(2));
end