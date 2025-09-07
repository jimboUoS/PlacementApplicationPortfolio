% geofence features
function [a,b,c] = geofenceToIneqConstraints(geofence)
    % geofence should be a table of cartesian geofence coordinates with
    % DistEast and DistNorth from RunwayCentre

    % fit the following inequality
    % ax + by + c < 0
    % where:
    % a = -deltaY
    % b = deltaX
    % c = -ax_1 - by_1
    % y is east
    % x is north
    
    a=zeros(1,size(geofence,1)-1);
    b=zeros(1,size(geofence,1)-1);
    c=zeros(1,size(geofence,1)-1);

    for i=2:size(geofence,1)
        a(1,i-1)=-geofence(i,:).DistEast+geofence(i-1,:).DistEast;
        b(1,i-1)=geofence(i,:).DistNorth-geofence(i-1,:).DistNorth;
        c(1,i-1)=-a(1,i-1)*geofence(i-1,:).DistNorth - b(1,i-1)*geofence(i-1,:).DistEast;
    end

end