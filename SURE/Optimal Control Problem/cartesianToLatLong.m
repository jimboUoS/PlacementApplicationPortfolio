function [latLongWaypoints] = cartesianToLatLong(origin, cartesianWaypoints)
    %formulae from https://www.movable-type.co.uk/scripts/latlong.html
    %Great Circle distance, some inaccuracy on the order of a few metres
    
    %Earth radius
    R=6371*10^3;
    latLongWaypoints = table('Size',size(cartesianWaypoints),'VariableTypes', ["double", "double"],'VariableNames',["Lat","Long"]);
    
    for i = 1:size(cartesianWaypoints,1)
        deltaEast = cartesianWaypoints.DistEast(i,:);
        deltaNorth = cartesianWaypoints.DistNorth(i,:);
        if (deltaEast==0) && (deltaNorth == 0)
            latLongWaypoints.Lat(i,:) = origin.Lat;
            latLongWaypoints.Long(i,:) = origin.Long;
        else
            theta = atan2(deltaEast,deltaNorth);
            d = sqrt(deltaEast^2 + deltaNorth^2);
            latLongWaypoints.Lat(i,:)=asind(sind(origin.Lat).*cos(d/R)+cosd(origin.Lat).*sin(d/R).*cos(theta));
            latLongWaypoints.Long(i,:) = origin.Long + atan2d(sin(theta).*sin(d/R).*cosd(origin.Lat), cos(d/R)-sind(origin.Lat).*sind(latLongWaypoints.Lat(i,:)));
        end
    end
end