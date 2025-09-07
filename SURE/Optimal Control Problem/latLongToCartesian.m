function [cartesianWaypoints] = latLongToCartesian(origin, latLongWaypoints)
    %formulae from https://www.movable-type.co.uk/scripts/latlong.html
    %Great Circle distance, some inaccuracy on the order of a few metres
    
    %Earth radius
    R=6371*10^3;
    cartesianWaypoints = table('Size',size(latLongWaypoints),'VariableTypes', ["double", "double"],'VariableNames',["DistNorth","DistEast"]);
    for i = 1:size(latLongWaypoints,1)
        if (latLongWaypoints(i,:).Lat==origin.Lat) && (latLongWaypoints(i,:).Long == origin.Long)
            cartesianWaypoints.DistEast(i,:) = 0;
            cartesianWaypoints.DistNorth(i,:) = 0;
        else
            deltaLat = latLongWaypoints(i,:).Lat - origin.Lat;
            deltaLong = latLongWaypoints(i,:).Long - origin.Long;
    
            a = sind(deltaLat./2).^2+cosd(origin.Lat).*cosd(latLongWaypoints(i,:).Lat).*sind(deltaLong./2).^2;
            c = 2.*atan2(sqrt(a),sqrt(1-a));
            d=R*c;
    
            theta = atan2(sind(deltaLong).*cosd(latLongWaypoints(i,:).Lat),cosd(origin.Lat).*sind(latLongWaypoints(i,:).Lat)-sind(origin.Lat).*cosd(latLongWaypoints(i,:).Lat).*cosd(deltaLong));
            cartesianWaypoints.DistEast(i,:) = d.*sin(theta);
            cartesianWaypoints.DistNorth(i,:) = d.*cos(theta);
        end
    end
end