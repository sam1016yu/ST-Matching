function [speed_limits] = getSpeedLimits(road_types)
%Get speed limit of each road type
    speed_limits = zeros(size(road_types));
    speed_limits = speed_limits';
    for type_idx = 1:length(road_types)
        speed_limits(type_idx) = type2limit(road_types(type_idx));
    end
end

function speed_limit = type2limit(rtype)
% https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed
% country specific. no standards for china yet
switch rtype
    case '#'
        speed_limit = 40;
    case 'motorway'
        speed_limit = 120;
    case 'primary '
        speed_limit = 100;
    case 'residential'
        speed_limit = 40;
    case 'secondary'
        speed_limit = 60;
    case 'service'
        speed_limit = 60;
    case 'tertiary'
        speed_limit = 40;
    case 'trunk'
        speed_limit = 60;
    case 'unclassified '
        speed_limit = 40;
    otherwise
        speed_limit = 40;
end
end