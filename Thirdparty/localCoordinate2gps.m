function lats_lons = localCoordinate2gps(points, heading, initial_gps)
% 	'''
% 	points: [[x0 y0]...[xN yN]]
% 	heading: heading to east, increment counter clockwise
% 	initial_gps: gps position for the first frame
% 
% 	return gps format:
% 	lat_lon = [
%     (37.771269, -122.511015),
%     (37.773495, -122.464830),
%     (37.774797, -122.454538),
%     (37.771988, -122.454018),
%     (37.773646, -122.440979),
%     (37.772742, -122.440797),
%     (37.771096, -122.453889),
%     (37.768669, -122.453518),
%     (37.766227, -122.460213),
%     (37.764028, -122.510347),
%     (37.771269, -122.511015)
%     ]	
% 
% 	'''

lats_lons = zeros(size(points,1),2);
[x,y,utmzone] = deg2utm(initial_gps(1), initial_gps(2)) ; 
for i = 1: size(points,1)
    pose = points(i,:);
    delta_x = cos(heading) * pose(1) - sin(heading) * pose(2);
    delta_y = sin(heading) * pose(1) + cos(heading) * pose(2);
    
    utm_x_curr = x + delta_x;
    utm_y_curr = y + delta_y;
    
    [Lat,Lon]   = utm2deg(utm_x_curr, utm_y_curr, utmzone); % easting, northing, zone number 
    lats_lons(i,:) = [Lat,Lon];
end
end