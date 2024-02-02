function showStreetViewAndGPS(hax, lat_lon_traj, points, poses, initial_gps)
heading = -1.1215;


%% google street view
figure;
headings = [45,135,225,315];
varargin.fov = 120;
varargin.key = 'AIzaSyArXN_gFzIDy131XzYE2HcSZwlt0qJIPP4';

for i = 1:numel(headings)
    varargin.heading = headings(i);
    
    varargout = get_google_streetview(lat_lon_traj(1,:),varargin);
    subplot(2,2,i);    
    imshow(varargout);   
    title(num2str(headings(i)));

    
end

%% poses to gps trajecotry 
poses_lats_lons = localCoordinate2gps(poses, heading, initial_gps);
% poses_size = repmat(1, size(poses_lats_lons,1), 1);
% poses_color = cellstr(repmat('poses', size(poses_lats_lons,1 ), 1));
% 
% map points to gps position
points_lats_lons = localCoordinate2gps(points, heading, initial_gps);
% points_size = repmat(1, size(points_lats_lons,1), 1);
% points_color = cellstr(repmat('point', size(points_lats_lons,1), 1));
% % 
% % gps trajectory on base map
% raw_gps_size = repmat(1, size(lat_lon_traj,1), 1);
% raw_gps_color = cellstr(repmat('gps', size(lat_lon_traj,1), 1));
% 
% 
% % concate all gps and plot
% gps = [poses_lats_lons; points_lats_lons; lat_lon_traj];
% sizedata = [poses_size; points_size; raw_gps_size];
% 
% colordata = categorical([poses_color;points_color;raw_gps_color]);
% 
% figure;
% name = 'openstreetmap';
% url = 'a.tile.openstreetmap.org';
% copyright = char(uint8(169));
% attribution = copyright + "OpenStreetMap contributors";
% addCustomBasemap(name,url,'Attribution',attribution)
% geobubble(gps(:,1), gps(:,2), sizedata, colordata, 'Basemap','openstreetmap');

%% show parsed openstreetmap and points


plot(hax, points_lats_lons(:,2),points_lats_lons(:,1),'c.', 'MarkerSize',0.2);
plot(hax, poses_lats_lons(:,2),poses_lats_lons(:,1),'m.', 'MarkerSize',1);

end