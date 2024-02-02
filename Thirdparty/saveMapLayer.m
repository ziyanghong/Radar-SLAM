name = 'openstreetmap';
url = 'https://a.tile.openstreetmap.org/${z}/${x}/${y}.png';
copyright = char(uint8(169));
attribution = copyright + "OpenStreetMap contributors";
addCustomBasemap(name,url,'Attribution',attribution)
% data = load('geoRoute.mat');
zoomLevel = 20;
player = geoplayer(lat_lon(1,1),lat_lon(1,2),zoomLevel);
plotRoute(player,lat_lon(:,1),lat_lon(:,2));
player.Basemap = 'openstreetmap';
plotRoute(player,lat_lon(:,1),lat_lon(:,2));
