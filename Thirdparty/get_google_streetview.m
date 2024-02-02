function varargout = get_google_streetview(loc_v, varargin)
%GET_GOOGLE_STREETVIEW request a Google StreetView image based on the
%specified location and other parameters.
%   filename: get_google_streetview.m
%   created: 01/30/2015
%   modified: 01/30/2015
%   copyright @ 2015 Dongxi Zheng
%
%   input:
%       loc_v - a [lat, lon] vector indicating the geo location, a
%               string describing the location, or a string describing the
%               panorama id of Google StreetView.
%       varargin - optional name-value pairs
%           'width' - requested image width
%           'height' - requested image height
%           'key' - StreetView API key
%           'heading' - camera heading of the image, in degree, 0 is north,
%                       90 is east
%           'fov' - field of view of the image, in degree
%           'pitch' - the pitch of the camera, in degree, 90 up, -90 down,
%                     and 0 straight ahead
%   output:
%       varargout - A image-data matrix or a [X, cmap] pair, consistent
%                   with what imread could output, representing the
%                   requested Google StreetView image

    % validate the location input
    location = struct();
    location.k = 'location';
    if isnumeric(loc_v)
        if numel(loc_v) == 2
            location.v = [num2str(loc_v(1)),',',num2str(loc_v(2))];
        end
    elseif ischar(loc_v)
        location.v = loc_v;
    end
    
    % if location input is in the right format
    if isfield(location,'v')
        
        % set default for optional input and get optional input
        p = inputParser;
        addParameter(p, 'width', 600, @validate_size);
        addParameter(p, 'height', 400, @validate_size);
        addParameter(p, 'key', '', @ischar);
        addParameter(p, 'heading', -1, @isnumeric);
        addParameter(p, 'fov', 90, @validate_fov);
        addParameter(p, 'pitch',0, @validate_pitch);
        parse(p, varargin{:});
        width = ceil(p.Results.width);
        height = ceil(p.Results.height);
        key = p.Results.key;
        heading = p.Results.heading;
        fov = p.Results.fov;
        pitch = p.Results.pitch;
        
        % request the image
        try
            st_url = gen_url(location, width, height, key, heading, fov, pitch);
            if nargout == 1
                varargout{1} = imread(st_url);
            elseif nargout == 2
                [varargout{1}, varargout{2}] = imread(st_url);
            end
        catch err
            % in case the user mean panorama id instead of description for
            % the location, try using the input as a panorama id
            if ischar(location.v)
                location.k = 'panoid';
                st_url = gen_url(location, width, height, key, heading, fov, pitch);
                if nargout == 1
                    varargout{1} = imread(st_url);
                elseif nargout == 2
                    [varargout{1}, varargout{2}] = imread(st_url);
                end
            else
                rethrow(err);
            end
        end

    else
        error('Input of location or api key is not in the right format.');
    end
end


function result = validate_size(wOrH)
% validate width or height input
    result = ((numel(wOrH) == 1) && isnumeric(wOrH) && (wOrH > 0));
end

function result = validate_fov(fov)
% validate field-of-view input
    result = (fov > 0) && isnumeric(fov) && (fov <= 120);
end

function result = validate_pitch(pitch)
% validate pitch input
    result = isnumeric(pitch) && (pitch <= 90) && (pitch >= -90);
end

function st_url = gen_url(location, width, height, key, heading, fov, pitch)
% construct the https request url
    st_url = 'https://maps.googleapis.com/maps/api/streetview?';
    st_url = [st_url, 'size=',num2str(width),'x',num2str(height)];
    st_url = [st_url, '&', location.k,'=',location.v];
    st_url = [st_url, '&fov=',num2str(fov)];
    st_url = [st_url, '&pitch=',num2str(pitch)];
    if ~isempty(key)
        st_url = [st_url, '&key=',key];
    end
    if (heading >= 0) && (heading <= 360)
        st_url = [st_url, '&heading=',num2str(heading)];
    end
end
