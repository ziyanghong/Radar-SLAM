close all;

%% Generate point cloud from polar image
sequence_path = ['/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/'...
    '2019-01-18-14-14-42-radar-oxford-10k'];
polar_path = strcat( sequence_path, '/radar_zfill_six/');
image_idx = '006789.png';

img = imread(strcat(polar_path, image_idx));
max_selected_distance = 87.5;
radar_resolution = 0.0438;
max_distance  = 163.0;

% Generate feture and point cloud
tic
[feature,pointcloud] = generateGlobalFeature(img, max_selected_distance, radar_resolution, max_distance);
[coeff, scores, latent] = pca(pointcloud(:,1:2)); % PCA
toc
figure;
scatter(pointcloud(:,1),pointcloud(:,2), 100,'.')
% hold on
% pcshow(pointcloud, 'MarkerSize'. 10);
% plotv([coeff(1,1)*50*latent(1)/1000; coeff(2,1)*50*latent(1)/1000], '-b');
% plotv([coeff(1,2)*50*latent(2)/1000; coeff(2,2)*50*latent(2)/1000], '-r');

% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')
% hold off




%% Cartesian image representation
cart_path = strcat( sequence_path, '/701_radar_cart/');
img_cart = imread(strcat(cart_path, image_idx));
% [rows,cols] =  size(img_cart);
% fig1 = figure;
figure;
imshow(img_cart);


% set(gca,'Units','normalized','Position',[0 0 1 1]);  %# Modify axes size
% set(gcf,'Units','pixels','Position',[200 200 cols rows]);  %# Modify figure size
% F = getframe(fig1);
% img = F.cdata;
% imwrite(img, 'loop_origin_image.png');
% 
% fig2=figure;
% imshow(img_cart);
% hold on;
% for i=1:1:size(pointcloud)
%     
%     col = cols/2 + pointcloud(i,1)/(max_selected_distance/(cols/2));
%     row = rows/2 - pointcloud(i,2)/(max_selected_distance/(cols/2));
%     
%     plot(col,row,'yo');    
% end
% set(gca,'Units','normalized','Position',[0 0 1 1]);  %# Modify axes size
% set(gcf,'Units','pixels','Position',[200 200 cols rows]);  %# Modify figure size
% F = getframe(fig2);
% img = F.cdata;
% imwrite(img, 'loop_filtered_pointcloud.png');

%% fft denoise
% % img_cart = imread('/home/hong/Desktop/Texture_Kit_02.jpg');
% figure;
% % subplot(131);
% imshow(img_cart,[]);
% % thresholding
% threshold = 0.3;
% max_intensity = max(img_cart(:));
% img_thres = img_cart;
% img_thres(img_thres< (threshold *max_intensity))=0;
% % subplot(132);
% figure;
% imshow(img_thres);
% % adaptive equalize
% image = adapthisteq(img_thres);
% % subplot(133);
% figure;
% imshow(image);
% 
% % image = img_cart;
% ft = fftshift(fft2(image));
% figure; imagesc(abs(ft));
% [x y ~] = size(ft);
% D = 200;
% mask = fspecial('disk', D) == 0;
% mask = imresize(padarray(mask, [floor((x/2)-D) floor((y/2)-D)], 1, 'both'), [x y]);
% mask = 1.-mask;
% figure; imshow(mask);
% masked_ft = ft .* mask;
% filtered_image = ifft2(ifftshift(masked_ft));
% figure; 
% imshow(filtered_image,[]);


