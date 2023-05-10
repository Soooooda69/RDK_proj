picture = imread('jhu.png');
% picture = imresize(picture,[800,800]);
% figure;
% imshow(picture)
% [image,map] = rgb2ind(picture,8,'nodither');

% blackPart = (picture(:,:,1)==0)&(picture(:,:,2)==0)&(picture(:,:,3)==0);
% whitePart = (picture(:,:,1)==255)&(picture(:,:,2)==255)&(picture(:,:,3)==255);

% figure;
% imshow(whitePart)
% figure;
% imshow(blackPart)
% picture=regionprops(blackPart,'all');
% whiteRegion=regionprops(whitePart,'all');

%%
gray = im2gray(picture);
% figure;
% imshow(gray)
image = wiener2(gray,[3 3]);
% image = wiener2(image,[2 2]);
image = edge(image,'canny');
image = bwareaopen(image, 110); 
image = wiener2(image,[2 2]);
image = wiener2(image,[2 2]);
image = edge(image,'canny');
image = bwareaopen(image, 110); 
% image = wiener2(image,[2 2]);
% image = wiener2(image,[2 2]);
% image = edge(image,'Canny');
% image = bwareaopen(image, 110); image = bwareaopen(image, 110);  
% image = imfill(image,'holes');
% imshow(image)
figure;
imshow(image)

% Index of pixel where color is black 
[whiteIdxR,whiteIdxC] = find(image==1);
whiteIdxR = whiteIdxR-min(whiteIdxR);
whiteIdxC = whiteIdxC-min(whiteIdxC);
figure;
plot(whiteIdxC,-whiteIdxR+max(whiteIdxR),'k.');
data = [whiteIdxC,-whiteIdxR+max(whiteIdxR)];


% % Index -> position in 15cm x 15cm
% whiteIdxR = whiteIdxR*max(whiteIdxR)/15;
% whiteIdxC = whiteIdxC*max(whiteIdxC)/15;
% 
% % whiteIdxR = smooth(1:numel(whiteIdxR),whiteIdxR);
% % whiteIdxC = smooth(1:numel(whiteIdxR),whiteIdxC);
% 
% plot(whiteIdxR,whiteIdxC,'o')
% 
% % 0.3 0.3 0.1
% frame_0 = [eye(3) [0.3 0.3 0.1]'; 0 0 0 1];   % starting point of drawing under base frame 
% for i = 1:numel(whiteIdxR)
%     frame_g(:,:,i) = frame_0 + [zeros(4,3) [whiteIdxR(i) whiteIdxC(i) 0 0]'];
% end

%%
% figure;
% imcontour(image,1)